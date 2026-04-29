#!/usr/bin/env python3

# Copyright 2026 Christopher Newport University
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of Christopher Newport University nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Validate FlexBE state docstring tags used by the WebUI state parser."""

import argparse
import ast
from dataclasses import dataclass
import importlib.util
import os
from pathlib import Path
import re
import sys
from typing import Iterable


TAG_PATTERN = re.compile(r'^(--|>#|#>)\s+([^\s]+)\s+([^\s]+)\s+(.+)$')
OUTCOME_PATTERN = re.compile(r'^(<=)\s+([^\s]+)\s+(.+)$')
TAG_PREFIX_PATTERN = re.compile(r'^(--|>#|#>|<=)')
TYPE_NAME_PATTERN = re.compile(r'^[^\s|]+(?:\|[^\s|]+)*$')
PARAM_TAG = '--'
INPUT_TAG = '>#'
OUTPUT_TAG = '#>'
OUTCOME_TAG = '<='


@dataclass
class DocEntry:
    """Parsed documentation tag entry."""

    symbol: str
    name: str
    type_name: str
    description: str
    line: int


@dataclass
class StateInterface:
    """State interface values read from the constructor."""

    parameters: list[str]
    outcomes: list[str] | None
    input_keys: list[str] | None
    output_keys: list[str] | None


def paths_from_environment(variable='FLEXBE_STATE_DOCSTRING_PATHS'):
    """Return state paths from a path-separator-delimited environment variable."""
    paths = os.environ.get(variable)
    if not paths:
        return []

    return [Path(path).expanduser().resolve() for path in paths.split(os.pathsep) if path]


def _python_files(paths):
    for path in paths:
        if path.is_file() and path.suffix == '.py':
            yield path
        elif path.is_dir():
            yield from sorted(
                candidate for candidate in path.rglob('*.py')
                if '__pycache__' not in candidate.parts
            )


def _literal_string_list(node):
    if isinstance(node, (ast.List, ast.Tuple)):
        values = []
        for element in node.elts:
            if not isinstance(element, ast.Constant) or not isinstance(element.value, str):
                return None
            values.append(element.value)
        return values

    return None


def _is_state_base(base):
    if isinstance(base, ast.Name):
        return base.id.endswith('State')
    if isinstance(base, ast.Attribute):
        return base.attr.endswith('State')
    return False


def _find_init(class_node):
    for node in class_node.body:
        if isinstance(node, ast.FunctionDef) and node.name == '__init__':
            return node
    return None


def _find_super_init(init_node):
    for node in ast.walk(init_node):
        if not isinstance(node, ast.Call):
            continue
        func = node.func
        if not isinstance(func, ast.Attribute) or func.attr != '__init__':
            continue
        value = func.value
        if (
            isinstance(value, ast.Call)
            and isinstance(value.func, ast.Name)
            and value.func.id == 'super'
        ):
            return node
    return None


def _constructor_parameters(init_node):
    return [
        arg.arg for arg in init_node.args.args[1:]
        if arg.arg not in ('self', 'userdata')
    ]


def _interface_from_class(class_node):
    init_node = _find_init(class_node)
    if init_node is None:
        return None

    super_init = _find_super_init(init_node)
    interface = StateInterface(
        parameters=_constructor_parameters(init_node),
        outcomes=None,
        input_keys=None,
        output_keys=None,
    )
    if super_init is None:
        return interface

    for keyword in super_init.keywords:
        if keyword.arg == 'outcomes':
            interface.outcomes = _literal_string_list(keyword.value)
        elif keyword.arg == 'input_keys':
            interface.input_keys = _literal_string_list(keyword.value)
        elif keyword.arg == 'output_keys':
            interface.output_keys = _literal_string_list(keyword.value)

    return interface


def _docstring_start_line(class_node):
    if not class_node.body:
        return class_node.lineno

    first_node = class_node.body[0]
    if (
        isinstance(first_node, ast.Expr)
        and isinstance(first_node.value, ast.Constant)
        and isinstance(first_node.value.value, str)
    ):
        return first_node.lineno

    return class_node.lineno


def _parse_doc_entries(class_node):
    docstring = ast.get_docstring(class_node, clean=False)
    if not docstring:
        return [], [f'{class_node.name}:{class_node.lineno}: missing class docstring']

    errors = []
    entries = []
    start_line = _docstring_start_line(class_node)
    for index, raw_line in enumerate(docstring.strip().splitlines()):
        line_number = start_line + index + 1
        line = raw_line.strip()
        if not line:
            continue

        if not TAG_PREFIX_PATTERN.match(line):
            continue

        if line.startswith(OUTCOME_TAG):
            match = OUTCOME_PATTERN.match(line)
            if not match:
                errors.append(
                    f'{class_node.name}:{line_number}: outcome tag must be '
                    f'"<= name description": {line!r}'
                )
                continue
            entries.append(DocEntry(match[1], match[2], '', match[3], line_number))
            continue

        match = TAG_PATTERN.match(line)
        if not match:
            errors.append(
                f'{class_node.name}:{line_number}: tag must be '
                f'"--|>#|#> name type description": {line!r}'
            )
            continue
        if not TYPE_NAME_PATTERN.match(match[3]) or match[4].startswith('|'):
            errors.append(
                f'{class_node.name}:{line_number}: type alternatives must use '
                f'"|" without whitespace, for example "str|bool": {line!r}'
            )
            continue
        entries.append(DocEntry(match[1], match[2], match[3], match[4], line_number))

    return entries, errors


def _entries_by_symbol(entries, symbol):
    return [entry.name for entry in entries if entry.symbol == symbol]


def _duplicates(names):
    seen = set()
    duplicates = []
    for name in names:
        if name in seen and name not in duplicates:
            duplicates.append(name)
        seen.add(name)
    return duplicates


def _compare_literal_interface(class_name, label, expected, documented):
    if expected is None:
        return []

    errors = []
    missing = [name for name in expected if name not in documented]
    extra = [name for name in documented if name not in expected]
    if missing:
        errors.append(f'{class_name}: missing documented {label}: {missing}')
    if extra:
        errors.append(f'{class_name}: documented unknown {label}: {extra}')
    return errors


def _validate_class(file_path, class_node, strict_interface):
    if not any(_is_state_base(base) for base in class_node.bases):
        return []

    interface = _interface_from_class(class_node)
    if interface is None:
        return []

    entries, errors = _parse_doc_entries(class_node)
    by_symbol = {
        PARAM_TAG: _entries_by_symbol(entries, PARAM_TAG),
        INPUT_TAG: _entries_by_symbol(entries, INPUT_TAG),
        OUTPUT_TAG: _entries_by_symbol(entries, OUTPUT_TAG),
        OUTCOME_TAG: _entries_by_symbol(entries, OUTCOME_TAG),
    }

    for symbol, names in by_symbol.items():
        duplicates = _duplicates(names)
        if duplicates:
            errors.append(f'{class_node.name}: duplicate {symbol} entries: {duplicates}')

    if strict_interface:
        documented_params = by_symbol[PARAM_TAG]
        missing_params = [name for name in interface.parameters if name not in documented_params]
        extra_params = [name for name in documented_params if name not in interface.parameters]
        if missing_params:
            errors.append(f'{class_node.name}: missing documented parameters: {missing_params}')
        if extra_params:
            errors.append(f'{class_node.name}: documented unknown parameters: {extra_params}')

        errors.extend(_compare_literal_interface(
            class_node.name, 'outcomes', interface.outcomes, by_symbol[OUTCOME_TAG]
        ))
        errors.extend(_compare_literal_interface(
            class_node.name, 'input keys', interface.input_keys, by_symbol[INPUT_TAG]
        ))
        errors.extend(_compare_literal_interface(
            class_node.name, 'output keys', interface.output_keys, by_symbol[OUTPUT_TAG]
        ))

    return [f'{file_path}: {error}' for error in errors]


def _validate_file(file_path, strict_interface):
    try:
        tree = ast.parse(file_path.read_text(encoding='utf-8'), filename=str(file_path))
    except SyntaxError as exc:
        return [f'{file_path}: cannot parse Python file: {exc}']

    errors = []
    for node in tree.body:
        if isinstance(node, ast.ClassDef):
            errors.extend(_validate_class(file_path, node, strict_interface))

    return errors


def validate_state_docstrings(paths: Iterable[Path | str], strict_interface=False, verbose=False):
    """Return docstring validation errors for Python state files under paths."""
    resolved_paths = [Path(path).expanduser().resolve() for path in paths]
    files = list(_python_files(resolved_paths))
    if not files:
        return [f'No Python state files found in: {resolved_paths}']

    errors = []
    for file_path in files:
        if verbose:
            print(f"Validating state docstring '{file_path}' ...")
        errors.extend(_validate_file(file_path, strict_interface))

    return errors


def assert_state_docstrings_valid(paths, strict_interface=None):
    """Assert that all FlexBE state docstring tags under paths are valid."""
    if strict_interface is None:
        strict_interface = os.environ.get('FLEXBE_STATE_DOCSTRING_STRICT_INTERFACE') == '1'

    errors = validate_state_docstrings(paths, strict_interface=strict_interface)
    assert not errors, '\n'.join(errors)


def _module_paths(package_name):
    spec = importlib.util.find_spec(package_name)
    if spec is None:
        return []

    if spec.submodule_search_locations:
        return [Path(location).resolve() for location in spec.submodule_search_locations]

    if spec.origin:
        return [Path(spec.origin).resolve()]

    return []


def _package_share_fallbacks(package_name):
    try:
        from ament_index_python.packages import get_package_share_directory
    except ImportError:
        return []

    try:
        share_dir = Path(get_package_share_directory(package_name)).resolve()
    except LookupError:
        return []

    python_version = f'python{sys.version_info.major}.{sys.version_info.minor}'
    install_prefix = share_dir.parents[1]
    candidates = [
        share_dir / package_name,
        install_prefix / 'lib' / python_version / 'site-packages' / package_name,
    ]
    return [candidate for candidate in candidates if candidate.exists()]


def paths_for_package(package_name):
    """Return likely Python state paths for an installed ROS package."""
    return _module_paths(package_name) or _package_share_fallbacks(package_name)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Validate FlexBE state docstring tags for a package or path.',
    )
    parser.add_argument(
        'target',
        help='ROS/Python package name to validate, such as flexbe_states.',
    )
    parser.add_argument(
        '--path',
        action='append',
        dest='paths',
        default=[],
        help=(
            'Explicit Python file or directory path to validate instead of package lookup. '
            'May be used more than once.'
        ),
    )
    parser.add_argument(
        '--no-strict-interface',
        action='store_false',
        dest='strict_interface',
        help='Only validate tag syntax, not constructor/interface coverage.',
    )
    parser.add_argument(
        '--quiet',
        action='store_true',
        help='Suppress per-file validation progress output.',
    )
    parser.set_defaults(strict_interface=True)
    return parser.parse_args(argv)


def main(argv=None):
    """Command-line entry point for state docstring validation."""
    args = _parse_args(sys.argv[1:] if argv is None else argv)
    paths = (
        [Path(path).expanduser() for path in args.paths]
        if args.paths
        else paths_for_package(args.target)
    )
    if not paths:
        print(f'Unable to find Python module paths for package {args.target!r}', file=sys.stderr)
        return 2

    errors = validate_state_docstrings(
        paths,
        strict_interface=args.strict_interface,
        verbose=not args.quiet,
    )
    if errors:
        print('\n'.join(errors), file=sys.stderr)
        return 1

    if not args.quiet:
        print(f'State docstring validation passed for {args.target}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
