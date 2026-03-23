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
#    * Neither the name of the Christopher Newport University nor the names of its
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

"""Unit tests for BehaviorLibrary package parsing guard paths."""

import os
import tempfile
import unittest
import zlib
from unittest.mock import patch

from flexbe_core.behavior_library import BehaviorLibrary


class _FakeExport:

    def __init__(self, tagname):
        self.tagname = tagname


class _FakePackage:

    def __init__(self):
        self.exports = [_FakeExport('flexbe_behaviors')]


class TestBehaviorLibrary(unittest.TestCase):
    """Validate behavior library parsing resilience."""

    def test_behavior_library_init_initializes_logger_and_loads_library(self):
        """Construction should initialize logging and immediately parse plus dump the library."""
        sentinel_node = object()

        with patch('flexbe_core.behavior_library.Logger.initialize') as initialize, \
                patch.object(BehaviorLibrary, 'parse_packages') as parse_packages, \
                patch.object(BehaviorLibrary, 'dump_packages') as dump_packages:
            lib = BehaviorLibrary(sentinel_node)

        self.assertIs(lib._node, sentinel_node)
        initialize.assert_called_once_with(sentinel_node)
        parse_packages.assert_called_once_with()
        dump_packages.assert_called_once_with()

    def test_dump_packages_prints_sorted_behavior_ids(self):
        """Debug dumps should present behavior entries ordered by their assigned key."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {
            20: {'class': 'Later', 'name': 'Later Name', 'file': 'later_sm', 'package': 'pkg.two'},
            10: {'class': 'Earlier', 'name': 'Earlier Name', 'file': 'earlier_sm', 'package': 'pkg.one'},
        }

        with patch('builtins.print') as print_mock:
            lib.dump_packages()

        rendered = '\n'.join(call.args[0] for call in print_mock.call_args_list)
        first_index = rendered.index('10')
        second_index = rendered.index('20')
        self.assertLess(first_index, second_index)

    def test_parse_packages_skips_missing_manifest_directory(self):
        """Missing manifest directories should not abort parsing of later packages."""
        lib = object.__new__(BehaviorLibrary)

        def _add_manifests(path, pkg_name):
            if pkg_name == 'broken_pkg':
                raise FileNotFoundError(path)
            lib._behavior_lib[123] = {
                'name': 'Valid Behavior',
                'package': 'valid_pkg.behaviors',
                'file': 'valid_behavior_sm',
                'class': 'ValidBehaviorSM'
            }

        with patch('flexbe_core.behavior_library.get_packages_with_prefixes',
                   return_value={'broken_pkg': '/tmp/broken', 'valid_pkg': '/tmp/valid'}), \
                patch('flexbe_core.behavior_library.parse_package', return_value=_FakePackage()), \
                patch.object(BehaviorLibrary, '_add_behavior_manifests', side_effect=_add_manifests), \
                patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn:
            lib.parse_packages()

        self.assertIn(123, lib._behavior_lib)
        log_warn.assert_called_once()

    def test_parse_packages_reports_unparseable_share_directory(self):
        """Package parse failures should be reported and skipped instead of aborting discovery."""
        lib = object.__new__(BehaviorLibrary)

        with patch('flexbe_core.behavior_library.get_packages_with_prefixes',
                   return_value={'broken_pkg': '/tmp/broken'}), \
                patch('flexbe_core.behavior_library.parse_package', side_effect=OSError('bad package')), \
                patch('builtins.print') as print_mock:
            lib.parse_packages()

        self.assertEqual(lib._behavior_lib, {})
        self.assertIn('cannot be parsed', print_mock.call_args.args[0])

    def test_get_sourcecode_filepath_appends_tmp_suffix(self):
        """Temporary behavior paths should keep the behavior stem and append _tmp before .py."""
        lib = object.__new__(BehaviorLibrary)

        be_entry = {
            'name': 'Valid Behavior',
            'package': 'valid_pkg.behaviors',
            'file': 'valid_behavior_sm',
            'class': 'ValidBehaviorSM'
        }

        class _ImportedModule:
            __path__ = ['/tmp/valid_pkg/behaviors']

        with patch.object(BehaviorLibrary, 'get_behavior', return_value=be_entry), \
                patch('builtins.__import__', return_value=_ImportedModule()):
            source_path = lib.get_sourcecode_filepath(123, add_tmp=False)
            tmp_path = lib.get_sourcecode_filepath(123, add_tmp=True)

        self.assertEqual(source_path, os.path.join('/tmp/valid_pkg/behaviors', 'valid_behavior_sm.py'))
        self.assertEqual(tmp_path, os.path.join('/tmp/valid_pkg/behaviors', 'valid_behavior_sm_tmp.py'))

    def test_add_behavior_manifests_recurses_and_filters_by_package(self):
        """Manifest loading should recurse into subdirectories and ignore manifests from other packages."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}

        with tempfile.TemporaryDirectory() as temp_dir:
            nested_dir = os.path.join(temp_dir, 'nested')
            os.mkdir(nested_dir)

            with open(os.path.join(temp_dir, 'ignore.xml'), 'w', encoding='utf-8') as handle:
                handle.write(
                    '<behavior name="Ignore">'
                    '<executable package_path="other_pkg.behaviors.ignore_sm" class="IgnoreSM"/>'
                    '</behavior>'
                )

            with open(os.path.join(nested_dir, 'match.xml'), 'w', encoding='utf-8') as handle:
                handle.write(
                    '<behavior name="Match">'
                    '<executable package_path="target_pkg.behaviors.match_sm" class="MatchSM"/>'
                    '</behavior>'
                )

            lib._add_behavior_manifests(temp_dir, pkg='target_pkg')

        expected_key = zlib.adler32('target_pkg.behaviors.match_sm'.encode()) & 0x7fffffff
        self.assertEqual(list(lib._behavior_lib.keys()), [expected_key])
        self.assertEqual(lib._behavior_lib[expected_key]['name'], 'Match')
        self.assertEqual(lib._behavior_lib[expected_key]['package'], 'target_pkg.behaviors')
        self.assertEqual(lib._behavior_lib[expected_key]['file'], 'match_sm')
        self.assertEqual(lib._behavior_lib[expected_key]['class'], 'MatchSM')

    def test_add_behavior_manifests_skips_malformed_xml(self):
        """Malformed manifest XML should be logged and skipped without aborting later files."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}

        with tempfile.TemporaryDirectory() as temp_dir:
            with open(os.path.join(temp_dir, 'broken.xml'), 'w', encoding='utf-8') as handle:
                handle.write('<behavior name="Broken"><executable')
            with open(os.path.join(temp_dir, 'valid.xml'), 'w', encoding='utf-8') as handle:
                handle.write(
                    '<behavior name="Valid">'
                    '<executable package_path="target_pkg.behaviors.valid_sm" class="ValidSM"/>'
                    '</behavior>'
                )

            with patch('flexbe_core.behavior_library.Logger.logerr') as logerr:
                lib._add_behavior_manifests(temp_dir, pkg='target_pkg')

        expected_key = zlib.adler32('target_pkg.behaviors.valid_sm'.encode()) & 0x7fffffff
        self.assertIn(expected_key, lib._behavior_lib)
        logerr.assert_called_once()

    def test_add_behavior_manifests_skips_invalid_manifest_structure(self):
        """Invalid manifest structures should be ignored instead of populating the behavior library."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}

        with tempfile.TemporaryDirectory() as temp_dir:
            for index, xml in enumerate([
                '<root name="WrongTag"><executable package_path="pkg.behavior.state" class="State"/></root>',
                '<behavior name="MissingExecutable"></behavior>',
                '<behavior name="MissingPackagePath"><executable class="State"/></behavior>',
                '<behavior name="ShortPackagePath"><executable package_path="state" class="State"/></behavior>',
            ]):
                with open(os.path.join(temp_dir, f'invalid_{index}.xml'), 'w', encoding='utf-8') as handle:
                    handle.write(xml)

            lib._add_behavior_manifests(temp_dir, pkg='pkg')

        self.assertEqual(lib._behavior_lib, {})

    def test_add_behavior_manifests_rejects_duplicate_behavior_ids(self):
        """Behavior ids derived from identical package paths should fail loudly instead of overwriting."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}

        with tempfile.TemporaryDirectory() as temp_dir:
            for filename in ['first.xml', 'second.xml']:
                with open(os.path.join(temp_dir, filename), 'w', encoding='utf-8') as handle:
                    handle.write(
                        '<behavior name="Duplicate">'
                        '<executable package_path="target_pkg.behaviors.same_sm" class="SameSM"/>'
                        '</behavior>'
                    )

            with self.assertRaises(KeyError):
                lib._add_behavior_manifests(temp_dir, pkg='target_pkg')

    def test_find_behavior_refreshes_library_for_package_qualified_lookup(self):
        """Package-qualified lookups should retry after refreshing the behavior library."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}
        expected = {
            'name': 'Target Behavior',
            'package': 'target_pkg.behaviors',
            'file': 'target_behavior_sm',
            'class': 'TargetBehaviorSM',
        }

        def _parse_packages():
            lib._behavior_lib = {321: expected}

        lib.parse_packages = _parse_packages

        with patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn, \
                patch('flexbe_core.behavior_library.Logger.logerr') as log_err:
            result = lib.find_behavior('target_pkg.behaviors/Target Behavior')

        self.assertEqual(result, (321, expected))
        log_warn.assert_called_once()
        log_err.assert_not_called()

    def test_find_behavior_returns_none_when_refresh_still_misses(self):
        """Behavior name lookups should fail cleanly after a refresh if nothing matches."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}
        lib.parse_packages = lambda: None

        with patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn, \
                patch('flexbe_core.behavior_library.Logger.logerr') as log_err:
            result = lib.find_behavior('Missing Behavior')

        self.assertEqual(result, (None, None))
        log_warn.assert_called_once()
        log_err.assert_called_once()

    def test_find_behavior_without_package_returns_first_matching_name(self):
        """Legacy unqualified lookups should return the first matching behavior name."""
        lib = object.__new__(BehaviorLibrary)
        first = {'name': 'Shared Name', 'package': 'pkg.one', 'file': 'one_sm', 'class': 'One'}
        second = {'name': 'Shared Name', 'package': 'pkg.two', 'file': 'two_sm', 'class': 'Two'}
        lib._behavior_lib = {11: first, 22: second}

        result = lib.find_behavior('Shared Name')

        self.assertEqual(result, (11, first))

    def test_get_behavior_refreshes_library_on_cache_miss(self):
        """ID lookups should retry once after refreshing the library."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}
        expected = {
            'name': 'Recovered Behavior',
            'package': 'target_pkg.behaviors',
            'file': 'recovered_behavior_sm',
            'class': 'RecoveredBehaviorSM',
        }

        def _parse_packages():
            lib._behavior_lib = {654: expected}

        lib.parse_packages = _parse_packages

        with patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn:
            result = lib.get_behavior(654)

        self.assertIs(result, expected)
        log_warn.assert_called_once()

    def test_get_behavior_returns_none_after_refresh_miss(self):
        """ID lookups should return None after a refresh if the key is still missing."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {}
        lib.parse_packages = lambda: None

        with patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn:
            result = lib.get_behavior(999)

        self.assertIsNone(result)
        log_warn.assert_called_once()

    def test_count_behaviors_reports_library_size(self):
        """Behavior counts should reflect the current library contents."""
        lib = object.__new__(BehaviorLibrary)
        lib._behavior_lib = {1: object(), 2: object(), 3: object()}

        self.assertEqual(lib.count_behaviors(), 3)

    def test_get_sourcecode_filepath_returns_none_when_behavior_is_missing(self):
        """Missing behavior ids should return None without attempting imports."""
        lib = object.__new__(BehaviorLibrary)

        with patch.object(BehaviorLibrary, 'get_behavior', return_value=None), \
                patch('builtins.__import__') as import_mock:
            source_path = lib.get_sourcecode_filepath(123)

        self.assertIsNone(source_path)
        import_mock.assert_not_called()

    def test_get_sourcecode_filepath_falls_back_to_share_directory(self):
        """Source lookup should fall back to package share lookup when importing the module fails."""
        lib = object.__new__(BehaviorLibrary)
        be_entry = {
            'name': 'Valid Behavior',
            'package': 'valid_pkg.behaviors',
            'file': 'valid_behavior_sm',
            'class': 'ValidBehaviorSM'
        }

        original_import = __import__

        def _fake_import(name, *args, **kwargs):
            if name == 'valid_pkg.behaviors':
                raise ImportError('missing module')
            return original_import(name, *args, **kwargs)

        with patch.object(BehaviorLibrary, 'get_behavior', return_value=be_entry), \
                patch('builtins.__import__', side_effect=_fake_import), \
                patch('ament_index_python.packages.get_package_share_directory',
                      return_value='/tmp/share/valid_pkg.behaviors'), \
                patch('flexbe_core.behavior_library.Logger.logwarn') as log_warn:
            source_path = lib.get_sourcecode_filepath(123)

        self.assertEqual(source_path, os.path.join('/tmp/share/valid_pkg.behaviors', 'valid_behavior_sm.py'))
        log_warn.assert_called_once()


if __name__ == '__main__':
    unittest.main()
