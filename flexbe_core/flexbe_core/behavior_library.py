# Copyright 2023 Philipp Schillinger, Team ViGIR, Christopher Newport University
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
#    * Neither the name of the Philipp Schillinger, Team ViGIR, Christopher Newport University nor the names of its
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


"""Provide access to all known behaviors."""
import os
import xml.etree.ElementTree as ET
import zlib

from ament_index_python import get_packages_with_prefixes
from catkin_pkg.package import parse_package

from flexbe_core.logger import Logger


class BehaviorLibrary:
    """Provide access to all known behaviors."""

    def __init__(self, node):
        self._node = node
        Logger.initialize(node)
        self.parse_packages()
        self.dump_packages()

    def dump_packages(self):
        """Dump to screen for debugging."""
        print("\n\n------------------ Available Behaviors Workspace (sorted by assigned key) ------------------", flush=True)
        sorted_keys = list(self._behavior_lib.keys())
        sorted_keys.sort()
        for be_key in sorted_keys:
            data = self._behavior_lib[be_key]
            print(f"{be_key:10d} : {data['class']:36s} - {data['name']:36s} - {data['file']:30s} - {data['package']}")
        print("-----------------------------------------------------------------------------------------\n\n", flush=True)

    def parse_packages(self):
        """Parse all ROS2 packages to update the internal behavior library."""
        self._behavior_lib = {}
        for pkg_name, pkg_path in get_packages_with_prefixes().items():
            pkg = parse_package(os.path.join(pkg_path, 'share', pkg_name))
            for export in pkg.exports:
                if export.tagname == "flexbe_behaviors":
                    try:
                        self._add_behavior_manifests(os.path.join(pkg_path, 'lib', pkg_name, 'manifest'), pkg_name)
                    except KeyError as exc:
                        print(f"Error : duplicate behavior name found in {pkg_name} \n  {exc}", flush=True)
                        raise exc

    def _add_behavior_manifests(self, path, pkg=None):
        """
        Recursively add all behavior manifests in the given folder to the internal library.

        If a package name is specified, only manifests referring to this package are added.

        @type path: string
        @param path: Path of the folder to be traversed.

        @type pkg: string
        @param pkg: Optional name of a package to only add manifests referring to this package.
        """
        for entry in os.listdir(path):
            entry_path = os.path.join(path, entry)
            if os.path.isdir(entry_path):
                self._add_behavior_manifests(entry_path, pkg)
            elif entry.endswith(".xml") and not entry.startswith("#"):
                mrt = ET.parse(entry_path).getroot()
                # structure sanity check
                if (mrt.tag != "behavior"
                        or len(mrt.findall(".//executable")) == 0
                        or mrt.find("executable").get("package_path") is None
                        or len(mrt.find("executable").get("package_path").split(".")) < 2):
                    continue
                exct = mrt.find("executable")
                if pkg is not None and exct.get("package_path").split(".")[0] != pkg:
                    continue  # ignore if manifest not in specified package
                be_key = zlib.adler32(exct.get("package_path").encode()) & 0x7fffffff
                if be_key in self._behavior_lib:
                    raise KeyError(f"Invalid behavior id key={be_key} for {exct.get('package_path')} "
                                   f"- already exists for {self._behavior_lib[be_key]['package']}")

                self._behavior_lib[be_key] = {
                    "name": mrt.get("name"),
                    "package": ".".join(exct.get("package_path").split(".")[:-1]),
                    "file": exct.get("package_path").split(".")[-1],
                    "class": exct.get("class")
                }

    def get_behavior(self, be_key):
        """
        Provide the library entry corresponding to the given ID.

        @type be_key: int
        @param be_key: Behavior ID key to look up.

        @return Corresponding library entry or None if not found.
        """
        try:
            return self._behavior_lib[be_key]
        except KeyError:
            Logger.logwarn(f"Did not find ID {be_key} in libary, updating...")
            self.parse_packages()
            return self._behavior_lib.get(be_key, None)

    def find_behavior(self, be_identifier):
        """
        Search for a behavior with the given name and returns it along with its ID.

        @type be_identifier: string
        @param be_identifier: Behavior identifier in form of package/"Behavior Name"

        @return Tuple (be_key, be_entry) corresponding to the name or (None, None) if not found.
        """
        if "/" in be_identifier:
            # Identifier in the form of package/Name
            # where only first slash delineates package
            be_split = be_identifier.split("/")
            be_package, be_name = be_split[0], "/".join(be_split[1:])

            def __find_behavior():
                return next((id, be) for (id, be)
                            in self._behavior_lib.items()
                            if be["name"] == be_name and be["package"] == be_package)
        else:
            # Accept older form of only Name, but this will return the first matching name!
            be_package = None
            be_name = be_identifier

            def __find_behavior():
                return next((id, be) for (id, be)
                            in self._behavior_lib.items()
                            if be["name"] == be_name)  # Returns first matching name regardless of package
        try:
            return __find_behavior()
        except StopIteration:
            Logger.logwarn("Did not find behavior '%s' in current libary, updating..." % be_name)
            self.parse_packages()
            try:
                return __find_behavior()
            except StopIteration:
                Logger.logerr("Still cannot find behavior '%s' in libary after update, giving up!" % be_name)
                return None, None

    def count_behaviors(self):
        """
        Count the available behaviors.

        @return Number of behaviors.
        """
        return len(self._behavior_lib)

    def get_sourcecode_filepath(self, be_key, add_tmp=False):
        """
        Construct a file path to the source code of corresponding to the given ID.

        @type be_key: int
        @param be_key: Behavior ID to look up.

        @type add_tmp: bool
        @param add_tmp: Append "_tmp" to the file to consider a temporary version.

        @return String containing the absolute path to the source code file.
        """
        be_entry = self.get_behavior(be_key)
        if be_entry is None:
            # rely on get_behavior to handle/log missing package
            return None

        try:
            module_path = __import__(be_entry["package"]).__path__[-1]
        except ImportError:
            try:
                # Attempt to replace prior use of ROS 1 package finder
                Logger.logwarn(f"""Cannot import behavior package '{be_entry["package"]}', """
                               f"""try using 'get_package_share_directory' instead""")
                from ament_index_python.packages import get_package_share_directory  # pylint: disable=C0415

                module_path = get_package_share_directory(be_entry["package"])
            except Exception as exc:
                Logger.logerr(f"""Cannot import behavior package '{be_entry["package"]}' """)
                raise exc

        filename = be_entry["file"] + '.py' if not add_tmp else '_tmp.py'
        return os.path.join(module_path, filename)
