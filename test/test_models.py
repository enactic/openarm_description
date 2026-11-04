# Copyright 2026 Enactic, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
from urllib.parse import urlparse

from support import ROOT, expand, packages


def test_shipped_presets_expand_and_resolve_resources():
    robots = []
    for version in ('v1.0', 'v2.0'):
        folder = ROOT/f'assets/robot/openarm_{version}/config/robot_presets'
        robots.append((version, '<default>', expand(version=version, ros2_control=True,
                       use_fake_hardware=True, emit_grasp_frame=True)))
        for path in sorted(folder.glob('*.yaml')):
            robot = expand(version=version, robot_preset=path.stem, ros2_control=True,
                           use_fake_hardware=True, emit_grasp_frame=True)
            robots.append((version, path.stem, robot))
    for path in sorted((ROOT/'assets/end_effector').glob('*/urdf/*_standalone.urdf.xacro')):
        robots.append(('standalone', path.name, expand(
            entry=str(path.relative_to(ROOT)))))
    for version, name, robot in robots:
        for mesh in robot.findall('.//geometry/mesh'):
            uri = urlparse(mesh.get('filename'))
            if uri.scheme == 'package':
                share = ROOT if uri.netloc == 'openarm_description' else Path(
                    packages.get_package_share_directory(uri.netloc))
                assert (share/uri.path.lstrip('/')
                        ).is_file(), (version, name, mesh.attrib)
            elif uri.scheme in ('', 'file'):
                assert Path(uri.path).is_file(), (version, name, mesh.attrib)
