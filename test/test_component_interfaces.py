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

"""Check default and component-owned model generation."""

import os
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET
from xml.parsers.expat import ExpatError

from helpers import ENTRY, ROOT, package_share_at
import xacro
import yaml


def expand(interface=None):
    with tempfile.TemporaryDirectory() as directory:
        share = Path(directory)
        shutil.copytree(ROOT / 'assets', share / 'assets',
                        copy_function=os.symlink)
        product = share / 'assets/body/fixture'
        product.mkdir(parents=True)
        source = ROOT / 'assets/body/v2.0/config'
        (product / 'config').symlink_to(source, target_is_directory=True)
        fallback = share / 'assets/body/fixture_default'
        fallback.mkdir()
        (fallback / 'config').symlink_to(source, target_is_directory=True)
        if interface is not None:
            (product / 'urdf').mkdir()
            (product / 'urdf/component.xacro').write_text(interface)
        preset = {
            'robot': {'root_frame': 'world', 'components': ['first', 'second']},
            'first': {'type': 'body', 'product': 'fixture', 'config': None,
                      'enabled': True, 'inertials': 'nominal',
                      'prefix': 'first_'},
            'second': {'type': 'body', 'product': 'fixture_default', 'config': None,
                       'enabled': True, 'inertials': 'nominal',
                       'prefix': 'second_',
                       'connection': {'attach_to': 'first'}},
        }
        target = share / (
            'assets/robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml')
        target.unlink()
        target.write_text(yaml.safe_dump(preset, sort_keys=False))
        with package_share_at(share):
            return ET.fromstring(xacro.process_file(
                str(ENTRY), mappings={'ros2_control': 'false'}).toxml())


class ComponentInterfaceTest(unittest.TestCase):
    def test_unknown_product_with_no_interface_uses_yaml(self):
        robot = expand()
        first = robot.find("link[@name='first_body_link0']/inertial/mass")
        second = robot.find("link[@name='second_body_link0']/inertial/mass")
        self.assertIsNotNone(first)
        self.assertEqual(first.attrib, second.attrib)

    def test_custom_describe_and_emit_do_not_leak_to_next_component(self):
        interface = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="describe_component" params="component_name component_cfg config_root options out_name">
    <xacro:openarm_describe_config_component component_name="@{component_name}" component_cfg="@{component_cfg}"
      config_root="@{config_root}" options="@{options}" out_name="description"/>
    <xacro:property name="base" value="@{description['topology']['links'][0]}"/>
    <xacro:property name="inertials" value="@{description['inertials']}"/>
    <xacro:property name="inertials" value="@{dict(list(inertials.items()) + [(base, dict(inertials[base], mass=1.0))])}" lazy_eval="false"/>
    <xacro:property name="@{out_name}" value="@{dict(description, inertials=inertials)}" scope="parent"/>
  </xacro:macro>
  <xacro:macro name="emit_component" params="component_desc component_runtime options">
    <xacro:openarm_emit_config_component component_desc="@{component_desc}" component_runtime="@{component_runtime}" options="@{options}"/>
    <link name="@{component_desc['prefix']}custom"/>
    <joint name="@{component_desc['prefix']}custom_joint" type="fixed">
      <parent link="@{component_desc['root_link_actual']}"/>
      <child link="@{component_desc['prefix']}custom"/>
    </joint>
  </xacro:macro>
</robot>'''.replace('@{', '$' + '{')
        robot = expand(interface)
        self.assertEqual(float(robot.find(
            "link[@name='first_body_link0']/inertial/mass").get('value')), 1.0)
        self.assertGreater(float(robot.find(
            "link[@name='second_body_link0']/inertial/mass").get('value')), 3.0)
        self.assertIsNotNone(robot.find("link[@name='first_custom']"))
        self.assertIsNone(robot.find("link[@name='second_custom']"))

    def test_present_interface_errors_do_not_fall_back(self):
        interfaces = [
            '<robot><broken',
            '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="emit_component" params="component_desc component_runtime options">
    <xacro:include filename="@{component_desc['component_dir']}/missing-model.xacro"/>
  </xacro:macro>
</robot>'''.replace('@{', '$' + '{'),
        ]
        for interface in interfaces:
            with self.subTest(interface=interface):
                with self.assertRaises((xacro.XacroException, ExpatError)):
                    expand(interface)


if __name__ == '__main__':
    unittest.main()
