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

from xml.parsers.expat import ExpatError
import pytest
import xacro

import support


def test_zed_adapter_forwards_configuration(tmp_path):
    from copy import deepcopy
    from unittest.mock import patch
    vendor = tmp_path/'zed'
    (vendor/'urdf').mkdir(parents=True)
    (vendor/'urdf/zed_macro.urdf.xacro').write_text('''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:macro name="zed_camera" params="name model custom_baseline enable_gnss *origin">
        <link name="${name}_camera_link"/>
        <delegate name="${name}" model="${model}" baseline="${custom_baseline}" gnss="${enable_gnss}">
          <xacro:insert_block name="origin"/>
        </delegate>
      </xacro:macro>
    </robot>''')
    original = support.packages.get_package_share_directory
    config = dict(camera_model='zedm', custom_baseline=.07,
                  enable_gnss=True, gnss_origin=support.pose(x=.02))
    mount = support.pose(.1, -.2, .3, .2, -.1, .4)
    for camera_name in (None, 'inspection'):
        cfg = deepcopy(config)
        if camera_name is not None:
            cfg['camera_name'] = camera_name
        spec = support.preset(support=support.component(product='fixture_support', prefix='support_'),
                              camera=dict(type='sensor', product='zed', config=cfg, prefix='front_', enabled=True,
                                          connection=dict(attach_to='support', origin=mount)),
                              child=support.component(connection=dict(attach_to='camera')))
        with patch.object(support.packages, 'get_package_share_directory', lambda name: str(vendor) if name == 'zed_description' else original(name)):
            robot = support.assemble(
                spec, {'support': support.model(), 'child': support.model()})
        name = camera_name or 'front_zed'
        call = robot.find('delegate')
        assert call.get('name') == name and call.get(
            'model') == cfg['camera_model']
        assert float(call.get('baseline')) == cfg['custom_baseline']
        assert call.get('gnss').lower() == 'true'
        support.near(support.transform(call.find('origin')),
                     support.transform(cfg['gnss_origin']))
        support.near(support.poses(robot)[
                     name+'_camera_link'], support.transform(mount))


def loader_input():
    spec = support.preset(a=support.component(product='fixture_a', prefix='first_'),
                          b=support.component(product='fixture_b', prefix='second_', connection=dict(attach_to='a')))
    second = support.model(('other',))
    second['inertials/nominal.yaml']['other']['mass'] = 4.2
    return support.files_for(spec, {'a': support.model(), 'b': second})


def test_unknown_product_uses_default_generator():
    robot = support.expand(
        loader_input(), robot_preset='fixture', ros2_control=False)
    assert set(support.poses(robot)) == {'world', 'first_root', 'second_other'}


def test_component_hooks_are_scoped_to_their_component():
    files = loader_input()
    files['body/fixture_a/urdf/component.xacro'] = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="describe_component" params="component_name component_cfg config_root options out_name">
    <xacro:openarm_describe_config_component component_name="${component_name}" component_cfg="${component_cfg}"
      config_root="${config_root}" options="${options}" out_name="d"/>
    <xacro:property name="i" value="${d['inertials']}"/>
    <xacro:property name="i" value="${dict(i, root=dict(i['root'], mass=7.1))}" lazy_eval="false"/>
    <xacro:property name="${out_name}" value="${dict(d, inertials=i)}" scope="parent"/>
  </xacro:macro>
  <xacro:macro name="emit_component" params="component_desc component_runtime options">
    <xacro:openarm_emit_config_component component_desc="${component_desc}" component_runtime="${component_runtime}" options="${options}"/>
    <hook name="${component_desc['prefix']}"/>
  </xacro:macro>
</robot>'''
    robot = support.expand(files, robot_preset='fixture', ros2_control=False)
    assert [item.get('name') for item in robot.findall('hook')] == ['first_']
    assert float(robot.find(
        "link[@name='first_root']/inertial/mass").get('value')) == 7.1
    expected = files['body/fixture_b/config/inertials/nominal.yaml']['other']['mass']
    assert float(robot.find(
        "link[@name='second_other']/inertial/mass").get('value')) == expected


@pytest.mark.parametrize('interface', ['<robot><broken', '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="emit_component" params="component_desc component_runtime options">
    <xacro:include filename="${component_desc['component_dir']}/missing.xacro"/>
  </xacro:macro>
</robot>'''])
def test_broken_component_hook_does_not_use_fallback(interface):
    files = loader_input()
    files['body/fixture_a/urdf/component.xacro'] = interface
    with pytest.raises((xacro.XacroException, ExpatError)):
        support.expand(files, robot_preset='fixture', ros2_control=False)
