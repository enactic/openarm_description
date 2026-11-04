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


def control_input():
    names = ['root'] + [f'link{i}' for i in range(1, 8)]
    data = support.model(names, {f'joint{i}': dict(
        type='revolute', parent=names[i-1], child=names[i]) for i in range(1, 8)})
    data['control/control_gains.yaml'] = {
        f'joint{i}': dict(kp=10+i, kd=i/10) for i in range(1, 8)}
    spec = support.preset(arm=support.component('robot', 'fixture_control', 'openarm_right_', control=dict(side='right')),
                          tool=support.component('end_effector', 'pinch_gripper', 'openarm_right_', connection=dict(attach_to='arm')))
    return spec, {'arm': data, 'tool': support.pinch()}


def test_control_joints_follow_model_names_and_enabled_components():
    spec, data = control_input()
    spec['arm']['prefix'], spec['tool']['prefix'] = 'drive_', 'tool_'
    arm_joints = data['arm']['struct/topology.yaml']['joints']
    arm_joints['wrist'] = arm_joints.pop('joint7')
    for filename in ('joint_origins', 'joint_axes', 'joint_limits'):
        values = data['arm'][f'joint/{filename}.yaml']
        values['wrist'] = values.pop('joint7')

    assert not support.assemble(
        spec, data, ros2_control=False).findall('ros2_control')
    for tool_enabled in (False, True):
        spec['tool']['enabled'] = tool_enabled
        robot = support.assemble(
            spec, data, ros2_control=True, use_fake_hardware=True)
        expected = {spec['arm']['prefix'] + name for name in arm_joints}
        if tool_enabled:
            expected.update(spec['tool']['prefix'] + name
                            for name in data['tool']['struct/topology.yaml']['joints']
                            if name not in data['tool']['joint/joint_mimics.yaml'])
        active = {joint.get('name') for joint in robot.findall('joint')
                  if joint.get('type') != 'fixed' and joint.find('mimic') is None}
        assert active == expected
        assert {joint.get('name')
                for joint in robot.findall('ros2_control/joint')} == active

    data['arm']['struct/topology.yaml']['collapse_empty_end'] = True
    spec['tool']['connection']['parent_reference_point'] = 'port'
    files = support.files_for(spec, data)
    files['end_effector/pinch_gripper/urdf/component.xacro'] = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:macro name="describe_component" params="component_name component_cfg config_root options out_name">
        <xacro:openarm_describe_config_component component_name="${component_name}" component_cfg="${component_cfg}"
          config_root="${config_root}" options="${options}" out_name="description"/>
        <xacro:property name="${out_name}" value="${dict(description, joint_mimics=dict())}" scope="parent" lazy_eval="false"/>
      </xacro:macro>
    </robot>'''
    robot = support.expand(files, robot_preset='fixture',
                           ros2_control=True, use_fake_hardware=True)
    assert robot.find("joint[@name='tool_finger_joint2']/mimic") is None
    expected.add('tool_finger_joint2')
    active = {joint.get('name') for joint in robot.findall('joint')
              if joint.get('type') != 'fixed' and joint.find('mimic') is None}
    assert active == expected
    assert {joint.get('name')
            for joint in robot.findall('ros2_control/joint')} == active


def test_hardware_parameters_follow_input():
    spec, data = control_input()
    for side in ('left', 'right'):
        spec['arm']['control']['side'] = side
        for name in ('arm', 'tool'):
            spec[name]['prefix'] = 'openarm_'+side+'_'
        robot = support.assemble(spec, data, ros2_control=True, use_fake_hardware=False,
                                 left_can_interface='can_left_test', right_can_interface='can_right_test', can_fd=False)
        system = robot.find('ros2_control/hardware')
        params = {p.get('name'): p.text for p in system.findall('param')}
        assert params['can_interface'] == 'can_'+side+'_test'
        assert params['can_fd'].lower() == 'false'
        assert params['ee_type'] == spec['tool']['product']
        for name, gains in data['arm']['control/control_gains.yaml'].items():
            for key, value in gains.items():
                assert float(params[key+name.removeprefix('joint')]) == value
