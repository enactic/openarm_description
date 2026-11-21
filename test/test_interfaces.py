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
    files['presets/v2/fixture.yaml']['a']['config'] = dict(mass=7.1)
    files['body/fixture_a/urdf/component.xacro'] = '''<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
  <xacro:macro name="describe_component" params="component_name component_cfg config_root options out_name">
    <xacro:openarm_describe_config_component component_name="${component_name}" component_cfg="${component_cfg}"
      config_root="${config_root}" options="${options}" out_name="d"/>
    <xacro:property name="i" value="${d['inertials']}"/>
    <xacro:property name="i" value="${dict(i, root=dict(i['root'], mass=d['config']['mass']))}" lazy_eval="false"/>
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


def control_input(product='openarm_v2.0'):
    names = ['root'] + [f'link{i}' for i in range(1, 8)]
    data = support.model(names, {f'joint{i}': dict(
        type='revolute', parent=names[i-1], child=names[i]) for i in range(1, 8)})
    data['control/control_gains.yaml'] = {'default': {
        f'joint{i}': dict(kp=10+i, kd=i/10) for i in range(1, 8)}}
    spec = support.preset(arm=support.component('arm', product, 'openarm_right_'),
                          tool=support.component('end_effector', 'pinch_gripper', 'openarm_right_', connection=dict(attach_to='arm')))
    spec['systems'] = {'fixture_system': dict(
        hardware_plugin='openarm_hardware/OpenArmHW', can_interface='can_test', can_fd=True,
        components=dict(arm=dict(joints=list(data['struct/topology.yaml']['joints']), idx=list(range(7)), control_gains='default'),
                        tool=dict(joints=['joint1'], idx=[8])))}
    return spec, {'arm': data, 'tool': support.pinch()}


def test_control_joints_follow_model_names_and_enabled_components():
    spec, data = control_input()
    spec['arm']['prefix'], spec['tool']['prefix'] = 'drive_', 'tool_'
    arm_joints = data['arm']['struct/topology.yaml']['joints']
    arm_joints['joint7']['name'] = 'wrist'
    selection = spec['systems']['fixture_system']['components']['arm']
    selection['joints'].reverse()
    selection['idx'].reverse()
    assert not support.assemble(
        spec, data, ros2_control=False).findall('ros2_control')
    for tool_enabled in (False, True):
        spec['tool']['enabled'] = tool_enabled
        robot = support.assemble(
            spec, data, ros2_control=True, use_fake_hardware=True)
        expected = {spec['arm']['prefix'] + cfg.get('name', name)
                    for name, cfg in arm_joints.items()}
        if tool_enabled:
            expected.update(spec['tool']['prefix'] + cfg.get('name', name)
                            for name, cfg in data['tool']['struct/topology.yaml']['joints'].items()
                            if 'mimic' not in cfg)
        active = {joint.get('name') for joint in robot.findall('joint')
                  if joint.get('type') != 'fixed' and joint.find('mimic') is None}
        assert active == expected
        assert {joint.get('name')
                for joint in robot.findall('ros2_control/joint')} == active
        expected_indices = {spec[component]['prefix'] + data[component]['struct/topology.yaml']['joints'][name].get('name', name): idx
                            for component, selected in spec['systems']['fixture_system']['components'].items()
                            if spec[component]['enabled'] for name, idx in zip(selected['joints'], selected['idx'])}
        actual_indices = {joint.get('name'): int(joint.find("param[@name='idx']").text)
                          for joint in robot.findall('ros2_control/joint')}
        assert actual_indices == expected_indices
        assert list(actual_indices.values()) == sorted(
            expected_indices.values())

    data['arm']['struct/topology.yaml']['collapse_empty_end'] = True
    spec['tool']['connection']['parent_reference_point'] = 'port'
    spec['systems']['fixture_system']['components']['tool']['joints'].append(
        'joint2')
    spec['systems']['fixture_system']['components']['tool']['idx'].append(9)
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

    spec, data = control_input()
    joints = list(data['arm']['struct/topology.yaml']['joints'])
    spec['systems'] = {
        'shoulder': dict(components=dict(arm=dict(joints=joints[:4], idx=[1, 3, 5, 7]))),
        'wrist': dict(components=dict(arm=dict(joints=joints[4:], idx=[1, 3, 5]),
                                      tool=dict(joints=['joint1'], idx=[7])))}
    robot = support.assemble(
        spec, data, ros2_control=True, use_fake_hardware=True)
    for name, cfg in spec['systems'].items():
        expected = {spec[component]['prefix'] + data[component]['struct/topology.yaml']['joints'][joint].get('name', joint) for component,
                    selected in cfg['components'].items() for joint in selected['joints']}
        assert {joint.get('name') for joint in robot.find(
            f"ros2_control[@name='{name}']").findall('joint')} == expected
    spec['systems']['shoulder'].update(
        hardware_plugin='openarm_hardware/OpenArmHW', can_interface='can_test')
    with pytest.raises(xacro.XacroException, match='seven arm joints'):
        support.assemble(spec, data, ros2_control=True,
                         use_fake_hardware=False)
    selection = spec['systems']['wrist']['components']
    selection['arm']['joints'].append(joints[0])
    selection['arm']['idx'].append(9)
    with pytest.raises(xacro.XacroException, match='duplicate joint assignment'):
        support.assemble(spec, data, ros2_control=True, use_fake_hardware=True)
    selection['arm']['joints'][-1] = 'missing'
    with pytest.raises(xacro.XacroException, match='unknown joint'):
        support.assemble(spec, data, ros2_control=True, use_fake_hardware=True)
    selection['arm']['joints'].pop()
    with pytest.raises(xacro.XacroException, match='equally sized'):
        support.assemble(spec, data, ros2_control=True, use_fake_hardware=True)
    selection['arm']['idx'].pop()
    for invalid_idx in (-1, True, 1.5):
        selection['tool']['idx'][0] = invalid_idx
        with pytest.raises(xacro.XacroException, match='non-negative integer idx'):
            support.assemble(spec, data, ros2_control=True,
                             use_fake_hardware=True)
    selection['tool']['idx'][0] = 1
    with pytest.raises(xacro.XacroException, match='duplicate idx'):
        support.assemble(spec, data, ros2_control=True, use_fake_hardware=True)


def test_hardware_parameters_follow_input():
    spec, data = control_input()
    control = spec['systems']['fixture_system']
    data['arm']['control/control_gains.yaml']['tuned'] = {
        name: dict(kp=values['kp']*2, kd=values['kd']*2)
        for name, values in data['arm']['control/control_gains.yaml']['default'].items()}
    control['components']['arm']['control_gains'] = 'tuned'
    control['components']['tool'].update(
        control_gains='default')
    data['tool']['control/control_gains.yaml'] = {
        'default': {'joint1': dict(kp=8., kd=.4)}}
    for prefix, can_interface in [('openarm_left_', 'can_left_test'),
                                  ('openarm_right_', 'can_right_test')]:
        control.update(can_interface=can_interface, can_fd=False)
        for name in ('arm', 'tool'):
            spec[name]['prefix'] = prefix
        robot = support.assemble(spec, data, ros2_control=True,
                                 use_fake_hardware=False)
        hardware = robot.find('ros2_control/hardware')
        assert hardware.find('plugin').text == control['hardware_plugin']
        params = {p.get('name'): p.text for p in hardware.findall('param')}
        assert len(params) == len(hardware.findall('param'))
        assert params['can_interface'] == control['can_interface']
        assert params['can_fd'].lower() == 'false'
        assert params['arm_prefix'] == prefix.removeprefix('openarm_')
        assert params['ee_type'] == spec['tool']['product']
        assert float(params['kp_hand']) == 8. and float(
            params['kd_hand']) == .4
        for name, gains in data['arm']['control/control_gains.yaml'][control['components']['arm']['control_gains']].items():
            for key, value in gains.items():
                assert float(params[key+name.removeprefix('joint')]) == value

    for side in ('left', 'right'):
        spec['systems'] = {f'openarm_{side}_hardware_interface': control}
        control.update(can_fd=True)
        overrides = dict(left_can_interface='runtime_left',
                         right_can_interface='runtime_right', can_fd=False)
        robot = support.assemble(spec, data, ros2_control=True,
                                 use_fake_hardware=False, **overrides)
        hardware = robot.find('ros2_control/hardware')
        params = {p.get('name'): p.text for p in hardware.findall('param')}
        assert params['can_interface'] == overrides[side + '_can_interface']
        assert params['can_fd'].lower() == 'false'

    spec['arm']['product'] = 'openarm_v1.0'
    spec['systems'] = {'openarm_hardware_interface': control}
    for name in ('arm', 'tool'):
        spec[name]['prefix'] = 'openarm_'
    for interface in (None, '', 'can3'):
        overrides = {} if interface is None else dict(can_interface=interface)
        robot = support.assemble(spec, data, version='v1.0', ros2_control=True,
                                 use_fake_hardware=False, **overrides)
        hardware = robot.find('ros2_control/hardware')
        assert hardware.find("param[@name='can_interface']").text == (
            interface or control['can_interface'])

    robot = support.assemble(spec, data, ros2_control=True,
                             use_fake_hardware=True, fake_sensor_commands=True)
    hardware = robot.find('ros2_control/hardware')
    assert hardware.find('plugin').text == 'mock_components/GenericSystem'
    params = {p.get('name'): p.text for p in hardware.findall('param')}
    assert params['mock_sensor_commands'].lower() == 'true'
    assert float(params['position_state_following_offset']) == 0.0
    assert 'fake_sensor_commands' not in params and 'state_following_offset' not in params

    control.update(command_interfaces=[
                   'position'], state_interfaces=['position'])
    robot = support.assemble(
        spec, data, ros2_control=True, use_fake_hardware=True)
    for joint in robot.findall('ros2_control/joint'):
        assert [i.get('name') for i in joint.findall(
            'command_interface')] == ['position']
        assert [i.get('name')
                for i in joint.findall('state_interface')] == ['position']

    # A runtime backend changes the driver, not the selected joints/indices.
    files = support.files_for(spec, data, version='v1.0')
    files['urdf/backend.xacro'] = """<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="$(find openarm_description)/assets/common/robot.xacro" />
      <xacro:openarm_robot robot_preset="v1/fixture" use_fake_hardware="false"
        hardware_overrides="${dict(openarm_hardware_interface=dict(hardware_plugin='fixture/Backend', websocket_port=1337, command_interfaces=['position']))}" />
    </robot>"""
    overridden = support.expand(files, entry='urdf/backend.xacro')
    hardware = overridden.find('ros2_control/hardware')
    assert hardware.find('plugin').text == 'fixture/Backend'
    assert {p.get('name'): p.text for p in hardware.findall('param')} == {
        'websocket_port': '1337'}
    assert [(j.get('name'), j.find("param[@name='idx']").text) for j in overridden.findall('ros2_control/joint')] == [
        (j.get('name'), j.find("param[@name='idx']").text) for j in robot.findall('ros2_control/joint')]


def test_articulated_body_control_uses_declared_names():
    data = support.chain()
    data['struct/topology.yaml']['joints']['pivot']['type'] = 'prismatic'
    spec = support.preset(lift=support.component(prefix='platform_'))
    spec['systems'] = {'lift_control': dict(hardware_plugin='fixture/Lift', device='lift_test',
                                            components=dict(lift=dict(joints=list(data['struct/topology.yaml']['joints']), idx=[0, 1])))}
    robot = support.assemble(spec, {'lift': data},
                             ros2_control=True, use_fake_hardware=False)
    system = robot.find('ros2_control')
    assert system.get('name') == 'lift_control'
    assert system.find(
        'hardware/plugin').text == spec['systems']['lift_control']['hardware_plugin']
    assert system.find("hardware/param[@name='device']").text == 'lift_test'
    expected = {spec['lift']['prefix']+name for name,
                joint in data['struct/topology.yaml']['joints'].items() if joint['type'] != 'fixed'}
    assert {j.get('name') for j in system.findall('joint')} == expected

    robot = support.assemble(spec, {'lift': data}, ros2_control=True,
                             use_fake_hardware=True, fake_sensor_commands=True)
    assert robot.find(
        'ros2_control/hardware/plugin').text == 'mock_components/GenericSystem'
    assert robot.find(
        "ros2_control/hardware/param[@name='mock_sensor_commands']").text.lower() == 'true'
