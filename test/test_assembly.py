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

from copy import deepcopy
import itertools
import pytest
import xacro

from support import assemble, chain, component, installed_tool, model, multiply, near, physical_equal, pinch, pose, poses, preset, transform


@pytest.mark.parametrize('kind', ['robot'])
def test_topology_and_values_come_from_configuration(kind):
    data = chain()
    data['struct/topology.yaml']['joints']['pivot']['type'] = 'prismatic'
    data['struct/topology.yaml']['joints']['end']['type'] = 'fixed'
    data['inertials/nominal.yaml']['root']['mass'] = 3.7
    spec = preset(unit=component(kind))
    first = assemble(spec, {'unit': data})
    for name, item in data['struct/topology.yaml']['joints'].items():
        joint = first.find(f"joint[@name='unit_{name}']")
        assert joint.get('type') == item['type']
        for end in ('parent', 'child'):
            assert joint.find(end).get('link') == 'unit_' + item[end]
        near(transform(joint.find('origin')), transform(
            data['joint/joint_origins.yaml'][name]['origin']))
        if item['type'] != 'fixed':
            assert [float(v) for v in joint.find('axis').get('xyz').split()] == list(
                data['joint/joint_axes.yaml'][name]['axis'].values())
            for key, value in data['joint/joint_limits.yaml'][name]['limit'].items():
                assert float(joint.find('limit').get(key)) == value
    inertial = first.find("link[@name='unit_root']/inertial")
    assert float(inertial.find('mass').get('value')
                 ) == data['inertials/nominal.yaml']['root']['mass']
    near(transform(inertial.find('origin')), transform(
        data['inertials/nominal.yaml']['root']['origin']))
    data['struct/topology.yaml']['joints'] = dict(
        reversed(list(data['struct/topology.yaml']['joints'].items())))
    second = assemble(spec, {'unit': data})
    assert poses(first) == poses(second)


@pytest.mark.parametrize('product', ['fixture_tool', 'pinch_gripper'])
def test_names_and_mimics_follow_configuration(product):
    data = pinch()
    topology = data['struct/topology.yaml']
    topology['joints']['joint1']['name'] = 'drive'
    topology['joints']['joint2']['name'] = 'follower'
    topology['links']['base_link']['name'] = 'tool_base'
    topology['links']['link1']['name'] = 'moving_finger'
    topology['links']['link2'].pop('name')
    for item in data['struct/topology.yaml']['joints'].values():
        item['type'] = 'prismatic'
    for joint in data['joint/joint_axes.yaml'].values():
        joint['axis'].update(x=.6, y=.8, z=0.)
    spec = preset(a=component('end_effector', product, 'alpha_'),
                  b=component('end_effector', product, 'beta_', connection=dict(attach_to='a'),
                              reflect=dict(x=1, y=-1, z=1)))
    robot = assemble(spec, {'a': data, 'b': data})
    names = {key: cfg.get('name', key) for key, cfg in
             data['struct/topology.yaml']['joints'].items()}
    assert {link.get('name') for link in robot.findall('link')} == {'world'} | {
        prefix + cfg.get('name', key) for prefix in ('alpha_', 'beta_')
        for key, cfg in topology['links'].items()}
    mimic = data['joint/joint_mimics.yaml']['joint2']
    for prefix in ('alpha_', 'beta_'):
        joint = robot.find(f"joint[@name='{prefix}follower']")
        assert joint.get('type') == 'prismatic'
        for name, configured in data['joint/joint_limits.yaml'].items():
            limit = robot.find(f"joint[@name='{prefix}{names[name]}']/limit")
            for key in ('lower', 'upper'):
                assert float(limit.get(key)) == configured['limit'][key]
        assert joint.find('mimic').get(
            'joint') == prefix + names[mimic['joint']]
        for key in ('multiplier', 'offset'):
            assert float(joint.find('mimic').get(key)) == mimic[key]

    reflection = [[float(i == j) * (1, -1, 1, 1)[i]
                   for j in range(4)] for i in range(4)]
    for name in data['struct/topology.yaml']['joints']:
        right = robot.find(f"joint[@name='alpha_{names[name]}']")
        left = robot.find(f"joint[@name='beta_{names[name]}']")
        for endpoint in ['lower', 'upper']:
            position = float(right.find('limit').get(endpoint))
            motions = []
            for joint in [right, left]:
                axis = [float(v)
                        for v in joint.find('axis').get('xyz').split()]
                translation = [[float(i == j) for j in range(3)] + [axis[i] * position]
                               for i in range(3)] + [[0., 0., 0., 1.]]
                motions.append(
                    multiply(transform(joint.find('origin')), translation))
            near(motions[1], multiply(reflection,
                 multiply(motions[0], reflection)))


def test_reference_emission_preserves_composed_mount_pose():
    parent, child = model(), model()
    offset = pose(.03, -.02, .01, .1, .2, -.3)
    spec = preset(parent=component(product='parent', prefix='parent_',
                                   reflect=dict(x=1, y=-1, z=1)),
                  child=component(product='child', prefix='child_',
                                  connection=dict(attach_to='parent', parent_reference_point='port', origin=offset)))
    reflection = [[float(i == j) * (1, -1, 1, 1)[i]
                   for j in range(4)] for i in range(4)]
    reference = transform(
        parent['struct/reference_points.yaml']['port']['origin'])
    expected = multiply(multiply(reflection, multiply(
        reference, reflection)), transform(offset))
    for emit in (False, True):
        parent['struct/reference_points.yaml']['port']['emit'] = emit
        robot = assemble(spec, {'parent': parent, 'child': child})
        assert ('parent_port' in poses(robot)) == emit
        near(poses(robot)['child_root'], expected)


def test_grasp_emission_combines_configuration_and_argument():
    spec = preset(tool=component('end_effector', 'pinch_gripper', 'tool_'))
    for emit, argument in itertools.product((False, True), repeat=2):
        data = pinch()
        data['struct/reference_points.yaml']['grasp_frame']['emit'] = emit
        robot = assemble(spec, {'tool': data}, emit_grasp_frame=argument)
        assert len(robot.findall(
            "link[@name='tool_grasp_frame']")) == int(emit or argument)


@pytest.mark.parametrize('tool', [False, True])
def test_overrides_are_applied_before_reflection(tool):
    data = pinch() if tool else chain()
    name = 'joint1' if tool else 'pivot'
    origin = pose(.03, .04, .05, .2, -.3, .4)
    limit = dict(lower=-.2, upper=.7, effort=12, velocity=4)
    spec = preset(unit=component('end_effector' if tool else 'robot',
                                 'pinch_gripper' if tool else 'fixture',
                                 reflect=dict(x=1, y=-1, z=1),
                                 origin_override_joints={name: origin}, limit_override_joints={name: limit}))
    robot = assemble(spec, {'unit': data})
    emitted_name = data['struct/topology.yaml']['joints'][name].get(
        'name', name)
    joint = robot.find(f"joint[@name='unit_{emitted_name}']")
    expected = deepcopy(origin)
    for key in ('y', 'roll', 'yaw'):
        expected[key] *= -1
    near(transform(joint.find('origin')), transform(expected))
    assert float(joint.find('limit').get('lower')) == -limit['upper']
    assert float(joint.find('limit').get('upper')) == -limit['lower']
    for key in ('effort', 'velocity'):
        assert float(joint.find('limit').get(key)) == limit[key]


@pytest.mark.parametrize('product', ['fixture_tool', 'pinch_gripper'])
def test_reflection_selection_preserves_unselected_joints(product):
    data = pinch()
    for joint in data['joint/joint_axes.yaml'].values():
        joint['axis'].update(x=.6, y=.8, z=0.)
    spec = preset(tool=component('end_effector', product, 'tool_',
                                 reflect=dict(x=1, y=-1, z=1, include=dict(links=['base_link'], joints=[], reference_points=[]))))
    robot = assemble(spec, {'tool': data})
    base = robot.find("link[@name='tool_ee_base_link']/inertial")
    source = data['inertials/nominal.yaml']['base_link']
    assert float(base.find('mass').get('value')) == source['mass']
    expected = deepcopy(source['origin'])
    expected['y'] *= -1
    near(transform(base.find('origin')), transform(expected))
    for key, value in source['inertia'].items():
        sign = -1 if key in ('xy', 'yz') else 1
        assert float(base.find('inertia').get('i'+key)) == sign * value
    for name in data['struct/topology.yaml']['joints']:
        emitted_name = data['struct/topology.yaml']['joints'][name].get(
            'name', name)
        joint = robot.find(f"joint[@name='tool_{emitted_name}']")
        near(transform(joint.find('origin')), transform(
            data['joint/joint_origins.yaml'][name]['origin']))
        assert [float(v) for v in joint.find('axis').get('xyz').split()] == list(
            data['joint/joint_axes.yaml'][name]['axis'].values())
        for key, value in data['joint/joint_limits.yaml'][name]['limit'].items():
            assert float(joint.find('limit').get(key)) == value


def test_folding_preserves_physical_properties_and_frames():
    spec, data = installed_tool(True)
    joints = data['arm']['struct/topology.yaml']['joints']
    joints['end']['parent'] = 'root'
    joints['pivot']['parent'] = 'tip'
    robot = assemble(spec, data)
    assert robot.find("link[@name='arm_tip']") is not None

    spec, data = installed_tool(False)
    data['tool']['struct/topology.yaml']['joints']['joint2']['parent'] = 'link1'
    data['arm']['struct/reference_points.yaml']['port']['emit'] = True
    geometry = dict(mesh='package://fixture_meshes/shape.stl',
                    scale=dict(x=1, y=1, z=1))
    shape = dict(origin=pose(.02, -.04, .01, .2, .1, -.3), geometry=geometry)
    data['tool']['link/visuals.yaml']['base_link'] = deepcopy(shape)
    data['tool']['link/collisions.yaml']['base_link'] = deepcopy(shape)
    spec['tool']['reflect'] = dict(x=1, y=-1, z=1, include=dict(
        links=['base_link'], joints=[], reference_points=[]))
    spec['tool']['origin_override_joints'] = {
        'joint1': pose(.04, .03, .02, .3, .2, .1)}
    first = assemble(spec, data, emit_grasp_frame=True)
    data['arm']['struct/topology.yaml']['collapse_empty_end'] = True
    second = assemble(spec, data, emit_grasp_frame=True)
    physical_equal(first, second)
    for frame in ('arm_port', 'tool_grasp_frame'):
        near(poses(first)[frame], poses(second)[frame])
    mass = data['tool']['inertials/nominal.yaml']['base_link']['mass']
    assert float(second.find(
        "link[@name='tool_ee_base_link']/inertial/mass").get('value')) == mass


@pytest.mark.parametrize('mode', ['missing', 'off', 'on', 'physical', 'unattached'])
def test_folding_requires_enabled_empty_tip_and_child(mode):
    spec, data = installed_tool(mode not in ('missing', 'off'))
    if mode == 'missing':
        del data['arm']['struct/topology.yaml']['collapse_empty_end']
    if mode == 'physical':
        data['arm']['inertials/nominal.yaml']['tip'] = deepcopy(
            data['arm']['inertials/nominal.yaml']['root'])
    if mode == 'unattached':
        spec = preset(arm=spec['arm'])
        del data['tool']
    robot = assemble(spec, data)
    assert (robot.find("link[@name='arm_tip']") is None) == (mode == 'on')


@pytest.mark.parametrize('error', ['joint_parent', 'mount_reference', 'parent_selectors'])
def test_invalid_attachment_references_are_rejected(error):
    spec, data = installed_tool()
    if error == 'joint_parent':
        data['arm']['struct/topology.yaml']['joints']['pivot']['parent'] = 'absent'
    elif error == 'mount_reference':
        spec['tool']['connection']['parent_reference_point'] = 'absent'
    else:
        spec['tool']['connection']['parent'] = 'world'
    with pytest.raises(xacro.XacroException):
        assemble(spec, data)
