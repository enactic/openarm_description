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

"""Check component mounting and reference frame emission."""

import copy
import os
from pathlib import Path
import shutil
import tempfile
import unittest
import xml.etree.ElementTree as ET

from helpers import (
    ENTRY,
    multiply,
    package_share_at,
    poses,
    ROOT,
    transform,
)
import xacro
import yaml


def expand_modified(changes=None, collapse_empty_end=None, **mappings):
    """Expand the V2 model with changed YAML files.

    changes maps a YAML path relative to assets/ to a function that
    receives the loaded YAML value and returns the changed value.
    """
    arguments = {
        'robot_preset': 'default_bimanual',
        'emit_grasp_frame': 'true',
    }
    arguments.update(
        {
            key: str(value).lower() if isinstance(value, bool) else value
            for key, value in mappings.items()
        }
    )
    with tempfile.TemporaryDirectory() as share:
        # Link all assets and replace only the changed YAML files.
        shutil.copytree(ROOT / 'assets', Path(share) / 'assets',
                        copy_function=os.symlink)
        for path, change in (changes or {}).items():
            target = Path(share) / 'assets' / path
            value = change(yaml.safe_load(target.read_text()))
            target.unlink()
            target.write_text(yaml.safe_dump(value, sort_keys=False))
        if collapse_empty_end is not None:
            target = Path(
                share) / 'assets/robot/openarm_v2.0/config/arm/struct/topology.yaml'
            value = yaml.safe_load(target.read_text())
            value['collapse_empty_end'] = collapse_empty_end
            target.unlink()
            target.write_text(yaml.safe_dump(value, sort_keys=False))
        with package_share_at(share):
            robot = ET.fromstring(
                xacro.process_file(str(ENTRY), mappings=arguments).toxml()
            )
    for tag in ['link', 'joint']:
        names = [item.get('name') for item in robot.findall(tag)]
        if len(names) != len(set(names)):
            raise AssertionError(f'Duplicate {tag} names')
    poses(robot)
    return robot


def set_reference_emit(reference, enabled):
    def change(value):
        point = value[reference]
        point['emit'] = enabled
        value[reference] = point
        return value

    return change


class AssemblyTest(unittest.TestCase):
    def assert_common_poses(self, first, second):
        a, b = poses(first), poses(second)
        for name in a.keys() & b.keys():
            with self.subTest(link=name):
                self.assertLess(
                    max(
                        abs(x - y)
                        for row_a, row_b in zip(a[name], b[name])
                        for x, y in zip(row_a, row_b)
                    ),
                    1e-10,
                )

    def test_rotated_offset_on_mirrored_gripper_reference(self):
        for offset in [None, {
            'x': 0.012, 'y': -0.008, 'z': 0.005,
            'roll': 0.3, 'pitch': -0.2, 'yaw': 0.4,
        }]:
            def add_probe(value):
                robot = value['robot']
                robot['components'] = list(
                    robot['components']) + ['mount_probe']
                value['robot'] = robot
                probe = copy.deepcopy(value['body'])
                probe['prefix'] = 'test_probe_'
                probe['connection'] = {
                    'attach_to': 'primary_arm_end_effector',
                    'parent_reference_point': 'camera_view_frame',
                    'origin': offset,
                }
                value['mount_probe'] = probe
                return value

            for collapse in [False, True]:
                robots = [expand_modified({
                    'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': add_probe,
                    'end_effector/pinch_gripper/config/struct/reference_points.yaml':
                        set_reference_emit('camera_view_frame', emit),
                }, collapse_empty_end=collapse) for emit in [False, True]]
                values = [poses(robot) for robot in robots]
                for robot in robots:
                    self.assertIsNotNone(robot.find(
                        "link[@name='test_probe_body_link0']"))
                self.assert_common_poses(*robots)
                delta = offset or dict.fromkeys(
                    ['x', 'y', 'z', 'roll', 'pitch', 'yaw'], 0.0)
                expected = multiply(
                    values[1]['openarm_left_camera_view_frame'],
                    transform(ET.Element('origin', {
                        'xyz': ' '.join(str(delta[k]) for k in ['x', 'y', 'z']),
                        'rpy': ' '.join(str(delta[k]) for k in ['roll', 'pitch', 'yaw']),
                    })),
                )
                for value in values:
                    self.assertLess(max(
                        abs(a - b) for row, target in zip(value['test_probe_body_link0'], expected)
                        for a, b in zip(row, target)), 1e-10)

    def test_arm_tip_reference_emit_with_and_without_collapse(self):
        for collapse in [False, True]:
            robots = [
                expand_modified(
                    {
                        'robot/openarm_v2.0/config/arm/struct/reference_points.yaml': set_reference_emit(
                            'ee_mount_point', emit
                        ),
                    },
                    collapse_empty_end=collapse,
                )
                for emit in [False, True]
            ]
            self.assert_common_poses(*robots)

    def test_grasp_frame_emission(self):
        # grasp_frame is emitted once when either the reference point's
        # emit or the emit_grasp_frame argument is enabled.
        for emit in [False, True]:
            for enabled in [False, True]:
                with self.subTest(emit=emit, emit_grasp_frame=enabled):
                    robot = expand_modified(
                        {
                            'end_effector/pinch_gripper/config/struct/reference_points.yaml': set_reference_emit(
                                'grasp_frame', emit
                            ),
                        },
                        emit_grasp_frame=enabled,
                    )
                    for side in ['left', 'right']:
                        self.assertEqual(
                            len(robot.findall(
                                f"link[@name='openarm_{side}_grasp_frame']")),
                            1 if emit or enabled else 0,
                        )

    def test_conflicting_parent_selectors_rejected(self):
        def conflict(value):
            gripper = value['primary_arm_end_effector']
            connection = gripper['connection']
            connection['parent'] = 'openarm_body_link0'
            gripper['connection'] = connection
            value['primary_arm_end_effector'] = gripper
            return value

        with self.assertRaisesRegex(
            xacro.XacroException, 'choose parent or parent_reference_point'
        ):
            expand_modified(
                {'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': conflict})

    def test_body_can_assemble_without_arms(self):
        def body_only(value):
            robot = value['robot']
            robot['components'] = ['body']
            value['robot'] = robot
            return value

        robot = expand_modified(
            {'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': body_only})
        body = robot.find("link[@name='openarm_body_link0']")
        self.assertIsNotNone(body)
        self.assertTrue(body.find('visual/geometry/mesh').get('filename').startswith(
            'package://openarm_description/assets/body/v2.0/'))
        self.assertFalse(robot.findall('ros2_control'))
        self.assertFalse([joint for joint in robot.findall('joint')
                          if joint.get('type') != 'fixed'])

    def test_camera_mount_emit_preserves_pose(self):
        for parent_file in [
            'body/v2.0/config/struct/reference_points.yaml',
            'accessory/zed_mount/config/struct/reference_points.yaml',
        ]:
            off, on = [
                expand_modified(
                    {parent_file: set_reference_emit(
                        'camera_mount_point', emit)}
                )
                for emit in [False, True]
            ]
            self.assert_common_poses(off, on)

    def test_controls_reference_existing_active_joints(self):
        preset_dir = ROOT / 'assets/robot/openarm_v2.0/config/robot_presets'
        for preset in sorted(preset_dir.glob('*.yaml')):
            with self.subTest(preset=preset.stem):
                robot = expand_modified(robot_preset=preset.stem)
                joints = {j.get('name'): j for j in robot.findall('joint')}
                controlled = [
                    j.get('name') for j in robot.findall('ros2_control/joint')
                ]
                active = {
                    name
                    for name, joint in joints.items()
                    if joint.get('type') != 'fixed' and joint.find('mimic') is None
                }
                self.assertEqual(set(controlled), active)
                self.assertEqual(len(controlled), len(set(controlled)))

    def test_control_output_can_be_disabled(self):
        robot = expand_modified(ros2_control=False)
        self.assertEqual(robot.findall('ros2_control'), [])
        self.assertIsNotNone(robot.find("joint[@name='openarm_right_joint1']"))

    def test_control_names_follow_component_prefixes(self):
        def rename(value):
            for name, prefix in [
                ('primary_arm', 'custom_arm_'),
                ('primary_arm_end_effector', 'custom_tool_'),
            ]:
                component = value[name]
                component['prefix'] = prefix
                value[name] = component
            return value

        robot = expand_modified(
            {'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': rename})
        controlled = {j.get('name')
                      for j in robot.findall('ros2_control/joint')}
        self.assertIn('custom_arm_joint1', controlled)
        self.assertIn('custom_tool_finger_joint1', controlled)
        self.assertNotIn('openarm_left_joint1', controlled)
        self.assertTrue(controlled <= {j.get('name')
                        for j in robot.findall('joint')})

    def test_real_hardware_parameters_are_preserved(self):
        robot = expand_modified(
            use_fake_hardware=False,
            left_can_interface='left_test',
            right_can_interface='right_test',
            can_fd=False,
        )
        gains = xacro.load_yaml(
            str(
                ROOT / 'assets/robot/openarm_v2.0/config/arm/control/control_gains.yaml'
            )
        )
        for side in ['left', 'right']:
            system = robot.find(
                f"ros2_control[@name='openarm_{side}_hardware_interface']"
            )
            hardware = system.find('hardware')
            self.assertEqual(hardware.find('plugin').text,
                             'openarm_hardware/OpenArmHW')
            params = {p.get('name'): p.text for p in hardware.findall('param')}
            self.assertEqual(params['arm_type'], 'v20')
            self.assertEqual(params['arm_prefix'], side + '_')
            self.assertEqual(params['can_interface'], side + '_test')
            self.assertEqual(params['can_fd'].lower(), 'false')
            self.assertEqual(params['ee_type'], 'pinch_gripper')
            for i in range(1, 8):
                for gain in ['kp', 'kd']:
                    self.assertEqual(
                        float(params[f'{gain}{i}']), gains[f'joint{i}'][gain]
                    )

    def test_joint_order_does_not_define_the_arm_chain(self):
        def reorder(value):
            value['joints'] = dict(reversed(list(value['joints'].items())))
            return value

        original = expand_modified()
        reordered = expand_modified(
            {'robot/openarm_v2.0/config/arm/struct/topology.yaml': reorder})
        self.assert_common_poses(original, reordered)

    def test_arm_joint_names_come_from_topology(self):
        def rename_topology(value):
            joints = value['joints']
            joints['wrist'] = joints.pop('joint7')
            value['joints'] = joints
            return value

        def rename_parameters(value):
            value['wrist'] = value.pop('joint7')
            return value

        changes = {
            'robot/openarm_v2.0/config/arm/struct/topology.yaml': rename_topology}
        for filename in ['joint_origins.yaml', 'joint_axes.yaml', 'joint_limits.yaml']:
            changes['robot/openarm_v2.0/config/arm/joint/' +
                    filename] = rename_parameters
        robot = expand_modified(changes, robot_preset='right_arm')
        self.assertIsNotNone(robot.find("joint[@name='openarm_right_wrist']"))
        self.assertIsNone(robot.find("joint[@name='openarm_right_joint7']"))
        self.assertIsNotNone(
            robot.find("ros2_control/joint[@name='openarm_right_wrist']")
        )

    def test_joint_type_and_parent_come_from_topology(self):
        def arm_change(value):
            joints = value['joints']
            joint = joints['joint7']
            joint['type'] = 'fixed'
            joint['parent'] = 'link5'
            joints['joint7'] = joint
            value['joints'] = joints
            return value

        robot = expand_modified(
            {'robot/openarm_v2.0/config/arm/struct/topology.yaml': arm_change}, robot_preset='right_arm'
        )
        joint = robot.find("joint[@name='openarm_right_joint7']")
        self.assertEqual(joint.get('type'), 'fixed')
        self.assertEqual(joint.find('parent').get(
            'link'), 'openarm_right_link5')
        self.assertIsNone(joint.find('axis'))
        self.assertIsNone(joint.find('limit'))

    def test_gripper_joint_type_comes_from_topology(self):
        def change(value):
            joints = value['joints']
            for name in joints:
                joint = joints[name]
                joint['type'] = 'prismatic'
                joints[name] = joint
            value['joints'] = joints
            return value

        robot = expand_modified(
            {'end_effector/pinch_gripper/config/struct/topology.yaml': change})
        for side in ['left', 'right']:
            for index in [1, 2]:
                self.assertEqual(
                    robot.find(
                        f"joint[@name='openarm_{side}_finger_joint{index}']"
                    ).get('type'),
                    'prismatic',
                )

    def test_pinch_overrides_precede_reflection_and_survive_collapse(self):
        def override(preset):
            component = preset['primary_arm_end_effector']
            component['origin_override_joints'] = {'finger_joint1': {
                'x': .03, 'y': .015, 'z': .01,
                'roll': .2, 'pitch': .1, 'yaw': .3}}
            component['limit_override_joints'] = {'finger_joint1': {
                'lower': -.2, 'upper': .6, 'effort': 7, 'velocity': 10}}
            preset['primary_arm_end_effector'] = component
            return preset

        first, second = [expand_modified(
            {'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': override},
            collapse_empty_end=collapse,
        ) for collapse in [False, True]]
        for robot in [first, second]:
            joint = robot.find("joint[@name='openarm_left_finger_joint1']")
            limit = joint.find('limit')
            self.assertEqual(float(limit.get('lower')), -.6)
            self.assertEqual(float(limit.get('upper')), .2)
            values = [float(v)
                      for v in joint.find('origin').get('rpy').split()]
            for actual, expected in zip(values, [-.2, .1, -.3]):
                self.assertAlmostEqual(actual, expected)
        a, b = poses(first), poses(second)
        for name in ['openarm_left_ee_link1', 'openarm_left_ee_link2']:
            self.assertLess(max(abs(x-y) for ra, rb in zip(a[name], b[name])
                                for x, y in zip(ra, rb)), 1e-10)

    def test_pinch_excluded_joints_keep_their_axes(self):
        def exclude(preset):
            component = preset['primary_arm_end_effector']
            component['reflect'] = {
                'x': -1, 'y': 1, 'z': 1,
                'include': {'links': ['ee_base_link'], 'joints': [],
                            'reference_points': []},
            }
            preset['primary_arm_end_effector'] = component
            return preset

        robot = expand_modified(
            {'robot/openarm_v2.0/config/robot_presets/default_bimanual.yaml': exclude})
        for index in [1, 2]:
            left = robot.find(
                f"joint[@name='openarm_left_finger_joint{index}']")
            right = robot.find(
                f"joint[@name='openarm_right_finger_joint{index}']")
            self.assertEqual(left.find('axis').attrib,
                             right.find('axis').attrib)
            self.assertEqual(left.find('limit').attrib,
                             right.find('limit').attrib)

    def test_unknown_topology_link_rejected(self):
        def change(value):
            joints = value['joints']
            joint = joints['joint7']
            joint['parent'] = 'missing'
            joints['joint7'] = joint
            value['joints'] = joints
            return value

        with self.assertRaisesRegex(xacro.XacroException, 'unknown link'):
            expand_modified(
                {'robot/openarm_v2.0/config/arm/struct/topology.yaml': change})

    def test_single_arm_control_uses_selected_side(self):
        for side in ['left', 'right']:
            robot = expand_modified(
                robot_preset=side + '_arm',
                use_fake_hardware=False,
                left_can_interface='left_test',
                right_can_interface='right_test',
            )
            systems = robot.findall('ros2_control')
            self.assertEqual(len(systems), 1)
            self.assertEqual(
                systems[0].get('name'), 'openarm_' +
                side + '_hardware_interface'
            )
            params = {
                p.get('name'): p.text for p in systems[0].findall('hardware/param')
            }
            self.assertEqual(params['can_interface'], side + '_test')
            self.assertEqual(params['ee_type'], None)
            self.assertEqual(len(systems[0].findall('joint')), 7)

    def test_arm_topology_selects_empty_tip_collapse(self):
        enabled = expand_modified()
        disabled = expand_modified(collapse_empty_end=False)
        for side in ['left', 'right']:
            self.assertIsNone(enabled.find(
                f"link[@name='openarm_{side}_link7']"))
            self.assertIsNotNone(disabled.find(
                f"link[@name='openarm_{side}_link7']"))
            joint = enabled.find(f"joint[@name='openarm_{side}_joint7']")
            self.assertEqual(joint.find('child').get('link'),
                             f'openarm_{side}_ee_base_link')
            self.assertIsNotNone(disabled.find(
                f"joint[@name='openarm_{side}_ee_base_link_mount_joint']"))

    def test_omitted_arm_collapse_flag_keeps_tip(self):
        def omit(value):
            value.pop('collapse_empty_end', None)
            return value
        robot = expand_modified(
            {'robot/openarm_v2.0/config/arm/struct/topology.yaml': omit})
        for side in ['left', 'right']:
            self.assertIsNotNone(robot.find(
                f"link[@name='openarm_{side}_link7']"))

    def test_collapse_requires_an_empty_tip_and_attached_child(self):
        standalone = expand_modified(
            robot_preset='right_arm', collapse_empty_end=True)
        self.assertIsNotNone(standalone.find(
            "link[@name='openarm_right_link7']"))

        def add_mass(value):
            value['link7'] = {
                'origin': {'x': 0, 'y': 0, 'z': 0, 'roll': 0, 'pitch': 0, 'yaw': 0},
                'mass': 0.1,
                'inertia': {'xx': 1e-5, 'yy': 1e-5, 'zz': 1e-5, 'xy': 0, 'xz': 0, 'yz': 0},
            }
            return value
        robot = expand_modified({'robot/openarm_v2.0/config/arm/inertials/nominal.yaml': add_mass},
                                collapse_empty_end=True)
        for side in ['left', 'right']:
            tip = robot.find(f"link[@name='openarm_{side}_link7']")
            self.assertIsNotNone(tip)
            self.assertEqual(
                float(tip.find('inertial/mass').get('value')), 0.1)


if __name__ == '__main__':
    unittest.main()
