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
    if collapse_empty_end is not None:
        arguments['collapse_internal_empty_links'] = str(
            collapse_empty_end).lower()
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
                probe['link_name'] = 'test_probe'
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
                        "link[@name='test_probe']"))
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
                        abs(a - b) for row, target in zip(value['test_probe'], expected)
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


if __name__ == '__main__':
    unittest.main()
