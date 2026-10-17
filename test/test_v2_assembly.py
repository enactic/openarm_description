"""Check component mounting and generated control interfaces."""

import copy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
from test_v2_camera_components import ENTRY, poses, ROOT
import xacro


def expand_modified(changes=None, **mappings):
    original_share = packages.get_package_share_directory
    symbols = xacro._global_symbols['xacro']
    original_load = symbols['load_yaml']

    def package_share(name):
        return str(ROOT) if name == 'openarm_description' else original_share(name)

    def load_yaml(filename):
        value = original_load(filename)
        for suffix, change in (changes or {}).items():
            if str(filename).endswith(suffix):
                value = change(copy.deepcopy(value))
        return value

    arguments = {
        'robot_preset': 'bimanual_with_zed',
        'emit_grasp_frame': 'true',
    }
    arguments.update(
        {
            key: str(value).lower() if isinstance(value, bool) else value
            for key, value in mappings.items()
        }
    )
    with patch.object(packages, 'get_package_share_directory', package_share):
        with patch.dict(symbols, {'load_yaml': load_yaml}):
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

    def test_camera_mount_emit_preserves_pose(self):
        for parent_file in [
            'config/body/struct/reference_points.yaml',
            'accessory/zed_mount/config/struct/reference_points.yaml',
        ]:
            off, on = [
                expand_modified(
                    {parent_file: set_reference_emit('camera_mount_point', emit)}
                )
                for emit in [False, True]
            ]
            self.assert_common_poses(off, on)

    def test_rotated_offset_on_mirrored_gripper_reference(self):
        def camera_on_gripper(value):
            camera = value['body_camera']
            camera['connection'] = {
                'attach_to': 'primary_arm_end_effector',
                'parent_reference_point': 'camera_view_frame',
                'origin': {
                    'x': 0.012,
                    'y': -0.008,
                    'z': 0.005,
                    'roll': 0.3,
                    'pitch': -0.2,
                    'yaw': 0.4,
                },
            }
            value['body_camera'] = camera
            return value

        for collapse in [False, True]:
            robots = [
                expand_modified(
                    {
                        'robot_presets/bimanual_with_zed.yaml': camera_on_gripper,
                        'pinch_gripper/config/struct/reference_points.yaml': set_reference_emit(
                            'camera_view_frame', emit
                        ),
                    },
                    collapse_internal_empty_links=collapse,
                )
                for emit in [False, True]
            ]
            self.assert_common_poses(*robots)

    def test_arm_tip_reference_emit_with_and_without_collapse(self):
        for collapse in [False, True]:
            robots = [
                expand_modified(
                    {
                        'config/arm/struct/reference_points.yaml': set_reference_emit(
                            'ee_mount_point', emit
                        ),
                    },
                    collapse_internal_empty_links=collapse,
                )
                for emit in [False, True]
            ]
            self.assert_common_poses(*robots)

    def test_grasp_frame_generated_once(self):
        for enabled in [False, True]:
            robot = expand_modified(
                {
                    'pinch_gripper/config/struct/reference_points.yaml': set_reference_emit(
                        'grasp_frame', True
                    ),
                },
                emit_grasp_frame=enabled,
            )
            self.assertEqual(
                len(robot.findall("link[@name='openarm_right_grasp_frame']")), 1
            )

    def test_conflicting_parent_selectors_rejected(self):
        def conflict(value):
            camera = value['body_camera']
            connection = camera['connection']
            connection['parent'] = 'openarm_body_link0'
            camera['connection'] = connection
            value['body_camera'] = camera
            return value

        with self.assertRaisesRegex(
            xacro.XacroException, 'choose parent or parent_reference_point'
        ):
            expand_modified({'robot_presets/bimanual_with_zed.yaml': conflict})


if __name__ == '__main__':
    unittest.main()
