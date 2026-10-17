"""Check component mounting and generated control interfaces."""

import copy
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
from test_v2_camera_components import ENTRY, poses, ROOT
import xacro


def expand_modified(changes=None, collapse_empty_end=None, **mappings):
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
        if (collapse_empty_end is not None
                and str(filename).endswith('config/arm/struct/topology.yaml')):
            value['collapse_empty_end'] = collapse_empty_end
        return value

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
                    {parent_file: set_reference_emit(
                        'camera_mount_point', emit)}
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
                        'robot_presets/default_bimanual.yaml': camera_on_gripper,
                        'pinch_gripper/config/struct/reference_points.yaml': set_reference_emit(
                            'camera_view_frame', emit
                        ),
                    },
                    collapse_empty_end=collapse,
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
                    collapse_empty_end=collapse,
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
                len(robot.findall(
                    "link[@name='openarm_right_grasp_frame']")), 1
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
            expand_modified({'robot_presets/default_bimanual.yaml': conflict})

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
            {'robot_presets/default_bimanual.yaml': rename})
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
            {'config/arm/struct/topology.yaml': reorder})
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

        changes = {'config/arm/struct/topology.yaml': rename_topology}
        for filename in ['joint_origins.yaml', 'joint_axes.yaml', 'joint_limits.yaml']:
            changes['config/arm/joint/' + filename] = rename_parameters
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
            {'config/arm/struct/topology.yaml': arm_change}, robot_preset='right_arm'
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
            {'pinch_gripper/config/struct/topology.yaml': change})
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
            expand_modified({'config/arm/struct/topology.yaml': change})

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
                f"joint[@name='openarm_{side}_ee_mount_joint']"))

    def test_omitted_arm_collapse_flag_keeps_tip(self):
        def omit(value):
            value.pop('collapse_empty_end', None)
            return value
        robot = expand_modified({'config/arm/struct/topology.yaml': omit})
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
        robot = expand_modified({'config/arm/inertials/nominal.yaml': add_mass},
                                collapse_empty_end=True)
        for side in ['left', 'right']:
            tip = robot.find(f"link[@name='openarm_{side}_link7']")
            self.assertIsNotNone(tip)
            self.assertEqual(
                float(tip.find('inertial/mass').get('value')), 0.1)


if __name__ == '__main__':
    unittest.main()
