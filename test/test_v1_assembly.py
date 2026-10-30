"""Check shared V1 assembly, centered joint frames, and tool ownership."""

import os
import shutil
import tempfile
import yaml
import copy
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET

from test_v2_camera_components import multiply, package_share_at, poses, ROOT, transform
import xacro


def expand(changes=None, collapse=None, **mappings):
    args = {'ros2_control': 'true'}
    args.update({k: str(v).lower() if isinstance(v, bool) else str(v)
                 for k, v in mappings.items()})
    with tempfile.TemporaryDirectory() as share:
        shutil.copytree(ROOT / 'assets', Path(share) / 'assets',
                        copy_function=os.symlink)
        for path, change in (changes or {}).items():
            target = Path(share) / 'assets' / path
            value = change(yaml.safe_load(target.read_text()))
            target.unlink()
            target.write_text(yaml.safe_dump(value, sort_keys=False))
        if collapse is not None:
            target = Path(
                share) / 'assets/robot/openarm_v1.0/config/arm/struct/topology.yaml'
            value = yaml.safe_load(target.read_text())
            value['collapse_empty_end'] = collapse
            target.unlink()
            target.write_text(yaml.safe_dump(value, sort_keys=False))
        with package_share_at(share):
            robot = ET.fromstring(xacro.process_file(
                str(ROOT / 'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'),
                mappings=args,
            ).toxml())
    for tag in ['link', 'joint']:
        names = [e.get('name') for e in robot.findall(tag)]
        if len(names) != len(set(names)):
            raise AssertionError(f'Duplicate {tag} names')
    poses(robot)
    return robot


def xyz(element):
    return [float(v) for v in element.get('xyz').split()]


def tensor(element):
    return [[float(element.get(k)) for k in row] for row in
            [('ixx', 'ixy', 'ixz'), ('ixy', 'iyy', 'iyz'), ('ixz', 'iyz', 'izz')]]


class V1AssemblyTest(unittest.TestCase):
    def assert_matrix(self, a, b):
        self.assertLess(max(abs(x-y) for ra, rb in zip(a, b)
                            for x, y in zip(ra, rb)), 1e-10)

    def test_all_presets_control_only_present_active_joints(self):
        directory = ROOT / 'assets/robot/openarm_v1.0/config/robot_presets'
        for path in directory.glob('*.yaml'):
            with self.subTest(preset=path.stem):
                robot = expand(robot_preset=path.stem)
                controlled = [e.get('name')
                              for e in robot.findall('ros2_control/joint')]
                expected = {j.get('name') for j in robot.findall('joint')
                            if j.get('type') != 'fixed' and j.find('mimic') is None}
                self.assertEqual(set(controlled), expected)
                self.assertEqual(len(controlled), len(set(controlled)))

    def test_external_body_keeps_closed_cover_mass(self):
        robot = expand()
        name = 'openarm_body_link0'
        body = robot.find(f"link[@name='{name}']")
        self.assertEqual(
            float(body.find('inertial/mass').get('value')), 3.74)
        self.assertTrue(body.find('visual/geometry/mesh').get('filename').startswith(
            'package://openarm_description/assets/body/v1.0/'))
        inertia = body.find('inertial/inertia')
        for key in ['ixy', 'ixz', 'iyz']:
            self.assertEqual(float(inertia.get(key)), 0)

    def test_tool_owns_link7_and_keeps_the_hand_tree(self):
        arm = xacro.load_yaml(
            str(ROOT / 'assets/robot/openarm_v1.0/config/arm/inertials/nominal.yaml'))
        tool = xacro.load_yaml(
            str(ROOT / 'assets/end_effector/parallel_link/config/inertials/nominal.yaml'))
        self.assertNotIn('link7', arm)
        self.assertAlmostEqual(tool['link7']['mass'], 0.4054971268153967)
        robot = expand()
        for side in ['left', 'right']:
            prefix = f'openarm_{side}_'
            self.assertIsNone(robot.find(
                f"link[@name='{prefix}arm_end_link']"))
            for name, parent, child in [
                ('joint7', 'link6', 'link7'),
                ('hand_joint', 'link7', 'hand'),
                ('finger_joint1', 'hand', 'right_finger'),
                ('finger_joint2', 'hand', 'left_finger'),
                ('hand_tcp_joint', 'hand', 'hand_tcp'),
            ]:
                j = robot.find(f"joint[@name='{prefix}{name}']")
                self.assertEqual(j.find('parent').get('link'), prefix+parent)
                self.assertEqual(j.find('child').get('link'), prefix+child)

    def test_only_link7_of_the_tool_is_mirrored(self):
        robot = expand()
        for name in ['link7', 'hand', 'right_finger', 'left_finger']:
            left = robot.find(f"link[@name='openarm_left_{name}']")
            right = robot.find(f"link[@name='openarm_right_{name}']")
            sign = -1 if name == 'link7' else 1
            a, b = xyz(left.find('inertial/origin')
                       ), xyz(right.find('inertial/origin'))
            self.assertEqual(a, [b[0], sign*b[1], b[2]])
            li, ri = left.find(
                'inertial/inertia'), right.find('inertial/inertia')
            for key in ['ixx', 'iyy', 'izz', 'ixz', 'ixy', 'iyz']:
                s = sign if key in ['ixy', 'iyz'] else 1
                self.assertEqual(float(li.get(key)), s*float(ri.get(key)))
            for tag in ['visual', 'collision']:
                a = left.find(tag+'/geometry/mesh')
                b = right.find(tag+'/geometry/mesh')
                self.assertEqual(a.get('filename'), b.get('filename'))
                scales_a = [float(v) for v in a.get('scale').split()]
                scales_b = [float(v) for v in b.get('scale').split()]
                self.assertEqual(
                    scales_a, [scales_b[0], sign*scales_b[1], scales_b[2]])

    def test_centered_origins_keep_original_axis_directions(self):
        robot = expand(bimanual=False)
        expected = [[0, 0, 1], [-1, 0, 0], [0, 0, 1], [0, 1, 0],
                    [0, 0, 1], [1, 0, 0], [0, 1, 0]]
        for i, axis in enumerate(expected, 1):
            j = robot.find(f"joint[@name='openarm_joint{i}']")
            self.assertEqual(xyz(j.find('axis')), axis)
            self.assertEqual(xyz(j.find('origin'))[:2], [0, 0])
        c = xyz(robot.find("link[@name='openarm_link4']/inertial/origin"))
        self.assertAlmostEqual(c[1], -0.03030575826634669 + 0.0315)

    def test_origin_and_limit_overrides_are_applied_before_mirroring(self):
        def change(preset):
            for name in ['primary_arm', 'secondary_arm']:
                arm = preset[name]
                arm['origin_override_joints'] = {'joint2': {
                    'x': .01, 'y': .02, 'z': .03, 'roll': .2, 'pitch': .3, 'yaw': .4}}
                arm['limit_override_joints'] = {'joint2': {
                    'lower': -.2, 'upper': 1.3, 'effort': 40, 'velocity': 16.754666}}
                preset[name] = arm
            return preset
        robot = expand(
            {'robot/openarm_v1.0/config/robot_presets/default_bimanual.yaml': change})
        for side, sign in [('right', 1), ('left', -1)]:
            j = robot.find(f"joint[@name='openarm_{side}_joint2']")
            self.assertEqual(xyz(j.find('origin')), [.01, sign*.02, .03])
            self.assertEqual([float(v) for v in j.find('origin').get('rpy').split()],
                             [sign*.2, .3, sign*.4])
            bounds = [float(j.find('limit').get(k))
                      for k in ['lower', 'upper']]
            self.assertEqual(bounds, [-.2, 1.3] if sign == 1 else [-1.3, .2])

    def test_collapse_with_rotated_mount_and_partial_reflection(self):
        def offset(preset):
            for name in ['primary_arm_end_effector', 'secondary_arm_end_effector']:
                component = preset[name]
                connection = component['connection']
                connection['origin'] = {'x': .008, 'y': .012, 'z': .004,
                                        'roll': .2, 'pitch': -.3, 'yaw': .1}
                component['connection'] = connection
                preset[name] = component
            return preset
        first, second = [expand({'robot/openarm_v1.0/config/robot_presets/default_bimanual.yaml': offset}, collapse=c)
                         for c in [False, True]]
        a, b = poses(first), poses(second)
        links = {link.get('name'): link for link in second.findall('link')}
        for link in first.findall('link'):
            name = link.get('name')
            if name not in links:
                continue
            for tag in ['visual', 'collision', 'inertial']:
                old, new = link.find(tag), links[name].find(tag)
                if old is None:
                    self.assertIsNone(new)
                    continue
                wa = multiply(a[name], transform(old.find('origin')))
                wb = multiply(b[name], transform(new.find('origin')))
                self.assert_matrix(wa, wb)
                if tag == 'inertial':
                    ra, rb = [r[:3] for r in wa[:3]], [r[:3] for r in wb[:3]]
                    ia = multiply(
                        multiply(ra, tensor(old.find('inertia'))), list(zip(*ra)))
                    ib = multiply(
                        multiply(rb, tensor(new.find('inertia'))), list(zip(*rb)))
                    self.assert_matrix(ia, ib)
            if name.endswith('hand_tcp'):
                self.assert_matrix(a[name], b[name])

    def test_arm_only_does_not_initialize_gripper_hardware(self):
        robot = expand(robot_preset='right_arm', use_fake_hardware=False)
        hardware = robot.find('ros2_control/hardware')
        self.assertEqual(hardware.find(
            "param[@name='hand']").text.lower(), 'false')
        self.assertIsNone(robot.find("link[@name='openarm_right_link7']"))
        self.assertIsNotNone(robot.find(
            "link[@name='openarm_right_arm_end_link']"))

    def test_legacy_base_mount_override_is_not_added_twice(self):
        robot = expand(right_arm_base_xyz='0.1 -0.2 0.7',
                       right_arm_base_rpy='.1 .2 .3')
        self.assert_matrix(poses(robot)['openarm_right_link0'],
                           transform({'xyz': '0.1 -0.2 0.7', 'rpy': '.1 .2 .3'}))

    def test_root_frame_and_placement_arguments(self):
        robot = expand(bimanual=False, parent='map',
                       xyz='.1 .2 .3', rpy='.2 .1 -.3')
        self.assertIsNotNone(robot.find("link[@name='map']"))
        self.assertIsNone(robot.find("link[@name='world']"))
        self.assert_matrix(poses(robot)['openarm_link0'],
                           transform({'xyz': '.1 .2 .3', 'rpy': '.2 .1 -.3'}))

    def test_emitted_mount_reference_keeps_tool_pose(self):
        def emit_reference(value):
            reference = value['ee_mount_point']
            reference['emit'] = True
            value['ee_mount_point'] = reference
            return value
        first = expand()
        second = expand(
            {'robot/openarm_v1.0/config/arm/struct/reference_points.yaml': emit_reference})
        a, b = poses(first), poses(second)
        for name in a.keys() & b.keys():
            self.assert_matrix(a[name], b[name])
        for side in ['left', 'right']:
            self.assertIsNotNone(second.find(
                f"link[@name='openarm_{side}_ee_mount_point']"))

    def test_standalone_gripper_includes_link7_and_custom_tcp(self):
        entry = ROOT / 'assets/end_effector/parallel_link/urdf'
        with package_share_at(ROOT):
            document = xacro.process_file(
                str(entry / 'parallel_link_standalone.urdf.xacro'),
                mappings={'reflect_y': '-1',
                          'tcp_xyz': '.01 .02 .10', 'tcp_rpy': '.2 .3 .4'},
            )
        robot = ET.fromstring(document.toxml())
        self.assertIsNotNone(robot.find("link[@name='link7']/inertial"))
        self.assertIsNotNone(robot.find("link[@name='hand']/inertial"))
        tcp = robot.find("joint[@name='hand_tcp_joint']")
        self.assertEqual(tcp.find('parent').get('link'), 'hand')
        self.assertEqual(xyz(tcp.find('origin')), [.01, .02, .1])
        self.assertIsNone(robot.find("link[@name='hand_tcp']/inertial"))
        for name in ['hand', 'left_finger', 'right_finger']:
            scale = robot.find(
                f"link[@name='{name}']/visual/geometry/mesh").get('scale')
            self.assertEqual([float(v)
                             for v in scale.split()], [.001, .001, .001])

    def test_unknown_joint_override_is_rejected(self):
        def change(preset):
            arm = preset['secondary_arm']
            arm['origin_override_joints'] = {'missing': {}}
            preset['secondary_arm'] = arm
            return preset
        with self.assertRaisesRegex(xacro.XacroException, 'Unknown joint override'):
            expand(
                {'robot/openarm_v1.0/config/robot_presets/default_bimanual.yaml': change})


if __name__ == '__main__':
    unittest.main()
