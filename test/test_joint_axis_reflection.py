"""Check oblique joint axes against reflected rigid-body motion."""

import itertools
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
import xacro

ROOT = Path(__file__).resolve().parents[1]
REFLECTIONS = [(1, 1, 1), (-1, 1, 1), (1, -1, 1), (1, 1, -1)]


def multiply(first, second):
    return [[sum(a * b for a, b in zip(row, col)) for col in zip(*second)]
            for row in first]


def rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ]


def expand(axis, reflect=(1, 1, 1), mirror_limits=False, joint_type='revolute'):
    origin = {'x': 0.12, 'y': -0.04, 'z': 0.07, 'roll': 0.31, 'pitch': -0.47, 'yaw': 0.29}
    limits = {'lower': -0.4, 'upper': 1.2, 'effort': 7, 'velocity': 3}
    symbols = {
        'axis_input': dict(zip(('x', 'y', 'z'), axis)),
        'origin_input': origin,
        'limit_input': limits,
        'reflect_input': reflect,
        'mirror_limits_input': mirror_limits,
        'joint_type_input': joint_type,
    }
    text = """<robot name="axis_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="$(find openarm_description)/assets/common/links.xacro" />
      <link name="base" /><link name="tip" />
      <xacro:openarm_emit_joint joint_name="joint" joint_type="${joint_type_input}"
        parent_link="base" child_link="tip" joint_origins="${dict()}"
        joint_axes="${dict()}" joint_limits="${dict()}"
        origin_override="${origin_input}" axis_override="${axis_input}"
        limit_override="${limit_input}" reflect_rotation="${True}"
        reflect_x="${reflect_input[0]}" reflect_y="${reflect_input[1]}"
        reflect_z="${reflect_input[2]}" mirror_limit_sign="${mirror_limits_input}" />
    </robot>"""
    original_share = packages.get_package_share_directory

    def package_share(name):
        return str(ROOT) if name == 'openarm_description' else original_share(name)

    with patch.object(packages, 'get_package_share_directory', package_share):
        with patch.dict(xacro._global_symbols, symbols):
            document = xacro.parse(text)
            xacro.process_doc(document)
    return ET.fromstring(document.toxml()).find('joint')


def joint_pose(joint, position):
    origin = joint.find('origin')
    xyz = [float(v) for v in origin.get('xyz').split()]
    roll, pitch, yaw = [float(v) for v in origin.get('rpy').split()]
    basis = multiply(rotation((0, 0, 1), yaw), multiply(
        rotation((0, 1, 0), pitch), rotation((1, 0, 0), roll)))
    axis = [float(v) for v in joint.find('axis').get('xyz').split()]
    if joint.get('type') == 'prismatic':
        delta = [row[0]*axis[0] + row[1]*axis[1] + row[2]*axis[2]
                 for row in basis]
        xyz = [p + position*d for p, d in zip(xyz, delta)]
    else:
        basis = multiply(basis, rotation(axis, position))
    return [row + [p] for row, p in zip(basis, xyz)] + [[0., 0., 0., 1.]]


class JointAxisReflectionTest(unittest.TestCase):
    def assert_matrix_close(self, first, second):
        self.assertLess(max(abs(a-b) for r, s in zip(first, second)
                            for a, b in zip(r, s)), 1e-12)

    def test_oblique_axis_keeps_in_plane_components(self):
        joint = expand((0.6, 0.8, 0), (1, -1, 1), True)
        self.assertEqual([float(v) for v in joint.find('axis').get('xyz').split()],
                         [0.6, -0.8, 0])
        self.assertEqual(float(joint.find('limit').get('lower')), -1.2)
        self.assertEqual(float(joint.find('limit').get('upper')), 0.4)

    def test_oblique_revolute_motion_matches_reflected_pose(self):
        right = expand((0.36, -0.48, 0.8))
        for reflect, position in itertools.product(REFLECTIONS[1:], [-0.3, 0, 0.7]):
            with self.subTest(reflect=reflect, position=position):
                left = expand((0.36, -0.48, 0.8), reflect, True)
                signs = (*reflect, 1)
                reflection = [[float(signs[i]) if i == j else 0.
                               for j in range(4)] for i in range(4)]
                expected = multiply(reflection, multiply(
                    joint_pose(right, position), reflection))
                self.assert_matrix_close(joint_pose(left, -position), expected)

    def test_oblique_prismatic_direction_matches_reflected_translation(self):
        right = expand((0.36, -0.48, 0.8), joint_type='prismatic')
        for reflect in REFLECTIONS[1:]:
            left = expand((0.36, -0.48, 0.8), reflect, joint_type='prismatic')
            signs = (*reflect, 1)
            reflection = [[float(signs[i]) if i == j else 0.
                           for j in range(4)] for i in range(4)]
            expected = multiply(reflection, multiply(
                joint_pose(right, 0.2), reflection))
            self.assert_matrix_close(joint_pose(left, 0.2), expected)

    def test_principal_axes_and_existing_limit_policy_are_preserved(self):
        axes = [tuple(sign if i == axis else 0 for i in range(3))
                for axis in range(3) for sign in [-1, 1]]
        for axis, reflect, explicit in itertools.product(axes, REFLECTIONS, [False, True]):
            with self.subTest(axis=axis, reflect=reflect, explicit=explicit):
                joint = expand(axis, reflect, explicit)
                flipped = any(r == -1 and a != 0 for a,
                              r in zip(axis, reflect))
                expected_axis = [(-1 if flipped else 1) * a for a in axis]
                self.assertEqual([float(v) for v in joint.find('axis').get('xyz').split()],
                                 expected_axis)
                expected_limits = (-1.2,
                                   0.4) if explicit or flipped else (-0.4, 1.2)
                self.assertEqual(tuple(float(joint.find('limit').get(k))
                                       for k in ['lower', 'upper']), expected_limits)
                self.assertEqual(float(joint.find('limit').get('effort')), 7)
                self.assertEqual(float(joint.find('limit').get('velocity')), 3)

    def test_no_reflection_preserves_oblique_axis(self):
        axis = (0.36, -0.48, 0.8)
        joint = expand(axis)
        self.assertEqual(tuple(float(v)
                         for v in joint.find('axis').get('xyz').split()), axis)
        self.assertAlmostEqual(sum(v*v for v in axis), 1)


if __name__ == '__main__':
    unittest.main()
