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

"""Check oblique joint axes against reflected rigid-body motion."""

import itertools
import math
import unittest
import xml.etree.ElementTree as ET

from helpers import (
    multiply,
    package_share_at,
    ROOT,
    transform,
)
import xacro

REFLECTIONS = [(1, 1, 1), (-1, 1, 1), (1, -1, 1), (1, 1, -1)]


def rotation(axis, angle):
    x, y, z = axis
    c, s, v = math.cos(angle), math.sin(angle), 1 - math.cos(angle)
    return [
        [c + x*x*v, x*y*v - z*s, x*z*v + y*s],
        [y*x*v + z*s, c + y*y*v, y*z*v - x*s],
        [z*x*v - y*s, z*y*v + x*s, c + z*z*v],
    ]


def literal(values):
    """Format values as a dict() expression for xacro."""
    return 'dict(' + ', '.join(f'{k}={v!r}' for k, v in values.items()) + ')'


def expand(axis, reflect=(1, 1, 1), mirror_limits=False, joint_type='revolute'):
    origin = {'x': 0.12, 'y': -0.04, 'z': 0.07,
              'roll': 0.31, 'pitch': -0.47, 'yaw': 0.29}
    limits = {'lower': -0.4, 'upper': 1.2, 'effort': 7, 'velocity': 3}
    text = """<robot name="axis_test" xmlns:xacro="http://www.ros.org/wiki/xacro">
      <xacro:include filename="$(find openarm_description)/assets/common/links.xacro" />
      <link name="base" /><link name="tip" />
      <xacro:openarm_emit_joint joint_name="joint" joint_type="%(joint_type)s"
        parent_link="base" child_link="tip" joint_origins="${dict()}"
        joint_axes="${dict()}" joint_limits="${dict()}"
        origin_override="${%(origin)s}" axis_override="${%(axis)s}"
        limit_override="${%(limits)s}" reflect_rotation="${True}"
        reflect_x="${%(reflect_x)r}" reflect_y="${%(reflect_y)r}"
        reflect_z="${%(reflect_z)r}" mirror_limit_sign="${%(mirror_limits)r}" />
    </robot>""" % {
        'joint_type': joint_type,
        'origin': literal(origin),
        'axis': literal(dict(zip(('x', 'y', 'z'), axis))),
        'limits': literal(limits),
        'reflect_x': reflect[0],
        'reflect_y': reflect[1],
        'reflect_z': reflect[2],
        'mirror_limits': mirror_limits,
    }
    with package_share_at(ROOT):
        document = xacro.parse(text)
        xacro.process_doc(document)
    return ET.fromstring(document.toxml()).find('joint')


def joint_pose(joint, position):
    axis = [float(v) for v in joint.find('axis').get('xyz').split()]
    if joint.get('type') == 'prismatic':
        motion = [[float(i == j) for j in range(3)] + [position * axis[i]]
                  for i in range(3)]
    else:
        motion = [row + [0.] for row in rotation(axis, position)]
    return multiply(transform(joint.find('origin')),
                    motion + [[0., 0., 0., 1.]])


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
