"""Preserve V1 motor coordinates while using shared mirroring helpers."""

from functools import lru_cache
import math
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
from test_v2_camera_components import ROOT
import xacro


@lru_cache(maxsize=None)
def expand(bimanual=True):
    original = packages.get_package_share_directory

    def package_share(name):
        return str(ROOT) if name == 'openarm_description' else original(name)

    with patch.object(packages, 'get_package_share_directory', package_share):
        robot = xacro.process_file(
            str(ROOT / 'assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro'),
            mappings={
                'bimanual': str(bimanual).lower(),
            },
        )
    return ET.fromstring(robot.toxml())


def numbers(element, attribute):
    return [float(value) for value in element.get(attribute).split()]


class V1MirroringTest(unittest.TestCase):
    def test_single_arm_retains_upright_zero_frames(self):
        robot = expand(False)
        prefix = 'openarm_'
        children = {j.find('child').get('link')
                    for j in robot.findall('joint')}
        roots = {link.get('name')
                 for link in robot.findall('link')} - children
        self.assertEqual(roots, {'world'})
        for i in range(1, 8):
            joint = robot.find(f"joint[@name='{prefix}joint{i}']")
            self.assertEqual(
                numbers(joint.find('origin'), 'rpy'), [0, 0, 0])
        joint2 = robot.find(f"joint[@name='{prefix}joint2']")
        self.assertAlmostEqual(
            float(joint2.find('limit').get('lower')), -1.745329)
        self.assertAlmostEqual(
            float(joint2.find('limit').get('upper')), 1.745329)

    def test_bimanual_j2_keeps_mounting_rotation_and_limits(self):
        robot = expand()
        for side, sign in [('left', -1), ('right', 1)]:
            joint = robot.find(f"joint[@name='openarm_{side}_joint2']")
            self.assertAlmostEqual(numbers(joint.find('origin'), 'rpy')[
                                   0], sign * math.pi / 2)
            self.assertEqual(numbers(joint.find('axis'), 'xyz'), [-1, 0, 0])
            limit = joint.find('limit')
            self.assertAlmostEqual(
                float(limit.get('lower')), -1.745329 + sign * math.pi / 2)
            self.assertAlmostEqual(
                float(limit.get('upper')), 1.745329 + sign * math.pi / 2)

    def test_elbow_joint_origins_are_on_the_centerline(self):
        robot = expand()
        for side in ['left', 'right']:
            for index, expected in [(4, 0.0), (5, 0.0)]:
                joint = robot.find(
                    f"joint[@name='openarm_{side}_joint{index}']")
                self.assertEqual(
                    numbers(joint.find('origin'), 'xyz')[1], expected)

    def test_joint_axes_keep_motor_positive_directions(self):
        expected = [[0, 0, 1], [-1, 0, 0], [0, 0, 1], [0, 1, 0],
                    [0, 0, 1], [1, 0, 0], [0, 1, 0]]
        for bimanual in [False, True]:
            robot = expand(bimanual)
            for side in (['left_', 'right_'] if bimanual else ['']):
                for index, axis in enumerate(expected, 1):
                    joint = robot.find(
                        f"joint[@name='openarm_{side}joint{index}']")
                    wanted = [
                        0, -1, 0] if side == 'left_' and index == 7 else axis
                    self.assertEqual(
                        numbers(joint.find('axis'), 'xyz'), wanted)


if __name__ == '__main__':
    unittest.main()
