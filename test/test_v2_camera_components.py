"""Check camera assembly, mirrored frames, and retained V2 model properties."""

from functools import lru_cache
import math
from pathlib import Path
import unittest
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
import xacro

ROOT = Path(__file__).resolve().parents[1]
ENTRY = ROOT / 'assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro'


def multiply(a, b):
    return [[sum(x * y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def transform(origin):
    xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
    r, p, y = [float(x) for x in origin.get('rpy', '0 0 0').split()]
    cr, sr, cp, sp, cy, sy = (
        math.cos(r),
        math.sin(r),
        math.cos(p),
        math.sin(p),
        math.cos(y),
        math.sin(y),
    )
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz[0]],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz[1]],
        [-sp, cp * sr, cp * cr, xyz[2]],
        [0, 0, 0, 1],
    ]


def poses(robot):
    pending = robot.findall('joint')
    children = {joint.find('child').get('link') for joint in pending}
    roots = {link.get('name') for link in robot.findall('link')} - children
    if len(roots) != 1:
        raise AssertionError('Expected one URDF root')
    values = {roots.pop(): identity()}
    while pending:
        progressed = False
        for joint in list(pending):
            parent = joint.find('parent').get('link')
            if parent in values:
                values[joint.find('child').get('link')] = multiply(
                    values[parent], transform(joint.find('origin'))
                )
                pending.remove(joint)
                progressed = True
        if not progressed:
            raise AssertionError('Disconnected or cyclic URDF tree')
    return values


@lru_cache(maxsize=None)
def expand(preset, collapse=None):
    original = packages.get_package_share_directory

    def package_share(name):
        return str(ROOT) if name == 'openarm_description' else original(name)

    with patch.object(packages, 'get_package_share_directory', package_share):
        document = xacro.process_file(
            str(ENTRY),
            mappings={
                'robot_preset': preset,
                'collapse_internal_empty_links':
                    str(collapse if collapse is not None else True).lower(),
                'emit_grasp_frame': 'true',
            },
        )
    robot = ET.fromstring(document.toxml())
    names = [link.get('name') for link in robot.findall('link')]
    if len(names) != len(set(names)):
        raise AssertionError('Duplicate link names')
    poses(robot)
    return robot


class CameraAssemblyTest(unittest.TestCase):
    def assertTransform(self, a, b, tolerance=1e-10):
        self.assertLess(
            max(abs(x - y) for ra, rb in zip(a, b)
                for x, y in zip(ra, rb)), tolerance
        )

    def test_gripper_view_reflects_position_and_orientation(self):
        robot = expand('default_bimanual')
        reflect = identity()
        reflect[1][1] = -1
        right = transform(
            robot.find(
                "joint[@name='openarm_right_camera_view_frame_joint']/origin")
        )
        left = transform(
            robot.find(
                "joint[@name='openarm_left_camera_view_frame_joint']/origin")
        )
        self.assertTransform(left, multiply(multiply(reflect, right), reflect))
        self.assertAlmostEqual(right[0][3], 0.0013353 + 0.0205, places=10)
        for side in ['left', 'right']:
            frame = robot.find(
                f"link[@name='openarm_{side}_camera_view_frame']")
            self.assertIsNone(frame.find('inertial'))

    def test_collapse_keeps_reference_world_poses(self):
        for preset in [
            'default_bimanual',
            'left_arm_with_pinch_gripper',
            'right_arm_with_pinch_gripper',
        ]:
            collapsed_robot, expanded_robot = (
                expand(preset, True),
                expand(preset, False),
            )
            collapsed, expanded = poses(collapsed_robot), poses(expanded_robot)
            for name in collapsed.keys() & expanded.keys():
                if name.endswith('_ee_base_link'):
                    for tag in ['visual', 'collision', 'inertial']:
                        first = collapsed_robot.find(
                            f"link[@name='{name}']/{tag}/origin"
                        )
                        second = expanded_robot.find(
                            f"link[@name='{name}']/{tag}/origin"
                        )
                        self.assertTransform(
                            multiply(collapsed[name], transform(first)),
                            multiply(expanded[name], transform(second)),
                        )
                else:
                    self.assertTransform(collapsed[name], expanded[name])

    def test_standalone_gripper_view_frame(self):
        entry = (
            ROOT
            / 'assets/end_effector/pinch_gripper/urdf/pinch_gripper_standalone.urdf.xacro'
        )
        original = packages.get_package_share_directory

        def package_share(name):
            return str(ROOT) if name == 'openarm_description' else original(name)

        frames = []
        with patch.object(packages, 'get_package_share_directory', package_share):
            for reflect_y in ['1', '-1']:
                robot = ET.fromstring(
                    xacro.process_file(
                        str(entry), mappings={'reflect_y': reflect_y}
                    ).toxml()
                )
                frames.append(
                    transform(
                        robot.find(
                            "joint[@name='camera_view_frame_joint']/origin")
                    )
                )
        self.assertAlmostEqual(frames[0][0][3], 0.0013353)
        reflect = identity()
        reflect[1][1] = -1
        self.assertTransform(frames[1], multiply(
            multiply(reflect, frames[0]), reflect))


if __name__ == '__main__':
    unittest.main()
