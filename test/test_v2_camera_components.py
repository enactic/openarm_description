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

"""Check camera assembly, mirrored frames, and retained V2 model properties."""

from functools import lru_cache
import unittest
import xml.etree.ElementTree as ET

from helpers import (
    ENTRY,
    identity,
    multiply,
    package_share_at,
    poses,
    ROOT,
    transform,
)
import xacro

STANDALONE_ENTRY = (
    ROOT
    / 'assets/end_effector/pinch_gripper/urdf/pinch_gripper_standalone.urdf.xacro'
)


@lru_cache(maxsize=None)
def expand(preset, collapse=None):
    with package_share_at(ROOT):
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


@lru_cache(maxsize=None)
def expand_standalone(reflect_y='1'):
    with package_share_at(ROOT):
        document = xacro.process_file(
            str(STANDALONE_ENTRY), mappings={'reflect_y': reflect_y})
    return ET.fromstring(document.toxml())


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
        frames = [
            transform(
                expand_standalone(reflect_y).find(
                    "joint[@name='camera_view_frame_joint']/origin")
            )
            for reflect_y in ['1', '-1']
        ]
        reflect = identity()
        reflect[1][1] = -1
        self.assertTransform(frames[1], multiply(
            multiply(reflect, frames[0]), reflect))


if __name__ == '__main__':
    unittest.main()
