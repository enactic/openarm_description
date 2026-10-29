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
import math
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
COLLADA_NS = {'c': 'http://www.collada.org/2005/11/COLLADASchema'}


def inverse(t):
    rotation = [[t[j][i] for j in range(3)] for i in range(3)]
    translation = [-sum(rotation[i][j] * t[j][3] for j in range(3))
                   for i in range(3)]
    return [row + [value] for row, value in zip(rotation, translation)] + [
        [0, 0, 0, 1]
    ]


def apply(t, points):
    return [[sum(t[i][j] * p[j] for j in range(3)) + t[i][3] for i in range(3)]
            for p in points]


def collada_node_vertices(path, node_id):
    document = ET.parse(path)
    unit = document.find('c:asset/c:unit', COLLADA_NS)
    meter = float(unit.get('meter')) if unit is not None else 1.0
    parents = {child: parent for parent in document.iter() for child in parent}
    node = document.find(f".//c:node[@id='{node_id}']", COLLADA_NS)
    # Apply <matrix> of the node and its ancestors. Exporters such as
    # Blender put a <matrix> on each node.
    node_transform = identity()
    current = node
    while current is not None and current.tag == node.tag:
        for matrix in reversed(current.findall('c:matrix', COLLADA_NS)):
            values = [float(value) for value in matrix.text.split()]
            node_transform = multiply(
                [values[i:i + 4] for i in range(0, 16, 4)], node_transform)
        current = parents.get(current)
    points = []
    for instance in node.findall('c:instance_geometry', COLLADA_NS):
        mesh = document.find(
            f".//c:geometry[@id='{instance.get('url')[1:]}']/c:mesh",
            COLLADA_NS)
        source = mesh.find("c:vertices/c:input[@semantic='POSITION']",
                           COLLADA_NS).get('source')[1:]
        values = [float(value) for value in mesh.find(
            f"c:source[@id='{source}']/c:float_array", COLLADA_NS).text.split()]
        points += [values[i:i + 3] for i in range(0, len(values), 3)]
    return [[value * meter for value in point]
            for point in apply(node_transform, points)]


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

    def test_camera_view_frame_matches_camera_geometry(self):
        # camera_view_frame's origin is the center of the camera PCB's
        # front face, its x axis is the optical axis and its y/z axes are
        # along the PCB edges. Check them against the camera parts in the
        # visual mesh instead of the configured values.
        robot = expand_standalone()
        joint = robot.find("joint[@name='camera_view_frame_joint']")
        visual = robot.find(
            f"link[@name='{joint.find('parent').get('link')}']/visual")
        mesh = visual.find('geometry/mesh')
        self.assertTrue(mesh.get('filename').endswith('/ee_base_link.dae'))
        path = ROOT / mesh.get('filename').split('/openarm_description/', 1)[1]
        scale = [float(value) for value in mesh.get('scale', '1 1 1').split()]
        to_frame = multiply(inverse(transform(joint.find('origin'))),
                            transform(visual.find('origin')))

        def camera_part(node_id):
            return apply(to_frame, [
                [value * factor for value, factor in zip(point, scale)]
                for point in collada_node_vertices(path, node_id)
            ])

        pcb = camera_part('wrist_camera_pcb')
        body = camera_part('wrist_camera_body')
        tolerance = 0.0005
        self.assertAlmostEqual(max(p[0] for p in pcb), 0.0, delta=tolerance)
        lens = [p for p in body if p[0] > tolerance]
        self.assertTrue(lens)
        for axis in [1, 2]:
            self.assertAlmostEqual(
                sum(p[axis] for p in lens) / len(lens), 0.0, delta=tolerance)

        # The PCB is the narrowest along y/z when they are along its edges.
        def widths(angle):
            c, s = math.cos(angle), math.sin(angle)
            rotated = [(c * p[1] - s * p[2], s * p[1] + c * p[2]) for p in pcb]
            return [max(q[i] for q in rotated) - min(q[i] for q in rotated)
                    for i in range(2)]

        narrowest = [min(width) for width in zip(
            *(widths(math.radians(degree)) for degree in range(90)))]
        for width, minimum in zip(widths(0.0), narrowest):
            self.assertAlmostEqual(width, minimum, delta=tolerance)


if __name__ == '__main__':
    unittest.main()
