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

"""Helpers shared by the tests."""

from contextlib import contextmanager
import math
from pathlib import Path
from unittest.mock import patch

import ament_index_python.packages as packages

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


@contextmanager
def package_share_at(path):
    """Use path as the share directory of openarm_description."""
    original = packages.get_package_share_directory

    def package_share(name):
        return str(path) if name == 'openarm_description' else original(name)

    with patch.object(packages, 'get_package_share_directory', package_share):
        yield
