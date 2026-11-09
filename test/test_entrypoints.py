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

import pytest
import xacro
import yaml

from support import expand, multiply, near, pinch, pose, transform


def write_configuration(directory, data):
    for name, value in data.items():
        path = directory/name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(yaml.safe_dump(value, sort_keys=False))


def test_pinch_entry_forwards_reference_options(tmp_path):
    data = pinch()
    reference = dict(parent='base_link', emit=True,
                     origin=pose(.01, .02, .03, .2, -.1, .3))
    data['struct/reference_points.yaml']['probe'] = reference
    write_configuration(tmp_path, data)
    robot = expand(entry='assets/end_effector/pinch_gripper/urdf/pinch_gripper_standalone.urdf.xacro',
                   ee_config_dir=tmp_path, reflect_y=-1, emit_grasp_frame=True)
    assert robot.find("link[@name='grasp_frame']") is not None
    reflection = [[float(i == j) * (1, -1, 1, 1)[i]
                   for j in range(4)] for i in range(4)]
    near(transform(robot.find("joint[@name='probe_joint']/origin")),
         multiply(reflection, multiply(transform(reference['origin']), reflection)))

    for invalid in ('unknown link', 'cycle'):
        data = pinch()
        joints = data['struct/topology.yaml']['joints']
        if invalid == 'unknown link':
            joints['joint1']['parent'] = 'missing'
        else:
            joints['joint1']['parent'] = joints['joint2']['child']
            joints['joint2']['parent'] = joints['joint1']['child']
        write_configuration(tmp_path, data)
        with pytest.raises(xacro.XacroException, match=invalid):
            expand(entry='assets/end_effector/pinch_gripper/urdf/pinch_gripper_standalone.urdf.xacro',
                   ee_config_dir=tmp_path)
