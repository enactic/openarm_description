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

from support import expand, pinch, pose, transform, near


@pytest.mark.parametrize('product', ['parallel_link', 'pinch_gripper'])
def test_component_entry_uses_own_configuration(product):
    data = pinch()
    reference = dict(parent='base_link', emit=True,
                     origin=pose(.01, .02, .10, .2, .3, .4))
    data['struct/reference_points.yaml'] = {
        'hand_tcp': reference,
        'hidden': dict(parent='base_link', emit=False, origin=pose())}
    files = {f'end_effector/{product}/config/{name}': value
             for name, value in data.items()}
    arguments = dict(entry='urdf/component.urdf.xacro',
                     component=f'end_effector/{product}')
    robot = expand(files=files, **arguments)
    assert robot.find("link[@name='world']") is not None
    assert robot.find("link[@name='hidden']") is None
    assert robot.find('ros2_control') is None
    joint = robot.find("joint[@name='hand_tcp_joint']")
    assert joint.find('parent').get('link') == 'ee_base_link'
    near(transform(joint.find('origin')), transform(reference['origin']))

    for invalid in ('unknown link', 'cycle'):
        joints = data['struct/topology.yaml']['joints']
        if invalid == 'unknown link':
            joints['joint1']['parent'] = 'missing'
        else:
            joints['joint1']['parent'] = joints['joint2']['child']
            joints['joint2']['parent'] = joints['joint1']['child']
        with pytest.raises(xacro.XacroException, match=invalid):
            expand(files=files, **arguments)
