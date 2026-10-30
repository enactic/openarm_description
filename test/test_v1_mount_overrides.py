"""Check legacy mount arguments against the selected parent assembly."""

import copy
import unittest

from test_v1_assembly import expand
from test_v2_camera_components import poses, transform

PRESET = 'robot/openarm_v1.0/config/robot_presets/default_bimanual.yaml'
BODY_REFERENCES = 'body/v2.0/config/struct/reference_points.yaml'


def select_v2_body(value):
    body = value.pop('body')
    body['product'] = 'v2.0'
    value['pedestal_component'] = body
    robot = value['robot']
    robot['components'] = ['pedestal_component' if c == 'body' else c
                           for c in robot['components']]
    value['robot'] = robot
    for name in robot['components']:
        component = value[name]
        connection = component.get('connection', {})
        if connection.get('attach_to') == 'body':
            connection['attach_to'] = 'pedestal_component'
            component['connection'] = connection
            value[name] = component
    return value


def arm_connection(value, origin):
    value = select_v2_body(value)
    arm = value['secondary_arm']
    connection = arm['connection']
    connection['origin'] = origin
    arm['connection'] = connection
    value['secondary_arm'] = arm
    return value


class V1MountOverrideTest(unittest.TestCase):
    def assert_pose(self, actual, expected):
        self.assertLess(max(abs(a-b) for x, y in zip(actual, expected)
                            for a, b in zip(x, y)), 1e-10)

    def test_xyz_override_uses_selected_parent_body(self):
        changes = {PRESET: select_v2_body}
        before = poses(expand(changes))
        for side, y in [('left', 0.03), ('right', -0.03)]:
            robot = expand(changes, **{side + '_arm_base_xyz': f'0 {y} 0.697'})
            name = f'openarm_{side}_link0'
            self.assert_pose(poses(robot)[name], before[name])

    def test_partial_overrides_preserve_composed_reference_pose(self):
        def with_offset(value):
            return arm_connection(value, {'x': 0.02, 'yaw': 0.17})

        for emit in [False, True]:
            def reference(value):
                ref = value['right_arm_mount_point']
                ref['emit'] = emit
                ref['origin'] = {'x': 0.1, 'y': -0.2, 'z': 0.7,
                                 'roll': 0.3, 'pitch': -0.2, 'yaw': 0.4}
                value['right_arm_mount_point'] = ref
                return value

            changes = {PRESET: with_offset, BODY_REFERENCES: reference}
            before = poses(expand(changes))['openarm_right_link0']
            xyz_robot = expand(changes, right_arm_base_xyz='0.11 -0.24 0.72')
            expected = copy.deepcopy(before)
            for i, v in enumerate([0.11, -0.24, 0.72]):
                expected[i][3] = v
            self.assert_pose(poses(xyz_robot)['openarm_right_link0'], expected)
            rpy_robot = expand(changes, right_arm_base_rpy='0.2 0.1 -0.3')
            expected = transform({'xyz': ' '.join(str(before[i][3]) for i in range(3)),
                                  'rpy': '0.2 0.1 -0.3'})
            self.assert_pose(poses(rpy_robot)['openarm_right_link0'], expected)

    def test_override_preserves_parent_reference_reflection(self):
        def reflect_parent(value):
            body = value['body']
            body['reflect'] = {'x': 1, 'y': -1, 'z': 1,
                               'include': {'links': [],
                                           'reference_points': ['right_arm_mount_point']}}
            value['body'] = body
            return value

        changes = {PRESET: reflect_parent}
        before = poses(expand(changes))['openarm_right_link0']
        robot = expand(changes, right_arm_base_xyz='0 0.03 0.697')
        self.assert_pose(poses(robot)['openarm_right_link0'], before)

    def test_override_without_reference_allows_identity_origin(self):
        def no_reference(value):
            arm = value['secondary_arm']
            connection = arm['connection']
            connection['parent_reference_point'] = None
            connection['origin'] = None
            arm['connection'] = connection
            value['secondary_arm'] = arm
            return value

        robot = expand({PRESET: no_reference},
                       right_arm_base_xyz='0.1 -0.2 0.7')
        self.assert_pose(poses(robot)['openarm_right_link0'],
                         transform({'xyz': '0.1 -0.2 0.7', 'rpy': '0 0 0'}))


if __name__ == '__main__':
    unittest.main()
