"""Check body root names and instance prefixes without relying on a particular canonical name."""

import unittest

from test_v1_assembly import expand as expand_v1
from test_v2_assembly import expand_modified as expand_v2
from test_v2_camera_components import poses


def renamed_body_changes(version, instance_prefix=None):
    def topology(value):
        value['links'] = ['support_post']
        return value

    def references(value):
        for name in list(value):
            point = value[name]
            point['parent'] = 'support_post'
            value[name] = point
        return value

    def data(value):
        value['support_post'] = value.pop('body_link0')
        return value

    prefix = f'body/{version}.0/config/'
    changes = {
        prefix + 'struct/topology.yaml': topology,
        prefix + 'struct/reference_points.yaml': references,
        prefix + 'inertials/nominal.yaml': data,
        prefix + 'link/visuals.yaml': data,
        prefix + 'link/collisions.yaml': data,
    }
    if instance_prefix is not None:
        def preset(value):
            body = value['body']
            body['prefix'] = instance_prefix
            value['body'] = body
            return value
        changes[f'robot/openarm_{version}.0/config/robot_presets/default_bimanual.yaml'] = preset
    return changes


class BodyRootMappingTest(unittest.TestCase):

    def test_local_root_and_prefix_update_body_references(self):
        for version, expand in [('v1', expand_v1), ('v2', expand_v2)]:
            with self.subTest(version=version):
                before = poses(expand())
                robot = expand(renamed_body_changes(version, 'fixture_'))
                after = poses(robot)
                self.assertIn('fixture_support_post', after)
                self.assertNotIn('support_post', after)
                self.assertNotIn('openarm_body_link0', after)
                self.assertEqual(after['fixture_support_post'],
                                 before['openarm_body_link0'])
                for name in before.keys() & after.keys():
                    self.assertEqual(after[name], before[name])
                if version == 'v2':
                    self.assertIn('fixture_camera_mount_point', after)


if __name__ == '__main__':
    unittest.main()
