"""Check body root aliases without relying on a particular canonical name."""

import unittest
import xml.etree.ElementTree as ET

from test_v1_assembly import expand as expand_v1
from test_v2_assembly import expand_modified as expand_v2
from test_v2_camera_components import poses


def renamed_body_changes(version, actual_name=None):
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
    if actual_name is not None:
        def preset(value):
            body = value['body']
            body['link_name'] = actual_name
            value['body'] = body
            return value
        changes[f'openarm_{version}.0/config/robot_presets/default_bimanual.yaml'] = preset
    return changes


class BodyRootMappingTest(unittest.TestCase):
    def test_canonical_root_rename_keeps_v1_and_v2_robot_definitions(self):
        for version, expand in [('v1', expand_v1), ('v2', expand_v2)]:
            with self.subTest(version=version):
                before = expand()
                after = expand(renamed_body_changes(version))
                self.assertEqual(
                    ET.canonicalize(ET.tostring(
                        before, encoding='unicode'), strip_text=True),
                    ET.canonicalize(ET.tostring(after, encoding='unicode'), strip_text=True))

    def test_custom_actual_name_maps_all_body_references(self):
        for version, expand in [('v1', expand_v1), ('v2', expand_v2)]:
            with self.subTest(version=version):
                before = poses(expand())
                robot = expand(renamed_body_changes(version, 'pedestal'))
                after = poses(robot)
                self.assertIn('pedestal', after)
                self.assertNotIn('support_post', after)
                self.assertNotIn('openarm_body_link0', after)
                self.assertEqual(after['pedestal'],
                                 before['openarm_body_link0'])
                for name in before.keys() & after.keys():
                    self.assertEqual(after[name], before[name])
                if version == 'v2':
                    self.assertIn('pedestal_camera_mount_point', after)


if __name__ == '__main__':
    unittest.main()
