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

"""Small YAML fixtures and checks shared by the contract tests."""

from contextlib import contextmanager
from copy import deepcopy
import math
import os
from pathlib import Path
import shutil
import tempfile
from unittest.mock import patch
import xml.etree.ElementTree as ET

import ament_index_python.packages as packages
import pytest
import xacro
import yaml

ROOT = Path(__file__).resolve().parents[1]


def pose(x=0, y=0, z=0, roll=0, pitch=0, yaw=0):
    return dict(x=x, y=y, z=z, roll=roll, pitch=pitch, yaw=yaw)


def identity():
    return [[float(i == j) for j in range(4)] for i in range(4)]


def multiply(a, b):
    return [[sum(x*y for x, y in zip(row, col)) for col in zip(*b)] for row in a]


def transform(origin):
    if not isinstance(origin, dict):
        xyz = [float(x) for x in origin.get('xyz', '0 0 0').split()]
        rpy = [float(x) for x in origin.get('rpy', '0 0 0').split()]
        origin = pose(*xyz, *rpy)
    r, p, y = [origin[k] for k in ('roll', 'pitch', 'yaw')]
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(
        r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return [[cy*cp, cy*sp*sr-sy*cr, cy*sp*cr+sy*sr, origin['x']],
            [sy*cp, sy*sp*sr+cy*cr, sy*sp*cr-cy*sr, origin['y']],
            [-sp, cp*sr, cp*cr, origin['z']], [0., 0., 0., 1.]]


def near(a, b):
    assert len(a) == len(b)
    for left, right in zip(a, b):
        assert left == pytest.approx(right, abs=1e-10)


def poses(robot):
    links = [link.get('name') for link in robot.findall('link')]
    joints = robot.findall('joint')
    names = [joint.get('name') for joint in joints]
    assert len(links) == len(set(links)) and len(names) == len(set(names))
    children = [j.find('child').get('link') for j in joints]
    assert len(children) == len(set(children))
    for joint in joints:
        assert joint.find('parent').get('link') in links
        assert joint.find('child').get('link') in links
        mimic = joint.find('mimic')
        if mimic is not None:
            assert mimic.get('joint') in names and mimic.get(
                'joint') != joint.get('name')
    roots = set(links) - set(children)
    assert len(roots) == 1
    result = {roots.pop(): identity()}
    pending = list(joints)
    while pending:
        progressed = False
        for joint in pending[:]:
            parent = joint.find('parent').get('link')
            if parent in result:
                result[joint.find('child').get('link')] = multiply(
                    result[parent], transform(joint.find('origin')))
                pending.remove(joint)
                progressed = True
        assert progressed, 'Disconnected or cyclic model'
    controlled = [j.get('name') for j in robot.findall('ros2_control/joint')]
    assert len(controlled) == len(set(controlled)
                                  ) and set(controlled) <= set(names)
    return result


@contextmanager
def package_share_at(path):
    original = packages.get_package_share_directory
    with patch.object(packages, 'get_package_share_directory',
                      lambda name: str(path) if name == 'openarm_description' else original(name)):
        yield


def expand(files=None, version='v2.0', entry=None, **arguments):
    entry = entry or f'assets/robot/openarm_{version}/urdf/openarm_{"v10" if version == "v1.0" else "v20"}.urdf.xacro'
    arguments = {k: str(v).lower() if isinstance(v, bool) else str(v)
                 for k, v in arguments.items()}
    with tempfile.TemporaryDirectory() as directory:
        share = Path(directory)
        if files:
            shutil.copytree(ROOT/'assets', share/'assets',
                            copy_function=os.symlink)
            for name, value in files.items():
                path = share/'assets'/name
                path.parent.mkdir(parents=True, exist_ok=True)
                if path.exists() or path.is_symlink():
                    path.unlink()
                path.write_text(value if isinstance(value, str)
                                else yaml.safe_dump(value, sort_keys=False))
        else:
            share = ROOT
        with package_share_at(share):
            robot = ET.fromstring(xacro.process_file(
                str(share/entry), mappings=arguments).toxml())
    poses(robot)
    return robot


def model(links=('root',), joints=None):
    joints = joints or {}
    return {'struct/topology.yaml': dict(links={name: {} for name in links}, joints=deepcopy(joints)),
            'struct/reference_points.yaml': {'port': dict(parent=links[-1], emit=False, origin=pose(.1, .2, .3, .2, -.1, .3))},
            'joint/joint_origins.yaml': {j: dict(origin=pose(z=.1)) for j in joints},
            'joint/joint_axes.yaml': {j: dict(axis=dict(x=0, y=0, z=1)) for j in joints},
            'joint/joint_limits.yaml': {j: dict(limit=dict(lower=-.4, upper=.8, effort=8, velocity=2)) for j in joints},
            'joint/joint_mimics.yaml': {},
            'inertials/nominal.yaml': {links[0]: dict(origin=pose(.01, .02, .03), mass=2.3,
                                                      inertia=dict(xx=.04, yy=.05, zz=.06, xy=.001, xz=.002, yz=.003))},
            'link/visuals.yaml': {}, 'link/collisions.yaml': {}}


def chain():
    return model(('root', 'middle', 'tip'), {
        'pivot': dict(type='revolute', parent='root', child='middle'),
        'end': dict(type='revolute', parent='middle', child='tip')})


def pinch():
    data = model(('link1', 'link2', 'link3'), {
        'joint1': dict(name='finger_joint1', type='revolute', parent='link1', child='link2'),
        'joint2': dict(name='finger_joint2', type='revolute', parent='link1', child='link3')})
    data['struct/topology.yaml']['links'] = {
        'link1': dict(name='ee_base_link'),
        'link2': dict(name='ee_link1'), 'link3': dict(name='ee_link2')}
    data['joint/joint_mimics.yaml'] = {'joint2': dict(
        joint='joint1', multiplier=-1, offset=.1)}
    data['struct/reference_points.yaml'] = {'grasp_frame': dict(
        parent='link1', emit=False, origin=pose(z=.12))}
    return data


def component(kind='body', product='fixture', prefix='unit_', **extra):
    return dict(type=kind, product=product, config='arm' if kind == 'robot' else None,
                prefix=prefix, enabled=True, inertials='nominal', **extra)


def preset(**components):
    return dict(robot=dict(root_frame='world', components=list(components)), **components)


def files_for(spec, models, version='v2.0'):
    files = {f'robot/openarm_{version}/config/robot_presets/fixture.yaml': spec}
    for name, data in models.items():
        cfg = spec[name]
        path = f"{cfg['type']}/{cfg['product']}/config" + \
            ('/arm' if cfg['type'] == 'robot' else '')
        files.update({path+'/'+key: value for key, value in data.items()})
    return files


def assemble(spec, models, version='v2.0', **arguments):
    args = dict(robot_preset='fixture', ros2_control=False)
    args.update(arguments)
    return expand(files_for(spec, models, version), version=version, **args)


def installed_tool(collapse=False):
    arm, tool = chain(), pinch()
    arm['struct/topology.yaml']['collapse_empty_end'] = collapse
    arm['struct/reference_points.yaml']['port']['origin'] = pose(
        .02, -.03, .04, .2, -.3, .1)
    spec = preset(arm=component('robot', 'fixture_arm', 'arm_'),
                  tool=component('end_effector', 'pinch_gripper', 'tool_',
                                 connection=dict(attach_to='arm', parent_reference_point='port', origin=pose(y=.02, yaw=.4))))
    return spec, {'arm': arm, 'tool': tool}


def physical_equal(first, second):
    a, b = poses(first), poses(second)
    joints = {joint.get('name'): joint for joint in second.findall('joint')}
    for joint in first.findall('joint'):
        if joint.get('type') == 'fixed':
            continue
        other = joints[joint.get('name')]
        assert joint.get('type') == other.get('type')
        for tag in ('limit', 'mimic'):
            before, after = joint.find(tag), other.find(tag)
            assert (before.attrib if before is not None else None) == (
                after.attrib if after is not None else None)
        axes = []
        for item, frames in ((joint, a), (other, b)):
            local = [[float(v)] for v in item.find('axis').get('xyz').split()]
            rotation = [row[:3]
                        for row in frames[item.find('child').get('link')][:3]]
            axes.append(multiply(rotation, local))
        near(*axes)
    links = {link.get('name'): link for link in second.findall('link')}
    for old in first.findall('link'):
        name = old.get('name')
        if name not in links:
            assert not list(old), 'Only empty links may disappear'
            continue
        new = links[name]
        for kind in ('visual', 'collision', 'inertial'):
            left, right = old.findall(kind), new.findall(kind)
            assert len(left) == len(right)
            for before, after in zip(left, right):
                wa, wb = multiply(a[name], transform(before.find('origin'))), multiply(
                    b[name], transform(after.find('origin')))
                near(wa, wb)
                if kind == 'inertial':
                    assert float(before.find('mass').get('value')) == float(
                        after.find('mass').get('value'))

                    def world_tensor(element, world):
                        raw = element.find('inertia')
                        tensor = [
                            [float(raw.get('i'+x+y if x <= y else 'i'+y+x)) for y in 'xyz'] for x in 'xyz']
                        rotation = [row[:3] for row in world[:3]]
                        return multiply(multiply(rotation, tensor), list(zip(*rotation)))
                    near(world_tensor(before, wa), world_tensor(after, wb))
                else:
                    assert ET.canonicalize(ET.tostring(before.find('geometry')), strip_text=True) == ET.canonicalize(
                        ET.tostring(after.find('geometry')), strip_text=True)
