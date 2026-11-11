# Copyright 2025 Enactic, Inc.
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
"""Preset positions and camera defaults shared by preview callers."""
import math
import xml.etree.ElementTree as ET

import xacro


def expand_preview_model(path, mappings, preset):
    """Read final joint names from the component descriptions in the same expansion."""
    xacro.init_stacks(path)
    doc = xacro.parse(None, path)
    bindings = []
    enabled = {name for name in preset['robot']
               ['components'] if preset[name]['enabled']}
    for system in preset.get('systems', {}).values():
        for component, joints in system['joints'].items():
            if component not in enabled:
                continue
            for key in joints:
                node = doc.createElement('preview_binding')
                node.setAttribute(
                    'name', '${component_descriptions[%r]["joint_name_map"][%r]}' % (component, key))
                doc.documentElement.appendChild(node)
                bindings.append((component, key, node))
    xacro.process_doc(doc, mappings=mappings)
    names = {}
    for component, key, node in bindings:
        names.setdefault(component, {})[key] = node.getAttribute('name')
        doc.documentElement.removeChild(node)
    return doc.toxml(), names


def resolve_initial_positions(preset, joint_name_maps, robot_description, overrides=None):
    """Resolve init_pos, optionally overridden by {system_name: {idx: position}}.

    Omitted positions stay omitted. Returned names are the generated URDF names.
    """
    overrides = overrides or {}
    systems = preset.get('systems', {})
    if set(overrides) - set(systems):
        raise ValueError('Unknown system in initial-position overrides')
    joints = {j.get('name'): j for j in ET.fromstring(
        robot_description).findall('joint')}
    positions = {}
    for system_name, system in systems.items():
        entries = overrides.get(system_name, {})
        seen = set()
        for component, selection in system['joints'].items():
            if component not in joint_name_maps:
                continue
            for key, config in selection.items():
                idx = config['idx']
                if idx in seen:
                    raise ValueError(f'Duplicate idx {idx} in {system_name}')
                seen.add(idx)
                if idx not in entries and 'init_pos' not in config:
                    continue
                value = float(
                    entries[idx] if idx in entries else config['init_pos'])
                name = joint_name_maps[component][key]
                joint = joints[name]
                if joint.get('type') not in ('revolute', 'continuous', 'prismatic') or joint.find('mimic') is not None:
                    raise ValueError(
                        f'Cannot initialize dependent or fixed joint {name}')
                if not math.isfinite(value):
                    raise ValueError(f'Non-finite init_pos for {name}')
                limit = joint.find('limit')
                if joint.get('type') != 'continuous' and limit is not None:
                    if not float(limit.get('lower')) <= value <= float(limit.get('upper')):
                        raise ValueError(
                            f'init_pos outside limits for {name}: {value}')
                if name in positions:
                    raise ValueError(f'Duplicate init_pos for {name}')
                positions[name] = value
        if set(entries) - seen:
            raise ValueError(f'Unknown or disabled joint idx in {system_name}')
    return positions


def apply_preview_view(config, view, root_frame):
    """Apply a Z-up, zero-roll eye/lookat view in the preset root frame."""
    eye = [float(view['eye'][axis]) for axis in 'xyz']
    target = [float(view['lookat'][axis]) for axis in 'xyz']
    if not all(math.isfinite(v) for v in eye + target):
        raise ValueError('Preview camera coordinates must be finite')
    dx, dy, dz = [a-b for a, b in zip(eye, target)]
    distance = math.sqrt(dx*dx + dy*dy + dz*dz)
    if distance == 0 or math.hypot(dx, dy) == 0:
        raise ValueError(
            'Preview Orbit camera needs distinct eye/lookat and a horizontal direction')
    manager = config['Visualization Manager']
    manager['Global Options']['Fixed Frame'] = root_frame
    camera = manager['Views']['Current']
    camera.update({'Class': 'rviz_default_plugins/Orbit', 'Target Frame': '<Fixed Frame>',
                   'Distance': distance, 'Yaw': math.atan2(dy, dx),
                   'Pitch': math.atan2(dz, math.hypot(dx, dy)),
                   'Focal Point': dict(zip('XYZ', target))})
    return config
