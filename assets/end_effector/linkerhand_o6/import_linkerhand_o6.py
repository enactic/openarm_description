#!/usr/bin/env python3
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

"""Manually import a pinned official O6 revision; never invoked by the build."""
import argparse
import json
from pathlib import Path
import subprocess
import xml.etree.ElementTree as ET

COMMIT = '49f26021b0a91ff5c33150df22970c07bb548672'
REPOSITORY = 'https://github.com/linker-bot/linkerhand-urdf'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True,
                        help='local official Git checkout')
    parser.add_argument('--commit', default=COMMIT, help='revision to import')
    parser.add_argument('--output', type=Path, default=Path(
        __file__).resolve().parent)
    args = parser.parse_args()

    def git(*params):
        return subprocess.check_output(['git', '-C', str(args.source), *params])

    revision = git('rev-parse', '--verify', args.commit +
                   '^{commit}').decode().strip()
    output = args.output
    for side in ('left', 'right'):
        source_root = f'O6/{side}'
        paths = git('ls-tree', '-r', '--name-only', revision,
                    source_root + '/meshes').decode().splitlines()
        if not paths:
            raise ValueError(f'No official meshes for {side} at {revision}')
        for path in paths:
            relative = Path(path).relative_to(source_root)
            target = output / side / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes(git('show', revision + ':' + path))
        root = ET.fromstring(
            git('show', revision + ':' + source_root + f'/linkerhand_o6_{side}.urdf'))
        for material in root.iter('material'):
            name = material.get('name')
            if name:
                material.set('name', f'linkerhand_o6_{side}_{name}')
        for mesh in root.iter('mesh'):
            path = Path(mesh.get('filename'))
            if path.is_absolute() or '..' in path.parts or path.parts[0] != 'meshes':
                raise ValueError(f'Unsupported official mesh path: {path}')
            if not (output / side / path).is_file():
                raise FileNotFoundError(output / side / path)
            mesh.set(
                'filename', f'package://openarm_description/assets/end_effector/linkerhand_o6/{side}/{path.as_posix()}')
        ET.indent(root)
        target = output / side / 'urdf' / f'linkerhand_o6_{side}.urdf'
        target.parent.mkdir(parents=True, exist_ok=True)
        ET.ElementTree(root).write(
            target, encoding='utf-8', xml_declaration=True)
    (output / 'LICENSE').write_bytes(git('show', revision + ':LICENSE'))
    (output / 'SOURCE.json').write_text(json.dumps(dict(repository=REPOSITORY,
                                                        commit=revision, source_directory='O6'), indent=2) + '\n')
    print(f'Imported {revision} into {output}; meshes are unchanged.')


if __name__ == '__main__':
    main()
