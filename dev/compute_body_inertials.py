"""Estimate the open-top chest cover from the measured closed-cover mass."""

import argparse
import hashlib
import json
from pathlib import Path

import collada
import numpy as np
import trimesh


def parallel_axis(position):
    return np.dot(position, position) * np.eye(3) - np.outer(position, position)


def integrate(vertices, faces):
    reference = (vertices.min(axis=0) + vertices.max(axis=0)) / 2
    triangles = vertices[faces] - reference
    a, b, c = triangles[:, 0], triangles[:, 1], triangles[:, 2]
    volumes = np.einsum('ij,ij->i', a, np.cross(b, c)) / 6
    volume = volumes.sum()
    sums = triangles.sum(axis=1)
    center = np.einsum('i,ij->j', volumes, sums) / (4 * volume)
    second = (
        np.einsum('i,ij,ik->jk', volumes, sums, sums)
        + np.einsum('i,iaj,iak->jk', volumes, triangles, triangles)
    ) / 20
    inertia = np.trace(second) * np.eye(3) - second - \
        volume * parallel_axis(center)
    return volume, center + reference, inertia, reference


def calculate(root, sources_path, mesh_override=None):
    inputs = json.loads(sources_path.read_text())
    selection = inputs['open_cover']
    mesh_path = mesh_override or root / selection['mesh']
    digest = hashlib.sha256(mesh_path.read_bytes()).hexdigest()
    if digest != selection['mesh_sha256']:
        raise ValueError(
            'The cover source mesh changed; update the source record first.')
    meshes = []
    document = collada.Collada(str(mesh_path))
    for geometry in document.scene.objects('geometry'):
        if geometry.original.id != selection['geometry_id']:
            continue
        for primitive in geometry.primitives():
            triangle = primitive.triangleset() if hasattr(
                primitive, 'triangleset') else primitive
            meshes.append(trimesh.Trimesh(
                vertices=np.asarray(
                    triangle.vertex, dtype=float) * selection['scale_to_m'],
                faces=triangle.vertex_index, process=True,
            ))
    if len(meshes) != 1:
        raise ValueError('Expected one chest-cover mesh.')
    mesh = meshes[0]
    if not mesh.is_winding_consistent:
        raise ValueError('The cover mesh has inconsistent triangle winding.')
    volume, center, unit_inertia, reference = integrate(
        mesh.vertices, mesh.faces)
    area_normals = mesh.face_normals * mesh.area_faces[:, None]
    flux_volumes = ((mesh.triangles_center - reference)
                    * area_normals).sum(axis=0)
    spread = np.ptp(flux_volumes) / volume
    if volume <= 0 or spread > 0.001:
        raise ValueError(
            'Cover surface integrals disagree by more than 0.1 percent.')
    density = (inputs['measurement']['closed_cover_mass_kg_approx']
               / inputs['cad_source']['closed_cover_volume_m3'])
    cover_mass = density * volume
    cover_inertia = density * unit_inertia
    metal = inputs['metal_base']
    metal_mass = metal['mass_kg']
    metal_center = np.asarray(metal['com_m'])
    mass = metal_mass + cover_mass
    combined_center = (metal_mass * metal_center + cover_mass * center) / mass
    full_inertia = (
        np.asarray(metal['inertia_kg_m2'])
        + metal_mass * parallel_axis(metal_center - combined_center)
        + cover_inertia + cover_mass * parallel_axis(center - combined_center)
    )
    reflection = np.diag([1., -1., 1.])
    symmetric_center = (combined_center + reflection @ combined_center) / 2
    symmetric_inertia = ((full_inertia + reflection @ full_inertia @ reflection) / 2
                         + mass * parallel_axis(combined_center - symmetric_center))
    diagonal = np.diag(symmetric_inertia)
    if min(diagonal) <= 0 or 2 * max(diagonal) >= sum(diagonal):
        raise ValueError('The resulting inertia is not physically admissible.')
    return {
        'closed_cover_volume_m3': inputs['cad_source']['closed_cover_volume_m3'],
        'open_cover_volume_m3': float(volume),
        'volume_ratio': float(volume / inputs['cad_source']['closed_cover_volume_m3']),
        'effective_density_kg_m3': float(density),
        'cover_mass_kg': float(cover_mass),
        'cover_mass_removed_kg': float(
            inputs['measurement']['closed_cover_mass_kg_approx'] - cover_mass),
        'cover_com_m': center.tolist(),
        'cover_inertia_kg_m2': cover_inertia.tolist(),
        'mesh_quality': {
            'watertight': bool(mesh.is_watertight),
            'winding_consistent': bool(mesh.is_winding_consistent),
            'integration_reference_m': reference.tolist(),
            'axis_volume_integrals_m3': flux_volumes.tolist(),
            'relative_axis_volume_spread': float(spread),
            'faces': len(mesh.faces),
        },
        'body': {
            'mass_kg': float(mass),
            'com_m': symmetric_center.tolist(),
            'diagonal_inertia_kg_m2': diagonal.tolist(),
            'products_of_inertia_kg_m2': [0., 0., 0.],
        },
        'mesh_sha256': digest,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path,
                        default=Path(__file__).resolve().parents[1])
    parser.add_argument('--sources', type=Path)
    parser.add_argument('--mesh', type=Path)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    result = calculate(args.root, args.sources or args.root /
                       'dev/body_inertial_sources.json', args.mesh)
    text = json.dumps(result, indent=2) + '\n'
    if args.output:
        args.output.write_text(text)
    print(text, end='')


if __name__ == '__main__':
    main()
