# Robot Description files for OpenArm

This package contains description files to generate OpenArm URDFs (Universal Robot Description Files). See [documentation](https://docs.openarm.dev/software/description) for details.

<!-- ## Project Entry Points

Use one of these entry documents depending on what you are working on:

- Active preset-driven robot pipeline:
  [docs/openarm_v2.0/00_OpenArm_v2.0_Description.md](docs/openarm_v2.0/00_OpenArm_v2.0_Description.md)
- Minimal legacy compatibility layer for old `v10` robots:
  [docs/openarm_v1.0/00_OpenArm_v1.0_Compatibility.md](docs/openarm_v1.0/00_OpenArm_v1.0_Compatibility.md)
- Extraction and helper scripts:
  [docs/openarm_v2.0/01_URDF_Extract_Scripts.md](docs/openarm_v2.0/01_URDF_Extract_Scripts.md)

Recommended starting point:

- for all new robot assembly, presets, xacro, and URDF generation work, start with `assets/robot/openarm_v2.0`
- only use `assets/robot/openarm_v1.0` when you need the old `v10 + parallel_link` compatibility path -->

## V2 camera assembly

The V2 `default_bimanual` preset includes both arms, pinch grippers, the camera mount, and the official ZED Mini model. Single-arm presets are also available. Use `ros2_control:=false` when exporting a model without control interfaces.

## Component interfaces

Robot entries load `assets/component_loader.xacro`. The loader locates each component using the existing `type`, `product`, and `config` fields, then optionally loads `<component directory>/urdf/component.xacro`. Components without this file use the standard YAML model generator.

A component interface can override either or both macros:

| Macro | Parameters | Result |
| --- | --- | --- |
| `describe_component` | `component_name component_cfg config_root options out_name` | Set the property named by `out_name` in the parent scope to a component descriptor. |
| `emit_component` | `component_desc component_runtime options` | Emit the component links, joints, mounting joint and reference frames. |

The default implementations are `openarm_describe_config_component` and `openarm_emit_config_component`. A descriptor supplies the root link, topology, model data and reference points; `supports_frame_shift` states whether its root can absorb a parent's empty terminal link. The runtime supplies resolved mounting, reflection and joint overrides.

Interface files declare macros; implementation and third-party model includes belong inside those macros. Each interface is loaded in a local scope, so its definitions do not affect other components. Presets need no implementation selector or product registry.

Component topology uses stable `link1`, `link2`, ... and `joint1`, `joint2`, ... keys. An optional `name` on each topology entry defines its local URDF name; otherwise the key is used. The instance prefix is added once. Geometry, inertia, joint parameters, parent/child links, mimic targets, references, reflection selections and overrides use the stable keys. Changing `name` preserves these config references. Component descriptors expose resolved `link_name_map` and `joint_name_map` entries for emitters and control generation.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/description)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE.txt)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
