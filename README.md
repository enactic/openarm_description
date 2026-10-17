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

Use `robot_preset:=bimanual_with_zed` for the bimanual model with a removable camera mount and the official ZED Mini model. `bimanual_with_camera_mount` adds only the mount; `default_bimanual` provides the updated pedestal and arms.

See [V2 visual models and camera assembly](dev/v2_camera_assembly.md) for the source geometry, mounting frames, measured pedestal parameters, and visualization commands.

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
