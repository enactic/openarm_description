# Robot Description files for OpenArm

This package contains the Xacro, configuration and mesh files that generate OpenArm URDFs (Unified Robot Description Format) for OpenArm 1.0, OpenArm 2.0 and OpenArm Cell. See the [documentation](https://docs.openarm.dev/api-reference/description) for details.

## Installation

Clone this repository into a ROS 2 workspace and build it (Jazzy example):

```bash
source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws/src
git clone https://github.com/enactic/openarm_description.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -y
colcon build --packages-select openarm_description
source install/setup.bash
```

## Robot presets

A preset is a YAML file under `presets/` that lists the components of a robot and how they are connected. It is selected with `robot_preset:=<group>/<name>`.

| Preset | Robot |
| --- | --- |
| `v2/default_bimanual` | OpenArm 2.0 body, both arms with pinch grippers and the body camera (default) |
| `v2/cell_bimanual` | OpenArm Cell with its lifter, both arms with pinch grippers and the body camera |
| `v2/left_arm`, `v2/right_arm` | One OpenArm 2.0 arm |
| `v2/left_arm_with_pinch_gripper`, `v2/right_arm_with_pinch_gripper` | One OpenArm 2.0 arm with a pinch gripper |
| `v1/default_bimanual` | OpenArm 1.0 body and both arms with parallel-link grippers |
| `v1/default_single` | One OpenArm 1.0 arm with a parallel-link gripper |
| `v1/left_arm`, `v1/right_arm` | One OpenArm 1.0 arm |
| `v1/left_arm_with_parallel_link`, `v1/right_arm_with_parallel_link` | One OpenArm 1.0 arm with a parallel-link gripper |

To describe your own robot, copy a preset to a new file in the same directory, edit it and pass its name as `robot_preset`.

## Usage

Preview a preset in RViz with a joint state GUI:

```bash
ros2 launch openarm_description display_openarm.launch.py robot_preset:=v2/default_bimanual
```

Generate a URDF:

```bash
xacro $(ros2 pkg prefix openarm_description)/share/openarm_description/urdf/openarm.urdf.xacro \
  robot_preset:=v2/default_bimanual > openarm.urdf
```

`urdf/openarm.urdf.xacro` accepts the following arguments:

| Argument | Default | Description |
| --- | --- | --- |
| `robot_preset` | `v2/default_bimanual` | Preset to generate. |
| `emit_grasp_frame` | `false` | Add a grasp frame to each end effector. |
| `ros2_control` | `true` | Emit the `ros2_control` section. |
| `use_fake_hardware` | `true` | Use mock hardware instead of the real hardware interface. |
| `fake_sensor_commands` | `false` | Enable fake sensor commands on the mock hardware. |
| `can_interface` | `""` | Override the standalone system CAN interface (fallback: `can0`). |
| `left_can_interface` | `""` | Override the left-arm CAN interface (fallback: `can1`). |
| `right_can_interface` | `""` | Override the right-arm CAN interface (fallback: `can0`). |
| `can_fd` | `""` | Override CAN FD mode (fallback: `true`); `false` selects classic CAN. |

The CAN arguments apply when `use_fake_hardware:=false`. Empty values preserve the preset system settings; omitted settings use the hardware defaults listed above.

## Compatibility with earlier versions

The presets moved from `assets/robot/openarm_v*/config/robot_presets/` to `presets/v1/` and `presets/v2/`. The previous preset directories are symbolic links to the new ones. The V1/V2 Xacro entry points remain at their previous paths.

Arm assets now live under `assets/arm/openarm_v*/`, with YAML files directly in `config/` and meshes in `meshes/`. In custom presets, replace arm components using `type: robot` and `config: arm` with `type: arm` and `config: null`; update any direct references to the old arm asset paths.

If a symlink build fails because an existing install contains real `robot_presets` directories where symlinks are now expected, remove this package's build and install directories and build again:

```bash
cd ~/ros2_ws
rm -rf build/openarm_description install/openarm_description
colcon build --packages-select openarm_description --symlink-install
source install/setup.bash
```

## Components with their own Xacro

Each component of a preset is a directory `assets/<type>/<product>/`, found from the `type` and `product` fields. The component types are `arm`, `body`, `end_effector`, `sensor` and `accessory`. For example, `type: arm` and `product: openarm_v2.0` select `assets/arm/openarm_v2.0/`.

The preset's `config` field is a mapping of component-specific parameters, or `null` when none are needed. It does not select a directory. The loader passes it to the component, which interprets and validates its own fields; for example, ZED reads `config.camera_model` and `config.camera_name`.

By default a component needs no Xacro. Its links, joints, meshes and inertials are generated from the YAML files in its `config/` directory, and it is attached to its parent with a fixed mounting joint.

A component that cannot be described this way, such as a third-party model, keeps its own Xacro in its directory and adds a `component.xacro` file under `urdf/` (example: [ZED cam](assets/sensor/zed/urdf/component.xacro), [pinch_gripper](assets/end_effector/pinch_gripper/urdf/component.xacro)) as the entry point that connects it to the assembly. The entry point is found automatically and needs no registration. It defines one or both of these macros; a macro it does not define keeps the default behavior:

| Macro | Parameters | What it must do |
| --- | --- | --- |
| `describe_component` | `component_name component_cfg config_root options out_name` | Set the property named by `out_name` in the parent scope to a descriptor of the component: its root link, topology, model data and reference points. |
| `emit_component` | `component_desc component_runtime options` | Emit the links and joints of the component, including the joint that mounts it on the parent given in `component_runtime`. |

The entry point must contain only these macro definitions. The model itself stays in separate files, which the macros include, so that it does not affect other components.

In the ZED camera example, the [wrapper](assets/sensor/zed/urdf_wrapper/openarm_zed_wrapper.xacro) includes the external `zed_description` package, and `emit_component` calls that wrapper. `describe_component` resolves the camera name and describes its root link.

## Development

See [dev/README.md](dev/README.md) for how to test and release.

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/api-reference/description)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

[Apache License 2.0](LICENSE.txt)

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
