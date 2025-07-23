# Robot Description files for OpenArm

This package contains description files to generate OpenArm URDFs (Universal Robot Description Files).

To generate or view the URDF, install ROS2, source the setup script, and clone this package into a workspace.

For example, for ROS2 Humble:
```sh
source /opt/ros/humble/setup.bash
mkdir -p ~/ros2_ws/src
git clone https://github.com/enactic/openarm_description.git ~/ros2_ws/src/openarm_description
colcon build
source ~/ros2_ws/install/setup.bash
```

Then to view the bimanual URDF:
```sh
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true
```

Or to generate a URDF:
```sh
URDF_NAME=openarm_bimanual.urdf
xacro ~/ros2_ws/src/openarm_description/urdf/robot/v10.urdf.xacro arm_type:=v10 bimanual:=true > $URDF_NAME
```

## Related links

- 📚 Read the [documentation](https://docs.openarm.dev/software/description)
- 💬 Join the community on [Discord](https://discord.gg/FsZaZ4z3We)
- 📬 Contact us through <openarm@enactic.ai>

## License

Licensed under the Apache License 2.0. See `LICENSE.txt` for details.

Copyright 2025 Enactic, Inc.

## Code of Conduct

All participation in the OpenArm project is governed by our
[Code of Conduct](CODE_OF_CONDUCT.md).
