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
import os
import importlib.util
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnShutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# All accepted arm_type values
V1_ARM_TYPES = {"v1.0", "v10", "v1_0",
                "openarm_v1.0", "openarm_v10", "openarm_v1_0"}
VALID_ARM_TYPES = {
    "v1.0", "v10", "v1_0", "openarm_v1.0", "openarm_v10", "openarm_v1_0",
    "v2.0", "v20", "v2_0", "openarm_v2.0", "openarm_v20", "openarm_v2_0",
}


def resolve_preset_id(arm_type, robot_preset, bimanual):
    """Return the full preset identifier used by the public xacro entry.

    A full identifier such as ``v2/cell_bimanual`` is used as given. A short
    name is completed from the selected arm type. No name keeps the historical
    default selection for that arm type.
    """
    if arm_type not in VALID_ARM_TYPES:
        raise ValueError(
            f"Invalid arm_type: '{arm_type}'. "
            f"Accepted values: {sorted(VALID_ARM_TYPES)}.")
    preset = robot_preset.strip()
    is_v1 = arm_type in V1_ARM_TYPES
    group = "v1" if is_v1 else "v2"
    if "/" in preset:
        return preset
    if preset:
        return group + "/" + preset
    if is_v1:
        return "v1/" + ("default_bimanual" if bimanual.lower() in ("true", "1")
                        else "default_single")
    return "v2/default_bimanual"


def robot_state_publisher_spawner(
    context: LaunchContext,
    arm_type,
    robot_preset,
    emit_grasp_frame,
    bimanual,
    ros2_control,
    use_fake_hardware,
    fake_sensor_commands,
    rviz_config,
):
    arm_type_str = context.perform_substitution(arm_type)
    preset_id = resolve_preset_id(arm_type_str,
                                  context.perform_substitution(robot_preset),
                                  context.perform_substitution(bimanual))

    pkg_share = get_package_share_directory("openarm_description")

    xacro_path = os.path.join(pkg_share, "urdf", "openarm.urdf.xacro")

    mappings = {
        "robot_preset": preset_id,
        "emit_grasp_frame": context.perform_substitution(emit_grasp_frame),
        "ros2_control": context.perform_substitution(ros2_control),
        "use_fake_hardware": context.perform_substitution(use_fake_hardware),
        "fake_sensor_commands": context.perform_substitution(fake_sensor_commands),
    }

    preset_path = os.path.join(pkg_share, "presets", preset_id + ".yaml")
    with open(preset_path) as stream:
        preset = yaml.safe_load(stream)
    spec = importlib.util.spec_from_file_location(
        'openarm_preview_config', os.path.join(pkg_share, 'scripts', 'preview_config.py'))
    preview = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(preview)
    robot_description, joint_names = preview.expand_preview_model(
        xacro_path, mappings, preset)
    positions = preview.resolve_initial_positions(
        preset, joint_names, robot_description)
    rviz_path = os.path.join(
        pkg_share, 'rviz', context.perform_substitution(rviz_config))
    cleanup = []
    view = preset.get('preview', {}).get('view')
    if view:
        with open(rviz_path) as stream:
            config = preview.apply_preview_view(
                yaml.safe_load(stream), view, preset['robot']['root_frame'])
        temporary = tempfile.TemporaryDirectory(prefix='openarm-preview-')
        rviz_path = os.path.join(temporary.name, 'preview.rviz')
        with open(rviz_path, 'w') as stream:
            yaml.safe_dump(config, stream)
        cleanup.append(RegisterEventHandler(OnShutdown(
            on_shutdown=lambda event, context: temporary.cleanup())))

    return cleanup + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            parameters=[{'zeros.' + name: value for name,
                         value in positions.items()}],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["--display-config", rviz_path],
        ),
    ]


def generate_launch_description():
    arm_type_arg = DeclareLaunchArgument(
        "arm_type",
        default_value="v20",
        description="Arm type used to complete a short preset name. "
                    "Accepts: v1.0, v10, openarm_v1.0, v2.0, v20, openarm_v2.0, etc.",
    )
    robot_preset_arg = DeclareLaunchArgument(
        "robot_preset",
        default_value="",
        description="Robot preset, either a full identifier such as v2/cell_bimanual "
                    "or a short name completed from arm_type; empty selects the default.",
    )
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="bimanual.rviz",
    )
    grasp_frame_arg = DeclareLaunchArgument(
        "emit_grasp_frame",
        default_value="false",
    )
    bimanual_arg = DeclareLaunchArgument(
        "bimanual",
        default_value="true",
        description="Enable bimanual configuration (v1.0 only).",
    )
    ros2_control_arg = DeclareLaunchArgument(
        "ros2_control",
        default_value="false",
    )
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="false",
    )
    fake_sensor_commands_arg = DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
    )

    arm_type = LaunchConfiguration("arm_type")
    robot_preset = LaunchConfiguration("robot_preset")
    rviz_config = LaunchConfiguration("rviz_config")
    grasp_frame = LaunchConfiguration("emit_grasp_frame")
    bimanual = LaunchConfiguration("bimanual")
    ros2_control = LaunchConfiguration("ros2_control")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    return LaunchDescription([
        arm_type_arg,
        robot_preset_arg,
        rviz_config_arg,
        grasp_frame_arg,
        bimanual_arg,
        ros2_control_arg,
        use_fake_hardware_arg,
        fake_sensor_commands_arg,
        OpaqueFunction(
            function=robot_state_publisher_spawner,
            args=[
                arm_type,
                robot_preset,
                grasp_frame,
                bimanual,
                ros2_control,
                use_fake_hardware,
                fake_sensor_commands,
                rviz_config,
            ],
        ),
    ])
