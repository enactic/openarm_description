# V1 arm coordinate frames

V1 uses a right-arm definition and local Y reflection for left links 0, 1, 2, 5, 6 and 7. Links 3 and 4 keep the same part geometry on both sides. In the default bimanual zero pose, arm link frames are aligned with the body axes, up to the original mounting-angle rounding.

Joint angles, positive motor directions, limits, masses and physical geometry are retained. The model still uses `link7 -> hand -> fingers / hand_tcp`; the TCP remains 90 mm from hand. The hand and finger frames retain their previous world poses.

## Change of basis

For the right arm, the following constant transform describes the new link frame in its former link frame. Left-arm rotations have the opposite sign; the translations below are unchanged.

| Link | Translation in former frame, m | Rotation around former X |
| --- | --- | --- |
| link0, link1 | (0, 0, 0) | -90 degrees |
| link2 | (0.0301, 0, 0) | -180 degrees |
| link3 | (0, 0, 0) | -180 degrees |
| link4 | (0, -0.0315, 0) | -180 degrees |
| link5 | (0, 0, 0) | -180 degrees |
| link6 | (-0.0375, 0, 0) | -180 degrees |
| link7 | (0, 0, 0) | -180 degrees |

The three origin translations lie on their joints' rotation axes. They move the reference point without moving the axis line.

If `C_i = old_link_T_new_link`, each fixed joint origin is converted with `C_parent^-1 * T_old * C_child`. The joint axis is expressed in the new child frame. Mesh origins and COM use `C_i^-1`; the inertia about the COM is rotated by `R * I * R^T`. Moving the link origin does not require a parallel-axis term in an inertia still expressed about its COM.

The visual and collision mesh files are retained. Their YAML origins express the coordinate change. Reflection transforms both the mesh origin rotation and position, as well as its scale, COM and inertia.

## Joint coordinates

| Joint | Right local axis | Left local axis |
| --- | --- | --- |
| J1 | -Y | +Y |
| J2 | -X | -X |
| J3 | -Z | -Z |
| J4 | -Y | -Y |
| J5 | -Z | -Z |
| J6 | +X | +X |
| J7 | -Y | +Y |

Each positive joint command follows the right-hand rule about its listed signed axis. The change of basis preserves that axis in world space for every joint configuration; no motor command or feedback sign conversion is added.

Single-arm J2 retains its original +/-100 degree range. Its -90 degree origin rotation and fixed world mount retain the original single-arm zero pose. Bimanual J2 retains the original side-specific offsets and limits. Single-arm output now has an empty `world` root with a fixed attachment to the physical `link0`.

The bimanual base mount roll defaults become +/-0.000003673205103416066 rad. This is the residual of the old +/-1.5708 rad mounts after the +/-90 degree basis change. The public `right_arm_base_rpy` and `left_arm_base_rpy` parameters now describe the new link0 frames. Convert an existing custom mount by post-multiplying its rotation with Rx(-90 degrees) for the right arm or Rx(+90 degrees) for the left arm.

The hand mounting transform absorbs the link7 frame rotation, leaving the hand, fingers and TCP definitions unchanged. Standalone gripper output is unchanged.

Consumers of arm link TFs must use the new origins and axes. Joint-space trajectories remain valid. Coordinates of points or tools previously expressed directly in an arm link frame must undergo the same change of basis.

## Validation

The comparison baseline is description commit `6deb462`. Eight V1 combinations cover single/bimanual, prefix/no-prefix and control enabled/disabled. Each combination was checked at zero and 32 sampled joint configurations, including gripper motion: 264 configurations in total.

- World visual/collision transforms, COM, inertia, joint axis lines, hand/finger/TCP poses, the generalized mass matrix and gravity generalized forces agree within floating-point rounding (maximum numeric error below 6e-15).
- Joint limits, mimic definitions and ROS2 control output are unchanged.
- All five V2 presets and standalone parallel gripper output retain their model definitions.

These checks establish model equivalence; no hardware motion was performed.
