# V2 视觉模型与相机装配

本版本更新 link4 排线罩和底座视觉模型，在夹爪上增加相机视角参考坐标，默认双臂包含独立相机安装座与 ZED Mini 组件。

## 基线

基于 `206eb8a`，整合此前确认的 V1 修复：`109076c`、`496500f`、`53c5710`、`407fb47`、`e4f5396`。保留主线已经修正的 V2 双臂安装原点 `(0, ±0.030, 0.697)` m。

底座继续使用称重结果：支架 3.55 kg，加约 0.19 kg 塑料罩，合计 **3.74 kg**。COM 与对角惯量沿用已确认参数。新导出 URDF 中约 9.864 kg 的底座质量未用于此模型。

## 组件组织

```text
world
└── openarm_body_link0
    ├── 左右机械臂与夹爪
    │   └── openarm_left/right_camera_view_frame
    └── openarm_body_link0_camera_mount_point
        └── openarm_body_zed_mount_base_link
            └── openarm_body_zed_mount_camera_mount_point
                └── openarm_body_zed_camera_link
                    └── 官方 ZED 中心、左右相机、光学与 IMU 坐标
```

安装座属于独立 `accessory` 组件，文件位于 `assets/accessory/zed_mount/`。其几何包含 Head_Cover、cameramount_head_zed 和两颗 SCB2-5 螺丝；相机模型由 `zed_description` 提供。

相机和安装座各有自己的连接变换；更换相机时调整相机组件，安装座作为独立刚体保留。

| 预设 | 内容 |
|---|---|
| `default_bimanual` | 双臂、夹爪、底座、安装座和 ZED Mini |
| `left_arm` / `right_arm` | 单臂 |
| `left_arm_with_pinch_gripper` / `right_arm_with_pinch_gripper` | 单臂与夹爪 |

新的 `accessory` 类型支持一个刚体 link。其配置使用现有的 topology、inertials、visuals、collisions 和 reference_points 结构。参考点设置 `emit: true` 时生成无质量的固定坐标；子组件通过 `parent_reference_point` 直接连接该坐标，`connection.origin` 表示相对该坐标的附加变换。

## ZED 型号与安装基准

OpenArm 2.0 官方 FAQ 将头部相机列为 **ZED Mini**；安装文档使用 `2b03:f682 STEREOLABS ZED-M camera`。对应官方 URDF 参数为 `camera_model: zedm`。

- [OpenArm 2.0 相机说明](https://docs.openarm.dev/faq/#the-cameras-disconnect)
- [OpenArm 2.0 安装说明](https://docs.openarm.dev/tutorial/setup/#3-check-the-vendor-and-product-id-for-the-head-stereo-camera)
- [Stereolabs 机器人集成接口](https://docs.stereolabs.com/docs/integrations/ros-2/robot-integration)
- [ZED Mini 官方机械图纸](https://cdn.sanity.io/files/s18ewfw4/staging/14fab5dad2a9f36ce7f1e8024c3959a9611b990a.pdf/ZED-Mini-Datasheet.pdf)

导出安装座中的两颗 SCB2-5 螺丝间距约 56 mm，与 ZED Mini 图纸的底部 M2 孔组一致。官方接口使用 `camera_link` 连接机器人；它没有单独提供两个 M2 安装孔中点的 TF。安装座的 CAD 参考点则位于这两个孔的中心连线中点。

本预设按官方 `zedm.stl` 的底部安装区对称中心估计安装孔组中点；它来自简化网格，而非精确螺纹孔 CAD。该点在 `camera_link` 坐标系下约为 `(-1.19247, 7.723415, -0.08746)` mm，因此预设中孔中点到 `camera_link` 的位置为：

```yaml
origin:
  x: 0.00119247
  y: -0.007723415
  z: 0.00008746
  roll: 0.0
  pitch: 0.0
  yaw: 0.0
```

这是针对官方可视模型的装配变换；实机外参调整入口为 `body_camera.connection.origin`。光学与 IMU 坐标继续由官方宏生成。

安装座使用单独导出的 CAD 惯性，质量为 **0.13704 kg**。官方 ZED 宏没有提供相机惯性，所以相机 link 当前没有独立 inertial 项。

## CAD 参考点

| 参考点 | 父坐标 | xyz，m | rpy，rad |
|---|---|---|---|
| 底座安装座接口 | body_link0 | `(0.03, 0, 0.743)` | `(0, 0, 0)` |
| 安装座上的双孔中点 | 安装座 base_link | `(0.0263085336684, 0.00869, 0.065829226007701)` | `(0, 1.08210413623648, 0)` |
| 双孔中点组合结果 | body_link0 | `(0.0563085336684035, 0.00869, 0.808829226007701)` | `(0, 1.08210413623648, 0)` |
| 右夹爪相机视角参考点 | 导出 EE base_link | `(0.0013353, -0.024136, -0.086895)` | `(2.1009, 1.1424, 2.1431)` |

安装座接口采用“带安装座的整体导出”中精度较高的坐标，和单独安装座的四舍五入值相符。分体装配与整体导出的轴对齐包围盒边界最大差小于 0.1 µm。

夹爪折叠空 link 后，原来的 20.5 mm 基座平移会同时作用于新参考点。左右视角采用 `p_left = S p_right`、`R_left = S R_right S`，其中 `S = diag(1, -1, 1)`。参考点的旋转和位置同时镜像，且折叠前后的世界位姿一致。

`camera_view_frame` 保留 CAD 导出的坐标轴定义；它是视角参考点，不会自动生成腕部相机模型或相机数据流。

## Mesh 与碰撞

新增视觉来源位于 toolset 的 `openarm_v2_urdf_additional/`：

- link4：`openarm_v2.0_no_gripper_urdf_with_cable_cover.SLDASM/meshes/link4_link4.dae`。
- 底座：`openarmv2.0bimanualURDF.SLDASM/meshes/base_link_base_link.dae`，不含独立安装座。
- 安装座：`ZED_mount.SLDASM/meshes/base_link_base_link.dae`。
- 夹爪参考点：`end-efector_follower_4310_v1.0_urdf_with_cam_view.SLDASM` 的 URDF。

这些新视觉模型使用米制坐标。底座 visual 的 scale 从 0.001 改为 1，link4 保持 scale 1。link4 外罩、两侧排线罩、Head_Cover 和 chest_cover_v2.0 的材质与已有 link3 对齐：使用 Lambert 哑光材质，diffuse 为 `(0.247, 0.247, 0.247, 1)`。这些塑料件原先使用 CAD 导出的灰色 `0.502`；其余材质保留导出值。调整只涉及 DAE 的材质定义，几何、法线与装配变换保持一致。

link4 与底座继续使用现有 collision mesh。新安装座的 collision 配置为空，等待后续生成；ZED 相机使用官方 collision mesh。其他手臂关节、限位、质量和惯量沿用既有定义。

## 使用

```bash
cd /home/li/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select openarm_description
source install/setup.bash

ros2 launch openarm_description display_openarm.launch.py \
  arm_type:=v20 robot_preset:=default_bimanual \
  emit_grasp_frame:=true rviz_config:=bimanual_camera.rviz
```

生成完整 URDF：

```bash
xacro /home/li/ros2_ws/src/openarm_description/assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro \
  robot_preset:=default_bimanual emit_grasp_frame:=true ros2_control:=false \
  > /home/li/ros2_ws/src/openarm_description/tmp/openarm_v2_bimanual.urdf
```

测试：

```bash
colcon test --packages-select openarm_description \
  --ctest-args -R v2_ --return-code-on-test-failure
```

测试覆盖默认相机装配、CAD 安装点、称重底座参数、左右旋转镜像、参考 frame 输出与折叠、拓扑驱动的关节定义、各 preset 控制接口，以及官方 ZED Mini 坐标树。

导出来源和官方模型哈希见 [v2_camera_sources.json](v2_camera_sources.json)。

## Xacro 组织与装配规则

V1 保留原入口和实现。V2 入口为 `assets/robot/openarm_v2.0/urdf/openarm_v20.urdf.xacro`，负责读取 preset、调用装配宏和按需输出控制接口。

| 位置 | 职责 |
|---|---|
| `assets/common/transforms.xacro` | 位姿组合、坐标反射 |
| `assets/common/links.xacro` | link、joint、惯性和刚性附件输出 |
| `assets/common/reference_frames.xacro` | 固定参考坐标输出 |
| `assets/common/components.xacro` | 组件配置加载与具体产品分发 |
| `assets/common/validation.xacro` | 组件连接和 link/joint 拓扑检查 |
| `assets/common/assembly.xacro` | 连接变换、末端空 link 折叠、组件树生成 |
| `assets/robot/openarm_v2.0/urdf/arm/` | V2 手臂生成宏 |
| `assets/end_effector/pinch_gripper/urdf/` | 夹爪宏和独立夹爪入口 |
| `assets/sensor/zed/urdf_wrapper/` | 官方 ZED 模型适配 |

`attach_to` 选择父组件。连接位置由 `parent_reference_point` 或 `parent` 二选一指定；两者不能同时填写非空值。`parent_reference_point` 查找父组件的安装接口，`parent` 直接指定最终 URDF link 名称。两者都省略时使用父组件的默认连接 link；根组件默认连接 `robot.root_frame`。

`connection.origin` 始终相对于选中的安装基准；省略时为零位姿。参考点设置 `emit: true` 时生成一个固定 frame，子组件连接到这个 frame；不输出 frame 时，将参考点与 `origin` 的变换相乘，直接连接父 link。两种表达保持同一世界位姿，没有新增 `offset` 字段。

`emit_grasp_frame` 保留原入口参数，并使用同一个参考坐标生成器。`assets/robot/openarm_v2.0/config/arm/struct/topology.yaml` 中的 `collapse_empty_end: true` 控制手臂末端折叠，左右臂共用该设置；省略该字段或设置为 `false` 时保留末端 link。开启时，当手臂末端 link 没有 visual、collision、inertial 且有一个合适的子组件时，将它折叠到子组件根 link。当前 V2 是将空 `link7` 折叠到 `ee_base_link`，并将 20.5 mm 安装变换传递给几何、COM、手指关节和参考点。折叠后同名 `ee_base_link` 的坐标原点改变，实体及参考点的世界位姿保持一致。

关节名称、类型和父子关系从 `topology.yaml` 读取；`links` 的首项是组件根 link，末项用于选择默认连接端。Pinch gripper 的基座与手指共用公共 link 输出宏。

## 控制与导出

V2 Xacro 的 `ros2_control` 默认 `true`；RViz launch 的该参数默认 `false`。控制接口只为启用的手臂和其夹爪生成，mimic 手指不重复生成主动控制接口。各手臂 preset 的 `control.side` 为 `left` 或 `right`，分别选择 `left_can_interface` 或 `right_can_interface`，与模型前缀分开配置。现有硬件插件、PD 增益和默认 CAN 参数保持原值。

导出纯模型可使用 `ros2_control:=false`，或在 description 目录运行：

```bash
bash scripts/generate_v20_urdfs.sh --all --no-ros2-control
```

需要保留末端空 link 时，将 arm topology 中的 `collapse_empty_end` 改为 `false`；Xacro、launch 和导出脚本不再提供折叠参数。`--grasp-frame` 输出抓取参考坐标。现有 V1 入口、配置和控制实现保持独立。
