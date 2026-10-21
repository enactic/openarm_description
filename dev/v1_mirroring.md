# V1 统一配置与装配

V1 与 V2 共用组件装配、运动链、参考坐标和 ROS2 control 生成逻辑，保留各自的 Xacro 入口。V1 以 `8624da6` 的物理模型为迁移基准，保留电机方向、角度零点、限位、质量、质心惯量及实体装配。默认 V1 不包含相机。

## 文件组织

```text
assets/common/
  transforms.xacro
  links.xacro
  kinematic_chain.xacro
  reference_frames.xacro
  components.xacro
  validation.xacro
  assembly.xacro
  ros2_control.xacro

assets/robot/openarm_v1.0/
  config/
    arm/{struct,joint,link,inertials,control}/
    robot_presets/
  urdf/
    openarm_v10.urdf.xacro
    example/v1.urdf

assets/body/v1.0/
  config/{struct,link,inertials}/
  meshes/{visual,collision}/

assets/end_effector/parallel_link/
  config/{struct,joint,link,inertials}/
  meshes/{visual,collision}/
  urdf/parallel_link_standalone.urdf.xacro
```

`kinematics_offset.yaml`、旧的扁平配置、V1 专用 arm/body/EE/control 生成宏和临时 `openarm_joint_from_config` 转换宏已移除。手臂与 parallel gripper 都由公共运动链根据 topology 输出；Pinch gripper 的组件实现仍留在自身目录。

## 关节原点靠近中线

本次只沿关节转轴平移参考原点，没有旋转 link 坐标轴：

| Link | 新原点在原 link 坐标系中的位置，m |
| --- | --- |
| link2 | (+0.0301, 0, 0) |
| link4 | (0, -0.0315, 0) |
| link6 | (-0.0375, 0, 0) |
| 其他实体 link | (0, 0, 0) |

关节原点、mesh 原点和 COM 同步换算。惯量继续表达在 COM，坐标轴未旋转，因此质心惯量张量数值不变。参考原点平移没有移动实体或转轴线。

J2/J3、J4/J5、J6/J7 原来的横向连接偏移由此消除。单臂零位时 link0–7 的轴仍与原单臂同向；双臂仍使用原来的旋转式安装。直接用旧 link2/4/6 坐标表达的外部点，需要减去上表的原点平移。

底座是独立的 `body` 组件，preset 使用 `type: body`、`product: v1.0`、`config: null`。完整胸罩版本保持 3.74 kg；开口胸罩版本见 [body 组件与惯量](body_components.md)。Preset 暂留在各版本 robot 目录，后续迁到独立装配配置目录。

## 单臂、双臂与镜像

基础 `arm/joint/joint_origins.yaml` 和 `joint_limits.yaml` 使用单臂右侧定义。双臂 preset 用完整的 `origin_override_joints.joint2` 和 `limit_override_joints.joint2` 声明安装结果；先覆盖右侧基准，再生成左侧镜像：

| 模式 | 根安装绕 X | J2 origin 绕 X | J2 限位 | J7 局部轴 |
| --- | ---: | ---: | --- | --- |
| 默认单臂 | 0 | 0 | 约 -100 到 +100 度 | +Y |
| 双臂右侧 | 约 +90 度 | 约 +90 度 | 约 -10 到 +190 度 | +Y |
| 双臂左侧 | 约 -90 度 | 约 -90 度 | 约 -190 到 +10 度 | -Y |

镜像名单直接写在 preset 的 `reflect.include` 中，link 与 joint 分开选择。中线化后，J5 已没有 -31.5 mm 的局部 Y 偏移，可以和 V2 一样参与关节镜像；J4 保留原转轴和非对称角度范围。

单臂增加一个固定的 `world` 根坐标，物理 link0 与世界之间是单位变换。控制关节名称和方向保持原定义。

## 完整夹爪组件

Parallel gripper 现在包含 `link7` 的配置和 mesh。默认生成树仍为：

```text
link6 --J7--> link7 --fixed--> hand
                              ├── right_finger
                              ├── left_finger
                              └── hand_tcp
```

- 左侧夹爪只镜像 **link7** 的 mesh、COM 和惯量。
- hand、手指、手指关节和 TCP 保持同向装配，`reflect.include.joints` 与 `reference_points` 均为空。
- 另一只手指已有的 mesh 绕 Z 旋转保持不变。
- TCP 仍为 hand 局部 `(0, 0, 0.09)` m，无质量。

手臂末端 `arm_end_link` 是空安装点；`arm/struct/topology.yaml` 的 `collapse_empty_end: true` 将它折叠到夹爪根 link7。设为 false 时保留空安装点和固定连接。移除夹爪组件时，arm_end_link 留作安装参考，link7 的实体质量随夹爪组件移除。

折叠时先镜像每项局部数据，再应用安装变换。该顺序允许组件中只有 link7 镜像，而 hand 和手指不镜像；带旋转、平移的自定义安装也保持折叠前后物理等效。

## 入口与控制

| Preset | 内容 |
| --- | --- |
| default_bimanual | 底座、双臂、两个完整 parallel gripper |
| default_single | 原默认单臂与完整夹爪，保留 openarm_ 前缀 |
| left_arm / right_arm | 单侧裸臂和空末端安装点 |
| left_arm_with_parallel_link / right_arm_with_parallel_link | 单侧手臂和完整夹爪 |

未指定 `robot_preset` 时，V1 的 `bimanual` 参数仍选择 default_bimanual/default_single。显式 preset 优先。保留 no_prefix、CAN、控制开关、根安装和双臂安装参数；原安装参数的默认值改为读取 preset。

控制接口来自实际启用组件的 topology。公共控制层根据是否安装夹爪输出 `hand`，裸臂不会初始化夹爪接口。V1 双臂的夹爪 PD 保持 kp=5、kd=0.1；单臂现在也显式输出这两个值，与原驱动默认值相同。七个手臂关节的 PD、硬件插件和默认 CAN 选择保持原值。

V1 default_single 的 `control.side: single` 对应 `can_interface`；left/right 对应 left_can_interface/right_can_interface。V1 的 ros2_control 默认 false，V2 保持 true；RViz launch 默认 false。

## 使用

```bash
source /opt/ros/jazzy/setup.bash
source /home/li/ros2_ws/install/setup.bash

ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=true
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 bimanual:=false
ros2 launch openarm_description display_openarm.launch.py arm_type:=v10 robot_preset:=right_arm

xacro assets/robot/openarm_v1.0/urdf/openarm_v10.urdf.xacro \
  robot_preset:=default_bimanual ros2_control:=false
xacro assets/end_effector/parallel_link/urdf/parallel_link_standalone.urdf.xacro
```

独立夹爪入口现在输出含 link7 的完整组件，并保留 tcp_xyz/tcp_rpy 和 mesh_root 参数。

## 验证

基线涵盖 16 种 V1 开关组合、20 种 V2 preset/折叠/参考点组合。268 组姿态对比验证了实体几何、世界 COM/惯量、关节转轴线、TCP、质量矩阵和重力广义力的等效性。仓库 mesh 文件只移动归属，字节内容保持一致。

回归测试另覆盖 origin/limit 覆盖顺序、夹爪部分镜像、带旋转安装的折叠、参考点 emit、控制命名、空夹爪配置、独立完整夹爪及旧入口参数。验证对象为生成模型和仿真数据，没有运行实机电机。
