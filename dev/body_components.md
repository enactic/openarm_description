# Body 组件与胸罩惯量

底座独立于机械臂，配置和 mesh 位于：

```text
assets/body/
├── v1.0/
│   ├── config/{struct,link,inertials}/
│   └── meshes/{visual,collision}/
└── v2.0/
    ├── config/{struct,link,inertials}/
    └── meshes/{visual,collision}/
```

V1 使用完整胸罩，V2 使用顶部开口胸罩。相机安装座仍属于 `assets/accessory/zed_mount/`，其 0.13704 kg 独立计入安装座 link。

Preset 通过独立类别引用 body：

```yaml
body:
  type: body
  enabled: true
  product: v2.0
  config: null
  inertials: nominal
  link_name: openarm_body_link0
  connection:
    parent: world
    origin: {x: 0, y: 0, z: 0, roll: 0, pitch: 0, yaw: 0}
```

`type: robot` 现在只负责 arm 配置。机械臂和相机安装座继续通过 body 的 reference_points 安装，生成的 link 名称和装配位姿保持一致。各版本 Xacro 入口仍分开；preset 暂留在 `assets/robot/openarm_v*/config/robot_presets/`，后续迁到独立装配配置目录。

## 质量边界与体积折算

称量的金属底座为 **3.55 kg**，不含两侧 arm、连接 link0 的螺丝和胸罩。完整胸罩另约 **190 g**。金属底座的 COM 与惯量沿用之前按 CAD 分配、整体归一化到 3.55 kg 的结果。

完整胸罩取 `openarm_hardware` 1.0.1 的 `OpenArm01_follower.step` 中 `Chest_Cover v3`：体积 **174718.587667 mm³**。顶部开口胸罩取本仓库 V2 body DAE 中的独立几何，按相同的有效密度折算：

```text
有效密度 = 0.190 / (174718.587667 × 10⁻⁹) = 1087.463 kg/m³
开口胸罩体积 ≈ 169948.036206 mm³
开口 / 完整体积 = 0.972695799
开口胸罩质量 ≈ 190 × 0.972695799 = 184.812202 g
减重 ≈ 5.187798 g
body 总质量 ≈ 3.55 + 0.184812202 = 3.734812202 kg
```

新胸罩网格包含局部未闭合边界，面朝向一致。计算以胸罩包围盒中心为局部积分参考点，使用有向四面体求体积、COM 和中心惯量；三个方向的表面积分体积差异为 **0.0421%**。这些参数是基于原称重值和新网格的估计。

## 合成惯量

各部件先通过平行轴定理移到共同质心，再求和。对总空间惯量做 Y=0 对称平均，最后将 Ixy、Ixz、Iyz 置零。URDF inertial 的 RPY 为零，惯量在 COM 处、沿 body 坐标轴表达。

| 参数 | V1 完整胸罩 | V2 顶部开口胸罩 |
| --- | ---: | ---: |
| 总质量，kg | 3.74 | 3.734812201783 |
| COM-X，m | −0.016206894902 | −0.016271695620 |
| COM-Y，m | 0 | 0 |
| COM-Z，m | 0.244091987626 | 0.243365327727 |
| Ixx，kg·m² | 0.248946829107 | 0.247519671767 |
| Iyy，kg·m² | 0.253033907704 | 0.251602232728 |
| Izz，kg·m² | 0.0140271775534 | 0.014018358766 |
| Ixy / Ixz / Iyz | 0 | 0 |

## 复算与来源

- [输入、称量边界与 CAD 来源](body_inertial_sources.json)
- [积分结果与网格质量检查](body_inertial_result.json)
- [计算脚本](compute_body_inertials.py)

使用安装了 NumPy、trimesh 和 pycollada 的开发环境，在 description 根目录运行：

```bash
python dev/compute_body_inertials.py --output /tmp/openarm_body_inertials.json
```

脚本按 geometry ID 选择胸罩，检查源 mesh 哈希、面朝向、体积积分一致性和惯量三角不等式。V1 的完整胸罩参数、两版几何与碰撞 mesh 保持原数据。
