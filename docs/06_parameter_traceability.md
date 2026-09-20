# 参数与实现追溯

本页区分三类数值：

- **课程给定**：直接来自 `project.pdf`；
- **公式导出**：由课程参数计算得到；
- **工程配置**：课程没有给出，由当前实现选取，必须通过仿真或实验整定。

这样可以避免把控制增益、solver 选项或可视化设置误认为飞行器物理参数。

## 1. 飞行器物理参数

| 参数 | 数值 | 来源 | Python 动力学 | ROS 配置 | Gazebo 物理模型 |
|---|---:|---|---|---|---|
| 质量 $m$ | 1.20 kg | 课程给定 | [`VehicleParameters.mass_kg`](../src/drone_gnc/drone_gnc/dynamics.py#L19) | [`mass_kg`](../src/drone_gnc/config/project.yaml#L41) | [`mass`](../src/drone_description/models/course_quadrotor/model.sdf#L10) |
| x 力臂 $d_x$ | 0.225 m | 课程给定 | [`dx_m`](../src/drone_gnc/drone_gnc/dynamics.py#L20) | [`dx_m`](../src/drone_gnc/config/project.yaml#L42) | [`dx`](../src/drone_description/models/course_quadrotor/model.sdf#L98) |
| y 力臂 $d_y$ | 0.225 m | 课程给定 | [`dy_m`](../src/drone_gnc/drone_gnc/dynamics.py#L21) | [`dy_m`](../src/drone_gnc/config/project.yaml#L43) | [`dy`](../src/drone_description/models/course_quadrotor/model.sdf#L99) |
| $J_{xx}$ | 0.0125 kg·m² | 课程给定 | [`jxx_kgm2`](../src/drone_gnc/drone_gnc/dynamics.py#L22) | [`jxx_kgm2`](../src/drone_gnc/config/project.yaml#L44) | [`ixx`](../src/drone_description/models/course_quadrotor/model.sdf#L12) |
| $J_{yy}$ | 0.0125 kg·m² | 课程给定 | [`jyy_kgm2`](../src/drone_gnc/drone_gnc/dynamics.py#L23) | [`jyy_kgm2`](../src/drone_gnc/config/project.yaml#L45) | [`iyy`](../src/drone_description/models/course_quadrotor/model.sdf#L13) |
| $J_{zz}$ | 0.0220 kg·m² | 课程给定 | [`jzz_kgm2`](../src/drone_gnc/drone_gnc/dynamics.py#L24) | [`jzz_kgm2`](../src/drone_gnc/config/project.yaml#L46) | [`izz`](../src/drone_description/models/course_quadrotor/model.sdf#L14) |
| 反扭矩比 $c_\tau$ | 0.015 m | 课程给定 | [`moment_ratio_m`](../src/drone_gnc/drone_gnc/dynamics.py#L25) | [`moment_ratio_m`](../src/drone_gnc/config/project.yaml#L47) | [`moment_ratio`](../src/drone_description/models/course_quadrotor/model.sdf#L100) |
| 重力 $g$ | 9.81 m/s² | 工程常数 | [`gravity_mps2`](../src/drone_gnc/drone_gnc/dynamics.py#L26) | 使用默认值 | Gazebo world gravity |

Python 动力学、NMPC 参数与 Gazebo 刚体使用同一组质量、惯量、力臂和反扭矩比。Gazebo
插件 [`RotorWrenchSystem`](../src/drone_gazebo/src/rotor_wrench_system.cpp) 只实现本项目
的力/力矩映射，不调用 Gazebo `MulticopterMotorModel`。

## 2. 执行器与状态约束

| 参数 | 数值 | 来源 | 配置/代码位置 |
|---|---:|---|---|
| 单旋翼最小推力 | 0.2 N | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L48)、[`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L101) |
| 单旋翼最大推力 | 5.5 N | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L49)、[`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L102) |
| 总推力范围 | 0.8–22 N | 公式导出 | $4T_{\min}$ 至 $4T_{\max}$ |
| 最大 roll/pitch | ±35° | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L50) |
| 最大 body rate | ±180/±180/±90 °/s | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L51) |
| 最大 roll/pitch torque | ±0.05 N·m | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L52) |
| 最大 yaw torque | ±0.015 N·m | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L53) |
| 最大推力变化率 | 40 N/s/rotor | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L54) |
| 命令超时 | 0.2 s | 工程配置 | [`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L103) |

悬停推力由课程质量导出：

$$
T_{i,h}=\frac{mg}{4}
=\frac{1.20\times9.81}{4}
=2.943\ \mathrm N.
$$

实现位置是 [`VehicleParameters.hover_thrust_per_rotor_n`](../src/drone_gnc/drone_gnc/dynamics.py#L34-L36)。

## 3. 传感器与估计器

| 参数 | 数值 | 来源 | Gazebo/仿真位置 | EKF 位置 |
|---|---:|---|---|---|
| IMU 频率 | 100 Hz | 课程给定 | [`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L79-L83) | IMU callback 驱动预测 |
| GNSS 频率 | 20 Hz | 课程给定 | [`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L85-L89) | position callback 驱动更新 |
| 加速度噪声 $\sigma_a$ | 0.08 m/s² | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L3) | [`project.yaml`](../src/drone_gnc/config/project.yaml#L14) |
| gyro 噪声 $\sigma_g$ | 0.015 rad/s | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L4) | [`project.yaml`](../src/drone_gnc/config/project.yaml#L15) |
| GNSS 位置噪声 $\sigma_p$ | 0.02 m | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L5) | [`project.yaml`](../src/drone_gnc/config/project.yaml#L16) |
| EKF 对角过程噪声率 | $[10^{-8}I_3,\sigma_a^2I_3,\sigma_g^2I_4/4,\sigma_g^2I_3]$ | 原始经验模型 | 不适用 | [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py) |
| 随机种子 | 6224 | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L7) | 不适用 |
| GNSS 参考点 | 1.3521°, 103.8198°, 0 m | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L8-L10) | 不适用 |

当前验收模型明确忽略加速度计和陀螺仪 bias：传感器仿真不生成 bias，EKF 也不包含
bias 状态。因此估计器状态与 `State13` 完全一致，均为 13 维。

## 4. 八字与任务参数

| 参数 | 数值 | 来源 | 配置 | 实现 |
|---|---:|---|---|---|
| $A_x$ | 3.0 m | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L31) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L10) |
| $A_y$ | 2.0 m | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L32) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L11) |
| $A_z$ | 0.5 m | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L33) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L12) |
| $z_0$ | -2.5 m NED | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L34) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L13) |
| $\omega$ | 0.25 rad/s | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L35) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L14) |
| 轨迹持续时间 | 60 s | 课程给定 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L36) | [`TrajectoryParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L15) |
| 初始 z | -0.035 m NED | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L22) | [`MissionParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L20) |
| 起飞时长 | 5 s | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L23) | [`MissionParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L21) |
| yaw 对准时长 | 5 s | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L24) | [`MissionParameters`](../src/drone_gnc/drone_gnc/trajectory.py#L22) |
| 八字入口相位渐入 | 3 s | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L25) | `smooth_lemniscate_reference` |

八字起点高度不是 $z_0$，而是：

$$
z_r(0)=z_0+A_z=-2.0\ \mathrm m.
$$

对应实现是 [`lemniscate_reference`](../src/drone_gnc/drone_gnc/trajectory.py#L120-L125)。

## 5. NMPC 工程参数

| 参数 | 数值 | 来源 | 位置 |
|---|---:|---|---|
| horizon steps $N$ | 12 | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L55) |
| prediction step | 0.05 s | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L56) |
| control rate | 20 Hz | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L57) |
| IPOPT max iterations | 80 | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L58) |
| IPOPT max CPU time | 0.08 s | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L59) |
| $Q_p$ diagonal | [20, 20, 25] | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L60) |
| $Q_v$ diagonal | [4, 4, 5] | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L61) |
| attitude weight | 12 | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L62) |
| $Q_\omega$ diagonal | [2, 2, 0.5] | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L63) |
| $R$ diagonal | [0.1, 0.1, 0.1, 0.1] | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L64) |
| terminal multiplier | 4 | 工程配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L65) |

课程允许使用 CasADi/IPOPT 或 acados，但没有给出上述 horizon、代价权重或 solver budget。
因此这些数值必须在报告中称为“控制器设计参数”，不能称为“飞行器物理参数”。

## 6. Gazebo 轨迹标记参数

| 参数 | 数值 | 来源 | 位置 |
|---|---:|---|---|
| 实际轨迹采样率 | 10 Hz | 可视化配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L83) |
| marker 发布率 | 5 Hz | 可视化配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L84) |
| 最大点数 | 1500 | 可视化配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L85) |
| 最小点间距 | 0.01 m | 可视化配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L86) |
| 实际/参考线宽 | 0.05/0.025 m | 可视化配置 | [`project.yaml`](../src/drone_gnc/config/project.yaml#L87-L88) |

这些参数仅影响显示，不进入动力学、EKF 或 NMPC。

## 7. 修改参数时的同步规则

修改参数前先判断其类别，并保持以下单一事实链：

1. 课程物理参数改变时，同步更新 `VehicleParameters`、`project.yaml` 和 `model.sdf`；
2. 传感器噪声改变时，同步更新 sensor simulator 和 EKF 的参数；
3. 轨迹参数改变时，同步更新 `trajectory_node` 和 `nmpc_node` 的配置段；
4. 控制权重、horizon 和回退增益属于工程设计，应记录调参依据与验证结果；
5. 修改公式或符号约定后，同步更新本 `docs` 目录和对应单元测试。

可用以下命令检查 YAML 中复制的关键参数：

```bash
rg -n "mass_kg|jxx_kgm2|amplitude_x_m|accel_std_mps2" src
```
