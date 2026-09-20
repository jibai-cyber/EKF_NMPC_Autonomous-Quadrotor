# `drone_gnc` 数学文档索引

本目录记录 `src/drone_gnc` 中实际使用的数学模型、推导过程和代码追溯关系。文档以用户
提供的课程需求文档 `project.pdf` 为基础，并明确标出代码为解决坐标符号、量纲或工程实现
问题而采用的补充假设。

## 文档结构

1. [坐标系、状态和四元数](01_frames_and_state.md)
2. [四旋翼动力学与执行器映射](02_quadrotor_dynamics.md)
3. [传感器模型与 EKF](03_sensor_and_ekf.md)
4. [任务与三维八字轨迹](04_trajectory_generation.md)
5. [NMPC 与安全回退控制](05_nmpc_and_fallback.md)
6. [参数与实现追溯](06_parameter_traceability.md)
7. [Rubric 验收报告](07_acceptance_report.md)

## 统一符号

| 符号 | 含义 |
|---|---|
| $\mathcal F_W$ | NED 世界坐标系 |
| $\mathcal F_B$ | FRD 机体坐标系 |
| $\mathbf p^W$ | 世界系位置，单位 m |
| $\mathbf v^W$ | 世界系速度，单位 m/s |
| $\mathbf q^{WB}$ | Hamilton 四元数 `[w,x,y,z]`，将机体系向量旋转到世界系 |
| $\boldsymbol\omega^B$ | FRD 机体系角速度 `[p,q,r]`，单位 rad/s |
| $R_{WB}(\mathbf q)$ | 机体系到世界系的旋转矩阵 |
| $\mathbf T=[T_0,T_1,T_2,T_3]^T$ | 四个正值旋翼推力，单位 N |
| $\hat{\mathbf x}$ | EKF 状态估计 |
| $P$ | EKF 状态误差协方差 |

## 状态排列

动力学、EKF、控制器和 ROS `State13` 消息统一使用：

$$
\mathbf x=
\begin{bmatrix}
\mathbf p^W & \mathbf v^W & \mathbf q^{WB} & \boldsymbol\omega^B
\end{bmatrix}^T\in\mathbb R^{13}.
$$

当前验收模型不考虑 IMU bias，因此不存在额外的 bias 状态。

**代码对应**

| 文件 | 符号/函数 | 当前行号 | 作用 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py) | `QuadrotorEkf.state_size` | - | 定义 13 维 EKF 状态 |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py) | `public_state` | - | 归一化后输出同一 13 维状态 |
| [`State13.msg`](../src/drone_interfaces/msg/State13.msg) | `State13` | 1-9 | ROS 侧 13 维状态布局 |

## 课程公式的两项必要解释

### 1. 推力方向

课程 PDF 将合推力写成正的机体 z 分量，但同时采用 NED/FRD，且每个 $T_i$ 是正推力。
在 FRD 中，机体 z 轴向下，而旋翼升力向上，因此本项目采用：

$$
\mathbf F_T^B=
\begin{bmatrix}0&0&-\sum_iT_i\end{bmatrix}^T.
$$

否则正推力会与 NED 重力同向，无法产生悬停。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L81-L98) | `rotor_wrench_frd` | 81-98 | 正旋翼推力映射为 FRD 负 z 合力 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L101) | `_continuous_symbolic` | 74-101 | NMPC 使用相同推力符号 |

### 2. 平移动力学中的质量

课程 PDF 的连续方程中未显式写出 $1/m$。力除以质量才得到加速度，因此本项目使用：

$$
\dot{\mathbf v}^W=\mathbf g^W+\frac{1}{m}R_{WB}\mathbf F_T^B.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L164-L196) | `continuous_dynamics` | 164-196 | NumPy 动力学中的质量项 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L107) | `_continuous_symbolic` | 74-107 | CasADi 动力学中的质量项 |

这两项差异在 [四旋翼动力学文档](02_quadrotor_dynamics.md) 中完整展开。

## 代码追溯规则

每个公式小节后都包含“代码对应”表，至少给出：

- 仓库相对路径；
- 函数或类名；
- 当前版本行号；
- 公式在代码中的具体作用。

函数名是长期追溯主键，行号用于当前提交快速定位。若后续重构导致行号变化，应优先按函数名
搜索，并同步更新本目录文档。

## 实现状态说明

本文档描述的是当前代码，而不是理想化的最终算法。已知的重要假设包括：

- EKF 使用 4 维四元数直接进入 13 维协方差，而不是 3 维姿态误差状态；
- 状态转移 Jacobian 使用中心差分数值计算；
- 悬停时仅凭 IMU 和 GNSS 位置不能观测绝对 yaw；
- 当前传感器与 EKF 均忽略 IMU bias，只考虑题目指定的白噪声；
- 任务阶段按固定时间切换，尚未实现稳定驻留条件；
- NMPC 后处理中的推力 slew-rate 投影没有进入预测模型约束。
