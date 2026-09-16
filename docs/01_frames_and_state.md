# 坐标系、状态和四元数

## 1. NED 世界系与 FRD 机体系

课程模型采用：

- 世界系 NED：North-East-Down；
- 机体系 FRD：Forward-Right-Down。

Gazebo 使用 ENU 世界系和 FLU 机体系，因此进入 GNC 前必须显式转换。

从 ENU 到 NED 的向量变换为：

$$
\mathbf v_{NED}=C_{NED\leftarrow ENU}\mathbf v_{ENU},\qquad
C_{NED\leftarrow ENU}=
\begin{bmatrix}
0&1&0\\
1&0&0\\
0&0&-1
\end{bmatrix}.
$$

即：

$$
[v_N,v_E,v_D]^T=[v_y^{ENU},v_x^{ENU},-v_z^{ENU}]^T.
$$

**代码对应**

| 文件 | 函数/符号 | 当前行号 | 实现 |
|---|---|---:|---|
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py#L8-L13) | `ENU_TO_NED`、`vector_enu_to_ned` | 8-13 | ENU 世界向量转 NED |
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L139-L180) | `odometry_callback` | 139-180 | 将 Gazebo 位置和速度转换后发布 truth |

从 FLU 到 FRD 的机体向量变换是绕 x 轴旋转 $\pi$：

$$
\mathbf v_{FRD}=C_{FRD\leftarrow FLU}\mathbf v_{FLU},\qquad
C_{FRD\leftarrow FLU}=\operatorname{diag}(1,-1,-1).
$$

该矩阵是自身的逆矩阵。

**代码对应**

| 文件 | 函数/符号 | 当前行号 | 实现 |
|---|---|---:|---|
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py#L9-L18) | `FRD_TO_FLU`、`vector_flu_to_frd` | 9-18 | FLU/FRD 机体向量互换 |
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L67-L115) | `imu_callback` | 67-115 | IMU 加速度与角速度转 FRD |

## 2. 姿态旋转矩阵的坐标变换

设 Gazebo odometry 中的 $R_{ENU\leftarrow FLU}$ 将 FLU 向量旋转到 ENU。所需的
FRD 到 NED 旋转矩阵为：

$$
R_{NED\leftarrow FRD}
=C_{NED\leftarrow ENU}
R_{ENU\leftarrow FLU}
C_{FLU\leftarrow FRD}.
$$

由于 $C_{FLU\leftarrow FRD}=C_{FRD\leftarrow FLU}$，代码可直接复用同一个对角矩阵。
得到旋转矩阵后再转换为 Hamilton 四元数。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py#L21-L68) | `rotation_to_quaternion` | 21-68 | 旋转矩阵转 `[w,x,y,z]` |
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py#L71-L74) | `quaternion_enu_flu_to_ned_frd` | 71-74 | 完整 ENU/FLU 到 NED/FRD 姿态转换 |

## 3. 13 维飞行器状态

控制器状态定义为：

$$
\mathbf x=
\begin{bmatrix}
x&y&z&u&v&w&q_w&q_x&q_y&q_z&p&q&r
\end{bmatrix}^T.
$$

其中：

$$
\mathbf p^W=[x,y,z]^T,\quad
\mathbf v^W=[u,v,w]^T,\quad
\mathbf q^{WB}=[q_w,q_x,q_y,q_z]^T,\quad
\boldsymbol\omega^B=[p,q,r]^T.
$$

四元数满足单位约束：

$$
\|\mathbf q^{WB}\|_2=1.
$$

代码在积分、EKF 更新和 NMPC 输入前都对四元数归一化，并将等价的 $q$ 与 $-q$ 固定到
$q_w\ge 0$ 的半球，以减少符号跳变。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L39-L46) | `normalize_quaternion` | 39-46 | 单位化与固定四元数半球 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L20-L28) | `state_from_message` | 20-28 | `State13` 消息组装为数值向量 |
| [`State13.msg`](../src/drone_interfaces/msg/State13.msg) | `State13` | 1-9 | ROS 消息字段和协方差布局 |

## 4. Hamilton 四元数乘法

对 $q_l=[l_w,\mathbf l_v]$ 和 $q_r=[r_w,\mathbf r_v]$：

$$
q_l\otimes q_r=
\begin{bmatrix}
l_wr_w-\mathbf l_v^T\mathbf r_v\\
l_w\mathbf r_v+r_w\mathbf l_v+\mathbf l_v\times\mathbf r_v
\end{bmatrix}.
$$

展开后对应代码中的四个标量分量。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L49-L60) | `quaternion_multiply` | 49-60 | Hamilton 四元数乘法 |

## 5. 四元数到旋转矩阵

归一化四元数 $q=[q_w,q_x,q_y,q_z]^T$ 对应：

$$
R_{WB}(q)=
\begin{bmatrix}
1-2(q_y^2+q_z^2) & 2(q_xq_y-q_zq_w) & 2(q_xq_z+q_yq_w)\\
2(q_xq_y+q_zq_w) & 1-2(q_x^2+q_z^2) & 2(q_yq_z-q_xq_w)\\
2(q_xq_z-q_yq_w) & 2(q_yq_z+q_xq_w) & 1-2(q_x^2+q_y^2)
\end{bmatrix}.
$$

该矩阵将 FRD 机体系向量映射到 NED 世界系。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L63-L73) | `quaternion_to_rotation` | 63-73 | NumPy 旋转矩阵 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L54-L61) | `_rotation_symbolic` | 54-61 | CasADi 符号旋转矩阵 |

## 6. 四元数运动学

机体系角速度写成纯四元数：

$$
\omega_q=\begin{bmatrix}0&p&q&r\end{bmatrix}^T.
$$

当 $q^{WB}$ 表示 body-to-world 旋转时：

$$
\dot q^{WB}=\frac{1}{2}q^{WB}\otimes\omega_q.
$$

分量形式为：

$$
\dot q=\frac12
\begin{bmatrix}
-q_xp-q_yq-q_zr\\
q_wp+q_yr-q_zq\\
q_wq-q_xr+q_zp\\
q_wr+q_xq-q_yp
\end{bmatrix}.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L76-L78) | `quaternion_derivative` | 76-78 | NumPy 连续运动学 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L63-L72) | `_quaternion_derivative_symbolic` | 63-72 | CasADi 符号运动学 |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L66-L69) | `_process_step` | 66-69 | IMU 角增量离散传播 |

## 7. Roll 与 pitch 提取

NMPC 的姿态约束和安全恢复需要从四元数计算：

$$
\phi=\operatorname{atan2}
\left(2(q_wq_x+q_yq_z),1-2(q_x^2+q_y^2)\right),
$$

$$
\theta=\arcsin\left(2(q_wq_y-q_zq_x)\right).
$$

CasADi 版本用 `atan2(s, sqrt(1-s^2))` 表示 pitch，以便形成符号表达式并避免直接
`asin` 邻域的数值问题。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L215-L220) | `roll_pitch_from_quaternion` | 215-220 | 安全恢复使用的数值角度 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L127-L133) | `_tilt_angles_symbolic` | 127-133 | NMPC 符号姿态约束 |
