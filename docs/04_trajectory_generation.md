# 任务阶段与三维八字轨迹

## 1. 八字轨迹的位置方程

设八字轨迹局部时间为 $t_f$。课程轨迹在 NED 坐标系中定义为：

$$
x_r(t_f)=A_x\sin(\omega t_f),
$$

$$
y_r(t_f)=A_y\sin(2\omega t_f),
$$

$$
z_r(t_f)=z_0+A_z\cos(\omega t_f).
$$

当前参数为 $A_x=3$ m、$A_y=2$ m、$A_z=0.5$ m、$z_0=-2.5$ m、
$\omega=0.25$ rad/s。NED 的 z 轴向下，因此更负的 z 表示更高的飞行高度。

**代码对应**

| 文件 | 函数/配置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L111-L126) | `lemniscate_reference` | 111-126 | 直接计算三维参考位置 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L23-L34) | `trajectory_node.ros__parameters` | 23-34 | 运行时轨迹参数 |

## 2. 解析速度与加速度

对位置方程逐项求导：

$$
\dot x_r=A_x\omega\cos(\omega t_f),\qquad
\dot y_r=2A_y\omega\cos(2\omega t_f),\qquad
\dot z_r=-A_z\omega\sin(\omega t_f).
$$

再次求导得到：

$$
\ddot x_r=-A_x\omega^2\sin(\omega t_f),
$$

$$
\ddot y_r=-4A_y\omega^2\sin(2\omega t_f),
$$

$$
\ddot z_r=-A_z\omega^2\cos(\omega t_f).
$$

采用解析导数而不是数值差分，可以避免参考速度和加速度中的离散噪声。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L127-L140) | `lemniscate_reference` | 127-140 | 解析速度与加速度 |
| [`TrajectoryPoint.msg`](../src/drone_interfaces/msg/TrajectoryPoint.msg) | `TrajectoryPoint` | 1-7 | ROS 参考位置、速度、加速度字段 |

## 3. 航向角及航向角速度

飞行器航向沿水平速度方向：

$$
\psi_r=\operatorname{atan2}(\dot y_r,\dot x_r).
$$

对 `atan2` 求导：

$$
\dot\psi_r=
\frac{\dot x_r\ddot y_r-\dot y_r\ddot x_r}
{\dot x_r^2+\dot y_r^2}.
$$

若水平速度平方小于 $10^{-10}$，代码将 $\dot\psi_r$ 置零，避免分母接近零。
仅含 yaw 的参考姿态和参考角速度为：

$$
q_r^{WB}=
\begin{bmatrix}
\cos(\psi_r/2)&0&0&\sin(\psi_r/2)
\end{bmatrix}^T,
\qquad
\boldsymbol\omega_r^B=
\begin{bmatrix}0&0&\dot\psi_r\end{bmatrix}^T.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L142-L151) | `lemniscate_reference` | 142-151 | yaw 与 yaw-rate 解析计算 |
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L26-L45) | `_state_reference` | 26-45 | yaw 转四元数并组成 13 维参考状态 |

## 4. 周期和八字起点

x 与 z 的周期为：

$$
T_{xz}=\frac{2\pi}{\omega},
$$

y 的周期为：

$$
T_y=\frac{\pi}{\omega}.
$$

完整三维轨迹的公共周期因此是：

$$
T=\frac{2\pi}{\omega}=25.1327\ \mathrm s.
$$

在 $t_f=0$：

$$
\mathbf p_r(0)=
\begin{bmatrix}0&0&z_0+A_z\end{bmatrix}^T
=\begin{bmatrix}0&0&-2.0\end{bmatrix}^T\ \mathrm m,
$$

$$
\mathbf v_r(0)=
\begin{bmatrix}A_x\omega&2A_y\omega&0\end{bmatrix}^T
=\begin{bmatrix}0.75&1.00&0\end{bmatrix}^T\ \mathrm{m/s},
$$

$$
\mathbf a_r(0)=
\begin{bmatrix}0&0&-A_z\omega^2\end{bmatrix}^T
=\begin{bmatrix}0&0&-0.03125\end{bmatrix}^T\ \mathrm{m/s^2}.
$$

初始航向为：

$$
\psi_r(0)=\operatorname{atan2}(1.0,0.75)=53.13^\circ.
$$

因此飞行器先爬升至 $(0,0,-2.0)$ m，再对准 $53.13^\circ$，随后从八字中心开始飞行。
一个周期后回到相同的位置、速度、加速度和航向。

**代码对应**

| 文件 | 函数/测试 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L111-L151) | `lemniscate_reference` | 111-151 | 八字起点与周期轨迹 |
| [`test_trajectory.py`](../src/drone_gnc/test/test_trajectory.py) | trajectory tests | 全文件 | 验证起点、周期及任务阶段 |

## 5. 五次多项式边界段

起飞和 yaw 对准使用五次多项式，而八字本体不使用多项式拟合。令：

$$
s(t)=a_0+a_1t+a_2t^2+a_3t^3+a_4t^4+a_5t^5,
\qquad 0\le t\le T_s.
$$

边界条件为：

$$
s(0)=s_0,\quad \dot s(0)=v_0,\quad \ddot s(0)=\alpha_0,
$$

$$
s(T_s)=s_1,\quad \dot s(T_s)=v_1,\quad \ddot s(T_s)=\alpha_1.
$$

前三个系数直接得到：

$$
a_0=s_0,\qquad a_1=v_0,\qquad a_2=\frac{\alpha_0}{2}.
$$

解剩余三元线性方程可得：

$$
a_3=
\frac{20(s_1-s_0)-(8v_1+12v_0)T_s-(3\alpha_0-\alpha_1)T_s^2}
{2T_s^3},
$$

$$
a_4=
\frac{30(s_0-s_1)+(14v_1+16v_0)T_s+(3\alpha_0-2\alpha_1)T_s^2}
{2T_s^4},
$$

$$
a_5=
\frac{12(s_1-s_0)-(6v_1+6v_0)T_s-(\alpha_0-\alpha_1)T_s^2}
{2T_s^5}.
$$

代码同时解析计算 $s(t)$、$\dot s(t)$ 和 $\ddot s(t)$；对向量位置逐轴使用同一公式。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L48-L108) | `_quintic_segment` | 48-108 | 边界系数和二阶解析导数 |

## 6. 三阶段任务

总任务时间 $t$ 依次经过：

1. `takeoff`，$0\le t<T_{\mathrm{to}}$：从近地面
   $\mathbf p_0=[0,0,-0.035]^T$ 以零速度、零加速度起飞，到八字起点
   $\mathbf p_f=[0,0,-2.0]^T$，同样以零速度、零加速度结束；
2. `yaw_align`，$T_{\mathrm{to}}\le t<T_{\mathrm{to}}+T_\psi$：位置保持在
   $\mathbf p_f$，用五次多项式从 $\psi=0$ 转到 $\psi_r(0)$；
3. `figure8`：只有估计状态连续满足入口条件 0.5 s 后才释放，并对解析八字的相位速度
   使用 3 s 半余弦渐入。

入口门控要求位置误差不超过 0.12 m、速度范数不超过 0.20 m/s、yaw 误差不超过 5°、
角速度范数不超过 10°/s。未满足时任务时钟停在 `yaw_align` 末端，因而强扰动不会触发
提前切换。

为保持空间八字公式不变，渐入只修改局部路径时间 $s(t)$。当 $0\le t<T_r$：

$$
s(t)=\frac12\left[t-\frac{T_r}{\pi}\sin\left(\frac{\pi t}{T_r}\right)\right],
$$

$$
\dot s(t)=\frac12\left[1-\cos\left(\frac{\pi t}{T_r}\right)\right],\qquad
\ddot s(t)=\frac{\pi}{2T_r}\sin\left(\frac{\pi t}{T_r}\right).
$$

当 $t\ge T_r$ 时，$s=t-T_r/2$、$\dot s=1$、$\ddot s=0$。参考导数通过链式法则
$\dot{\mathbf p}=\mathbf p'(s)\dot s$、
$\ddot{\mathbf p}=\mathbf p''(s)\dot s^2+\mathbf p'(s)\ddot s$ 得到。因此入口的位置、
速度和加速度连续，且没有拟合新的 Cartesian 过渡曲线。

当前 $T_{\mathrm{to}}=5$ s、$T_\psi=5$ s、$T_r=3$ s。最终仿真在 10.51 s 通过状态
门控；入口 5 s 三维 RMSE 为 0.0490 m，最大三维误差为 0.0708 m。

**代码对应**

| 文件 | 函数/配置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L154-L207) | `mission_reference` | 154-207 | 起飞、yaw 对准和八字三阶段 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L25-L33) | mission/trajectory parameters | 25-33 | 阶段时长和轨迹参数 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L110-L171) | `control_callback` | 110-171 | 根据阶段选择起飞、yaw 或 NMPC 控制 |

## 7. ROS 参考发布

`TrajectoryNode` 以 20 Hz 计算任务时间并发布 `/drone/reference`。消息包含位置、速度、
加速度、姿态四元数、机体系角速度和任务开始后的时间。NMPC 节点以消息时间为当前参考时间，
再向前采样 $N+1$ 个预测节点。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`trajectory_node.py`](../src/drone_gnc/drone_gnc/trajectory_node.py#L9-L33) | `TrajectoryNode.__init__` | 9-33 | 参数、publisher 和 20 Hz timer |
| [`trajectory_node.py`](../src/drone_gnc/drone_gnc/trajectory_node.py#L35-L48) | `publish_reference` | 35-48 | 构造并发布 `TrajectoryPoint` |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L107-L120) | `reference_callback`、`control_callback` | 107-120 | 构造 NMPC 参考时域 |
