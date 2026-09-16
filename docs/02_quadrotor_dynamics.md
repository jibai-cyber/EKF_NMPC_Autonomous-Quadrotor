# 四旋翼动力学与执行器映射

## 1. 物理参数

使用对角惯量矩阵：

$$
J=\operatorname{diag}(J_{xx},J_{yy},J_{zz}).
$$

默认值为：

$$
m=1.20\ \mathrm{kg},\quad
d_x=d_y=0.225\ \mathrm{m},
$$

$$
J=\operatorname{diag}(0.0125,0.0125,0.0220)\ \mathrm{kg\,m^2},\quad
c_\tau=0.015\ \mathrm m.
$$

**代码对应**

| 文件 | 类/属性 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L17-L36) | `VehicleParameters`、`inertia` | 17-36 | 动力学参数和惯量矩阵 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L36-L51) | `nmpc_node.ros__parameters` | 36-51 | 运行时控制参数 |

## 2. 旋翼编号与正推力

四个旋翼命令定义为：

$$
\mathbf T=[T_0,T_1,T_2,T_3]^T,
\qquad T_i\ge 0.
$$

FRD 的 z 轴向下，而旋翼升力向上。因此机体合力不是 PDF 中字面上的正 z，而是：

$$
\mathbf F_T^B=
\begin{bmatrix}
0\\0\\-(T_0+T_1+T_2+T_3)
\end{bmatrix}.
$$

这个负号保证水平姿态下正推力抵消 NED 的正向重力。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L81-L98) | `rotor_wrench_frd` | 81-98 | 四推力到 FRD 合力/力矩 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L101) | `_continuous_symbolic` | 74-101 | NMPC 内同一推力方向 |

## 3. 四旋翼力矩映射

课程给出的 X 构型力矩为：

$$
\boldsymbol\tau^B=
\begin{bmatrix}
\tau_x\\\tau_y\\\tau_z
\end{bmatrix}
=
\begin{bmatrix}
d_y(-T_0-T_1+T_2+T_3)\\
d_x(-T_0+T_1+T_2-T_3)\\
c_\tau(-T_0+T_1-T_2+T_3)
\end{bmatrix}.
$$

写成线性混控矩阵：

$$
\begin{bmatrix}
T_\Sigma\\\tau_x\\\tau_y\\\tau_z
\end{bmatrix}
=
\begin{bmatrix}
1&1&1&1\\
-d_y&-d_y&d_y&d_y\\
-d_x&d_x&d_x&-d_x\\
-c_\tau&c_\tau&-c_\tau&c_\tau
\end{bmatrix}
\begin{bmatrix}T_0\\T_1\\T_2\\T_3\end{bmatrix}.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L81-L98) | `rotor_wrench_frd` | 81-98 | NumPy 力矩映射 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L79-L86) | `_continuous_symbolic` | 79-86 | NMPC 动力学中的力矩 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L109-L115) | `_torque_symbolic` | 109-115 | NMPC 力矩约束表达式 |

## 4. 逆混控推导

定义归一化力矩项：

$$
r_x=\frac{\tau_x}{d_y},\qquad
r_y=\frac{\tau_y}{d_x},\qquad
r_z=\frac{\tau_z}{c_\tau}.
$$

对上一节的四元一次方程组求逆，得到：

$$
T_0=\frac14(T_\Sigma-r_x-r_y-r_z),
$$

$$
T_1=\frac14(T_\Sigma-r_x+r_y+r_z),
$$

$$
T_2=\frac14(T_\Sigma+r_x+r_y-r_z),
$$

$$
T_3=\frac14(T_\Sigma+r_x-r_y+r_z).
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L101-L118) | `thrusts_from_wrench_frd` | 101-118 | 合推力/力矩到四旋翼推力 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L232-L281) | `geometric_fallback` | 232-281 | 回退控制力矩混控 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L283-L294) | `yaw_alignment_control` | 283-294 | yaw 力矩混控 |

## 5. 悬停推力

水平悬停时 $R_{WB}=I$、$\dot{\mathbf v}=0$。由：

$$
\mathbf 0=
\begin{bmatrix}0\\0\\g\end{bmatrix}
+\frac1m
\begin{bmatrix}0\\0\\-T_\Sigma\end{bmatrix}
$$

得到：

$$
T_\Sigma=mg,\qquad T_i=\frac{mg}{4}.
$$

代入 $m=1.2$ kg、$g=9.81$ m/s²：

$$
T_\Sigma=11.772\ \mathrm N,\qquad T_i=2.943\ \mathrm N.
$$

**代码对应**

| 文件 | 属性/函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L34-L36) | `hover_thrust_per_rotor_n` | 34-36 | 单旋翼悬停推力 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L140-L149) | `_stage_cost` | 140-149 | 输入成本相对悬停推力 |

## 6. 连续平移动力学

位置运动学：

$$
\dot{\mathbf p}^W=\mathbf v^W.
$$

NED 重力为：

$$
\mathbf g^W=\begin{bmatrix}0&0&g\end{bmatrix}^T.
$$

牛顿第二定律给出：

$$
m\dot{\mathbf v}^W=m\mathbf g^W+R_{WB}(q)\mathbf F_T^B,
$$

所以：

$$
\dot{\mathbf v}^W=\mathbf g^W+\frac1mR_{WB}(q)\mathbf F_T^B.
$$

课程 PDF 连续方程未显式写 $1/m$；代码根据量纲和牛顿第二定律补上这一项。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L164-L196) | `continuous_dynamics` | 164-196 | 数值连续动力学 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L107) | `_continuous_symbolic` | 74-107 | CasADi 连续动力学 |

## 7. 刚体转动动力学

Euler 刚体方程为：

$$
J\dot{\boldsymbol\omega}^B+
\boldsymbol\omega^B\times(J\boldsymbol\omega^B)
=\boldsymbol\tau^B.
$$

整理得到：

$$
\dot{\boldsymbol\omega}^B=J^{-1}
\left(\boldsymbol\tau^B-
\boldsymbol\omega^B\times(J\boldsymbol\omega^B)\right).
$$

结合位置、速度和四元数运动学，完整 13 维连续模型为：

$$
\dot{\mathbf x}=f(\mathbf x,\mathbf T)=
\begin{bmatrix}
\mathbf v^W\\
\mathbf g^W+\frac1mR_{WB}\mathbf F_T^B\\
\frac12q^{WB}\otimes[0,(\boldsymbol\omega^B)^T]^T\\
J^{-1}(\boldsymbol\tau^B-\boldsymbol\omega^B\times J\boldsymbol\omega^B)
\end{bmatrix}.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L164-L196) | `continuous_dynamics` | 164-196 | NumPy 13 维模型 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L107) | `_continuous_symbolic` | 74-107 | CasADi 13 维模型 |

## 8. RK4 离散化

对采样周期 $h$：

$$
k_1=f(x_k,u_k),
$$

$$
k_2=f(x_k+\tfrac h2k_1,u_k),\quad
k_3=f(x_k+\tfrac h2k_2,u_k),
$$

$$
k_4=f(x_k+hk_3,u_k),
$$

$$
x_{k+1}=x_k+\frac h6(k_1+2k_2+2k_3+k_4).
$$

积分后重新归一化四元数，避免数值积分造成单位范数漂移。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L199-L212) | `rk4_step` | 199-212 | 数值 RK4 和四元数归一化 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L117-L125) | `_rk4_symbolic` | 117-125 | NMPC 符号 RK4 |

## 9. 执行器安全投影

控制器输出在发布前经过四步投影：

1. 单旋翼饱和：$T_i\leftarrow\operatorname{clip}(T_i,T_{min},T_{max})$；
2. 将推力转换为 $(T_\Sigma,\tau)$；
3. 分别限制 roll/pitch/yaw 力矩，再逆混控；
4. 限制相邻控制周期的最大推力变化量。

设投影后的目标为 $\mathbf T_p$、上一周期为 $\mathbf T_{k-1}$：

$$
\Delta\mathbf T=\mathbf T_p-\mathbf T_{k-1},\qquad
\Delta T_{max}=\dot T_{max}\Delta t.
$$

定义：

$$
\alpha=\min\left(1,
\frac{\Delta T_{max}}{\|\Delta\mathbf T\|_\infty}\right),
$$

最终命令为：

$$
\mathbf T_k=\mathbf T_{k-1}+\alpha\Delta\mathbf T.
$$

线段插值保持前后两个端点已有的凸约束，但该 slew-rate 目前不在 NMPC 预测模型内部。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`dynamics.py`](../src/drone_gnc/drone_gnc/dynamics.py#L121-L161) | `condition_rotor_thrusts` | 121-161 | 饱和、力矩限制和 slew-rate |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L173-L188) | `control_callback` | 173-188 | 发布前调用安全投影 |
