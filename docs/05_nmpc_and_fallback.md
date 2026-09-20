# NMPC 与安全回退控制

## 1. 离散最优控制问题

NMPC 使用 [四旋翼动力学](02_quadrotor_dynamics.md) 中独立实现的 13 维模型，不调用
PX4 或其他开源飞控的动力学。令：

$$
\mathbf x_k\in\mathbb R^{13},\qquad
\mathbf u_k=
\begin{bmatrix}T_{0,k}&T_{1,k}&T_{2,k}&T_{3,k}\end{bmatrix}^T
\in\mathbb R^4.
$$

RK4 离散模型记为：

$$
\mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k).
$$

在每个控制周期求解 multiple-shooting 非线性规划：

$$
\min_{\mathbf X,\mathbf U}
\sum_{k=0}^{N-1}\ell(\mathbf x_k,\mathbf u_k,\mathbf x_{r,k})
+\ell_f(\mathbf x_N,\mathbf x_{r,N}),
$$

满足：

$$
\mathbf x_0=\hat{\mathbf x},\qquad
\mathbf x_{k+1}-f_d(\mathbf x_k,\mathbf u_k)=0.
$$

当前 $N=12$、$\Delta t=0.05$ s，预测时域为：

$$
T_H=N\Delta t=0.6\ \mathrm s.
$$

**代码对应**

| 文件 | 类/函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L38-L52) | `NmpcController` | 38-52 | 状态/输入维度及 solver 初始化 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L152-L176) | `_build_solver` | 152-176 | 决策变量、初值约束和 RK4 shooting 约束 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L52-L56) | horizon/solver parameters | 52-56 | 实际预测时域和 IPOPT 限制 |

## 2. 阶段代价

定义误差：

$$
\mathbf e_p=\mathbf p-\mathbf p_r,\qquad
\mathbf e_v=\mathbf v-\mathbf v_r,\qquad
\mathbf e_\omega=\boldsymbol\omega-\boldsymbol\omega_r.
$$

四元数 $q$ 与 $-q$ 表示相同姿态，因此姿态误差使用：

$$
e_q=1-(q^Tq_r)^2.
$$

该代价对四元数符号不敏感，且单位四元数完全对齐时为零。令悬停输入为：

$$
\mathbf u_h=\frac{mg}{4}\mathbf 1_4.
$$

阶段代价为：

$$
\ell=
\mathbf e_p^TQ_p\mathbf e_p+
\mathbf e_v^TQ_v\mathbf e_v+
w_qe_q+
\mathbf e_\omega^TQ_\omega\mathbf e_\omega+
(\mathbf u-\mathbf u_h)^TR(\mathbf u-\mathbf u_h).
$$

末端代价复用同一函数，并乘以 $\gamma_f=4$：

$$
\ell_f=\gamma_f\,
\ell(\mathbf x_N,\mathbf u_h,\mathbf x_{r,N}).
$$

当前权重为：

$$
Q_p=\operatorname{diag}(20,20,25),\quad
Q_v=\operatorname{diag}(4,4,5),
$$

$$
w_q=12,\quad
Q_\omega=\operatorname{diag}(2,2,0.5),\quad
R=0.1I_4.
$$

**代码对应**

| 文件 | 类/函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L15-L22) | `NmpcWeights` | 15-22 | 默认权重 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L135-L150) | `_stage_cost` | 135-150 | 五项阶段代价 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L200-L205) | `_build_solver` terminal block | 200-205 | 末端代价 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L57-L62) | runtime weights | 57-62 | 当前运行时权重 |

## 3. 约束

### 3.1 单旋翼推力

每个预测步满足：

$$
0.2\le T_{i,k}\le5.5\ \mathrm N.
$$

因此总推力的可用区间是：

$$
0.8\le T_{\Sigma,k}\le22.0\ \mathrm N.
$$

### 3.2 倾角

由预测四元数求 roll/pitch，并施加：

$$
|\phi_k|\le35^\circ,\qquad |\theta_k|\le35^\circ.
$$

### 3.3 机体系角速度

$$
|p_k|\le180^\circ/\mathrm s,\qquad
|q_k|\le180^\circ/\mathrm s,\qquad
|r_k|\le90^\circ/\mathrm s.
$$

代码求解时将这些角速度限制转换成 rad/s。

### 3.4 机体系力矩

工程安全限制为：

$$
|\tau_x|\le0.05\ \mathrm{N\,m},\qquad
|\tau_y|\le0.05\ \mathrm{N\,m},\qquad
|\tau_z|\le0.015\ \mathrm{N\,m}.
$$

这些力矩上限是当前控制器配置，不是课程 PDF 给出的飞行器物理参数。动力学本身仍由四旋翼
推力映射产生力矩。

### 3.5 推力变化率

上一周期实际发布的推力 $\mathbf u_{-1}$ 作为 NLP 参数。预测域内直接施加

$$
-\dot T_{\max}\Delta t\le
\mathbf u_0-\mathbf u_{-1}\le
\dot T_{\max}\Delta t,
$$

$$
-\dot T_{\max}\Delta t\le
\mathbf u_k-\mathbf u_{k-1}\le
\dot T_{\max}\Delta t.
$$

当前 $\dot T_{\max}=40$ N/s、$\Delta t=0.05$ s，即每个预测步每个旋翼最多变化 2 N。
求解后的 `condition_rotor_thrusts` 继续保留，作为求解失败、回退控制和数值误差的安全层。

**代码对应**

| 文件 | 函数/位置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L127-L133) | `_tilt_angles_symbolic` | 127-133 | roll/pitch 符号表达式 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L166-L208) | `_build_solver` constraints | 166-208 | 倾角、动力学和力矩约束 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L227-L237) | `_build_solver` variable bounds | 227-237 | 角速度和旋翼推力边界 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L45-L51) | actuator/safety limits | 45-51 | 运行时约束值 |

## 4. 四元数单位范数处理

符号动力学每次计算旋转矩阵前使用：

$$
\bar q=\frac{q}{\sqrt{q^Tq+10^{-12}}},
$$

RK4 一步完成后再次归一化：

$$
q_{k+1}\leftarrow\frac{q_{k+1}}
{\sqrt{q_{k+1}^Tq_{k+1}+10^{-12}}}.
$$

因此 NMPC 没有额外加入 $\|q_k\|_2=1$ 等式约束，而是把归一化嵌入离散状态转移。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L74-L77) | `_continuous_symbolic` | 74-77 | 连续模型入口归一化 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L117-L125) | `_rk4_symbolic` | 117-125 | 离散积分后归一化 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L241-L249) | `solve` | 241-249 | 当前状态和参考状态归一化 |

## 5. NLP 求解与 warm start

CasADi 构造决策向量：

$$
\mathbf z=
\begin{bmatrix}
\operatorname{vec}(\mathbf X)\\
\operatorname{vec}(\mathbf U)
\end{bmatrix}.
$$

IPOPT 当前限制为最多 80 次迭代、每次求解最多 0.08 s，容差为 $10^{-4}$。第一次求解以
悬停输入滚动 13 维动力学，得到状态初猜；输入初猜全部取 $\mathbf u_h$。若上次求解成功，
下一周期将上次最优状态和控制序列向前移动一个预测步：

$$
\mathbf U_k^{(0)}=[\mathbf u_{1|k-1}^\star,\ldots,
\mathbf u_{N-1|k-1}^\star,\mathbf u_{N-1|k-1}^\star].
$$

状态序列同样前移，首项由当前估计状态覆盖，末项使用最后控制量再传播一步。这是标准的
receding-horizon shifted warm start。

**代码对应**

| 文件 | 函数/位置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L210-L225) | `_build_solver` NLP/IPOPT | 210-225 | NLP 和 solver 选项 |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L251-L265) | `solve` initial guess | 251-265 | 悬停 rollout 和 warm start |
| [`nmpc.py`](../src/drone_gnc/drone_gnc/nmpc.py#L266-L296) | `solve` | 266-296 | 求解、第一控制量和统计结果 |

## 6. 控制器阶段选择

当前控制流程为：

$$
\text{takeoff}\to\text{vertical fallback},
$$

$$
\text{yaw align}\to\text{yaw alignment control},
$$

$$
\text{figure8}\to\text{NMPC}.
$$

八字阶段若 IPOPT 返回失败、抛出异常，或飞行器进入安全恢复状态，则使用几何回退控制。
无论命令来自哪个控制器，发布前都通过同一执行器安全投影。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L110-L181) | `control_callback` | 110-181 | 阶段分配、NMPC 求解、失败回退和安全投影 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L184-L197) | `control_callback` publish block | 184-197 | 推力与求解统计发布 |

## 7. 垂直起飞回退控制

NED 中 z 向下。参考加速度前馈加 PD 反馈为：

$$
a_{z,d}=\ddot z_r+
k_{p,z}(z_r-z)+k_{v,z}(\dot z_r-\dot z),
$$

其中 $k_{p,z}=2.0$、$k_{v,z}=1.6$。水平姿态时：

$$
\ddot z=g-\frac{T_\Sigma}{m}.
$$

所以：

$$
T_i=\frac{m(g-a_{z,d})}{4},\qquad i=0,\ldots,3.
$$

四个旋翼使用完全相同的命令，因此该控制律不会主动产生 roll、pitch 或 yaw 力矩。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L199-L217) | `vertical_fallback` | 199-217 | z 轴 PD、重力补偿和旋翼饱和 |

## 8. 几何回退控制

### 8.1 期望平动加速度

$$
\mathbf a_d=\mathbf a_r+
K_p(\mathbf p_r-\mathbf p)+
K_v(\mathbf v_r-\mathbf v),
$$

其中：

$$
K_p=\operatorname{diag}(1.5,1.5,2.0),\qquad
K_v=\operatorname{diag}(1.4,1.4,1.6).
$$

水平加速度模长限制为 $2.5$ m/s²，垂向加速度限制为 $[-3,3]$ m/s²。

### 8.2 期望姿态

由 NED 平移动力学，期望机体 z 轴在世界系中的方向为：

$$
\mathbf b_{3,d}=
\frac{\mathbf g-\mathbf a_d}
{\|\mathbf g-\mathbf a_d\|_2}.
$$

参考 yaw 提供水平航向：

$$
\mathbf h_d=
\begin{bmatrix}\cos\psi_d&\sin\psi_d&0\end{bmatrix}^T.
$$

构造正交基：

$$
\mathbf b_{2,d}=
\frac{\mathbf b_{3,d}\times\mathbf h_d}
{\|\mathbf b_{3,d}\times\mathbf h_d\|_2},
\qquad
\mathbf b_{1,d}=\mathbf b_{2,d}\times\mathbf b_{3,d},
$$

$$
R_d=
\begin{bmatrix}
\mathbf b_{1,d}&\mathbf b_{2,d}&\mathbf b_{3,d}
\end{bmatrix}.
$$

### 8.3 姿态误差与力矩

姿态误差取：

$$
e_R=\frac12
\left(R_d^TR-R^TR_d\right)^\vee,
$$

其中 $(\cdot)^\vee$ 将反对称矩阵映射为三维向量。回退力矩为：

$$
\boldsymbol\tau=
-K_Re_R-K_\omega\boldsymbol\omega,
$$

$$
K_R=\operatorname{diag}(0.10,0.10,0.04),\qquad
K_\omega=\operatorname{diag}(0.04,0.04,0.02).
$$

### 8.4 合推力

期望加速度所需推力向量模长为 $m\|\mathbf g-\mathbf a_d\|_2$。投影到当前机体 z 轴：

$$
T_\Sigma=
m\,(\mathbf b_{3,d}^TR\mathbf e_3)\,
\|\mathbf g-\mathbf a_d\|_2.
$$

最后把 $(T_\Sigma,\boldsymbol\tau)$ 通过逆混控变成四个旋翼推力。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L232-L245) | `geometric_fallback` | 232-245 | 位置/速度反馈和加速度限制 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L247-L258) | `geometric_fallback` | 247-258 | 期望旋转矩阵 |
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L260-L281) | `geometric_fallback` | 260-281 | 姿态误差、力矩、合推力和混控 |

## 9. Yaw 对准控制

yaw 阶段先用几何控制保持位置和 roll/pitch，再用 gyro-rate feedback 覆盖 yaw 力矩：

$$
\tau_z=J_{zz}
\left(\ddot\psi_r+
k_r(\dot\psi_r-r)\right),
\qquad k_r=5.
$$

该控制不使用不可观的绝对 yaw 反馈，而是执行已知的 yaw-rate 和 yaw-acceleration 轨迹。
因此它可在只有 IMU 与 GNSS 位置的传感器组合下工作，但长期绝对航向仍可能漂移。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L283-L294) | `yaw_alignment_control` | 283-294 | yaw 角加速度前馈和角速度反馈 |
| [`trajectory.py`](../src/drone_gnc/drone_gnc/trajectory.py#L180-L203) | `mission_reference` yaw phase | 180-203 | yaw、yaw-rate 和 yaw-acceleration 参考 |

## 10. 安全恢复迟滞

令：

$$
\eta=\max(|\phi|,|\theta|),\qquad
\rho=\max(|p|,|q|).
$$

进入恢复状态的条件是：

$$
\eta>28^\circ\quad\text{或}\quad\rho>100^\circ/\mathrm s.
$$

退出条件更严格：

$$
\eta<15^\circ\quad\text{且}\quad\rho<60^\circ/\mathrm s.
$$

进入和退出阈值不同形成迟滞，避免 NMPC 与回退控制在边界附近频繁切换。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`nmpc_node.py`](../src/drone_gnc/drone_gnc/nmpc_node.py#L219-L230) | `recovery_required` | 219-230 | 倾角/角速度迟滞状态机 |

## 11. 当前实现边界

- 回退控制增益和安全阈值是工程参数，尚未由 Lyapunov 证明或系统辨识整定；
- NMPC 不显式估计风、气动阻力或电机一阶动态；

这些限制不会改变上述公式与代码的对应关系，但应在性能评价和后续改进中明确说明。
