# 传感器模型与 13 维扩展卡尔曼滤波器

## 1. 传感器频率与噪声

课程规范给出：

| 传感器 | 频率 | 白噪声标准差 |
|---|---:|---:|
| accelerometer | 100 Hz | $\sigma_a=0.08$ m/s² |
| gyroscope | 100 Hz | $\sigma_g=0.015$ rad/s |
| GNSS position | 20 Hz | $\sigma_p=0.02$ m |

Gazebo SDF 产生理想原始 IMU/NavSat，`SensorSimulatorNode` 再显式加入上述白噪声。
当前验收模型不生成或估计 IMU bias。

**代码对应**

| 文件 | 类/位置 | 实现 |
|---|---|---|
| [`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf) | `imu_raw`、`navsat_raw` | 100/20 Hz 原始传感器 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml) | sensor/EKF 参数 | 三项白噪声标准差 |
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py) | `SensorSimulatorNode` | 坐标转换、白噪声和 topic |

## 2. IMU 测量模型与 bias 假设

加速度计测量的是 specific force，而不是直接的世界系加速度。忽略 bias 后：

$$
\mathbf f_m^B=R_{WB}^T(\mathbf a^W-\mathbf g^W)+\mathbf n_a.
$$

陀螺仪模型为：

$$
\boldsymbol\omega_m^B=\boldsymbol\omega^B+\mathbf n_g.
$$

其中：

$$
\mathbf n_a\sim\mathcal N(0,\sigma_a^2I),\qquad
\mathbf n_g\sim\mathcal N(0,\sigma_g^2I).
$$

也就是在完整测量模型中直接采用

$$
\mathbf b_a\equiv\mathbf0,\qquad
\mathbf b_g\equiv\mathbf0,
$$

且不为它们建立随机游走方程或 EKF 状态。传感器仿真先把 Gazebo FLU 测量转为 FRD，
再加入零均值白噪声。

**代码对应**

| 文件 | 函数 | 实现 |
|---|---|---|
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py) | `imu_callback` | FLU/FRD 转换并加入白噪声 |
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py) | `vector_flu_to_frd` | FLU 到 FRD |

## 3. GNSS 局部 NED 位置

对参考纬度 $\varphi_0$、经度 $\lambda_0$、高度 $h_0$，在小范围内用平面近似：

$$
p_N=R_E(\varphi-\varphi_0),
$$

$$
p_E=R_E\cos\varphi_0(\lambda-\lambda_0),
$$

$$
p_D=-(h-h_0).
$$

角度差使用弧度，$R_E=6\,378\,137$ m。加入位置白噪声后：

$$
\mathbf z_p=\mathbf p^W+\mathbf n_p,\qquad
\mathbf n_p\sim\mathcal N(0,\sigma_p^2I).
$$

## 4. EKF 13 维状态

EKF、控制器和 ROS 消息使用完全相同的状态：

$$
\mathbf x_E=
\begin{bmatrix}
\mathbf p^W&\mathbf v^W&q^{WB}&\boldsymbol\omega^B
\end{bmatrix}^T\in\mathbb R^{13}.
$$

维数为：

$$
3+3+4+3=13.
$$

EKF 内部 `state`、协方差 $P$、发布的 `State13` 和 NMPC 输入均采用这一排列，
不存在额外的 bias 状态或维度裁剪。

**代码对应**

| 文件 | 类/属性 | 实现 |
|---|---|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py) | `QuadrotorEkf.state_size` | 状态维数固定为 13 |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py) | `state`、`covariance` | 13 维状态和 $13\times13$ 协方差 |
| [`ekf_node.py`](../src/drone_gnc/drone_gnc/ekf_node.py) | `imu_callback` | 发布同一 13 维状态 |

## 5. IMU 预测模型

由于不估计 bias，EKF 直接使用测量 specific force 和 gyro：

$$
\hat{\mathbf a}^W=R_{WB}(\hat q)\mathbf f_m^B+\mathbf g^W.
$$

在单个 IMU 周期内假设加速度不变：

$$
\mathbf p_{k+1}=\mathbf p_k+\mathbf v_k\Delta t
+\frac12\mathbf a_k\Delta t^2,
$$

$$
\mathbf v_{k+1}=\mathbf v_k+\mathbf a_k\Delta t.
$$

角增量的一阶四元数近似为：

$$
\delta q_k\approx
\begin{bmatrix}
1\\\frac12\boldsymbol\omega_{m,k}\Delta t
\end{bmatrix},
$$

$$
q_{k+1}=\operatorname{normalize}(q_k\otimes\delta q_k).
$$

公开角速度状态直接使用高频 gyro：

$$
\boldsymbol\omega_{k+1}=\boldsymbol\omega_{m,k}.
$$

## 6. 数值状态转移 Jacobian

设离散过程模型为

$$
\mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k).
$$

EKF 需要

$$
F_k=\left.\frac{\partial f_d}{\partial\mathbf x}\right|_{\hat x_k,u_k}.
$$

代码对 13 个状态分量使用中心差分：

$$
F_k[:,i]\approx
\frac{f_d(\hat x_k+\epsilon e_i,u_k)-
f_d(\hat x_k-\epsilon e_i,u_k)}{2\epsilon},
\qquad i=0,\ldots,12,
$$

其中 $\epsilon=10^{-6}$。

## 7. 过程协方差传播

EKF 预测为：

$$
P_{k+1}^-=F_kP_k^+F_k^T+Q_k.
$$

当前实现先对 IMU 输入计算离散过程 Jacobian

$$
G_k=\frac{\partial f_d}{\partial[\mathbf f_m^T\ \boldsymbol\omega_m^T]^T},
$$

并传播实际离散测量噪声：

$$
Q_{\mathrm{imu},k}=G_k\operatorname{diag}
(\sigma_a^2I_3,\sigma_g^2I_3)G_k^T.
$$

因此位置—速度、姿态—角速度之间由同一 IMU 样本造成的相关项会被保留。对未建模加速度
再使用连续白噪声的定加速度精确离散。令
$S_a=\operatorname{diag}(s_N^2,s_E^2,s_D^2)$，则

$$
Q_{pv}=\begin{bmatrix}
S_a\Delta t^3/3&S_a\Delta t^2/2\\
S_a\Delta t^2/2&S_a\Delta t
\end{bmatrix}.
$$

当前 $[s_N,s_E,s_D]=[0.50,0.08,0.08]$ m/s²/$\sqrt{\mathrm{Hz}}$。North 分量根据
验收创新一致性整定；修改前 X 轴 $\pm2\sigma$ 覆盖率为 82.42%，当前完整仿真提高到
92.09%，同时 Y/Z 保持在 97% 以上。该参数属于估计器工程参数，不是飞行器物理参数。

## 8. GNSS 位置更新

位置观测模型为

$$
\mathbf z_k=H\mathbf x_k+\mathbf n_p,
$$

其中

$$
H=\begin{bmatrix}I_3&0_{3\times10}\end{bmatrix},\qquad
R_p=\sigma_p^2I_3.
$$

创新与创新协方差：

$$
\mathbf y_k=\mathbf z_k-H\hat{\mathbf x}_k^-,
$$

$$
S_k=HP_k^-H^T+R_p.
$$

Kalman gain 与状态更新：

$$
K_k=P_k^-H^TS_k^{-1},
$$

$$
\hat{\mathbf x}_k^+=\hat{\mathbf x}_k^-+K_k\mathbf y_k.
$$

代码使用线性求解代替显式矩阵求逆。

## 9. Joseph 协方差更新

定义 $A=I-KH$，Joseph form 为：

$$
P_k^+=AP_k^-A^T+KR_pK^T.
$$

相对于简化式 $(I-KH)P$，Joseph form 在有限精度计算中更容易保持对称和半正定。
更新后四元数再次归一化，并对协方差执行对称化和最小特征值正则化。

## 10. $\pm2\sigma$ 验证量

对第 $i$ 个状态：

$$
\sigma_i=\sqrt{\max(P_{ii},0)}.
$$

报告中比较估计误差

$$
e_i=\hat x_i-x_{i,true}
$$

与边界 $\pm2\sigma_i$。理想高斯且一致的滤波器中，误差落入该区间的比例应接近 95%，
但实际还会受模型误差、时间同步和非线性影响。

## 11. 当前 EKF 数学限制

当前协方差直接包含四元数四个分量，但四元数受单位范数约束，真正局部姿态误差只有三维。
代码在归一化四元数后没有使用归一化 Jacobian 变换 $P$。因此：

- `State13.covariance` 中的四元数协方差不能直接等同于严格误差状态 EKF 的姿态协方差；
- 后续若需要严格一致性，建议改成名义四元数 + 3 维小角度误差的 ESKF；
- GNSS 更新目前没有 innovation/Mahalanobis gating，异常点会直接进入更新；
- 因为 bias 被明确忽略，真实硬件存在的零偏无法由当前 13 维 EKF 估计或补偿。

这些是对当前实现的明确边界说明，不改变上述代码实际执行的公式。
