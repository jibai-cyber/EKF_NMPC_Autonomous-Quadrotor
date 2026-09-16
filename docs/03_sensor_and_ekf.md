# 传感器模型与扩展卡尔曼滤波器

## 1. 传感器频率与噪声

课程规范给出：

| 传感器 | 频率 | 白噪声标准差 |
|---|---:|---:|
| accelerometer | 100 Hz | $\sigma_a=0.08$ m/s² |
| gyroscope | 100 Hz | $\sigma_g=0.015$ rad/s |
| GNSS position | 20 Hz | $\sigma_p=0.02$ m |

Gazebo SDF 产生理想原始 IMU/NavSat，`SensorSimulatorNode` 再显式加入课程噪声和 bias。

**代码对应**

| 文件 | 类/位置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`model.sdf`](../src/drone_description/models/course_quadrotor/model.sdf#L77-L89) | `imu_raw`、`navsat_raw` | 77-89 | 100/20 Hz 原始传感器 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L1-L21) | sensor/EKF 参数 | 1-21 | 噪声和 bias random walk |
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L25-L65) | `SensorSimulatorNode.__init__` | 25-65 | 参数与 topic 初始化 |

## 2. IMU 测量模型

加速度计测量的是 specific force，而不是直接的世界系加速度：

$$
\mathbf f_m^B=R_{WB}^T(\mathbf a^W-\mathbf g^W)
+\mathbf b_a+\mathbf n_a.
$$

陀螺仪模型为：

$$
\boldsymbol\omega_m^B=\boldsymbol\omega^B+\mathbf b_g+\mathbf n_g.
$$

其中：

$$
\mathbf n_a\sim\mathcal N(0,\sigma_a^2I),\qquad
\mathbf n_g\sim\mathcal N(0,\sigma_g^2I).
$$

传感器仿真先把 Gazebo FLU 测量转为 FRD，再加入白噪声和 bias。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L67-L115) | `imu_callback` | 67-115 | 坐标转换、bias 和白噪声 |
| [`frames.py`](../src/drone_gnc/drone_gnc/frames.py#L16-L18) | `vector_flu_to_frd` | 16-18 | FLU 到 FRD |

## 3. Bias 随机游走

课程给出连续随机游走：

$$
\dot{\mathbf b}_a=\mathbf w_{ba},\qquad
\dot{\mathbf b}_g=\mathbf w_{bg}.
$$

离散 Euler-Maruyama 形式为：

$$
\mathbf b_{a,k+1}=\mathbf b_{a,k}
+\sigma_{ba}\sqrt{\Delta t}\,\boldsymbol\xi_{a,k},
$$

$$
\mathbf b_{g,k+1}=\mathbf b_{g,k}
+\sigma_{bg}\sqrt{\Delta t}\,\boldsymbol\xi_{g,k},
$$

其中 $\boldsymbol\xi\sim\mathcal N(0,I)$。课程 PDF 没有给出 $\sigma_{ba}$、
$\sigma_{bg}$，当前值 `0.002` 和 `0.0002` 是可调工程假设。

**代码对应**

| 文件 | 函数/配置 | 当前行号 | 实现 |
|---|---|---:|---|
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L67-L76) | `imu_callback` | 67-76 | bias 离散随机游走 |
| [`project.yaml`](../src/drone_gnc/config/project.yaml#L6-L10) | bias 参数 | 6-10 | 仿真强度与随机种子 |

## 4. GNSS 局部 NED 位置

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

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`sensor_simulator_node.py`](../src/drone_gnc/drone_gnc/sensor_simulator_node.py#L117-L137) | `navsat_callback` | 117-137 | 经纬高到局部 NED 并加噪声 |

## 5. EKF 19 维名义状态

EKF 内部状态为：

$$
\mathbf x_E=
\begin{bmatrix}
\mathbf p^W&\mathbf v^W&q^{WB}&\boldsymbol\omega^B&\mathbf b_a&\mathbf b_g
\end{bmatrix}^T.
$$

维数为：

$$
3+3+4+3+3+3=19.
$$

只将前 13 维发布给 NMPC。bias 作为滤波器内部中间量。

**代码对应**

| 文件 | 类/属性 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L19-L40) | `QuadrotorEkf` | 19-40 | 状态、初始值和协方差 |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L146-L159) | `public_state`、`internal_biases` | 146-159 | 公开 13 维与内部 bias |
| [`ekf_node.py`](../src/drone_gnc/drone_gnc/ekf_node.py#L54-L63) | `imu_callback` 输出段 | 54-63 | 发布 `State13` |

## 6. IMU 去 bias 与世界系加速度

预测时先校正 IMU：

$$
\hat{\mathbf f}^B=\mathbf f_m^B-\hat{\mathbf b}_a,
$$

$$
\hat{\boldsymbol\omega}^B=
\boldsymbol\omega_m^B-\hat{\mathbf b}_g.
$$

由 specific force 恢复世界系加速度：

$$
\hat{\mathbf a}^W=R_{WB}(\hat q)\hat{\mathbf f}^B+\mathbf g^W.
$$

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L47-L62) | `_process_step` | 47-62 | IMU 校正与加速度恢复 |

## 7. EKF 离散过程模型

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
1\\\frac12\hat{\boldsymbol\omega}_k\Delta t
\end{bmatrix},
$$

$$
q_{k+1}=\operatorname{normalize}(q_k\otimes\delta q_k).
$$

公开角速度状态直接使用去 bias 的高频 gyro：

$$
\boldsymbol\omega_{k+1}=\hat{\boldsymbol\omega}_k.
$$

名义 bias 的确定性导数为零；随机游走通过过程噪声协方差进入滤波器。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L47-L73) | `_process_step` | 47-73 | 完整离散名义传播 |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L75-L107) | `predict` | 75-107 | 传播入口与协方差预测 |

## 8. 数值状态转移 Jacobian

设离散过程模型为：

$$
\mathbf x_{k+1}=f_d(\mathbf x_k,\mathbf u_k).
$$

EKF 需要：

$$
F_k=\left.\frac{\partial f_d}{\partial\mathbf x}\right|_{\hat x_k,u_k}.
$$

代码使用中心差分。对第 $i$ 个标准基向量 $e_i$：

$$
F_k[:,i]\approx
\frac{f_d(\hat x_k+\epsilon e_i,u_k)-
f_d(\hat x_k-\epsilon e_i,u_k)}{2\epsilon},
$$

其中 $\epsilon=10^{-6}$。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L84-L96) | `predict` | 84-96 | 19 列中心差分 Jacobian |

## 9. 过程协方差传播

EKF 预测为：

$$
P_{k+1}^-=F_kP_k^+F_k^T+Q_k.
$$

当前实现构造对角过程噪声：

$$
Q_k=\operatorname{diag}(q_p,q_v,q_q,q_\omega,q_{ba},q_{bg})\Delta t.
$$

其中速度、姿态、角速度和 bias 项分别由配置的 IMU 标准差与 random-walk 标准差平方
得到；位置使用很小的正则噪声。该 $Q$ 是工程离散近似，不是由完整连续噪声输入矩阵严格
离散得到。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L98-L107) | `predict` | 98-107 | $Q_k$ 与 $P^-$ |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L140-L144) | `_stabilize_covariance` | 140-144 | 对称化与最小特征值正则化 |

## 10. GNSS 位置更新

位置观测模型：

$$
\mathbf z_k=H\mathbf x_k+\mathbf n_p,
$$

其中：

$$
H=\begin{bmatrix}I_3&0_{3\times16}\end{bmatrix},\qquad
R_p=\sigma_p^2I_3.
$$

创新与创新协方差：

$$
\mathbf y_k=\mathbf z_k-H\hat{\mathbf x}_k^-,
$$

$$
S_k=HP_k^-H^T+R_p.
$$

Kalman gain：

$$
K_k=P_k^-H^TS_k^{-1}.
$$

状态更新：

$$
\hat{\mathbf x}_k^+=\hat{\mathbf x}_k^-+K_k\mathbf y_k.
$$

代码使用线性求解代替显式矩阵求逆。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L109-L117) | `update_position` | 109-117 | 构造 $H$、$R_p$ |
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L119-L129) | `_linear_update` | 119-129 | 创新、$S$、$K$ 与状态更新 |
| [`ekf_node.py`](../src/drone_gnc/drone_gnc/ekf_node.py#L33-L34) | `position_callback` | 33-34 | GNSS 更新入口 |

## 11. Joseph 协方差更新

定义：

$$
A=I-KH.
$$

Joseph form 为：

$$
P_k^+=AP_k^-A^T+KR_pK^T.
$$

相对于简化式 $(I-KH)P$，Joseph form 在有限精度计算中更容易保持对称和半正定。
更新后四元数再次归一化。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`ekf.py`](../src/drone_gnc/drone_gnc/ekf.py#L130-L138) | `_linear_update` | 130-138 | Joseph form 和四元数归一化 |

## 12. ±2σ 验证量

对第 $i$ 个公开状态：

$$
\sigma_i=\sqrt{\max(P_{ii},0)},
$$

报告中比较估计误差：

$$
e_i=\hat x_i-x_{i,true}
$$

与边界 $\pm2\sigma_i$。理想高斯且一致的滤波器中，误差落入该区间的比例应接近 95%，
但实际还会受模型误差、时间同步和非线性影响。

**代码对应**

| 文件 | 函数 | 当前行号 | 实现 |
|---|---|---:|---|
| [`logger_node.py`](../src/drone_gnc/drone_gnc/logger_node.py#L64-L81) | `estimate_callback` | 64-81 | 从对角线计算标准差并写 CSV |
| [`generate_plots.py`](../scripts/generate_plots.py#L30-L41) | `main` | 30-41 | 绘制误差与 ±2σ |

## 13. 当前 EKF 数学限制

当前协方差直接包含四元数四个分量，但四元数受单位范数约束，真正局部姿态误差只有三维。
代码在归一化四元数后没有使用归一化 Jacobian 变换 $P$。因此：

- `State13.covariance` 中的四元数协方差不能直接等同于严格误差状态 EKF 的姿态协方差；
- 后续若需要严格一致性，建议改成名义四元数 + 3 维小角度误差的 ESKF；
- GNSS 更新目前没有 innovation/Mahalanobis gating，异常点会直接进入更新。

这些是对当前实现的明确边界说明，不改变上述代码实际执行的公式。
