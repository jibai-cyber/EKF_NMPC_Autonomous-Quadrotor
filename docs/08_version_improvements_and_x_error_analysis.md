# 本版本改进记录与 X 轴周期性估计误差分析

## 1. 文档目的与范围

本文记录当前工作区相对上一验收版本的实现改动、实施步骤、测试方法和量化效果，并对
`ekf_position_error_2sigma.png` 中近似正弦的 X 轴估计误差进行数学分析。

需要区分三类内容：

- 第 2～4 节记录提交 `2b17b87` 已经完成的轨迹、EKF 和 NMPC 改进；
- 第 5～6 节记录该版本 X 轴周期误差的离线诊断和 WGS84 根因推导；
- 第 7 节记录 WGS84 ECEF→NED 修正，以及随后恢复简单对角过程噪声的稳定性对照。

当前 `results/` 中的日志、图片和指标均来自“WGS84 修正 + 简单对角 $Q_k$”的完整仿真。

分析使用以下数据：

- `results/acceptance_log.csv`：当前验收运行的整理后日志；
- `results/flight_log.csv`：ROS logger 输出的原始日志；
- `results/acceptance_summary.txt`：验收指标汇总；
- `results/plots/ekf_position_error_2sigma.png`：EKF 位置误差与协方差图。

![EKF 位置估计误差](../results/plots/ekf_position_error_2sigma.png "EKF 位置估计误差与 2σ 边界")

## 2. 已实施的主要改动

### 2.1 八字入口由速度阶跃改为解析路径的平滑时间参数化

原实现从 yaw 对准阶段的零速度直接切换到八字入口速度

$$
\mathbf v_f(0)=[0.75,1.0,0]^T\ \mathrm{m/s},
$$

因而产生明显入口瞬态。本版本没有拟合新的空间过渡曲线，而是保留原八字几何公式，
只对路径时间 $s$ 使用半余弦速度渐入。设八字释放后的时间为 $\tau$，渐入时长为
$T_r=3$ s，则在 $0\le\tau<T_r$ 内：

$$
\dot s(\tau)=\frac12\left[1-\cos\left(\frac{\pi\tau}{T_r}\right)\right],
$$

$$
s(\tau)=\frac12\left[\tau-\frac{T_r}{\pi}
\sin\left(\frac{\pi\tau}{T_r}\right)\right],
$$

$$
\ddot s(\tau)=\frac{\pi}{2T_r}
\sin\left(\frac{\pi\tau}{T_r}\right).
$$

渐入完成后使用 $s=\tau-T_r/2$、$\dot s=1$、$\ddot s=0$。参考速度和加速度按链式法则
计算：

$$
\mathbf v_r=\frac{\partial\mathbf p}{\partial s}\dot s,
$$

$$
\mathbf a_r=\frac{\partial^2\mathbf p}{\partial s^2}\dot s^2
+\frac{\partial\mathbf p}{\partial s}\ddot s.
$$

因此八字入口的位置不变，同时满足 $\mathbf v_r(0)=\mathbf0$、
$\mathbf a_r(0)=\mathbf0$；渐入结束后恢复原解析轨迹的速度和加速度。空间曲线仍是同一条
三维八字，不存在额外的笛卡尔多项式连接段。

相关实现：

- `trajectory.py`：新增 `smooth_lemniscate_reference` 和带时间缩放的解析求导；
- `project.yaml`：新增 `figure8_entry_ramp_duration_s: 3.0`；
- `test_trajectory.py`：验证入口位置、速度、加速度连续，以及渐入结束后与原解析轨迹一致。

### 2.2 八字释放改为状态门控

原版本只根据任务时间切换阶段。本版本在 yaw 对准结束后保持八字起点，只有估计状态连续
满足以下条件 0.5 s 才释放八字任务时钟：

| 条件 | 阈值 |
|---|---:|
| 位置误差范数 | $\le 0.12$ m |
| 速度范数 | $\le 0.20$ m/s |
| yaw 误差 | $\le 5^\circ$ |
| 机体系角速度范数 | $\le 10^\circ$/s |
| 连续驻留时间 | $\ge 0.5$ s |

当前验收运行在仿真时间 10.51 s 释放八字，而不是在未确认状态稳定时强制切换。
`reference_time_s` 被写入原始日志，使绘图和统计按有效任务时间划分阶段，不受门控等待时间
影响。

相关实现：`trajectory_node.py`、`logger_node.py`、`generate_plots.py` 和 `project.yaml`。

### 2.3 EKF 过程噪声方案及后续回退测试

提交 `2b17b87` 曾将原始经验对角过程噪声改为结构化离散：先对真实离散过程函数计算 IMU
输入 Jacobian $G_k$，再传播加速度计和陀螺仪测量噪声：

$$
Q_{imu,k}=G_kR_{imu}G_k^T.
$$

模型误差采用连续白加速度密度 $Q_a$，按定加速度模型精确离散到位置—速度块：

$$
Q_{pv,k}=
\begin{bmatrix}
\frac{\Delta t^3}{3}Q_a & \frac{\Delta t^2}{2}Q_a\\
\frac{\Delta t^2}{2}Q_a & \Delta tQ_a
\end{bmatrix}.
$$

当时 NED 三轴模型误差密度采用 $[0.50,0.08,0.08]$
m/s²/$\sqrt{\mathrm{Hz}}$，并保留位置—速度交叉协方差。

完成 WGS84 修正后，为单独测试原始简单模型的稳定性，当前代码已经恢复：

$$
Q_k=\operatorname{diag}\left(
10^{-8}I_3,\sigma_a^2I_3,\frac14\sigma_g^2I_4,\sigma_g^2I_3
\right)\Delta t.
$$

它不计算 $G_k$，不使用额外的 N/E/D 模型加速度密度，也不显式保留交叉协方差。
第 7.5 节给出同一 WGS84 坐标模型下两种 $Q_k$ 的实测对照。

当前相关实现：`ekf.py` 和 `test_ekf.py`；旧的模型误差密度参数已从 `ekf_node.py` 和
`project.yaml` 移除。

### 2.4 NMPC shifted warm start

上一周期的最优预测序列不再原样复用，而是按 receding-horizon 向前移动一步：

$$
\mathbf X^{(0)}=
[\mathbf x_k,\mathbf X^*_{2},\ldots,\mathbf X^*_{N},
f_d(\mathbf X^*_{N},\mathbf U^*_{N-1})],
$$

$$
\mathbf U^{(0)}=
[\mathbf U^*_{1},\ldots,\mathbf U^*_{N-1},\mathbf U^*_{N-1}].
$$

首状态替换为当前 EKF 状态，末状态用 RK4 再传播一步。这样初值与新的预测时刻对齐，减少
求解器从过期序列重新调整的工作。

相关实现：`nmpc.py` 中 `_shifted_warm_start`，并由 `test_nmpc.py` 验证移位关系。

### 2.5 推力变化率进入 NMPC 预测约束

上一周期实际发布推力 $\mathbf u_{k-1}$ 作为 NLP 参数。优化器直接约束：

$$
-\Delta T_{max}\le \mathbf U_0-\mathbf u_{k-1}\le\Delta T_{max},
$$

$$
-\Delta T_{max}\le \mathbf U_j-\mathbf U_{j-1}\le\Delta T_{max},
\qquad j=1,\ldots,N-1.
$$

控制频率为 20 Hz，旋翼变化率上限为 40 N/s，因此

$$
\Delta T_{max}=40\times0.05=2.0\ \mathrm{N/update}.
$$

求解后的执行器投影仍保留为数值误差和回退控制的最终保护，但正常 NMPC 预测现在已经知道
并遵守同一变化率边界。

相关实现：`nmpc.py`、`nmpc_node.py`、`project.yaml`、`generate_plots.py` 和
`test_nmpc.py`。

## 3. 实施与验证步骤

本版本按以下顺序完成：

1. 在轨迹层引入半余弦路径时间缩放，并用解析链式法则计算速度、加速度和 yaw rate；
2. 订阅 13 维 EKF 状态，加入八字入口阈值和连续驻留计时；
3. 在日志中加入有效参考时间，并让验收脚本按参考时间统计阶段；
4. 用离散输入 Jacobian 和精确位置—速度块重建 EKF 的 $Q_k$；
5. 将上一周期实际推力加入 NMPC 参数，在整个预测域施加 slew-rate 约束；
6. 将上一最优解执行 receding-horizon shift 后用作下一次初值；
7. 增加对应单元测试，重新运行完整无界面 Gazebo 验收仿真；
8. 通过谐波和位置—速度回归识别 WGS84 North 尺度误差；
9. 实现完整 geodetic→ECEF→NED 换算，增加 4 项 geodesy 测试并再次运行完整仿真；
10. 恢复原始经验对角 $Q_k$，增加对角方差率和零交叉项测试；
11. 再次运行完整仿真，并从新日志生成图片、详细 CSV 和指标汇总。

Docker 内的自动化测试结果为：

```text
........................                                                 [100%]
24 passed
```

复现测试：

```bash
cd /absolute/path/to/drone
docker compose run --rm --no-deps drone-dev bash -lc '
source /opt/ros/jazzy/setup.bash
source /opt/drone_venv/bin/activate
source install/setup.bash
python3 -m pytest -q src/drone_gnc/test
'
```

## 4. 第一阶段改动的量化效果

下表比较最初版本和提交 `2b17b87`；WGS84 修正的独立前后对比见第 7.3 节。

| 指标 | 最初版本 | `2b17b87` | 变化 |
|---|---:|---:|---:|
| 八字入口前 5 s 三维 RMSE | 0.2369 m | 0.0490 m | 降低 79.3% |
| 八字入口前 5 s 最大三维误差 | 0.5849 m | 0.0708 m | 降低 87.9% |
| 八字全阶段三维 RMSE | 0.0794 m | 0.0510 m | 降低 35.8% |
| X 位置 $\pm2\sigma$ 覆盖率 | 82.42% | 92.09% | 提高 9.67 个百分点 |
| 最大倾角 | 20.89° | 6.44° | 降低 69.2% |
| 超过 50 ms 的求解样本 | 5 | 3 | 减少 2 个 |
| 推力 slew-rate 越界样本 | 未作预测域统计 | 0 | 最大 0.5443 N/update |

WGS84 修正并恢复简单对角 $Q_k$ 后，当前完整八字阶段核心指标为：

- 三轴位置跟踪 RMSE：X 0.0207 m、Y 0.0273 m、Z 0.0097 m；
- 三维位置跟踪 RMSE：0.0356 m，最大误差 0.0808 m；
- EKF 位置 RMSE：X 0.01115 m、Y 0.01095 m、Z 0.00987 m；
- NMPC 日志成功率：99.9838%；中位 11.43 ms，P95 15.62 ms；
- 推力、推力变化率、力矩、倾角和机体系角速度越界数均为 0。

第一阶段改动曾使 yaw RMSE 从 1.613° 增加到 3.750°，求解时间中位数从 11.91 ms
增加到 12.88 ms。当前简单 $Q_k$ 运行中，两者分别为 1.675° 和 11.43 ms。求解耗时会受
主机调度影响，因此不能把这部分变化完全归因于过程噪声；但飞行性能和实时统计均未退化。

![三维任务轨迹](../results/plots/trajectory_3d.png "地面起飞、yaw 对准和三维八字")

![三轴跟踪误差](../results/plots/tracking_errors.png "八字阶段三轴跟踪误差")

## 5. X 轴误差是否确实具有正弦相关性

### 5.1 轨迹基频

八字的 North 方向参考为

$$
x_r(s)=A_x\sin(\omega s),\qquad A_x=3\ \mathrm m,\quad
\omega=0.25\ \mathrm{rad/s}.
$$

其基频和周期为

$$
f_0=\frac{\omega}{2\pi}=0.03979\ \mathrm{Hz},\qquad
T_0=\frac{2\pi}{\omega}=25.13\ \mathrm s.
$$

对八字阶段的 X 轴估计误差

$$
e_x=\hat x-x_{truth}
$$

进行固定基频的最小二乘谐波回归：

$$
e_x=b+c_s\sin(\omega s)+c_c\cos(\omega s)+\varepsilon,
$$

主谐波幅值和相位为

$$
A_e=\sqrt{c_s^2+c_c^2},\qquad
\phi_e=\operatorname{atan2}(c_c,c_s).
$$

为区分 100 Hz 白噪声和低频确定性误差，同时对误差作 1 s 居中滑动平均后重复回归。

| 数据 | 主谐波幅值 | 相位 | $R^2$ | 谐波外残差 RMSE |
|---|---:|---:|---:|---:|
| 原始 100 Hz 误差 | 0.02121 m | +0.015 rad | 49.36% | 0.01505 m |
| 1 s 平滑误差 | 0.02110 m | +0.017 rad | 93.04% | 0.00407 m |

结论是：**图中的慢变部分确实高度接近轨迹基频的正弦项**。原始数据的 $R^2$ 较低并不
否定该结论，因为其上叠加了约 1.5 cm RMSE 的随机高频分量；平滑后主谐波解释了 93.04%
的低频变化。相位接近零，说明误差与 North 位置近似同相。

### 5.2 尺度项与时延项的区分

进一步拟合

$$
e_x=b+k_xx_{truth}+k_v\dot x_{truth}+\varepsilon.
$$

其中：

- $k_xx$ 表示位置尺度误差；
- 若测量存在小延迟 $\tau$，则
  $x(t-\tau)\approx x(t)-\tau\dot x(t)$，因此速度系数可解释为等效延迟项。

坐标修正前的验收日志得到：

| 参数 | 估计值 | 标准误差 |
|---|---:|---:|
| 截距 $b$ | +0.000206 m | 0.000195 m |
| 位置系数 $k_x$ | +0.007134 | 0.000092 |
| 速度系数 $k_v$ | +0.000710 s | 0.000378 s |

仅使用位置项时 $R^2=49.39\%$；仅使用速度项时 $R^2=0.092\%$。因此主要现象是约
**+0.714% 的 North 位置尺度误差**，而不是传感器时间延迟。独立的后一份原始日志得到
$k_x=+0.7028\%$，说明该结果能够跨运行复现。

由于稳态段 $a_x=-\omega^2x$，误差会自然与 X 加速度呈负相关；yaw rate 也随路径相位
周期变化。因此“误差与加速度或 yaw rate 相关”本身不能证明动力学或航向是根因，二者与
位置之间存在共同的轨迹相位。位置—速度回归和下一节的量级闭合提供了更强证据。

## 6. 根因：WGS84 纬向曲率半径不一致

Gazebo world 明确配置：

```xml
<surface_model>EARTH_WGS84</surface_model>
```

但修正前的 `SensorSimulatorNode.navsat_callback` 用 WGS84 长半轴
$a=6378137$ m 同时换算纬度差和经度差：

$$
x_{used}=a\,\Delta\varphi.
$$

在 WGS84 椭球上，小范围纬向距离应使用参考纬度处的子午圈曲率半径：

$$
M(\varphi_0)=
\frac{a(1-e^2)}{(1-e^2\sin^2\varphi_0)^{3/2}},
$$

$$
x_{WGS84}=M(\varphi_0)\Delta\varphi.
$$

参考纬度 $\varphi_0=1.3521^\circ$ 时：

$$
M=6335474.749\ \mathrm m,
$$

$$
\frac{a}{M}-1=0.00673387=0.6734\%.
$$

因此修正前的转换会产生

$$
e_{geo}\approx0.00673387x.
$$

代入 $x=3\sin(\omega s)$，理论正弦误差幅值为

$$
A_{geo}=3\times0.00673387=0.02020\ \mathrm m.
$$

这与实测 0.02121 m 只差约 1.0 mm，且理论 +0.6734% 与回归 +0.7138% 方向、相位和量级
全部一致。因此 WGS84 纬向换算不一致是当前 X 轴正弦误差的主要根因，而不只是一个泛化的
“模型误差”猜测。

East 方向应使用卯酉圈曲率半径

$$
N(\varphi_0)=\frac{a}{\sqrt{1-e^2\sin^2\varphi_0}},
$$

并计算 $y=N\cos\varphi_0\Delta\lambda$。在当前低纬度，$a/N-1=-0.000186\%$，远小于
North 方向的 0.6734%，这也解释了为什么 Y 轴没有同量级的周期项。

## 7. WGS84 坐标修正的实现与验证

### 7.1 实现方法

修正已写入 `geodesy.py` 和 `sensor_simulator_node.py`。实现没有只替换局部比例系数，而是
采用适用于更大局部范围的完整两步变换：

1. 将 NavSat 的 WGS84 geodetic 纬度、经度、高度转换为 ECEF；
2. 计算相对参考点的 ECEF 位移，再旋转到参考点的 NED 切平面。

给定纬度 $\varphi$、经度 $\lambda$、椭球高 $h$，ECEF 坐标为

$$
\begin{aligned}
X&=(N+h)\cos\varphi\cos\lambda,\\
Y&=(N+h)\cos\varphi\sin\lambda,\\
Z&=[N(1-e^2)+h]\sin\varphi.
\end{aligned}
$$

以参考点 $(\varphi_0,\lambda_0,h_0)$ 的 ECEF 坐标为原点，旋转矩阵为

$$
R_{E\to NED}=
\begin{bmatrix}
-\sin\varphi_0\cos\lambda_0 & -\sin\varphi_0\sin\lambda_0 & \cos\varphi_0\\
-\sin\lambda_0 & \cos\lambda_0 & 0\\
-\cos\varphi_0\cos\lambda_0 & -\cos\varphi_0\sin\lambda_0 & -\sin\varphi_0
\end{bmatrix}.
$$

最终位置为

$$
\mathbf p^{NED}=R_{E\to NED}
(\mathbf p^{ECEF}-\mathbf p_0^{ECEF}).
$$

该方法在当前几米范围内退化为第 6 节的 $M\Delta\varphi$、
$N\cos\varphi_0\Delta\lambda$，同时正确处理高度和三个方向的耦合，不再使用单一球半径。

### 7.2 自动化验证

新增 `test_geodesy.py`，验证：

- WGS84 参考点严格映射到 NED 原点；
- 按 $M$ 和 $N$ 构造的 3 m North、2 m East 位移能以微米级误差恢复；
- 高度增加 2 m 映射为 Down = -2 m；
- 旧球面公式在当前纬度确实产生 0.673387% 的 North 尺度误差。

工作区重新构建成功，完整测试结果为 24 passed。

### 7.3 完整 Gazebo 仿真结果

修正后重新运行了地面起飞、yaw 对准、状态门控和完整 60 s 八字路径，并重新生成
`flight_log.csv`、`acceptance_log.csv`、全部图片和指标汇总。八字阶段为 10.51～72.04 s，
共 6154 个 100 Hz 样本。

本小节记录首次 WGS84 验证，当时仍使用结构化 $Q_k$；当前简单对角 $Q_k$ 的结果见第 7.5 节。

| X 轴估计指标 | 修正前 | WGS84 修正后 | 变化 |
|---|---:|---:|---:|
| 位置 RMSE | 21.33 mm | 15.15 mm | 降低 29.0% |
| $\pm2\sigma$ 覆盖率 | 92.09% | 98.28% | 提高 6.19 个百分点 |
| 位置尺度系数 $k_x$ | +0.7138% | +0.00614% | 降低 99.1% |
| 原始误差主谐波幅值 | 21.21 mm | 0.155 mm | 降低 99.3% |
| 1 s 平滑后主谐波幅值 | 21.10 mm | 0.169 mm | 降低 99.2% |
| 原始误差谐波 $R^2$ | 49.36% | 0.0051% | 周期项消失 |
| 平滑误差谐波 $R^2$ | 93.04% | 0.0792% | 周期项消失 |

实际主谐波幅值 0.155 mm 甚至低于离线预计的 1.26 mm。位置尺度系数的标准误差为
0.00922%，而估计值仅 0.00614%，已经无法在该单次运行中与零显著区分。这说明修复的是
真实测量模型错误，而不是通过滤波参数把曲线暂时压平。

整体闭环指标也保持稳定或略有改善：

| 闭环指标 | 修正前 | WGS84 修正后 |
|---|---:|---:|
| 八字三维跟踪 RMSE | 0.05096 m | 0.04967 m |
| 八字最大三维误差 | 0.12817 m | 0.11258 m |
| yaw RMSE | 3.750° | 2.059° |
| NMPC 成功率 | 99.9675% | 99.9838% |
| 50 ms deadline miss | 3 | 2 |
| 控制约束越界 | 0 | 0 |

### 7.4 WGS84 修正结论

修正后的 X 轴误差主要是约 15.15 mm RMSE 的随机分量，与 20 mm GNSS 白噪声和 EKF
平滑后的量级相符。确定性八字基频项已不再是主要误差来源。

结构化 $Q_k$ 下 X 轴覆盖率为 98.28%，略高于理论约 95%。这提示此前 North 方向 0.50
的模型误差密度可能在坐标尺度修复后偏保守，也构成恢复简单 $Q_k$ 做对照试验的动机。

不建议按轨迹相位减去拟合正弦项；该做法会依赖当前八字轨迹并掩盖测量模型问题。

### 7.5 恢复简单对角过程噪声后的稳定性

保持 WGS84、轨迹、NMPC、传感器噪声和随机种子不变，只将过程噪声恢复为第 2.3 节的
经验对角形式。完整八字仍在 10.51～72.04 s 完成，共 6154 个 100 Hz 样本，没有出现
状态发散、任务门控失败或约束越界。

| 指标 | 结构化 $Q_k$ | 简单对角 $Q_k$ | 变化 |
|---|---:|---:|---:|
| EKF X RMSE | 15.15 mm | 11.15 mm | 降低 26.4% |
| EKF Y RMSE | 10.17 mm | 10.95 mm | 增加 7.6% |
| EKF Z RMSE | 10.34 mm | 9.87 mm | 降低 4.5% |
| X/Y/Z $\pm2\sigma$ 覆盖率 | 98.28/97.82/96.34% | 97.58/97.56/97.84% | 均略高于 95% |
| 八字三维跟踪 RMSE | 49.67 mm | 35.63 mm | 降低 28.3% |
| 最大三维跟踪误差 | 112.58 mm | 80.80 mm | 降低 28.2% |
| yaw RMSE | 2.059° | 1.675° | 降低 18.7% |
| 控制约束越界 | 0 | 0 | 无退化 |

X 轴谐波回归结果为：

| 指标 | 结构化 $Q_k$ | 简单对角 $Q_k$ |
|---|---:|---:|
| 主谐波幅值 | 0.155 mm | 0.885 mm |
| 主谐波 $R^2$ | 0.0051% | 0.3077% |
| 1 s 平滑后幅值 | 0.169 mm | 0.865 mm |
| 1 s 平滑后 $R^2$ | 0.0792% | 1.662% |
| 位置尺度系数 $k_x$ | +0.00614% | +0.02977% |

简单模型下低频谐波略有增加，但幅值仍低于 0.9 mm，且只能解释约 0.31% 的原始误差，
与修正前 21.21 mm、49.36% 的确定性周期项不在同一量级。因此 WGS84 修正仍然有效，
没有因恢复简单 $Q_k$ 而重新引入坐标尺度问题。

当前单次仿真说明简单 $Q_k$ 在此环境下稳定且表现更好，但不能据此断言它在所有工况下
优于结构化离散。结构化模型物理含义更完整；最终选型应增加多随机种子、风扰动、质量与惯量
偏差试验，并比较创新白化和 NEES/NIS，而不是只比较一条理想仿真轨迹。

## 8. 当前版本文件变更索引

| 类别 | 文件 | 主要内容 |
|---|---|---|
| 地理坐标 | `src/drone_gnc/drone_gnc/geodesy.py` | WGS84 曲率、geodetic→ECEF→NED |
| 传感器 | `src/drone_gnc/drone_gnc/sensor_simulator_node.py` | NavSat 使用 WGS84 局部 NED 投影 |
| 轨迹 | `src/drone_gnc/drone_gnc/trajectory.py` | 半余弦路径时间渐入与解析导数 |
| 任务门控 | `src/drone_gnc/drone_gnc/trajectory_node.py` | 状态阈值、驻留计时和任务时钟冻结 |
| EKF | `src/drone_gnc/drone_gnc/ekf.py` | 当前经验对角 $Q_k$ 和 13 维状态传播 |
| EKF 测试 | `src/drone_gnc/test/test_ekf.py` | 对角方差率、零交叉项和 GNSS 更新 |
| NMPC | `src/drone_gnc/drone_gnc/nmpc.py` | shifted warm start 与预测域 slew-rate 约束 |
| 控制节点 | `src/drone_gnc/drone_gnc/nmpc_node.py` | 传入上一实际推力和有效参考时间 |
| 日志 | `src/drone_gnc/drone_gnc/logger_node.py` | 记录 `reference_time_s` |
| 配置 | `src/drone_gnc/config/project.yaml` | 渐入、门控、EKF 和 NMPC 新参数 |
| 验收工具 | `scripts/generate_plots.py` | 按任务时钟分段并统计推力变化率 |
| 测试 | `src/drone_gnc/test/` | geodesy、连续性、协方差、warm start 和 slew-rate 测试 |
| 文档 | `docs/03`～`docs/07`、`README.md` | 公式、参数追溯、限制和验收结果同步 |

## 9. 最终结论

本版本已经解决八字入口速度阶跃、纯时间切换、未移位 warm start 和预测模型缺少推力变化率
约束四个问题，并完成结构化与简单对角过程噪声对照。入口最大三维误差降低 87.9%，全部
控制约束无越界；加入 geodesy 覆盖后，24 项自动化测试通过。

X 轴估计误差并非视觉上“碰巧像正弦”：修正前平滑误差有 93.04% 可由八字基频解释，
位置回归、相位和理论量级共同指向 WGS84 纬向尺度错误。完成 geodetic→ECEF→NED 修正后，
主谐波幅值从 21.21 mm 降至 0.155 mm，降低 99.3%；X 轴 RMSE 降低 29.0%，完整闭环
约束仍全部满足。新日志证明坐标修正达到了预期目的。

在此基础上恢复简单对角 $Q_k$ 后，完整飞行仍稳定，三维跟踪 RMSE 进一步降至 35.63 mm，
X/Y/Z 覆盖率均约为 97.6%～97.8%，全部约束继续满足。当前可以使用简单版本继续测试，
但最终工程选型仍应通过 Monte Carlo 和扰动试验决定。
