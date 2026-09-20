# 本版本改进记录与 X 轴周期性估计误差分析

## 1. 文档目的与范围

本文记录当前工作区相对上一验收版本的实现改动、实施步骤、测试方法和量化效果，并对
`ekf_position_error_2sigma.png` 中近似正弦的 X 轴估计误差进行数学分析。

需要区分两类内容：

- 第 2～4 节是**已经进入当前代码并通过仿真验证的改动**；
- 第 5～7 节是对 X 轴周期误差的**离线诊断和下一步修正建议**。WGS84 换算修正尚未写入
  当前代码，因此现有验收日志、图片和指标仍与当前实现严格对应。

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

最终验收运行在仿真时间 10.52 s 释放八字，而不是在未确认状态稳定时强制切换。
`reference_time_s` 被写入原始日志，使绘图和统计按有效任务时间划分阶段，不受门控等待时间
影响。

相关实现：`trajectory_node.py`、`logger_node.py`、`generate_plots.py` 和 `project.yaml`。

### 2.3 EKF 过程噪声离散化与 X 轴一致性

原 EKF 使用互不相关的经验对角过程噪声。本版本先对真实离散过程函数计算 IMU 输入
Jacobian $G_k$，再传播加速度计和陀螺仪测量噪声：

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

当前 NED 三轴模型误差密度为 $[0.50,0.08,0.08]$
m/s²/$\sqrt{\mathrm{Hz}}$。改动保留了位置—速度交叉协方差，不再用互不相关的对角项近似。

> North 方向的 0.50 是依据本轮一致性结果作出的工程整定。第 6 节识别出确定性的 WGS84
> 尺度误差后，下一版本应先修正坐标换算，再重新辨识这个参数；不应继续用增大 $Q$ 的方式
> 掩盖确定性测量模型误差。

相关实现：`ekf.py`、`ekf_node.py`、`project.yaml` 和 `test_ekf.py`。

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
8. 从同一份日志生成图片、详细 CSV 和 `acceptance_summary.txt`。

Docker 内的自动化测试结果为：

```text
....................                                                     [100%]
20 passed in 0.47s
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

## 4. 已实现改动的量化效果

| 指标 | 修改前 | 当前版本 | 变化 |
|---|---:|---:|---:|
| 八字入口前 5 s 三维 RMSE | 0.2369 m | 0.0490 m | 降低 79.3% |
| 八字入口前 5 s 最大三维误差 | 0.5849 m | 0.0708 m | 降低 87.9% |
| 八字全阶段三维 RMSE | 0.0794 m | 0.0510 m | 降低 35.8% |
| X 位置 $\pm2\sigma$ 覆盖率 | 82.42% | 92.09% | 提高 9.67 个百分点 |
| 最大倾角 | 20.89° | 6.44° | 降低 69.2% |
| 超过 50 ms 的求解样本 | 5 | 3 | 减少 2 个 |
| 推力 slew-rate 越界样本 | 未作预测域统计 | 0 | 最大 0.5443 N/update |

当前完整八字阶段的核心指标为：

- 三轴位置跟踪 RMSE：X 0.0354 m、Y 0.0350 m、Z 0.0109 m；
- 三维位置跟踪 RMSE：0.0510 m，最大误差 0.1282 m；
- EKF 位置 RMSE：X 0.0213 m、Y 0.0101 m、Z 0.0101 m；
- NMPC 日志成功率：99.9675%；中位 12.88 ms，P95 18.01 ms；
- 推力、推力变化率、力矩、倾角和机体系角速度越界数均为 0。

需要同时记录两个权衡：yaw RMSE 从 1.613° 增加到 3.750°，求解时间中位数从
11.91 ms 增加到 12.88 ms。前者与入口时间缩放后的切向航向动态有关，后者与新增预测
约束及运行时波动有关；二者仍处于稳定可用范围，但不能只报告改善项而忽略。

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

验收日志得到：

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

但当前 `SensorSimulatorNode.navsat_callback` 用 WGS84 长半轴
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

因此当前转换会产生

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

## 7. 能否进一步降低振幅

可以。首选方案是修正 GNSS→NED 的局部切平面换算，而不是继续调整 EKF 的 $Q$：

1. 根据 WGS84 扁率计算 $e^2$、$M(\varphi_0)$ 和 $N(\varphi_0)$；
2. 使用 $M\Delta\varphi$ 计算 North；
3. 使用 $N\cos\varphi_0\Delta\lambda$ 计算 East；
4. 保持 Down 的高度差定义不变；
5. 重新运行完整验收仿真，再根据创新一致性重新整定 North 过程噪声密度。

对现有日志离线减去理论项 $0.00673387x_{truth}$ 后：

| 指标 | 修正前 | 理论尺度修正后 | 预计变化 |
|---|---:|---:|---:|
| X 主谐波幅值 | 21.21 mm | 1.26 mm | 降低 94.1% |
| X 位置总 RMSE | 21.33 mm | 15.07 mm | 降低 29.3% |
| 主谐波 $R^2$ | 49.36% | 0.34% | 确定性周期项基本消失 |

这里的“修正后”是基于已有日志的离线反事实估计，不是新仿真的验收结果。实际代码修正后的
最终数值应以重新运行 Gazebo 所得到的日志为准。

不建议采用以下两种方式：

- 直接增大 X 轴 $Q$：这主要改变 Kalman 增益和 $\pm2\sigma$ 宽度，不能消除确定性尺度误差；
- 按已知八字相位减去正弦拟合项：这会依赖当前轨迹并掩盖测量模型错误，换轨迹后不能泛化。

时间戳对齐和延迟补偿仍可作为后续精细优化，但当前速度项解释度不足 0.1%，优先级明显低于
WGS84 曲率修正。

## 8. 当前版本文件变更索引

| 类别 | 文件 | 主要内容 |
|---|---|---|
| 轨迹 | `src/drone_gnc/drone_gnc/trajectory.py` | 半余弦路径时间渐入与解析导数 |
| 任务门控 | `src/drone_gnc/drone_gnc/trajectory_node.py` | 状态阈值、驻留计时和任务时钟冻结 |
| EKF | `src/drone_gnc/drone_gnc/ekf.py` | $G_kR_{imu}G_k^T$ 与结构化 $Q_{pv}$ |
| EKF 参数 | `src/drone_gnc/drone_gnc/ekf_node.py` | 三轴模型误差密度参数 |
| NMPC | `src/drone_gnc/drone_gnc/nmpc.py` | shifted warm start 与预测域 slew-rate 约束 |
| 控制节点 | `src/drone_gnc/drone_gnc/nmpc_node.py` | 传入上一实际推力和有效参考时间 |
| 日志 | `src/drone_gnc/drone_gnc/logger_node.py` | 记录 `reference_time_s` |
| 配置 | `src/drone_gnc/config/project.yaml` | 渐入、门控、EKF 和 NMPC 新参数 |
| 验收工具 | `scripts/generate_plots.py` | 按任务时钟分段并统计推力变化率 |
| 测试 | `src/drone_gnc/test/` | 连续性、协方差、warm start 和 slew-rate 测试 |
| 文档 | `docs/03`～`docs/07`、`README.md` | 公式、参数追溯、限制和验收结果同步 |

## 9. 最终结论

本版本已经解决八字入口速度阶跃、纯时间切换、未移位 warm start 和预测模型缺少推力变化率
约束四个问题，并改善了 EKF 协方差结构。入口最大三维误差降低 87.9%，完整八字三维 RMSE
降低 35.8%，全部控制约束无越界，20 项自动化测试通过。

X 轴估计误差并非视觉上“碰巧像正弦”：平滑后有 93.04% 的变化可由八字基频解释。
位置回归、相位和理论量级共同表明，主要原因是 WGS84 纬向曲率半径使用不一致。下一步将
North 换算从 $a\Delta\varphi$ 改为 $M(\varphi_0)\Delta\varphi$，预计可去除约 94% 的
低频正弦幅值；但该修正尚未进入当前版本，必须在实施后重新仿真并更新正式验收指标。
