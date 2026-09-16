# MA6224 Independent Quadrotor EKF + NMPC

这是一个不依赖 PX4 飞控或开源四旋翼控制模型的 ROS 2/Gazebo 项目骨架。飞行器动力学、19 维 EKF 和约束 NMPC 均根据课程需求独立实现；Gazebo 只提供通用刚体物理、可视化和原始传感器。

## 固定的设计边界

- 世界坐标系：NED；机体坐标系：FRD。
- 控制器状态：`p(3) + v(3) + q(4) + omega(3) = 13`。
- EKF 名义状态：上述 13 维加 `b_a(3) + b_g(3) = 19`。
- bias 只保留在 `QuadrotorEkf` 内部，不包含在 `/drone/state_estimate` 中。
- 输入为四个物理旋翼推力 `[T0, T1, T2, T3]`，单位 N。
- Gazebo 插件直接实现课程给出的 `T_B` 和 `tau_B` 映射，不调用 PX4 或 Gazebo `MulticopterMotorModel`。
- 平移动力学包含需求文档遗漏的 `1/m` 项。
- 在 NED/FRD 中使用 `T_B = [0, 0, -sum(T_i)]`，因此 11.772 N 可与重力平衡。

## 软件结构

| Package | 作用 |
|---|---|
| `drone_interfaces` | `State13`、`RotorThrusts`、GNSS fix、轨迹点和求解器统计消息 |
| `drone_description` | 仅使用课程质量、惯量和臂长构造的 SDF 模型 |
| `drone_gazebo` | Gazebo world、自编 rotor-wrench 插件、ROS/Gazebo bridge 和总启动文件 |
| `drone_gnc` | 独立动力学、坐标转换、传感器噪声、19 维 EKF、分阶段任务、NMPC 和日志 |

关键文件：

- `src/drone_gnc/drone_gnc/dynamics.py`：13 维连续模型、旋翼映射、RK4。
- `src/drone_gnc/drone_gnc/ekf.py`：19 维名义 EKF，向控制器只暴露 13 维。
- `src/drone_gnc/drone_gnc/nmpc.py`：CasADi/IPOPT multiple-shooting NMPC。
- `src/drone_gazebo/src/rotor_wrench_system.cpp`：自编 Gazebo 力/力矩插件。
- `src/drone_gnc/config/project.yaml`：全部课程参数、噪声和控制权重。
- `scripts/generate_plots.py`：轨迹、EKF ±2 sigma、推力和求解时间图。

## 为什么使用容器

当前宿主机是 Ubuntu 26.04，而 ROS 2 Jazzy 的官方平台是 Ubuntu 24.04。`docker/Dockerfile` 固定为 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic 和 Python/CasADi，避免在宿主机上混装不受支持的软件包。

## 第一次配置

安装 Docker 并构建开发镜像：

```bash
./scripts/bootstrap_host.sh
```

如果脚本刚刚安装 Docker 或配置了用户组，它会要求重新登录。重新登录后再次运行：

```bash
./scripts/bootstrap_host.sh
```

也可以直接构建镜像：

```bash
docker compose build
```

Gazebo GUI 使用 X11。启动前只允许当前本机用户连接显示服务：

```bash
xhost +SI:localuser:"$(id -un)"
```

## 构建工作区

进入容器：

```bash
docker compose run --rm drone-dev
```

在容器内：

```bash
./scripts/build_workspace.sh
source install/setup.bash
```

## 测试

纯算法测试：

```bash
pytest -q
```

ROS package 测试：

```bash
python3 -m colcon test --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

顶层动力学 smoke test：

```bash
PYTHONPATH=src/drone_gnc python3 scripts/main_simulation.py
```

## 启动 Gazebo + GNC

在容器内运行：

```bash
./scripts/run_simulation.sh
```

启动文件会运行：

1. Gazebo Harmonic 和课程 SDF 四旋翼；
2. IMU、NavSat、odometry 的 `ros_gz_bridge`；
3. 四推力 ROS-to-Gazebo adapter；
4. 规定噪声和 bias random walk 的传感器节点；
5. 19 维 EKF；
6. 地面起飞至八字起点、原地 yaw 对准和 3D 八字参考；
7. CasADi/IPOPT NMPC、力矩约束和几何姿态安全恢复；
8. Gazebo 实时轨迹标记和 CSV logger。

Gazebo 中默认显示两条轨迹带：蓝色粗带为飞行器实际轨迹，黄色细带为参考轨迹。
可视化节点按 10 Hz 采样、5 Hz 刷新并最多保留 1500 个点，避免轨迹无限增长。
如需关闭可视化以测试纯控制性能：

```bash
ros2 launch drone_gazebo simulation.launch.py show_trajectory:=false
```

默认任务时间线（NED 高度为负值）：

- `0–5 s`：从地面垂直起飞至八字起点 `(0,0,z0+Az)=(0,0,-2.0 m)`；
- `5–10 s`：保持起点位置，原地对准初始速度方向 `yaw=53.13°` 并稳定角速度；
- `10 s` 以后：不使用位置过渡曲线，直接从局部 `t=0` 开始 NMPC 八字跟踪。

项目采用 NED 坐标系，z 越负表示高度越高；轨迹严格采用给定脚本中的
`z=z0+Az*cos(omega*t)`。八字开始瞬间位置和 yaw 连续，但参考速度按原公式
从零切换为 `(Ax*omega, 2*Ay*omega, 0)=(0.75,1.0,0) m/s`。

这些时间、高度和轨迹参数都可在 `src/drone_gnc/config/project.yaml` 中修改。

结果默认写入 `results/flight_log.csv`。生成报告图片：

```bash
python3 scripts/generate_plots.py
```

## 主要 ROS topics

| Topic | 类型 | 使用者 |
|---|---|---|
| `/drone/imu_raw` | `sensor_msgs/Imu` | 仅传感器仿真节点 |
| `/drone/navsat_raw` | `sensor_msgs/NavSatFix` | 仅传感器仿真节点 |
| `/drone/imu` | `sensor_msgs/Imu` | EKF |
| `/drone/gnss/position_ned` | `drone_interfaces/PositionFix` | EKF |
| `/drone/state_estimate` | `drone_interfaces/State13` | NMPC、logger |
| `/drone/reference` | `drone_interfaces/TrajectoryPoint` | NMPC、logger |
| `/drone/rotor_thrusts` | `drone_interfaces/RotorThrusts` | 自编 Gazebo actuator bridge |
| `/drone/ground_truth/state_ned` | `drone_interfaces/State13` | 仅 logger/验证 |
| `/drone/nmpc/stats` | `drone_interfaces/SolverStats` | logger |

控制器和 EKF 不订阅 ground-truth topic。

## acados 可选升级

当前可运行后端是 CasADi + IPOPT，符合项目允许的软件范围。需要 SQP-RTI 时，在容器内执行：

```bash
sudo ./scripts/install_acados.sh /opt/acados
```

该脚本只安装求解器；NMPC 的状态、动力学和约束仍应由本项目代码生成，不能导入 acados 示例中的四旋翼模型。后续可新增 `AcadosNmpcController`，保留当前 `NmpcController.solve()` 接口。

## 调参与验证说明

- 文档没有给出 accelerometer/gyroscope bias random-walk 的谱密度；YAML 中的值是明确标注的初始假设。
- 当前 EKF 使用数值状态转移 Jacobian，便于逐项审查；提交前可推导解析 Jacobian并比较结果与运行时间。
- 当前 NMPC 权重与力矩限制已通过容器内闭环 smoke test，最终课程指标仍应据评分标准继续调参。
- NMPC 超时或姿态越界时会暂时使用同一独立动力学/混控约定下的几何控制恢复，稳定后再交回 NMPC。
- 完整 60 秒试验后必须检查姿态、角速度、四旋翼推力、求解可行性和 ±2 sigma 一致性。
