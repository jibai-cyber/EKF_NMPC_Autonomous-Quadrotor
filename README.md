# MA6224 独立四旋翼 EKF + NMPC 仿真平台

本仓库实现一套可复现的 ROS 2/Gazebo 四旋翼 Guidance, Navigation and Control
(GNC) 仿真环境。飞行器动力学、19 维 EKF、约束 NMPC、任务轨迹和安全回退控制均由
本项目独立实现，不调用 PX4 或 Gazebo 的现成多旋翼动力学/飞控模型。

默认任务由三个阶段组成：从地面垂直起飞到八字起点、原地调整初始 yaw、直接进入
三维八字轨迹。Gazebo 中会同时显示实际轨迹和参考轨迹。

## 1. 当前软件栈

| 组件 | 版本或约束 |
|---|---|
| 宿主机 | Linux；当前验证环境为 Ubuntu 26.04 |
| 容器基础系统 | Ubuntu 24.04 系列 ROS 官方镜像 |
| ROS | ROS 2 Jazzy |
| Gazebo | Gazebo Harmonic / gz-sim 8.11.0 |
| Python | 3.12.3 |
| 优化器 | CasADi 3.8.0 + IPOPT |
| 构建工具 | colcon + CMake + ament |
| 容器工具 | Docker Engine + Docker Compose v2 |

ROS 2 Jazzy 的官方目标系统是 Ubuntu 24.04。使用容器可以避免在较新的宿主系统中
混装 ROS、Gazebo 和 Python 依赖。

## 2. 设计边界与坐标约定

- 世界坐标系采用 NED：x 向北、y 向东、z 向下；高度越高，z 越负。
- 机体坐标系采用 FRD：x 向前、y 向右、z 向下。
- 控制器状态为 13 维：`p(3) + v(3) + q(4) + omega(3)`。
- EKF 内部状态为 19 维：13 维飞行器状态加 `b_a(3) + b_g(3)`。
- 六维 IMU bias 只保留在 EKF 内部，不发布给控制器。
- 控制输入是四个物理旋翼推力 `[T0, T1, T2, T3]`，单位为 N。
- Gazebo 仅提供刚体物理、碰撞、原始传感器和可视化。
- 自编 Gazebo 插件直接将四旋翼推力映射成机体合力和力矩。

完整公式、符号和代码对应关系见 [公式推导索引](docs/README.md)。

## 3. 系统架构

```text
Gazebo rigid body
  ├─ raw IMU 100 Hz ─┐
  ├─ raw NavSat 20 Hz├─> sensor_simulator_node ─> EKF ─> State13 ─┐
  └─ ground truth ───┘                                            │
                                                                  ├─> NMPC / fallback
trajectory_node ────────────────────────> trajectory reference ───┘       │
                                                                          v
Gazebo rotor-wrench plugin <─ actuator_bridge <─ four rotor thrusts ──────┘

ground truth + reference ─> trajectory_visualizer ─> Gazebo Marker Manager
all main signals ─────────> logger_node ────────────> results/flight_log.csv
```

控制器和 EKF 不订阅 ground-truth topic。ground truth 只用于日志、性能验证和轨迹显示。

## 4. 仓库结构

| 路径 | 内容 |
|---|---|
| `docker/` | ROS 2 Jazzy、Gazebo Harmonic 和 Python 依赖镜像 |
| `scripts/` | 宿主机配置、构建、启动、绘图和可选 acados 安装脚本 |
| `src/drone_interfaces/` | 自定义 ROS 消息 |
| `src/drone_description/` | 独立 SDF 四旋翼模型 |
| `src/drone_gazebo/` | Gazebo world、自编力/力矩插件、bridge、轨迹可视化 |
| `src/drone_gnc/` | 动力学、坐标转换、传感器、EKF、轨迹、NMPC 和日志 |
| `docs/` | `drone_gnc` 数学模型、公式推导及代码追溯 |
| `results/` | 运行生成的 CSV 和图片；默认不进入 Git |

## 5. 宿主机准备

### 5.1 平台要求

推荐使用带 X11 桌面环境的 Linux 主机。当前 Docker Compose 配置使用：

- 宿主机 X11 socket：`/tmp/.X11-unix`
- 图形设备：`/dev/dri`
- host network
- `video` 和 `render` 用户组

macOS、Windows 和无桌面服务器需要额外的显示转发或 Compose 配置，本仓库当前没有对这些
平台做正式验证。

### 5.2 自动安装 Docker

在仓库根目录运行：

```bash
./scripts/bootstrap_host.sh
```

脚本会在缺少 Docker 时安装 `docker.io` 和 `docker-compose-v2`，并将当前用户加入
`docker` 用户组。若脚本提示重新登录，请注销并重新登录，然后再次运行同一命令。

注意：`docker` 用户组具有较高的宿主机权限，只应向可信用户开放。

### 5.3 手动检查 Docker

```bash
docker --version
docker compose version
docker info
```

`docker info` 不应返回 socket permission denied。

### 5.4 配置 X11 权限

启动 Gazebo 前执行：

```bash
xhost +SI:localuser:"$(id -un)"
```

仿真结束后可撤销该授权：

```bash
xhost -SI:localuser:"$(id -un)"
```

### 5.5 检查图形设备和用户组

```bash
echo "$DISPLAY"
ls -l /dev/dri
getent group video
getent group render
```

Compose 默认假定 `video` GID 为 44、`render` GID 为 990。如果宿主机不同，可以在启动前
覆盖：

```bash
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"
export LOCAL_VIDEO_GID="$(getent group video | cut -d: -f3)"
export LOCAL_RENDER_GID="$(getent group render | cut -d: -f3)"
```

## 6. 构建 Docker 镜像

自动配置脚本会执行镜像构建。也可以手动运行：

```bash
docker compose build
```

默认镜像名为：

```text
ma6224-drone:jazzy-harmonic
```

首次构建需要从网络下载 ROS 和 Python 依赖，耗时取决于网络和 Docker 缓存状态。

## 7. 进入开发容器

```bash
docker compose run --rm drone-dev
```

容器将仓库挂载到 `/workspace`，并自动执行：

- `source /opt/ros/jazzy/setup.bash`
- 激活 `/opt/drone_venv`
- 若存在则加载 `/workspace/install/setup.bash`

后续没有特别说明的命令都在容器内、`/workspace` 目录执行。

## 8. 构建 ROS 工作空间

```bash
./scripts/build_workspace.sh
source install/setup.bash
```

脚本会运行 `rosdep`，然后以 `RelWithDebInfo` 模式构建四个 package：

```text
drone_interfaces
drone_description
drone_gnc
drone_gazebo
```

每次修改 C++、ROS 消息、launch 文件或 package 配置后都应重新构建并重新加载
`install/setup.bash`。

## 9. 测试

### 9.1 Python 算法测试

必须使用虚拟环境解释器调用 pytest：

```bash
python3 -m pytest -q
```

不要直接使用 `pytest -q`。容器中的 `/usr/bin/pytest` 使用系统 Python，可能找不到虚拟环境
内的 CasADi，从而跳过 NMPC 测试。

### 9.2 ROS package 测试

```bash
python3 -m colcon test --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

当前基线应得到 15 项 Python 测试通过、0 failure、0 error、0 skipped。

### 9.3 顶层动力学 smoke test

```bash
PYTHONPATH=src/drone_gnc python3 scripts/main_simulation.py
```

该测试不启动 Gazebo，用独立 RK4 模型检查悬停推力和平衡状态。

## 10. 启动仿真

```bash
./scripts/run_simulation.sh
```

等价命令为：

```bash
ros2 launch drone_gazebo simulation.launch.py
```

启动内容包括：

1. Gazebo Harmonic 和课程四旋翼 SDF；
2. IMU、NavSat、odometry 的 ROS/Gazebo bridge；
3. 自编 rotor-wrench 插件和 actuator bridge；
4. 传感器噪声与 bias random walk；
5. 19 维 EKF；
6. 分阶段任务参考；
7. CasADi/IPOPT NMPC 和几何回退控制；
8. Gazebo 实时轨迹带；
9. CSV logger。

### 10.1 默认任务时间线

| 仿真时间 | 阶段 | 参考行为 |
|---:|---|---|
| 0-5 s | `takeoff` | 从地面垂直起飞到 `(0, 0, -2.0 m)` |
| 5-10 s | `yaw_align` | 保持位置并将 yaw 调整到约 53.13 deg |
| 10 s 以后 | `figure8` | 从八字局部时间 `t=0` 直接开始跟踪 |

八字开始时位置和 yaw 连续，但参考速度由零直接切换为
`(0.75, 1.0, 0) m/s`；这是当前任务定义的一部分，不是额外的位置过渡曲线。

### 10.2 Gazebo 轨迹显示

- 蓝色轨迹带：实际 ground-truth 轨迹；
- 黄色轨迹带：参考轨迹；
- 默认 10 Hz 采样、5 Hz 刷新；
- 最多保存 1500 个采样点。

关闭轨迹显示：

```bash
ros2 launch drone_gazebo simulation.launch.py show_trajectory:=false
```

## 11. 参数配置

主要参数集中在：

```text
src/drone_gnc/config/project.yaml
```

参数组包括：

- IMU/GNSS 噪声和 bias random walk；
- 起飞、yaw 对准和八字轨迹；
- 质量、惯量、臂长和旋翼范围；
- NMPC horizon、步长、权重、力矩和姿态约束；
- 求解器迭代数与 CPU 时间上限；
- 日志路径和轨迹显示刷新率。

修改物理参数时必须同步核对以下三处：

1. `src/drone_gnc/config/project.yaml`
2. `src/drone_gnc/drone_gnc/dynamics.py`
3. `src/drone_description/models/course_quadrotor/model.sdf`

参数逐项追溯见 [参数与实现追溯](docs/06_parameter_traceability.md)。

## 12. 运行结果与绘图

默认日志：

```text
results/flight_log.csv
```

生成报告图片：

```bash
python3 scripts/generate_plots.py
```

输出包括：

- `results/plots/trajectory_3d.png`
- `results/plots/ekf_position_error_2sigma.png`
- `results/plots/rotor_thrusts.png`
- `results/plots/solver_time.png`

每次启动仿真时 logger 会覆盖现有 `flight_log.csv`。需要保留实验结果时，请在下一次启动前
复制或重命名该文件。

## 13. 主要 ROS topics

| Topic | 消息类型 | 用途 |
|---|---|---|
| `/drone/imu_raw` | `sensor_msgs/Imu` | Gazebo 原始 IMU |
| `/drone/navsat_raw` | `sensor_msgs/NavSatFix` | Gazebo 原始 NavSat |
| `/drone/imu` | `sensor_msgs/Imu` | 加入噪声和 bias 后的 IMU |
| `/drone/gnss/position_ned` | `drone_interfaces/PositionFix` | NED 位置观测 |
| `/drone/state_estimate` | `drone_interfaces/State13` | EKF 对控制器公开的 13 维状态 |
| `/drone/reference` | `drone_interfaces/TrajectoryPoint` | 任务参考 |
| `/drone/rotor_thrusts` | `drone_interfaces/RotorThrusts` | 四旋翼推力命令 |
| `/drone/ground_truth/state_ned` | `drone_interfaces/State13` | 日志和可视化 |
| `/drone/nmpc/stats` | `drone_interfaces/SolverStats` | 求解耗时、状态和成本 |

## 14. 常见问题

### Docker daemon permission denied

确认当前用户属于 `docker` 组，并在加组后重新登录：

```bash
groups
docker info
```

### Gazebo 窗口没有出现

依次检查：

```bash
echo "$DISPLAY"
ls -l /tmp/.X11-unix
xhost +SI:localuser:"$(id -un)"
```

同时确认 Compose 运行时能够访问 `/dev/dri`。

### Gazebo 中没有轨迹带

确认工作空间已重新构建，关闭旧 Gazebo 进程后重新启动：

```bash
./scripts/build_workspace.sh
source install/setup.bash
./scripts/run_simulation.sh
```

启动日志中应出现 `Gazebo trajectory markers enabled`。

### Python 能导入 NumPy，但不能导入 CasADi

确认使用容器内虚拟环境：

```bash
which python3
python3 -c "import casadi; print(casadi.__version__)"
```

正确解释器应位于 `/opt/drone_venv/bin/python3`。

### 修改代码后行为没有变化

重新构建并重新加载 overlay：

```bash
./scripts/build_workspace.sh
source install/setup.bash
```

对于正在运行的 Gazebo/C++ 节点，需要完全停止后重新启动。

## 15. 可选 acados 支持

当前默认、已验证的 NMPC 后端是 CasADi + IPOPT。可选安装 acados：

```bash
sudo ./scripts/install_acados.sh /opt/acados
```

脚本只安装求解器。使用 acados 时仍必须由本项目代码生成状态、动力学和约束，不能导入
acados 示例中的四旋翼动力学模型。

## 16. 当前假设和已知限制

- 需求文档没有提供 accelerometer/gyroscope bias random-walk 谱密度；YAML 中为明确记录的
  工程假设，需要结合实验重新标定。
- 仅使用 IMU 和 GNSS 位置时，悬停状态下绝对 yaw 不可观；当前仿真假定初始 yaw 已知。
- 当前 EKF 使用四元数名义状态和数值 Jacobian，便于审查，但不是误差状态四元数 EKF。
- 任务阶段按固定时间切换，不检查高度、速度和姿态是否已连续稳定。
- NMPC 求解失败或姿态越界时使用独立实现的几何回退控制。
- Dockerfile 中 Python 包尚未锁定精确版本；正式发布时建议增加依赖锁文件和镜像 digest。

## 17. 数学与代码文档

- [文档索引与符号表](docs/README.md)
- [坐标系、状态和四元数](docs/01_frames_and_state.md)
- [四旋翼动力学与执行器映射](docs/02_quadrotor_dynamics.md)
- [传感器模型与 EKF](docs/03_sensor_and_ekf.md)
- [任务与三维八字轨迹](docs/04_trajectory_generation.md)
- [NMPC 与安全回退控制](docs/05_nmpc_and_fallback.md)
- [参数与实现追溯](docs/06_parameter_traceability.md)

## 18. License

本项目使用 BSD 3-Clause License，详见 `LICENSE`。
