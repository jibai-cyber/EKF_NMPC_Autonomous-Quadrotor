# EKF-NMPC Autonomous Quadrotor

A reproducible ROS 2 and Gazebo simulation platform for nonlinear state estimation and
constraint-aware trajectory tracking of a quadrotor.

The quadrotor dynamics, 19-state EKF, constrained NMPC, mission generator, actuator mixer,
and safety fallback controllers are implemented directly in this repository. The project does
not call PX4 or Gazebo's ready-made multicopter dynamics or flight-control models. Gazebo is
used only for generic rigid-body physics, collision, raw sensors, and visualization.

The default mission has three phases:

1. vertical takeoff from the ground to the figure-8 start point;
2. in-place yaw alignment with the initial trajectory direction;
3. direct tracking of the three-dimensional figure-8 trajectory.

Gazebo displays both the ground-truth flight trail and the reference trail.

## Quick Start

The commands below assume a Linux host with an X11 desktop, Git, sudo access, and an active
internet connection.

### 1. Clone and enter the repository

```bash
git clone https://github.com/jibai-cyber/EKF_NMPC_Autonomous-Quadrotor.git
cd EKF_NMPC_Autonomous-Quadrotor
```

### 2. Install Docker and build the development image

```bash
./scripts/bootstrap_host.sh
```

If the script adds your account to the `docker` group, log out and back in, return to the
repository, and run the same command again.

### 3. Allow the container to use the X11 display

```bash
xhost +SI:localuser:"$(id -un)"
```

### 4. Enter the development container

Export the host user and graphics-group IDs so files created in the mounted workspace retain
the correct ownership:

```bash
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"
export LOCAL_VIDEO_GID="$(getent group video | cut -d: -f3)"
export LOCAL_RENDER_GID="$(getent group render | cut -d: -f3)"
```

```bash
docker compose run --rm drone-dev
```

### 5. Build, test, and launch inside the container

```bash
./scripts/build_workspace.sh
source install/setup.bash
python3 -m pytest -q
./scripts/run_simulation.sh
```

The Gazebo window should open with the quadrotor on the ground. It takes off to
`(0, 0, -2.0 m)` in NED coordinates, aligns yaw to approximately `53.13 deg`, and then
starts the figure-8.

After closing the container, the optional X11 permission can be revoked on the host:

```bash
xhost -SI:localuser:"$(id -un)"
```

For installation details and troubleshooting, continue with the sections below.

## 1. Tested Software Stack

| Component | Version or requirement |
|---|---|
| Host operating system | Linux; verified on Ubuntu 26.04 |
| Container base | Ubuntu 24.04-based official ROS image |
| ROS | ROS 2 Jazzy |
| Gazebo | Gazebo Harmonic / gz-sim 8.11.0 |
| Python | 3.12.3 |
| Optimizer | CasADi 3.8.0 with IPOPT |
| Build tools | colcon, CMake, and ament |
| Container runtime | Docker Engine with Docker Compose v2 |

ROS 2 Jazzy officially targets Ubuntu 24.04. The container prevents ROS, Gazebo, and Python
dependencies from being mixed with packages from a newer host distribution.

## 2. Design Boundaries and Coordinate Conventions

- World frame: North-East-Down (NED). Increasing altitude makes z more negative.
- Body frame: Forward-Right-Down (FRD).
- Controller state: `p(3) + v(3) + q(4) + omega(3) = 13` states.
- EKF internal state: the 13 vehicle states plus `b_a(3) + b_g(3) = 19` states.
- The six IMU bias states remain internal to the EKF and are not published to the controller.
- Control input: four physical rotor thrusts `[T0, T1, T2, T3]` in newtons.
- Gazebo provides rigid-body integration, collision, raw sensors, and visualization.
- The custom Gazebo plugin maps the four rotor thrusts directly to body force and torque.

The complete notation, derivations, assumptions, and source-code mappings are in the
[mathematical documentation](docs/README.md).

## 3. System Architecture

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

The EKF and controller do not subscribe to ground truth. Ground truth is used only for
logging, validation, and trajectory visualization.

## 4. Repository Layout

| Path | Purpose |
|---|---|
| `docker/` | ROS 2 Jazzy, Gazebo Harmonic, and Python development image |
| `scripts/` | Host setup, workspace build, launch, plotting, and optional acados installation |
| `src/drone_interfaces/` | Custom ROS message definitions |
| `src/drone_description/` | Independent SDF quadrotor model |
| `src/drone_gazebo/` | Gazebo world, custom wrench plugin, bridge, and trajectory visualization |
| `src/drone_gnc/` | Dynamics, frames, sensors, EKF, trajectory generation, NMPC, and logging |
| `docs/` | Mathematical derivations and formula-to-code traceability |
| `results/` | Generated CSV files and plots; ignored by Git |

## 5. Host Preparation

### 5.1 Platform requirements

A Linux workstation with an X11 desktop is recommended. The current Compose configuration
uses:

- the host X11 socket at `/tmp/.X11-unix`;
- direct rendering through `/dev/dri`;
- host networking;
- the host `video` and `render` groups.

macOS, Windows, Wayland-only systems, and headless servers require additional display
forwarding or Compose changes and are not part of the verified configuration.

### 5.2 Automated Docker setup

From the repository root:

```bash
./scripts/bootstrap_host.sh
```

When Docker is absent, the script installs `docker.io` and `docker-compose-v2`, adds the
current user to the `docker` group, and builds the image. Group membership normally requires
logging out and back in.

The `docker` group grants substantial access to the host. Add only trusted accounts.

### 5.3 Manual Docker checks

```bash
docker --version
docker compose version
docker info
```

`docker info` must complete without a socket permission error.

### 5.4 X11 permission

Before launching Gazebo:

```bash
xhost +SI:localuser:"$(id -un)"
```

Revoke the permission after the simulation:

```bash
xhost -SI:localuser:"$(id -un)"
```

### 5.5 Graphics devices and group IDs

```bash
echo "$DISPLAY"
ls -l /dev/dri
getent group video
getent group render
```

Compose defaults to video GID 44 and render GID 990. Override them when the host uses
different IDs:

```bash
export LOCAL_UID="$(id -u)"
export LOCAL_GID="$(id -g)"
export LOCAL_VIDEO_GID="$(getent group video | cut -d: -f3)"
export LOCAL_RENDER_GID="$(getent group render | cut -d: -f3)"
```

## 6. Build the Docker Image

The bootstrap script builds the image automatically. To build it manually:

```bash
docker compose build
```

The default image name is:

```text
ma6224-drone:jazzy-harmonic
```

The first build downloads ROS and Python dependencies and may take several minutes.

## 7. Enter the Development Container

```bash
docker compose run --rm drone-dev
```

The repository is mounted at `/workspace`. The entrypoint automatically:

- sources `/opt/ros/jazzy/setup.bash`;
- activates `/opt/drone_venv`;
- sources `/workspace/install/setup.bash` when it already exists.

Unless explicitly stated otherwise, all remaining commands are run inside the container from
`/workspace`.

## 8. Build the ROS Workspace

```bash
./scripts/build_workspace.sh
source install/setup.bash
```

The script resolves ROS dependencies and builds these packages in `RelWithDebInfo` mode:

```text
drone_interfaces
drone_description
drone_gnc
drone_gazebo
```

Rebuild and source `install/setup.bash` after changing C++, ROS messages, launch files, or
package metadata.

## 9. Tests

### 9.1 Python algorithm tests

Use the active virtual-environment interpreter:

```bash
python3 -m pytest -q
```

Do not invoke `pytest -q` directly. The container's `/usr/bin/pytest` uses the system Python
and may not see CasADi from the virtual environment.

The current baseline is 15 passed tests with no skips.

### 9.2 ROS package tests

```bash
python3 -m colcon test --event-handlers console_direct+
python3 -m colcon test-result --verbose
```

The expected result is 15 tests, 0 errors, 0 failures, and 0 skipped.

### 9.3 Dynamics smoke test

```bash
PYTHONPATH=src/drone_gnc python3 scripts/main_simulation.py
```

This test does not start Gazebo. It checks the hover equilibrium using the independently
implemented RK4 dynamics.

## 10. Run the Simulation

```bash
./scripts/run_simulation.sh
```

Equivalent ROS command:

```bash
ros2 launch drone_gazebo simulation.launch.py
```

The launch file starts:

1. Gazebo Harmonic and the course quadrotor SDF;
2. ROS/Gazebo bridges for IMU, NavSat, and odometry;
3. the custom rotor-wrench plugin and actuator bridge;
4. sensor noise and bias random-walk simulation;
5. the 19-state EKF;
6. the phased mission-reference generator;
7. CasADi/IPOPT NMPC and geometric fallback control;
8. Gazebo trajectory trails;
9. the CSV logger.

### 10.1 Default mission timeline

| Simulation time | Phase | Reference behavior |
|---:|---|---|
| 0-5 s | `takeoff` | Rise from the ground to `(0, 0, -2.0 m)` |
| 5-10 s | `yaw_align` | Hold position and align yaw to approximately 53.13 deg |
| 10-70 s | `figure8` | Track the 60 s figure-8 from local trajectory time `t=0` |
| after 70 s | `figure8` hold | Hold the final reference sample at local trajectory time `t=60 s` |

Position and yaw are continuous when the figure-8 starts. The horizontal reference velocity
changes directly from zero to `(0.75, 1.0, 0) m/s`, and the vertical reference acceleration
changes from zero to `-0.03125 m/s^2`. These discontinuities are intentional: the original
figure-8 begins directly, without an additional transition curve.

### 10.2 Gazebo trajectory trails

- Blue thick trail: ground-truth flight path.
- Yellow thin trail: reference path.
- Default sampling rate: 10 Hz.
- Default marker refresh rate: 5 Hz.
- Maximum retained points: 1500.

Disable the visualization when measuring controller-only performance:

```bash
ros2 launch drone_gazebo simulation.launch.py show_trajectory:=false
```

## 11. Configuration

Runtime parameters are centralized in:

```text
src/drone_gnc/config/project.yaml
```

The file contains:

- IMU/GNSS noise and bias random walks;
- takeoff, yaw-alignment, and figure-8 parameters;
- mass, inertia, arm lengths, and rotor limits;
- NMPC horizon, time step, weights, torque limits, and attitude constraints;
- solver iteration and CPU-time limits;
- logger and trajectory-visualizer settings.

When changing physical parameters, keep these three locations consistent:

1. `src/drone_gnc/config/project.yaml`
2. `src/drone_gnc/drone_gnc/dynamics.py`
3. `src/drone_description/models/course_quadrotor/model.sdf`

See [parameter and implementation traceability](docs/06_parameter_traceability.md) for a
field-by-field map.

## 12. Results and Plots

The default flight log is:

```text
results/flight_log.csv
```

Generate report plots:

```bash
python3 scripts/generate_plots.py
```

Outputs:

- `results/plots/trajectory_3d.png`
- `results/plots/ekf_position_error_2sigma.png`
- `results/plots/rotor_thrusts.png`
- `results/plots/solver_time.png`

The logger overwrites `flight_log.csv` when a new simulation starts. Copy or rename a result
before launching the next experiment when it must be retained.

## 13. Main ROS Topics

| Topic | Message type | Purpose |
|---|---|---|
| `/drone/imu_raw` | `sensor_msgs/Imu` | Raw Gazebo IMU |
| `/drone/navsat_raw` | `sensor_msgs/NavSatFix` | Raw Gazebo NavSat |
| `/drone/imu` | `sensor_msgs/Imu` | IMU with configured noise and bias |
| `/drone/gnss/position_ned` | `drone_interfaces/PositionFix` | Local NED position measurement |
| `/drone/state_estimate` | `drone_interfaces/State13` | Public 13-state EKF estimate |
| `/drone/reference` | `drone_interfaces/TrajectoryPoint` | Mission reference |
| `/drone/rotor_thrusts` | `drone_interfaces/RotorThrusts` | Four rotor-thrust commands |
| `/drone/ground_truth/state_ned` | `drone_interfaces/State13` | Logging and visualization only |
| `/drone/nmpc/stats` | `drone_interfaces/SolverStats` | Solver timing, status, and cost |

## 14. Troubleshooting

### Docker daemon permission denied

Confirm group membership and log in again after a group change:

```bash
groups
docker info
```

### Gazebo window does not open

Check the display, X11 socket, and permission:

```bash
echo "$DISPLAY"
ls -l /tmp/.X11-unix
xhost +SI:localuser:"$(id -un)"
```

Also verify that the container can access `/dev/dri`.

### No trajectory trails appear in Gazebo

Rebuild the workspace, stop all old Gazebo processes, and relaunch:

```bash
./scripts/build_workspace.sh
source install/setup.bash
./scripts/run_simulation.sh
```

The launch log should contain `Gazebo trajectory markers enabled`.

### NumPy imports successfully but CasADi does not

Verify the active interpreter:

```bash
which python3
python3 -c "import casadi; print(casadi.__version__)"
```

The interpreter should be `/opt/drone_venv/bin/python3`.

### Changes do not affect runtime behavior

Rebuild and source the overlay:

```bash
./scripts/build_workspace.sh
source install/setup.bash
```

Restart Gazebo after changing C++, launch files, or installed package data.

## 15. Optional acados Support

CasADi with IPOPT is the default and verified NMPC backend. To install acados:

```bash
sudo ./scripts/install_acados.sh /opt/acados
```

The script installs only the solver. Any acados backend must still generate the state,
dynamics, and constraints from this repository. It must not import a sample quadrotor
dynamics model.

## 16. Assumptions and Known Limitations

- The specification defines accelerometer and gyroscope bias random walks but does not give
  their spectral densities. The YAML values are documented engineering assumptions.
- Absolute yaw is unobservable during hover with only IMU and GNSS position measurements;
  the simulation assumes a known initial yaw.
- The current EKF uses a nominal quaternion inside a direct 19-dimensional covariance and a
  numerical process Jacobian. It is not a multiplicative error-state quaternion EKF.
- Mission phases switch at fixed times rather than waiting for verified position, velocity,
  and attitude convergence.
- NMPC failure or an excessive attitude/rate condition activates the independently
  implemented geometric fallback controller.
- The post-solve thrust slew-rate projection is not included in the NMPC prediction model.
- Python dependencies are not pinned to exact patch versions. A release should add a lock
  file and a container-image digest.

## 17. Mathematical Documentation

- [Documentation index and notation](docs/README.md)
- [Frames, state, and quaternions](docs/01_frames_and_state.md)
- [Quadrotor dynamics and actuator mapping](docs/02_quadrotor_dynamics.md)
- [Sensor models and EKF](docs/03_sensor_and_ekf.md)
- [Mission and three-dimensional figure-8](docs/04_trajectory_generation.md)
- [NMPC and safety fallback control](docs/05_nmpc_and_fallback.md)
- [Parameter and implementation traceability](docs/06_parameter_traceability.md)

The mathematical documents are currently written in Chinese to preserve the terminology used
during model development. All equations use the same NED/FRD conventions as the source code.

## 18. License

This project is distributed under the BSD 3-Clause License. See [LICENSE](LICENSE).
