"""Mission reference: ground takeoff, yaw alignment, then the specified figure-8."""

from dataclasses import dataclass, field

import numpy as np


@dataclass(frozen=True)
class TrajectoryParameters:
    amplitude_x_m: float = 3.0
    amplitude_y_m: float = 2.0
    amplitude_z_m: float = 0.5
    base_height_ned_m: float = -2.5
    omega_radps: float = 0.25
    duration_s: float = 60.0


@dataclass(frozen=True)
class MissionParameters:
    initial_height_ned_m: float = -0.035
    takeoff_duration_s: float = 5.0
    yaw_alignment_duration_s: float = 5.0
    figure8_entry_ramp_duration_s: float = 3.0
    trajectory: TrajectoryParameters = field(default_factory=TrajectoryParameters)


def _state_reference(
    time_s: float,
    position: np.ndarray,
    velocity: np.ndarray,
    acceleration: np.ndarray,
    yaw: float,
    yaw_rate: float,
    phase: str,
) -> dict[str, np.ndarray | float | str]:
    quaternion = np.array([np.cos(0.5 * yaw), 0.0, 0.0, np.sin(0.5 * yaw)])
    state = np.concatenate((position, velocity, quaternion, [0.0, 0.0, yaw_rate]))
    return {
        "time_s": float(time_s),
        "position": position,
        "velocity": velocity,
        "acceleration": acceleration,
        "yaw": float(yaw),
        "state": state,
        "phase": phase,
    }


def _quintic_segment(
    start_position,
    start_velocity,
    start_acceleration,
    end_position,
    end_velocity,
    end_acceleration,
    duration_s: float,
    time_s: float,
):
    """Evaluate a quintic satisfying position, velocity and acceleration endpoints."""
    p0 = np.asarray(start_position, dtype=float)
    v0 = np.asarray(start_velocity, dtype=float)
    acc0 = np.asarray(start_acceleration, dtype=float)
    p1 = np.asarray(end_position, dtype=float)
    v1 = np.asarray(end_velocity, dtype=float)
    acc1 = np.asarray(end_acceleration, dtype=float)
    duration = float(duration_s)
    time = float(np.clip(time_s, 0.0, duration))

    coefficient_0 = p0
    coefficient_1 = v0
    coefficient_2 = 0.5 * acc0
    coefficient_3 = (
        20.0 * (p1 - p0)
        - (8.0 * v1 + 12.0 * v0) * duration
        - (3.0 * acc0 - acc1) * duration**2
    ) / (2.0 * duration**3)
    coefficient_4 = (
        30.0 * (p0 - p1)
        + (14.0 * v1 + 16.0 * v0) * duration
        + (3.0 * acc0 - 2.0 * acc1) * duration**2
    ) / (2.0 * duration**4)
    coefficient_5 = (
        12.0 * (p1 - p0)
        - (6.0 * v1 + 6.0 * v0) * duration
        - (acc0 - acc1) * duration**2
    ) / (2.0 * duration**5)

    position = (
        coefficient_0
        + coefficient_1 * time
        + coefficient_2 * time**2
        + coefficient_3 * time**3
        + coefficient_4 * time**4
        + coefficient_5 * time**5
    )
    velocity = (
        coefficient_1
        + 2.0 * coefficient_2 * time
        + 3.0 * coefficient_3 * time**2
        + 4.0 * coefficient_4 * time**3
        + 5.0 * coefficient_5 * time**4
    )
    acceleration = (
        2.0 * coefficient_2
        + 6.0 * coefficient_3 * time
        + 12.0 * coefficient_4 * time**2
        + 20.0 * coefficient_5 * time**3
    )
    return position, velocity, acceleration


def _lemniscate_reference_with_time_scaling(
    output_time_s: float,
    phase_time_s: float,
    phase_rate: float,
    phase_acceleration: float,
    parameters: TrajectoryParameters,
) -> dict[str, np.ndarray | float]:
    """Evaluate the analytic path with derivatives from an arbitrary time scaling."""
    t = float(np.clip(phase_time_s, 0.0, parameters.duration_s))
    w = parameters.omega_radps
    ax = parameters.amplitude_x_m
    ay = parameters.amplitude_y_m
    az = parameters.amplitude_z_m

    position = np.array(
        [
            ax * np.sin(w * t),
            ay * np.sin(2.0 * w * t),
            parameters.base_height_ned_m + az * np.cos(w * t),
        ]
    )
    path_velocity = np.array(
        [
            ax * w * np.cos(w * t),
            2.0 * ay * w * np.cos(2.0 * w * t),
            -az * w * np.sin(w * t),
        ]
    )
    path_acceleration = np.array(
        [
            -ax * w * w * np.sin(w * t),
            -4.0 * ay * w * w * np.sin(2.0 * w * t),
            -az * w * w * np.cos(w * t),
        ]
    )
    velocity = path_velocity * phase_rate
    acceleration = (
        path_acceleration * phase_rate**2
        + path_velocity * phase_acceleration
    )

    # Heading follows the geometric tangent even when the time scale starts at rest.
    yaw = float(np.arctan2(path_velocity[1], path_velocity[0]))
    denominator = path_velocity[0] ** 2 + path_velocity[1] ** 2
    yaw_rate = 0.0
    if denominator > 1e-10:
        yaw_rate = float(
            (
                path_velocity[0] * path_acceleration[1]
                - path_velocity[1] * path_acceleration[0]
            )
            / denominator
            * phase_rate
        )

    result = _state_reference(
        output_time_s, position, velocity, acceleration, yaw, yaw_rate, "figure8"
    )
    result["path_time_s"] = t
    result["path_time_rate"] = float(phase_rate)
    return result


def lemniscate_reference(
    time_s: float, parameters: TrajectoryParameters = TrajectoryParameters()
) -> dict[str, np.ndarray | float]:
    t = float(np.clip(time_s, 0.0, parameters.duration_s))
    return _lemniscate_reference_with_time_scaling(t, t, 1.0, 0.0, parameters)


def smooth_lemniscate_reference(
    time_s: float,
    parameters: TrajectoryParameters = TrajectoryParameters(),
    ramp_duration_s: float = 3.0,
) -> dict[str, np.ndarray | float]:
    """Enter the same analytic path with continuous velocity and acceleration.

    A half-cosine phase-rate ramp changes only the path timing.  The spatial
    lemniscate is unchanged and no Cartesian transition curve is fitted.
    """
    elapsed = max(float(time_s), 0.0)
    ramp = float(ramp_duration_s)
    if ramp <= 0.0:
        return lemniscate_reference(elapsed, parameters)
    if elapsed < ramp:
        angle = np.pi * elapsed / ramp
        phase_time = 0.5 * (elapsed - ramp * np.sin(angle) / np.pi)
        phase_rate = 0.5 * (1.0 - np.cos(angle))
        phase_acceleration = 0.5 * np.pi * np.sin(angle) / ramp
    else:
        phase_time = elapsed - 0.5 * ramp
        phase_rate = 1.0
        phase_acceleration = 0.0
    if phase_time >= parameters.duration_s:
        phase_time = parameters.duration_s
        phase_rate = 0.0
        phase_acceleration = 0.0
    return _lemniscate_reference_with_time_scaling(
        elapsed,
        phase_time,
        phase_rate,
        phase_acceleration,
        parameters,
    )


def mission_reference(
    time_s: float, parameters: MissionParameters = MissionParameters()
) -> dict[str, np.ndarray | float | str]:
    """Take off to the figure-8 start, align yaw, then start it at local t=0."""
    time = max(float(time_s), 0.0)
    zero = np.zeros(3)
    initial_position = np.array([0.0, 0.0, parameters.initial_height_ned_m])
    first_figure8 = lemniscate_reference(0.0, parameters.trajectory)
    figure8_start_position = np.asarray(first_figure8["position"])
    figure8_start_yaw = float(first_figure8["yaw"])

    if time < parameters.takeoff_duration_s:
        position, velocity, acceleration = _quintic_segment(
            initial_position,
            zero,
            zero,
            figure8_start_position,
            zero,
            zero,
            parameters.takeoff_duration_s,
            time,
        )
        return _state_reference(
            time, position, velocity, acceleration, 0.0, 0.0, "takeoff"
        )

    figure8_start_s = parameters.takeoff_duration_s + parameters.yaw_alignment_duration_s
    if time < figure8_start_s:
        alignment_time = time - parameters.takeoff_duration_s
        yaw, yaw_rate, yaw_acceleration = _quintic_segment(
            0.0,
            0.0,
            0.0,
            figure8_start_yaw,
            0.0,
            0.0,
            parameters.yaw_alignment_duration_s,
            alignment_time,
        )
        reference = _state_reference(
            time,
            figure8_start_position,
            zero,
            zero,
            float(yaw),
            float(yaw_rate),
            "yaw_align",
        )
        reference["yaw_acceleration"] = float(yaw_acceleration)
        return reference

    result = smooth_lemniscate_reference(
        time - figure8_start_s,
        parameters.trajectory,
        parameters.figure8_entry_ramp_duration_s,
    )
    result["time_s"] = time
    return result
