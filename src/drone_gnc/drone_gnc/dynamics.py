"""Course quadrotor dynamics implemented directly from project.pdf.

Conventions
-----------
World: NED (x north, y east, z down)
Body: FRD (x forward, y right, z down)
Quaternion: Hamilton [w, x, y, z], rotating body vectors into world.
State: [p_NED(3), v_NED(3), q_WB(4), omega_FRD(3)].
Input: per-rotor thrusts [T0, T1, T2, T3] in newtons.
"""

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class VehicleParameters:
    mass_kg: float = 1.20
    dx_m: float = 0.225
    dy_m: float = 0.225
    jxx_kgm2: float = 1.25e-2
    jyy_kgm2: float = 1.25e-2
    jzz_kgm2: float = 2.20e-2
    moment_ratio_m: float = 0.015
    gravity_mps2: float = 9.81
    rotor_min_thrust_n: float = 0.2
    rotor_max_thrust_n: float = 5.5

    @property
    def inertia(self) -> np.ndarray:
        return np.diag([self.jxx_kgm2, self.jyy_kgm2, self.jzz_kgm2])

    @property
    def hover_thrust_per_rotor_n(self) -> float:
        return self.mass_kg * self.gravity_mps2 / 4.0


def normalize_quaternion(quaternion: np.ndarray) -> np.ndarray:
    quaternion = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("Quaternion norm is zero")
    result = quaternion / norm
    # q and -q encode the same attitude; a fixed hemisphere avoids discontinuities.
    return result if result[0] >= 0.0 else -result


def quaternion_multiply(left: np.ndarray, right: np.ndarray) -> np.ndarray:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    return np.array(
        [
            lw * rw - lx * rx - ly * ry - lz * rz,
            lw * rx + lx * rw + ly * rz - lz * ry,
            lw * ry - lx * rz + ly * rw + lz * rx,
            lw * rz + lx * ry - ly * rx + lz * rw,
        ],
        dtype=float,
    )


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    """Return the body-to-world rotation matrix."""
    qw, qx, qy, qz = normalize_quaternion(quaternion)
    return np.array(
        [
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
        ],
        dtype=float,
    )


def quaternion_derivative(quaternion: np.ndarray, body_rates: np.ndarray) -> np.ndarray:
    omega_quaternion = np.concatenate(([0.0], np.asarray(body_rates, dtype=float)))
    return 0.5 * quaternion_multiply(quaternion, omega_quaternion)


def rotor_wrench_frd(
    thrusts_n: np.ndarray, parameters: VehicleParameters
) -> tuple[np.ndarray, np.ndarray]:
    """Map four rotor thrusts to body force and torque using the supplied equations."""
    t0, t1, t2, t3 = np.asarray(thrusts_n, dtype=float)
    total = t0 + t1 + t2 + t3
    # In body FRD, lift points along -z. This sign makes the documented
    # 11.77 N hover thrust balance +g in world NED.
    force_body = np.array([0.0, 0.0, -total], dtype=float)
    torque_body = np.array(
        [
            parameters.dy_m * (-t0 - t1 + t2 + t3),
            parameters.dx_m * (-t0 + t1 + t2 - t3),
            parameters.moment_ratio_m * (-t0 + t1 - t2 + t3),
        ],
        dtype=float,
    )
    return force_body, torque_body


def thrusts_from_wrench_frd(
    total_thrust_n: float,
    torque_body_nm: np.ndarray,
    parameters: VehicleParameters,
) -> np.ndarray:
    """Invert the documented X-quadrotor mixer for a total thrust and body torque."""
    tau_x, tau_y, tau_z = np.asarray(torque_body_nm, dtype=float)
    roll_term = tau_x / parameters.dy_m
    pitch_term = tau_y / parameters.dx_m
    yaw_term = tau_z / parameters.moment_ratio_m
    return 0.25 * np.array(
        [
            total_thrust_n - roll_term - pitch_term - yaw_term,
            total_thrust_n - roll_term + pitch_term + yaw_term,
            total_thrust_n + roll_term + pitch_term - yaw_term,
            total_thrust_n + roll_term - pitch_term + yaw_term,
        ]
    )


def condition_rotor_thrusts(
    requested_thrusts_n: np.ndarray,
    previous_thrusts_n: np.ndarray,
    dt_s: float,
    parameters: VehicleParameters,
    max_roll_pitch_torque_nm: float,
    max_yaw_torque_nm: float,
    max_thrust_slew_nps: float,
) -> np.ndarray:
    """Project a rotor command onto the actuator safety envelope."""
    requested = np.clip(
        np.asarray(requested_thrusts_n, dtype=float),
        parameters.rotor_min_thrust_n,
        parameters.rotor_max_thrust_n,
    )
    previous = np.asarray(previous_thrusts_n, dtype=float)
    if requested.shape != (4,) or previous.shape != (4,):
        raise ValueError("Expected four requested and four previous rotor thrusts")
    if dt_s <= 0.0 or max_thrust_slew_nps <= 0.0:
        raise ValueError("Time step and thrust slew rate must be positive")

    total_thrust = float(np.sum(requested))
    _, requested_torque = rotor_wrench_frd(requested, parameters)
    limited_torque = np.clip(
        requested_torque,
        [-max_roll_pitch_torque_nm, -max_roll_pitch_torque_nm, -max_yaw_torque_nm],
        [max_roll_pitch_torque_nm, max_roll_pitch_torque_nm, max_yaw_torque_nm],
    )
    projected = thrusts_from_wrench_frd(total_thrust, limited_torque, parameters)
    projected = np.clip(
        projected,
        parameters.rotor_min_thrust_n,
        parameters.rotor_max_thrust_n,
    )
    # Move along one line segment rather than clipping each rotor separately.
    # Convex interpolation preserves the torque limits at both endpoints.
    delta = projected - previous
    max_delta = float(np.max(np.abs(delta)))
    max_step = max_thrust_slew_nps * dt_s
    interpolation = min(1.0, max_step / max_delta) if max_delta > 0.0 else 1.0
    return previous + interpolation * delta


def continuous_dynamics(
    state: np.ndarray,
    thrusts_n: np.ndarray,
    parameters: VehicleParameters = VehicleParameters(),
) -> np.ndarray:
    state = np.asarray(state, dtype=float)
    if state.shape != (13,):
        raise ValueError(f"Expected a 13-state vector, got {state.shape}")
    thrusts_n = np.asarray(thrusts_n, dtype=float)
    if thrusts_n.shape != (4,):
        raise ValueError(f"Expected four rotor thrusts, got {thrusts_n.shape}")

    velocity = state[3:6]
    quaternion = normalize_quaternion(state[6:10])
    body_rates = state[10:13]
    force_body, torque_body = rotor_wrench_frd(thrusts_n, parameters)

    rotation_wb = quaternion_to_rotation(quaternion)
    gravity_world = np.array([0.0, 0.0, parameters.gravity_mps2])
    acceleration_world = gravity_world + rotation_wb @ force_body / parameters.mass_kg
    angular_acceleration = np.linalg.solve(
        parameters.inertia,
        torque_body - np.cross(body_rates, parameters.inertia @ body_rates),
    )

    return np.concatenate(
        (
            velocity,
            acceleration_world,
            quaternion_derivative(quaternion, body_rates),
            angular_acceleration,
        )
    )


def rk4_step(
    state: np.ndarray,
    thrusts_n: np.ndarray,
    dt_s: float,
    parameters: VehicleParameters = VehicleParameters(),
) -> np.ndarray:
    state = np.asarray(state, dtype=float)
    k1 = continuous_dynamics(state, thrusts_n, parameters)
    k2 = continuous_dynamics(state + 0.5 * dt_s * k1, thrusts_n, parameters)
    k3 = continuous_dynamics(state + 0.5 * dt_s * k2, thrusts_n, parameters)
    k4 = continuous_dynamics(state + dt_s * k3, thrusts_n, parameters)
    result = state + dt_s * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
    result[6:10] = normalize_quaternion(result[6:10])
    return result


def roll_pitch_from_quaternion(quaternion: np.ndarray) -> tuple[float, float]:
    qw, qx, qy, qz = normalize_quaternion(quaternion)
    roll = np.arctan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx * qx + qy * qy))
    pitch_argument = np.clip(2 * (qw * qy - qz * qx), -1.0, 1.0)
    pitch = np.arcsin(pitch_argument)
    return float(roll), float(pitch)
