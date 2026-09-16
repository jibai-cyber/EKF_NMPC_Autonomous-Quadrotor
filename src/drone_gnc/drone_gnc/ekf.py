"""19-state EKF: 13 public vehicle states plus six internal IMU biases."""

from dataclasses import dataclass

import numpy as np

from .dynamics import normalize_quaternion, quaternion_multiply, quaternion_to_rotation


@dataclass(frozen=True)
class EkfNoise:
    accel_std_mps2: float = 0.08
    gyro_std_radps: float = 0.015
    position_std_m: float = 0.02
    accel_bias_rw_std: float = 0.002
    gyro_bias_rw_std: float = 0.0002


class QuadrotorEkf:
    """A compact EKF suitable for extension during controller/observer tuning.

    Nominal state ordering is [p(3), v(3), q(4), omega(3), b_a(3), b_g(3)].
    Only the first 13 components are exposed to the controller.
    """

    state_size = 19
    public_state_size = 13

    def __init__(self, noise: EkfNoise = EkfNoise(), gravity_mps2: float = 9.81):
        self.noise = noise
        self.gravity = float(gravity_mps2)
        self.state = np.zeros(self.state_size)
        self.state[6] = 1.0
        self.covariance = np.eye(self.state_size) * 0.1
        self.covariance[6:10, 6:10] = np.eye(4) * 0.01
        self.covariance[13:16, 13:16] = np.eye(3) * 0.02
        # The simulated gyro bias starts at zero and follows only the specified
        # small random walk.  A large initial yaw-bias covariance lets GNSS
        # position updates inject an unobservable fictitious yaw bias in hover.
        self.covariance[16:19, 16:19] = np.eye(3) * 1e-6
        self.initialized = False

    def initialize_position(self, position_ned_m: np.ndarray) -> None:
        self.state[0:3] = np.asarray(position_ned_m, dtype=float)
        self.initialized = True

    def _process_step(
        self, state: np.ndarray, accel_frd: np.ndarray, gyro_frd: np.ndarray, dt_s: float
    ) -> np.ndarray:
        result = state.copy()
        position = state[0:3]
        velocity = state[3:6]
        quaternion = normalize_quaternion(state[6:10])
        accel_bias = state[13:16]
        gyro_bias = state[16:19]

        corrected_accel = np.asarray(accel_frd) - accel_bias
        corrected_gyro = np.asarray(gyro_frd) - gyro_bias
        acceleration_ned = (
            quaternion_to_rotation(quaternion) @ corrected_accel
            + np.array([0.0, 0.0, self.gravity])
        )

        result[0:3] = position + velocity * dt_s + 0.5 * acceleration_ned * dt_s**2
        result[3:6] = velocity + acceleration_ned * dt_s
        delta_quaternion = np.concatenate(([1.0], 0.5 * corrected_gyro * dt_s))
        result[6:10] = normalize_quaternion(
            quaternion_multiply(quaternion, delta_quaternion)
        )
        # Angular velocity is a public state; the bias-corrected gyro is its high-rate estimate.
        result[10:13] = corrected_gyro
        # Bias states follow random walks, so their deterministic derivative is zero.
        return result

    def predict(
        self, accel_frd_mps2: np.ndarray, gyro_frd_radps: np.ndarray, dt_s: float
    ) -> None:
        if not self.initialized:
            return
        dt_s = float(np.clip(dt_s, 1e-4, 0.1))
        previous = self.state.copy()
        self.state = self._process_step(previous, accel_frd_mps2, gyro_frd_radps, dt_s)

        # Numerical Jacobian keeps the implementation auditable against the supplied equations.
        epsilon = 1e-6
        transition = np.zeros((self.state_size, self.state_size))
        for index in range(self.state_size):
            perturbation = np.zeros(self.state_size)
            perturbation[index] = epsilon
            plus = self._process_step(
                previous + perturbation, accel_frd_mps2, gyro_frd_radps, dt_s
            )
            minus = self._process_step(
                previous - perturbation, accel_frd_mps2, gyro_frd_radps, dt_s
            )
            transition[:, index] = (plus - minus) / (2.0 * epsilon)

        process_diagonal = np.zeros(self.state_size)
        process_diagonal[0:3] = 1e-8
        process_diagonal[3:6] = self.noise.accel_std_mps2**2
        process_diagonal[6:10] = self.noise.gyro_std_radps**2 * 0.25
        process_diagonal[10:13] = self.noise.gyro_std_radps**2
        process_diagonal[13:16] = self.noise.accel_bias_rw_std**2
        process_diagonal[16:19] = self.noise.gyro_bias_rw_std**2
        process_noise = np.diag(process_diagonal) * dt_s
        self.covariance = transition @ self.covariance @ transition.T + process_noise
        self._stabilize_covariance()

    def update_position(self, position_ned_m: np.ndarray) -> None:
        measurement = np.asarray(position_ned_m, dtype=float)
        if not self.initialized:
            self.initialize_position(measurement)
            return
        observation = np.zeros((3, self.state_size))
        observation[:, 0:3] = np.eye(3)
        measurement_noise = np.eye(3) * self.noise.position_std_m**2
        self._linear_update(measurement, observation, measurement_noise)

    def _linear_update(
        self, measurement: np.ndarray, observation: np.ndarray, measurement_noise: np.ndarray
    ) -> None:
        innovation = measurement - observation @ self.state
        innovation_covariance = (
            observation @ self.covariance @ observation.T + measurement_noise
        )
        gain = np.linalg.solve(
            innovation_covariance.T, (self.covariance @ observation.T).T
        ).T
        self.state += gain @ innovation
        identity = np.eye(self.state_size)
        correction = identity - gain @ observation
        # Joseph form maintains symmetry and positive semi-definiteness.
        self.covariance = (
            correction @ self.covariance @ correction.T
            + gain @ measurement_noise @ gain.T
        )
        self.state[6:10] = normalize_quaternion(self.state[6:10])
        self._stabilize_covariance()

    def _stabilize_covariance(self) -> None:
        self.covariance = 0.5 * (self.covariance + self.covariance.T)
        minimum_eigenvalue = float(np.min(np.linalg.eigvalsh(self.covariance)))
        if minimum_eigenvalue < 1e-12:
            self.covariance += np.eye(self.state_size) * (1e-12 - minimum_eigenvalue)

    @property
    def public_state(self) -> np.ndarray:
        output = self.state[: self.public_state_size].copy()
        output[6:10] = normalize_quaternion(output[6:10])
        return output

    @property
    def public_covariance(self) -> np.ndarray:
        return self.covariance[: self.public_state_size, : self.public_state_size].copy()

    @property
    def internal_biases(self) -> tuple[np.ndarray, np.ndarray]:
        """Internal-only bias estimates; never included in State13 messages."""
        return self.state[13:16].copy(), self.state[16:19].copy()
