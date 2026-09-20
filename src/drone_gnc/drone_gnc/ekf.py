"""13-state EKF aligned with the vehicle state published to the controller."""

from dataclasses import dataclass

import numpy as np

from .dynamics import normalize_quaternion, quaternion_multiply, quaternion_to_rotation


@dataclass(frozen=True)
class EkfNoise:
    accel_std_mps2: float = 0.08
    gyro_std_radps: float = 0.015
    position_std_m: float = 0.02
    model_accel_noise_density_ned: tuple[float, float, float] = (0.50, 0.08, 0.08)


class QuadrotorEkf:
    """A compact EKF suitable for extension during controller/observer tuning.

    State ordering is [p(3), v(3), q(4), omega(3)]. IMU biases are deliberately
    not modeled; the acceptance sensor model assumes both physical biases are zero.
    """

    state_size = 13
    public_state_size = 13

    def __init__(self, noise: EkfNoise = EkfNoise(), gravity_mps2: float = 9.81):
        self.noise = noise
        self.gravity = float(gravity_mps2)
        self.state = np.zeros(self.state_size)
        self.state[6] = 1.0
        self.covariance = np.eye(self.state_size) * 0.1
        self.covariance[6:10, 6:10] = np.eye(4) * 0.01
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

        acceleration_ned = (
            quaternion_to_rotation(quaternion) @ np.asarray(accel_frd)
            + np.array([0.0, 0.0, self.gravity])
        )

        result[0:3] = position + velocity * dt_s + 0.5 * acceleration_ned * dt_s**2
        result[3:6] = velocity + acceleration_ned * dt_s
        delta_quaternion = np.concatenate(([1.0], 0.5 * np.asarray(gyro_frd) * dt_s))
        result[6:10] = normalize_quaternion(
            quaternion_multiply(quaternion, delta_quaternion)
        )
        # The gyro is the high-rate estimate of the public angular-velocity state.
        result[10:13] = gyro_frd
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

        # Discrete IMU measurement noise is propagated through the actual process
        # step so that position/velocity, attitude, and body-rate correlations are
        # retained instead of being approximated by unrelated diagonal entries.
        input_jacobian = np.zeros((self.state_size, 6))
        nominal_input = np.concatenate((accel_frd_mps2, gyro_frd_radps))
        for index in range(6):
            perturbation = np.zeros(6)
            perturbation[index] = epsilon
            plus = self._process_step(
                previous,
                nominal_input[0:3] + perturbation[0:3],
                nominal_input[3:6] + perturbation[3:6],
                dt_s,
            )
            minus = self._process_step(
                previous,
                nominal_input[0:3] - perturbation[0:3],
                nominal_input[3:6] - perturbation[3:6],
                dt_s,
            )
            input_jacobian[:, index] = (plus - minus) / (2.0 * epsilon)
        measurement_variances = np.array(
            [self.noise.accel_std_mps2**2] * 3
            + [self.noise.gyro_std_radps**2] * 3
        )
        process_noise = (
            input_jacobian
            @ np.diag(measurement_variances)
            @ input_jacobian.T
        )

        # Continuous white-acceleration model error uses the exact constant-
        # acceleration discretization.  The larger North component is an
        # empirical consistency correction for the observed X-axis residuals.
        model_density = np.asarray(
            self.noise.model_accel_noise_density_ned, dtype=float
        )
        model_covariance = np.diag(model_density**2)
        process_noise[0:3, 0:3] += model_covariance * dt_s**3 / 3.0
        process_noise[0:3, 3:6] += model_covariance * dt_s**2 / 2.0
        process_noise[3:6, 0:3] += model_covariance * dt_s**2 / 2.0
        process_noise[3:6, 3:6] += model_covariance * dt_s
        process_noise += np.eye(self.state_size) * 1e-12
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
