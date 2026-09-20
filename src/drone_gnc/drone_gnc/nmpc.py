"""Constrained NMPC built from the course equations, without PX4 dynamics."""

from dataclasses import dataclass, field

import numpy as np

from .dynamics import VehicleParameters, normalize_quaternion, rk4_step

try:
    import casadi as ca
except ImportError:  # Allows dynamics/EKF tests before the optional solver is installed.
    ca = None


@dataclass(frozen=True)
class NmpcWeights:
    position: tuple[float, float, float] = (20.0, 20.0, 25.0)
    velocity: tuple[float, float, float] = (4.0, 4.0, 5.0)
    attitude: float = 12.0
    body_rate: tuple[float, float, float] = (2.0, 2.0, 0.5)
    input: tuple[float, float, float, float] = (0.1, 0.1, 0.1, 0.1)
    terminal_multiplier: float = 4.0


@dataclass(frozen=True)
class NmpcConfig:
    horizon_steps: int = 20
    step_s: float = 0.05
    max_tilt_deg: float = 35.0
    max_body_rates_degps: tuple[float, float, float] = (180.0, 180.0, 90.0)
    max_roll_pitch_torque_nm: float = 0.05
    max_yaw_torque_nm: float = 0.015
    max_thrust_slew_nps: float = 40.0
    ipopt_max_iterations: int = 80
    ipopt_max_cpu_time_s: float = 0.08
    weights: NmpcWeights = field(default_factory=NmpcWeights)


class NmpcController:
    state_size = 13
    input_size = 4

    def __init__(
        self,
        vehicle: VehicleParameters = VehicleParameters(),
        config: NmpcConfig = NmpcConfig(),
    ):
        if ca is None:
            raise RuntimeError("CasADi is not installed; install the container dependencies first")
        self.vehicle = vehicle
        self.config = config
        self._last_solution: np.ndarray | None = None
        self._build_solver()

    @staticmethod
    def _rotation_symbolic(quaternion):
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        return ca.vertcat(
            ca.horzcat(1 - 2 * (qy**2 + qz**2), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
            ca.horzcat(2 * (qx * qy + qz * qw), 1 - 2 * (qx**2 + qz**2), 2 * (qy * qz - qx * qw)),
            ca.horzcat(2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx**2 + qy**2)),
        )

    @staticmethod
    def _quaternion_derivative_symbolic(quaternion, body_rates):
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        wx, wy, wz = body_rates[0], body_rates[1], body_rates[2]
        return 0.5 * ca.vertcat(
            -qx * wx - qy * wy - qz * wz,
            qw * wx + qy * wz - qz * wy,
            qw * wy - qx * wz + qz * wx,
            qw * wz + qx * wy - qy * wx,
        )

    def _continuous_symbolic(self, state, thrusts):
        velocity = state[3:6]
        quaternion = state[6:10] / ca.sqrt(ca.dot(state[6:10], state[6:10]) + 1e-12)
        omega = state[10:13]

        t0, t1, t2, t3 = thrusts[0], thrusts[1], thrusts[2], thrusts[3]
        total_thrust = t0 + t1 + t2 + t3
        force_body = ca.vertcat(0.0, 0.0, -total_thrust)
        torque_body = ca.vertcat(
            self.vehicle.dy_m * (-t0 - t1 + t2 + t3),
            self.vehicle.dx_m * (-t0 + t1 + t2 - t3),
            self.vehicle.moment_ratio_m * (-t0 + t1 - t2 + t3),
        )
        inertia = ca.diag(
            ca.vertcat(
                self.vehicle.jxx_kgm2,
                self.vehicle.jyy_kgm2,
                self.vehicle.jzz_kgm2,
            )
        )
        gravity = ca.vertcat(0.0, 0.0, self.vehicle.gravity_mps2)
        acceleration = (
            gravity
            + self._rotation_symbolic(quaternion)
            @ force_body
            / self.vehicle.mass_kg
        )
        omega_dot = ca.solve(inertia, torque_body - ca.cross(omega, inertia @ omega))
        return ca.vertcat(
            velocity,
            acceleration,
            self._quaternion_derivative_symbolic(quaternion, omega),
            omega_dot,
        )

    def _torque_symbolic(self, thrusts):
        t0, t1, t2, t3 = thrusts[0], thrusts[1], thrusts[2], thrusts[3]
        return ca.vertcat(
            self.vehicle.dy_m * (-t0 - t1 + t2 + t3),
            self.vehicle.dx_m * (-t0 + t1 + t2 - t3),
            self.vehicle.moment_ratio_m * (-t0 + t1 - t2 + t3),
        )

    def _rk4_symbolic(self, state, thrusts):
        dt = self.config.step_s
        k1 = self._continuous_symbolic(state, thrusts)
        k2 = self._continuous_symbolic(state + 0.5 * dt * k1, thrusts)
        k3 = self._continuous_symbolic(state + 0.5 * dt * k2, thrusts)
        k4 = self._continuous_symbolic(state + dt * k3, thrusts)
        result = state + dt * (k1 + 2 * k2 + 2 * k3 + k4) / 6.0
        normalized_q = result[6:10] / ca.sqrt(ca.dot(result[6:10], result[6:10]) + 1e-12)
        return ca.vertcat(result[0:6], normalized_q, result[10:13])

    @staticmethod
    def _tilt_angles_symbolic(quaternion):
        qw, qx, qy, qz = quaternion[0], quaternion[1], quaternion[2], quaternion[3]
        roll = ca.atan2(2 * (qw * qx + qy * qz), 1 - 2 * (qx**2 + qy**2))
        pitch_sine = 2 * (qw * qy - qz * qx)
        pitch = ca.atan2(pitch_sine, ca.sqrt(ca.fmax(1 - pitch_sine**2, 1e-9)))
        return ca.vertcat(roll, pitch)

    def _stage_cost(self, state, thrusts, reference):
        weights = self.config.weights
        position_error = state[0:3] - reference[0:3]
        velocity_error = state[3:6] - reference[3:6]
        quaternion_dot = ca.dot(state[6:10], reference[6:10])
        body_rate_error = state[10:13] - reference[10:13]
        hover = self.vehicle.hover_thrust_per_rotor_n
        input_error = thrusts - ca.DM.ones(4, 1) * hover

        return (
            ca.dot(position_error * ca.DM(weights.position), position_error)
            + ca.dot(velocity_error * ca.DM(weights.velocity), velocity_error)
            + weights.attitude * (1.0 - quaternion_dot**2)
            + ca.dot(body_rate_error * ca.DM(weights.body_rate), body_rate_error)
            + ca.dot(input_error * ca.DM(weights.input), input_error)
        )

    def _build_solver(self) -> None:
        horizon = self.config.horizon_steps
        states = ca.SX.sym("X", self.state_size, horizon + 1)
        controls = ca.SX.sym("U", self.input_size, horizon)
        # Parameter vector: current state, last applied input, then N+1 references.
        parameters = ca.SX.sym(
            "P",
            self.state_size
            + self.input_size
            + self.state_size * (horizon + 1),
        )
        current_state = parameters[0 : self.state_size]
        previous_control = parameters[
            self.state_size : self.state_size + self.input_size
        ]
        reference_offset = self.state_size + self.input_size

        constraints = [states[:, 0] - current_state]
        lower_constraints = [np.zeros(self.state_size)]
        upper_constraints = [np.zeros(self.state_size)]
        objective = 0
        tilt_limit = np.deg2rad(self.config.max_tilt_deg)

        for step in range(horizon):
            reference_start = reference_offset + step * self.state_size
            reference = parameters[reference_start : reference_start + self.state_size]
            objective += self._stage_cost(states[:, step], controls[:, step], reference)
            constraints.append(
                states[:, step + 1] - self._rk4_symbolic(states[:, step], controls[:, step])
            )
            lower_constraints.append(np.zeros(self.state_size))
            upper_constraints.append(np.zeros(self.state_size))
            constraints.append(self._tilt_angles_symbolic(states[6:10, step]))
            lower_constraints.append(np.array([-tilt_limit, -tilt_limit]))
            upper_constraints.append(np.array([tilt_limit, tilt_limit]))
            constraints.append(self._torque_symbolic(controls[:, step]))
            lower_constraints.append(
                np.array(
                    [
                        -self.config.max_roll_pitch_torque_nm,
                        -self.config.max_roll_pitch_torque_nm,
                        -self.config.max_yaw_torque_nm,
                    ]
                )
            )
            upper_constraints.append(
                np.array(
                    [
                        self.config.max_roll_pitch_torque_nm,
                        self.config.max_roll_pitch_torque_nm,
                        self.config.max_yaw_torque_nm,
                    ]
                )
            )
            control_delta = (
                controls[:, step] - previous_control
                if step == 0
                else controls[:, step] - controls[:, step - 1]
            )
            constraints.append(control_delta)
            maximum_delta = self.config.max_thrust_slew_nps * self.config.step_s
            lower_constraints.append(-np.ones(self.input_size) * maximum_delta)
            upper_constraints.append(np.ones(self.input_size) * maximum_delta)

        terminal_reference = parameters[-self.state_size :]
        objective += self.config.weights.terminal_multiplier * self._stage_cost(
            states[:, horizon],
            ca.DM.ones(4, 1) * self.vehicle.hover_thrust_per_rotor_n,
            terminal_reference,
        )
        constraints.append(self._tilt_angles_symbolic(states[6:10, horizon]))
        lower_constraints.append(np.array([-tilt_limit, -tilt_limit]))
        upper_constraints.append(np.array([tilt_limit, tilt_limit]))

        decision = ca.vertcat(ca.reshape(states, -1, 1), ca.reshape(controls, -1, 1))
        problem = {
            "x": decision,
            "f": objective,
            "g": ca.vertcat(*constraints),
            "p": parameters,
        }
        options = {
            "print_time": False,
            "ipopt.print_level": 0,
            "ipopt.sb": "yes",
            "ipopt.max_iter": self.config.ipopt_max_iterations,
            "ipopt.max_cpu_time": self.config.ipopt_max_cpu_time_s,
            "ipopt.tol": 1e-4,
        }
        self._solver = ca.nlpsol("course_nmpc", "ipopt", problem, options)

        state_variable_count = self.state_size * (horizon + 1)
        decision_count = state_variable_count + self.input_size * horizon
        self._lower_bounds = np.full(decision_count, -np.inf)
        self._upper_bounds = np.full(decision_count, np.inf)
        body_rate_limits = np.deg2rad(np.asarray(self.config.max_body_rates_degps))
        for step in range(horizon + 1):
            start = step * self.state_size
            self._lower_bounds[start + 10 : start + 13] = -body_rate_limits
            self._upper_bounds[start + 10 : start + 13] = body_rate_limits
        self._lower_bounds[state_variable_count:] = self.vehicle.rotor_min_thrust_n
        self._upper_bounds[state_variable_count:] = self.vehicle.rotor_max_thrust_n
        self._lower_constraints = np.concatenate(lower_constraints)
        self._upper_constraints = np.concatenate(upper_constraints)

    def _shifted_warm_start(
        self, current_state: np.ndarray, default_guess: np.ndarray
    ) -> np.ndarray:
        """Shift the previous state/control sequence by one receding-horizon step."""
        if self._last_solution is None or self._last_solution.shape != default_guess.shape:
            return default_guess
        horizon = self.config.horizon_steps
        state_count = self.state_size * (horizon + 1)
        old_states = self._last_solution[:state_count].reshape(
            (self.state_size, horizon + 1), order="F"
        )
        old_controls = self._last_solution[state_count:].reshape(
            (self.input_size, horizon), order="F"
        )
        shifted_controls = np.column_stack(
            (old_controls[:, 1:], old_controls[:, -1])
        )
        shifted_states = np.empty_like(old_states)
        shifted_states[:, 0] = current_state
        if horizon > 1:
            shifted_states[:, 1:horizon] = old_states[:, 2 : horizon + 1]
        shifted_states[:, horizon] = rk4_step(
            old_states[:, horizon],
            shifted_controls[:, horizon - 1],
            self.config.step_s,
            self.vehicle,
        )
        return np.concatenate(
            (
                shifted_states.reshape(-1, order="F"),
                shifted_controls.reshape(-1, order="F"),
            )
        )

    def solve(
        self,
        current_state: np.ndarray,
        reference_horizon: np.ndarray,
        previous_thrusts_n: np.ndarray | None = None,
    ) -> dict:
        current_state = np.asarray(current_state, dtype=float).copy()
        current_state[6:10] = normalize_quaternion(current_state[6:10])
        references = np.asarray(reference_horizon, dtype=float).copy()
        expected_shape = (self.config.horizon_steps + 1, self.state_size)
        if references.shape != expected_shape:
            raise ValueError(f"Expected reference horizon {expected_shape}, got {references.shape}")
        for step in range(references.shape[0]):
            references[step, 6:10] = normalize_quaternion(references[step, 6:10])

        hover_control = np.full(4, self.vehicle.hover_thrust_per_rotor_n)
        previous_control = (
            hover_control.copy()
            if previous_thrusts_n is None
            else np.asarray(previous_thrusts_n, dtype=float).copy()
        )
        if previous_control.shape != (self.input_size,):
            raise ValueError(
                f"Expected previous thrust vector ({self.input_size},), "
                f"got {previous_control.shape}"
            )
        previous_control = np.clip(
            previous_control,
            self.vehicle.rotor_min_thrust_n,
            self.vehicle.rotor_max_thrust_n,
        )
        rollout = [current_state.copy()]
        for _ in range(self.config.horizon_steps):
            rollout.append(
                rk4_step(rollout[-1], hover_control, self.config.step_s, self.vehicle)
            )
        state_guess = np.asarray(rollout).T.reshape(-1, order="F")
        input_guess = np.tile(
            hover_control,
            self.config.horizon_steps,
        )
        initial_guess = np.concatenate((state_guess, input_guess))
        initial_guess = self._shifted_warm_start(current_state, initial_guess)

        solution = self._solver(
            x0=initial_guess,
            lbx=self._lower_bounds,
            ubx=self._upper_bounds,
            lbg=self._lower_constraints,
            ubg=self._upper_constraints,
            p=np.concatenate((current_state, previous_control, references.reshape(-1))),
        )
        decision = np.asarray(solution["x"]).reshape(-1)
        state_count = self.state_size * (self.config.horizon_steps + 1)
        controls = decision[state_count:].reshape(
            (self.input_size, self.config.horizon_steps), order="F"
        )
        predicted_states = decision[:state_count].reshape(
            (self.state_size, self.config.horizon_steps + 1), order="F"
        ).T
        statistics = self._solver.stats()
        success = bool(statistics.get("success", False))
        self._last_solution = decision if success else None
        return {
            "thrusts_n": np.clip(
                controls[:, 0],
                self.vehicle.rotor_min_thrust_n,
                self.vehicle.rotor_max_thrust_n,
            ),
            "control_sequence": controls.T.copy(),
            "predicted_states": predicted_states,
            "success": success,
            "status": str(statistics.get("return_status", "unknown")),
            "iterations": int(statistics.get("iter_count", -1)),
            "cost": float(solution["f"]),
        }
