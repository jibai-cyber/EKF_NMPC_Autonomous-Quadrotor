import time

import numpy as np
import rclpy
from drone_interfaces.msg import RotorThrusts, SolverStats, State13, TrajectoryPoint
from rclpy.node import Node

from .dynamics import (
    VehicleParameters,
    condition_rotor_thrusts,
    quaternion_to_rotation,
    rotor_wrench_frd,
    roll_pitch_from_quaternion,
    thrusts_from_wrench_frd,
)
from .nmpc import NmpcConfig, NmpcController, NmpcWeights
from .trajectory import MissionParameters, TrajectoryParameters, mission_reference


def state_from_message(message: State13) -> np.ndarray:
    return np.concatenate(
        (
            np.asarray(message.position_ned_m),
            np.asarray(message.velocity_ned_mps),
            np.asarray(message.attitude_wb),
            np.asarray(message.body_rates_frd_radps),
        )
    )


class NmpcNode(Node):
    def __init__(self) -> None:
        super().__init__("nmpc_node")
        self.vehicle = VehicleParameters(
            mass_kg=self.declare_parameter("mass_kg", 1.20).value,
            dx_m=self.declare_parameter("dx_m", 0.225).value,
            dy_m=self.declare_parameter("dy_m", 0.225).value,
            jxx_kgm2=self.declare_parameter("jxx_kgm2", 1.25e-2).value,
            jyy_kgm2=self.declare_parameter("jyy_kgm2", 1.25e-2).value,
            jzz_kgm2=self.declare_parameter("jzz_kgm2", 2.20e-2).value,
            moment_ratio_m=self.declare_parameter("moment_ratio_m", 0.015).value,
            rotor_min_thrust_n=self.declare_parameter("rotor_min_thrust_n", 0.2).value,
            rotor_max_thrust_n=self.declare_parameter("rotor_max_thrust_n", 5.5).value,
        )
        weights = NmpcWeights(
            position=tuple(self.declare_parameter("weight_position", [20.0, 20.0, 25.0]).value),
            velocity=tuple(self.declare_parameter("weight_velocity", [4.0, 4.0, 5.0]).value),
            attitude=self.declare_parameter("weight_attitude", 12.0).value,
            body_rate=tuple(self.declare_parameter("weight_body_rate", [2.0, 2.0, 0.5]).value),
            input=tuple(self.declare_parameter("weight_input", [0.1] * 4).value),
            terminal_multiplier=self.declare_parameter("terminal_multiplier", 4.0).value,
        )
        self.config = NmpcConfig(
            horizon_steps=self.declare_parameter("horizon_steps", 20).value,
            step_s=self.declare_parameter("prediction_step_s", 0.05).value,
            max_tilt_deg=self.declare_parameter("max_tilt_deg", 35.0).value,
            max_body_rates_degps=tuple(
                self.declare_parameter("max_body_rates_degps", [180.0, 180.0, 90.0]).value
            ),
            max_roll_pitch_torque_nm=self.declare_parameter(
                "max_roll_pitch_torque_nm", 0.05
            ).value,
            max_yaw_torque_nm=self.declare_parameter("max_yaw_torque_nm", 0.015).value,
            max_thrust_slew_nps=self.declare_parameter(
                "max_thrust_slew_nps", 40.0
            ).value,
            ipopt_max_iterations=self.declare_parameter("ipopt_max_iterations", 80).value,
            ipopt_max_cpu_time_s=self.declare_parameter("ipopt_max_cpu_time_s", 0.08).value,
            weights=weights,
        )
        trajectory = TrajectoryParameters(
            amplitude_x_m=self.declare_parameter("amplitude_x_m", 3.0).value,
            amplitude_y_m=self.declare_parameter("amplitude_y_m", 2.0).value,
            amplitude_z_m=self.declare_parameter("amplitude_z_m", 0.5).value,
            base_height_ned_m=self.declare_parameter("base_height_ned_m", -2.5).value,
            omega_radps=self.declare_parameter("omega_radps", 0.25).value,
            duration_s=self.declare_parameter("duration_s", 60.0).value,
        )
        self.mission_parameters = MissionParameters(
            initial_height_ned_m=self.declare_parameter(
                "initial_height_ned_m", -0.035
            ).value,
            takeoff_duration_s=self.declare_parameter("takeoff_duration_s", 5.0).value,
            yaw_alignment_duration_s=self.declare_parameter(
                "yaw_alignment_duration_s", 5.0
            ).value,
            figure8_entry_ramp_duration_s=self.declare_parameter(
                "figure8_entry_ramp_duration_s", 3.0
            ).value,
            trajectory=trajectory,
        )

        self.controller = NmpcController(vehicle=self.vehicle, config=self.config)
        self.current_state: np.ndarray | None = None
        self.reference_time_s: float | None = None
        self.solve_count = 0
        self.publisher = self.create_publisher(RotorThrusts, "/drone/rotor_thrusts", 10)
        self.stats_publisher = self.create_publisher(SolverStats, "/drone/nmpc/stats", 10)
        self.create_subscription(State13, "/drone/state_estimate", self.state_callback, 10)
        self.create_subscription(TrajectoryPoint, "/drone/reference", self.reference_callback, 10)
        control_rate_hz = self.declare_parameter("control_rate_hz", 20.0).value
        self.control_step_s = 1.0 / control_rate_hz
        self.previous_thrusts = np.full(4, self.vehicle.hover_thrust_per_rotor_n)
        self.safety_recovery_active = False
        self.timer = self.create_timer(self.control_step_s, self.control_callback)

    def state_callback(self, message: State13) -> None:
        self.current_state = state_from_message(message)

    def reference_callback(self, message: TrajectoryPoint) -> None:
        self.reference_time_s = float(message.time_from_start_s)

    def control_callback(self) -> None:
        if self.current_state is None or self.reference_time_s is None:
            return
        references = [
            mission_reference(
                self.reference_time_s + step * self.config.step_s,
                self.mission_parameters,
            )
            for step in range(self.config.horizon_steps + 1)
        ]
        reference_horizon = np.vstack([reference["state"] for reference in references])
        started = time.perf_counter()
        phase = str(references[0]["phase"])
        if phase == "takeoff":
            thrusts = self.vertical_fallback(references[0])
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            solution = {
                "success": True,
                "status": f"mission_{phase}",
                "iterations": 0,
                "cost": float("nan"),
            }
        elif phase == "yaw_align":
            thrusts = self.yaw_alignment_control(references[0])
            elapsed_ms = (time.perf_counter() - started) * 1000.0
            solution = {
                "success": True,
                "status": "mission_yaw_align",
                "iterations": 0,
                "cost": float("nan"),
            }
        else:
            try:
                solution = self.controller.solve(
                    self.current_state,
                    reference_horizon,
                    self.previous_thrusts,
                )
                thrusts = solution["thrusts_n"]
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                if not solution["success"]:
                    self.get_logger().warning(
                        f"NMPC status={solution['status']}; applying geometric fallback"
                    )
                    thrusts = self.geometric_fallback(references[0])
                elif self.recovery_required():
                    thrusts = self.geometric_fallback(references[0])
                    solution["status"] = f"{solution['status']}_safety_recovery"
                self.solve_count += 1
                if self.solve_count % 20 == 0:
                    self.get_logger().info(
                        f"NMPC {solution['status']}: {elapsed_ms:.2f} ms, "
                        f"cost={solution['cost']:.3f}, iterations={solution['iterations']}"
                    )
            except Exception as exception:  # Safe fallback for infeasible constraints.
                self.get_logger().error(
                    f"NMPC solve failed: {exception}; applying geometric fallback"
                )
                thrusts = self.geometric_fallback(references[0])
                elapsed_ms = (time.perf_counter() - started) * 1000.0
                solution = {
                    "success": False,
                    "status": str(exception),
                    "iterations": -1,
                    "cost": float("nan"),
                }

        thrusts = condition_rotor_thrusts(
            thrusts,
            self.previous_thrusts,
            self.control_step_s,
            self.vehicle,
            self.config.max_roll_pitch_torque_nm,
            self.config.max_yaw_torque_nm,
            self.config.max_thrust_slew_nps,
        )
        self.previous_thrusts = thrusts

        output = RotorThrusts()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "base_link_frd"
        output.thrusts_n = np.asarray(thrusts, dtype=float).tolist()
        self.publisher.publish(output)

        stats = SolverStats()
        stats.header = output.header
        stats.solve_time_ms = elapsed_ms
        stats.cost = float(solution["cost"])
        stats.iterations = int(solution["iterations"])
        stats.success = bool(solution["success"])
        stats.status = str(solution["status"])
        self.stats_publisher.publish(stats)

    def vertical_fallback(self, reference: dict) -> np.ndarray:
        """Symmetric thrust fallback that cannot introduce roll or pitch torque."""
        assert self.current_state is not None
        desired_acceleration_z = (
            float(np.asarray(reference["acceleration"])[2])
            + 2.0 * (float(np.asarray(reference["position"])[2]) - self.current_state[2])
            + 1.6 * (float(np.asarray(reference["velocity"])[2]) - self.current_state[5])
        )
        thrust_per_rotor = self.vehicle.mass_kg * (
            self.vehicle.gravity_mps2 - desired_acceleration_z
        ) / 4.0
        return np.full(
            4,
            np.clip(
                thrust_per_rotor,
                self.vehicle.rotor_min_thrust_n,
                self.vehicle.rotor_max_thrust_n,
            ),
        )

    def recovery_required(self) -> bool:
        """Use hysteresis so NMPC is restored only after attitude has settled."""
        assert self.current_state is not None
        roll, pitch = roll_pitch_from_quaternion(self.current_state[6:10])
        tilt = max(abs(roll), abs(pitch))
        rate = float(np.max(np.abs(self.current_state[10:12])))
        if self.safety_recovery_active:
            if tilt < np.deg2rad(15.0) and rate < np.deg2rad(60.0):
                self.safety_recovery_active = False
        elif tilt > np.deg2rad(28.0) or rate > np.deg2rad(100.0):
            self.safety_recovery_active = True
        return self.safety_recovery_active

    def geometric_fallback(self, reference: dict) -> np.ndarray:
        """Track position while recovering attitude without an optimization solve."""
        assert self.current_state is not None
        position_error = np.asarray(reference["position"]) - self.current_state[0:3]
        velocity_error = np.asarray(reference["velocity"]) - self.current_state[3:6]
        desired_acceleration = (
            np.asarray(reference["acceleration"])
            + np.array([1.5, 1.5, 2.0]) * position_error
            + np.array([1.4, 1.4, 1.6]) * velocity_error
        )
        horizontal_norm = np.linalg.norm(desired_acceleration[0:2])
        if horizontal_norm > 2.5:
            desired_acceleration[0:2] *= 2.5 / horizontal_norm
        desired_acceleration[2] = np.clip(desired_acceleration[2], -3.0, 3.0)

        current_rotation = quaternion_to_rotation(self.current_state[6:10])
        reference_rotation = quaternion_to_rotation(np.asarray(reference["state"])[6:10])
        desired_yaw = np.arctan2(reference_rotation[1, 0], reference_rotation[0, 0])
        desired_body_z = np.array([0.0, 0.0, self.vehicle.gravity_mps2]) - desired_acceleration
        desired_body_z /= np.linalg.norm(desired_body_z)
        heading = np.array([np.cos(desired_yaw), np.sin(desired_yaw), 0.0])
        desired_body_y = np.cross(desired_body_z, heading)
        desired_body_y /= np.linalg.norm(desired_body_y)
        desired_body_x = np.cross(desired_body_y, desired_body_z)
        desired_rotation = np.column_stack(
            (desired_body_x, desired_body_y, desired_body_z)
        )

        attitude_skew = (
            desired_rotation.T @ current_rotation
            - current_rotation.T @ desired_rotation
        )
        attitude_error = 0.5 * np.array(
            [attitude_skew[2, 1], attitude_skew[0, 2], attitude_skew[1, 0]]
        )
        torque = (
            -np.array([0.10, 0.10, 0.04]) * attitude_error
            - np.array([0.04, 0.04, 0.02]) * self.current_state[10:13]
        )
        collective = self.vehicle.mass_kg * float(
            desired_body_z @ current_rotation[:, 2]
        ) * np.linalg.norm(
            np.array([0.0, 0.0, self.vehicle.gravity_mps2]) - desired_acceleration
        )
        collective = np.clip(
            collective,
            4.0 * self.vehicle.rotor_min_thrust_n,
            4.0 * self.vehicle.rotor_max_thrust_n,
        )
        return thrusts_from_wrench_frd(collective, torque, self.vehicle)

    def yaw_alignment_control(self, reference: dict) -> np.ndarray:
        """Execute the known yaw profile using gyro-rate feedback, not absolute yaw."""
        assert self.current_state is not None
        base_thrusts = self.geometric_fallback(reference)
        _, torque = rotor_wrench_frd(base_thrusts, self.vehicle)
        desired_yaw_rate = float(np.asarray(reference["state"])[12])
        desired_yaw_acceleration = float(reference["yaw_acceleration"])
        torque[2] = self.vehicle.jzz_kgm2 * (
            desired_yaw_acceleration
            + 5.0 * (desired_yaw_rate - self.current_state[12])
        )
        return thrusts_from_wrench_frd(float(np.sum(base_thrusts)), torque, self.vehicle)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NmpcNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
