import numpy as np
import rclpy
from drone_interfaces.msg import State13, TrajectoryPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from .trajectory import MissionParameters, TrajectoryParameters, mission_reference


class TrajectoryNode(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_node")
        trajectory = TrajectoryParameters(
            amplitude_x_m=self.declare_parameter("amplitude_x_m", 3.0).value,
            amplitude_y_m=self.declare_parameter("amplitude_y_m", 2.0).value,
            amplitude_z_m=self.declare_parameter("amplitude_z_m", 0.5).value,
            base_height_ned_m=self.declare_parameter("base_height_ned_m", -2.5).value,
            omega_radps=self.declare_parameter("omega_radps", 0.25).value,
            duration_s=self.declare_parameter("duration_s", 60.0).value,
        )
        self.parameters = MissionParameters(
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
        self.entry_position_tolerance_m = self.declare_parameter(
            "entry_position_tolerance_m", 0.12
        ).value
        self.entry_speed_tolerance_mps = self.declare_parameter(
            "entry_speed_tolerance_mps", 0.20
        ).value
        self.entry_yaw_tolerance_rad = np.deg2rad(
            self.declare_parameter("entry_yaw_tolerance_deg", 5.0).value
        )
        self.entry_body_rate_tolerance_radps = np.deg2rad(
            self.declare_parameter("entry_body_rate_tolerance_degps", 10.0).value
        )
        self.entry_stable_duration_s = self.declare_parameter(
            "entry_stable_duration_s", 0.5
        ).value
        publish_rate_hz = self.declare_parameter("publish_rate_hz", 20.0).value
        self.publisher = self.create_publisher(TrajectoryPoint, "/drone/reference", 10)
        self.create_subscription(
            State13,
            "/drone/state_estimate",
            self.state_callback,
            qos_profile_sensor_data,
        )
        self.current_state: np.ndarray | None = None
        self.stable_since_wall_s: float | None = None
        self.figure8_release_wall_s: float | None = None
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_reference)

    def state_callback(self, message: State13) -> None:
        self.current_state = np.concatenate(
            (
                np.asarray(message.position_ned_m),
                np.asarray(message.velocity_ned_mps),
                np.asarray(message.attitude_wb),
                np.asarray(message.body_rates_frd_radps),
            )
        )

    @staticmethod
    def _wrapped_angle(angle: float) -> float:
        return float((angle + np.pi) % (2.0 * np.pi) - np.pi)

    def entry_state_is_stable(self) -> bool:
        if self.current_state is None:
            return False
        target = mission_reference(
            self.parameters.takeoff_duration_s
            + self.parameters.yaw_alignment_duration_s
            - 1e-6,
            self.parameters,
        )
        target_state = np.asarray(target["state"])
        qw, qx, qy, qz = self.current_state[6:10]
        yaw = np.arctan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        target_qw, target_qx, target_qy, target_qz = target_state[6:10]
        target_yaw = np.arctan2(
            2.0 * (target_qw * target_qz + target_qx * target_qy),
            1.0 - 2.0 * (target_qy * target_qy + target_qz * target_qz),
        )
        return bool(
            np.linalg.norm(self.current_state[0:3] - target_state[0:3])
            <= self.entry_position_tolerance_m
            and np.linalg.norm(self.current_state[3:6]) <= self.entry_speed_tolerance_mps
            and abs(self._wrapped_angle(yaw - target_yaw))
            <= self.entry_yaw_tolerance_rad
            and np.linalg.norm(self.current_state[10:13])
            <= self.entry_body_rate_tolerance_radps
        )

    def effective_mission_time(self, wall_time_s: float) -> float:
        figure8_start_s = (
            self.parameters.takeoff_duration_s
            + self.parameters.yaw_alignment_duration_s
        )
        if wall_time_s < figure8_start_s:
            return wall_time_s
        if self.figure8_release_wall_s is None:
            if self.entry_state_is_stable():
                if self.stable_since_wall_s is None:
                    self.stable_since_wall_s = wall_time_s
                elif wall_time_s - self.stable_since_wall_s >= self.entry_stable_duration_s:
                    self.figure8_release_wall_s = wall_time_s
                    self.get_logger().info(
                        "Figure-8 entry released after the estimated state remained stable"
                    )
            else:
                self.stable_since_wall_s = None
        if self.figure8_release_wall_s is None:
            return figure8_start_s - 1e-6
        return figure8_start_s + wall_time_s - self.figure8_release_wall_s

    def publish_reference(self) -> None:
        wall_time_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
        elapsed_s = self.effective_mission_time(wall_time_s)
        reference = mission_reference(elapsed_s, self.parameters)
        state = np.asarray(reference["state"])
        output = TrajectoryPoint()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = "world_ned"
        output.time_from_start_s = float(reference["time_s"])
        output.position_ned_m = state[0:3].tolist()
        output.velocity_ned_mps = state[3:6].tolist()
        output.acceleration_ned_mps2 = np.asarray(reference["acceleration"]).tolist()
        output.attitude_wb = state[6:10].tolist()
        output.body_rates_frd_radps = state[10:13].tolist()
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
