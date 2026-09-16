import numpy as np
import rclpy
from drone_interfaces.msg import TrajectoryPoint
from rclpy.node import Node

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
            trajectory=trajectory,
        )
        publish_rate_hz = self.declare_parameter("publish_rate_hz", 20.0).value
        self.publisher = self.create_publisher(TrajectoryPoint, "/drone/reference", 10)
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(1.0 / publish_rate_hz, self.publish_reference)

    def publish_reference(self) -> None:
        elapsed_s = (self.get_clock().now() - self.start_time).nanoseconds * 1e-9
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
