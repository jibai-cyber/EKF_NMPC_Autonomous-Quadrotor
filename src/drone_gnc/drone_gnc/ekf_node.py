import numpy as np
import rclpy
from drone_interfaces.msg import PositionFix, State13
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

from .ekf import EkfNoise, QuadrotorEkf
from .sensor_simulator_node import stamp_seconds


class EkfNode(Node):
    def __init__(self) -> None:
        super().__init__("ekf_node")
        noise = EkfNoise(
            accel_std_mps2=self.declare_parameter("accel_std_mps2", 0.08).value,
            gyro_std_radps=self.declare_parameter("gyro_std_radps", 0.015).value,
            position_std_m=self.declare_parameter("position_std_m", 0.02).value,
        )
        self.filter = QuadrotorEkf(noise=noise)
        self.last_imu_time: float | None = None
        self.publisher = self.create_publisher(State13, "/drone/state_estimate", 10)
        self.create_subscription(Imu, "/drone/imu", self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(
            PositionFix,
            "/drone/gnss/position_ned",
            self.position_callback,
            qos_profile_sensor_data,
        )

    def position_callback(self, message: PositionFix) -> None:
        self.filter.update_position(np.asarray(message.position_ned_m, dtype=float))

    def imu_callback(self, message: Imu) -> None:
        now_s = stamp_seconds(message.header.stamp)
        dt_s = 0.01 if self.last_imu_time is None else now_s - self.last_imu_time
        self.last_imu_time = now_s
        accel = np.array(
            [
                message.linear_acceleration.x,
                message.linear_acceleration.y,
                message.linear_acceleration.z,
            ]
        )
        gyro = np.array(
            [message.angular_velocity.x, message.angular_velocity.y, message.angular_velocity.z]
        )
        self.filter.predict(accel, gyro, dt_s)
        if not self.filter.initialized:
            return

        state = self.filter.public_state
        output = State13()
        output.header = message.header
        output.header.frame_id = "world_ned"
        output.position_ned_m = state[0:3].tolist()
        output.velocity_ned_mps = state[3:6].tolist()
        output.attitude_wb = state[6:10].tolist()
        output.body_rates_frd_radps = state[10:13].tolist()
        output.covariance = self.filter.public_covariance.reshape(-1).tolist()
        self.publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = EkfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
