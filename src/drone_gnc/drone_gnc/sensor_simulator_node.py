"""Add the specified white noise and expose NED/FRD sensor topics."""

import numpy as np
import rclpy
from drone_interfaces.msg import PositionFix, State13
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, NavSatFix

from .dynamics import quaternion_to_rotation
from .frames import (
    quaternion_enu_flu_to_ned_frd,
    vector_enu_to_ned,
    vector_flu_to_frd,
)
from .geodesy import Wgs84LocalFrame


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class SensorSimulatorNode(Node):
    def __init__(self) -> None:
        super().__init__("sensor_simulator_node")
        self.accel_std = self.declare_parameter("accel_std_mps2", 0.08).value
        self.gyro_std = self.declare_parameter("gyro_std_radps", 0.015).value
        self.position_std = self.declare_parameter("position_std_m", 0.02).value
        self.reference_latitude_deg = self.declare_parameter(
            "reference_latitude_deg", 1.3521
        ).value
        self.reference_longitude_deg = self.declare_parameter(
            "reference_longitude_deg", 103.8198
        ).value
        self.reference_altitude_m = self.declare_parameter("reference_altitude_m", 0.0).value
        self.local_geodetic_frame = Wgs84LocalFrame(
            self.reference_latitude_deg,
            self.reference_longitude_deg,
            self.reference_altitude_m,
        )
        seed = int(self.declare_parameter("random_seed", 6224).value)

        self.rng = np.random.default_rng(seed)
        self.truth_gyro_frd = np.zeros(3)

        self.imu_publisher = self.create_publisher(Imu, "/drone/imu", qos_profile_sensor_data)
        self.gnss_publisher = self.create_publisher(
            PositionFix, "/drone/gnss/position_ned", qos_profile_sensor_data
        )
        self.truth_publisher = self.create_publisher(
            State13, "/drone/ground_truth/state_ned", qos_profile_sensor_data
        )
        self.create_subscription(
            Imu, "/drone/imu_raw", self.imu_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            NavSatFix, "/drone/navsat_raw", self.navsat_callback, qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            "/drone/ground_truth/odometry_enu",
            self.odometry_callback,
            qos_profile_sensor_data,
        )

    def imu_callback(self, message: Imu) -> None:
        accel_frd = vector_flu_to_frd(
            np.array(
                [
                    message.linear_acceleration.x,
                    message.linear_acceleration.y,
                    message.linear_acceleration.z,
                ]
            )
        )
        gyro_frd = vector_flu_to_frd(
            np.array(
                [
                    message.angular_velocity.x,
                    message.angular_velocity.y,
                    message.angular_velocity.z,
                ]
            )
        )
        # Gazebo's odometry twist can contain isolated angular-rate spikes when
        # the reported heading wraps.  The raw simulated IMU is a direct,
        # noise-free body-rate source for the ground-truth State13 message.
        self.truth_gyro_frd = gyro_frd.copy()
        accel_frd += self.accel_std * self.rng.standard_normal(3)
        gyro_frd += self.gyro_std * self.rng.standard_normal(3)

        output = Imu()
        output.header = message.header
        output.header.frame_id = "base_link_frd"
        output.orientation_covariance[0] = -1.0  # Orientation is deliberately not measured.
        (
            output.linear_acceleration.x,
            output.linear_acceleration.y,
            output.linear_acceleration.z,
        ) = accel_frd
        output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = gyro_frd
        output.linear_acceleration_covariance = [
            self.accel_std**2 if index in (0, 4, 8) else 0.0 for index in range(9)
        ]
        output.angular_velocity_covariance = [
            self.gyro_std**2 if index in (0, 4, 8) else 0.0 for index in range(9)
        ]
        self.imu_publisher.publish(output)

    def navsat_callback(self, message: NavSatFix) -> None:
        position_ned = self.local_geodetic_frame.position_ned(
            message.latitude,
            message.longitude,
            message.altitude,
        )
        position_ned += self.position_std * self.rng.standard_normal(3)

        output = PositionFix()
        output.header = message.header
        output.header.frame_id = "world_ned"
        output.position_ned_m = position_ned.tolist()
        covariance = np.eye(3) * self.position_std**2
        output.covariance = covariance.reshape(-1).tolist()
        self.gnss_publisher.publish(output)

    def odometry_callback(self, message: Odometry) -> None:
        position_enu = np.array(
            [
                message.pose.pose.position.x,
                message.pose.pose.position.y,
                message.pose.pose.position.z,
            ]
        )
        q_enu_flu = np.array(
            [
                message.pose.pose.orientation.w,
                message.pose.pose.orientation.x,
                message.pose.pose.orientation.y,
                message.pose.pose.orientation.z,
            ]
        )
        q_ned_frd = quaternion_enu_flu_to_ned_frd(q_enu_flu)
        velocity_flu = np.array(
            [
                message.twist.twist.linear.x,
                message.twist.twist.linear.y,
                message.twist.twist.linear.z,
            ]
        )
        velocity_enu = quaternion_to_rotation(q_enu_flu) @ velocity_flu
        output = State13()
        output.header = message.header
        output.header.frame_id = "world_ned"
        output.position_ned_m = vector_enu_to_ned(position_enu).tolist()
        output.velocity_ned_mps = vector_enu_to_ned(velocity_enu).tolist()
        output.attitude_wb = q_ned_frd.tolist()
        output.body_rates_frd_radps = self.truth_gyro_frd.tolist()
        output.covariance = np.zeros((13, 13)).reshape(-1).tolist()
        self.truth_publisher.publish(output)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SensorSimulatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
