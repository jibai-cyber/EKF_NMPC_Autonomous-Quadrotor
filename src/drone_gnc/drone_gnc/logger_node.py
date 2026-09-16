import csv
import math
from pathlib import Path

import rclpy
from drone_interfaces.msg import RotorThrusts, SolverStats, State13, TrajectoryPoint
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class LoggerNode(Node):
    def __init__(self) -> None:
        super().__init__("logger_node")
        output_path = Path(self.declare_parameter("output_csv", "results/flight_log.csv").value)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = output_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.stream)
        self.writer.writerow(
            [
                "time_s",
                *[f"estimate_{name}" for name in self.state_names()],
                *[f"sigma_{name}" for name in self.state_names()],
                *[f"truth_{name}" for name in self.state_names()],
                *[f"reference_{name}" for name in self.state_names()],
                "thrust_0_n",
                "thrust_1_n",
                "thrust_2_n",
                "thrust_3_n",
                "solver_time_ms",
                "solver_cost",
                "solver_iterations",
                "solver_success",
            ]
        )
        self.truth = [float("nan")] * 13
        self.reference = [float("nan")] * 13
        self.thrusts = [float("nan")] * 4
        self.solver_stats = [float("nan"), float("nan"), -1, False]
        self.create_subscription(State13, "/drone/state_estimate", self.estimate_callback, 10)
        self.create_subscription(
            State13,
            "/drone/ground_truth/state_ned",
            self.truth_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(TrajectoryPoint, "/drone/reference", self.reference_callback, 10)
        self.create_subscription(RotorThrusts, "/drone/rotor_thrusts", self.thrust_callback, 10)
        self.create_subscription(SolverStats, "/drone/nmpc/stats", self.stats_callback, 10)
        self.get_logger().info(f"Logging to {output_path.resolve()}")

    @staticmethod
    def state_names() -> list[str]:
        return ["x", "y", "z", "vx", "vy", "vz", "qw", "qx", "qy", "qz", "p", "q", "r"]

    @staticmethod
    def flatten_state(message: State13) -> list[float]:
        return [
            *message.position_ned_m,
            *message.velocity_ned_mps,
            *message.attitude_wb,
            *message.body_rates_frd_radps,
        ]

    def estimate_callback(self, message: State13) -> None:
        time_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        standard_deviations = [
            math.sqrt(max(message.covariance[index * 13 + index], 0.0))
            for index in range(13)
        ]
        self.writer.writerow(
            [
                time_s,
                *self.flatten_state(message),
                *standard_deviations,
                *self.truth,
                *self.reference,
                *self.thrusts,
                *self.solver_stats,
            ]
        )
        self.stream.flush()

    def truth_callback(self, message: State13) -> None:
        self.truth = self.flatten_state(message)

    def reference_callback(self, message: TrajectoryPoint) -> None:
        self.reference = [
            *message.position_ned_m,
            *message.velocity_ned_mps,
            *message.attitude_wb,
            *message.body_rates_frd_radps,
        ]

    def thrust_callback(self, message: RotorThrusts) -> None:
        self.thrusts = list(message.thrusts_n)

    def stats_callback(self, message: SolverStats) -> None:
        self.solver_stats = [
            message.solve_time_ms,
            message.cost,
            message.iterations,
            message.success,
        ]

    def destroy_node(self):
        self.stream.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = LoggerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
