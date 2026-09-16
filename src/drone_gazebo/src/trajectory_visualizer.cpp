#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include <drone_interfaces/msg/state13.hpp>
#include <drone_interfaces/msg/trajectory_point.hpp>
#include <gz/msgs/marker.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

namespace
{

gz::msgs::Vector3d NedToEnu(const double north, const double east, const double down)
{
  gz::msgs::Vector3d point;
  point.set_x(east);
  point.set_y(north);
  point.set_z(-down);
  return point;
}

double SquaredDistance(const gz::msgs::Vector3d & left, const gz::msgs::Vector3d & right)
{
  const double dx = left.x() - right.x();
  const double dy = left.y() - right.y();
  const double dz = left.z() - right.z();
  return dx * dx + dy * dy + dz * dz;
}

}  // namespace

class TrajectoryVisualizer final : public rclcpp::Node
{
public:
  TrajectoryVisualizer()
  : Node("trajectory_visualizer")
  {
    const double sample_rate_hz = declare_parameter<double>("sample_rate_hz", 10.0);
    const double publish_rate_hz = declare_parameter<double>("publish_rate_hz", 5.0);
    max_points_ = static_cast<std::size_t>(declare_parameter<int>("max_points", 1500));
    minimum_distance_m_ = declare_parameter<double>("minimum_distance_m", 0.01);
    actual_trail_width_m_ = declare_parameter<double>("actual_trail_width_m", 0.05);
    reference_trail_width_m_ = declare_parameter<double>("reference_trail_width_m", 0.025);
    marker_service_ = declare_parameter<std::string>("marker_service", "/marker");

    if (sample_rate_hz <= 0.0 || publish_rate_hz <= 0.0 || max_points_ < 2) {
      throw std::invalid_argument(
              "sample_rate_hz and publish_rate_hz must be positive and max_points must be >= 2");
    }
    sample_period_s_ = 1.0 / sample_rate_hz;

    actual_subscription_ = create_subscription<drone_interfaces::msg::State13>(
      "/drone/ground_truth/state_ned", rclcpp::SensorDataQoS(),
      [this](const drone_interfaces::msg::State13::SharedPtr message) {
        const double stamp_s = rclcpp::Time(message->header.stamp).seconds();
        AddSample(
          actual_points_, last_actual_sample_s_, stamp_s,
          NedToEnu(
            message->position_ned_m[0], message->position_ned_m[1],
            message->position_ned_m[2]));
      });

    reference_subscription_ = create_subscription<drone_interfaces::msg::TrajectoryPoint>(
      "/drone/reference", 10,
      [this](const drone_interfaces::msg::TrajectoryPoint::SharedPtr message) {
        const double stamp_s = rclcpp::Time(message->header.stamp).seconds();
        AddSample(
          reference_points_, last_reference_sample_s_, stamp_s,
          NedToEnu(
            message->position_ned_m[0], message->position_ned_m[1],
            message->position_ned_m[2]));
      });

    publish_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate_hz),
      [this]() {PublishMarkers();});

    RCLCPP_INFO(
      get_logger(),
      "Gazebo trajectory markers enabled: sample %.1f Hz, publish %.1f Hz, max %zu points",
      sample_rate_hz, publish_rate_hz, max_points_);
  }

private:
  void AddSample(
    std::deque<gz::msgs::Vector3d> & points, double & last_sample_s,
    const double stamp_s, const gz::msgs::Vector3d & point)
  {
    if (!std::isfinite(stamp_s)) {
      return;
    }
    if (stamp_s < last_sample_s) {
      points.clear();
      last_sample_s = -std::numeric_limits<double>::infinity();
    }
    if (stamp_s - last_sample_s + 1e-9 < sample_period_s_) {
      return;
    }
    last_sample_s = stamp_s;

    const double minimum_distance_squared = minimum_distance_m_ * minimum_distance_m_;
    if (!points.empty() && SquaredDistance(points.back(), point) < minimum_distance_squared) {
      return;
    }
    points.push_back(point);
    while (points.size() > max_points_) {
      points.pop_front();
    }
  }

  gz::msgs::Marker MakeRibbon(
    const std::deque<gz::msgs::Vector3d> & points, const std::uint64_t id,
    const double width_m, const double red, const double green, const double blue,
    const double alpha) const
  {
    gz::msgs::Marker marker;
    marker.set_action(gz::msgs::Marker::ADD_MODIFY);
    marker.set_ns("drone_trajectory");
    marker.set_id(id);
    marker.set_layer(1);
    // Gazebo LINE_STRIP is only one pixel wide and its scale transforms the
    // point coordinates. A triangle ribbon gives the trail a world-unit width.
    marker.set_type(gz::msgs::Marker::TRIANGLE_STRIP);
    marker.set_visibility(gz::msgs::Marker::GUI);
    marker.mutable_pose()->mutable_orientation()->set_w(1.0);
    marker.mutable_scale()->set_x(1.0);
    marker.mutable_scale()->set_y(1.0);
    marker.mutable_scale()->set_z(1.0);

    auto * diffuse = marker.mutable_material()->mutable_diffuse();
    diffuse->set_r(red);
    diffuse->set_g(green);
    diffuse->set_b(blue);
    diffuse->set_a(alpha);
    auto * ambient = marker.mutable_material()->mutable_ambient();
    ambient->CopyFrom(*diffuse);
    auto * emissive = marker.mutable_material()->mutable_emissive();
    emissive->set_r(0.35 * red);
    emissive->set_g(0.35 * green);
    emissive->set_b(0.35 * blue);
    emissive->set_a(alpha);

    const double half_width = 0.5 * width_m;
    for (std::size_t index = 0; index < points.size(); ++index) {
      const auto & previous = points[index == 0 ? index : index - 1];
      const auto & next = points[index + 1 < points.size() ? index + 1 : index];
      const double tangent_x = next.x() - previous.x();
      const double tangent_y = next.y() - previous.y();
      const double horizontal_norm = std::hypot(tangent_x, tangent_y);
      const double lateral_x = horizontal_norm > 1e-9 ? -tangent_y / horizontal_norm : 1.0;
      const double lateral_y = horizontal_norm > 1e-9 ? tangent_x / horizontal_norm : 0.0;

      auto * left = marker.add_point();
      left->set_x(points[index].x() + half_width * lateral_x);
      left->set_y(points[index].y() + half_width * lateral_y);
      left->set_z(points[index].z() + 0.01);
      auto * right = marker.add_point();
      right->set_x(points[index].x() - half_width * lateral_x);
      right->set_y(points[index].y() - half_width * lateral_y);
      right->set_z(points[index].z() + 0.01);
    }
    return marker;
  }

  void PublishMarkers()
  {
    if (actual_points_.size() >= 2) {
      const auto marker = MakeRibbon(
        actual_points_, 1, actual_trail_width_m_, 0.05, 0.45, 1.0, 1.0);
      if (!transport_node_.Request(marker_service_, marker)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gazebo marker service [%s] is not available yet", marker_service_.c_str());
      }
    }
    if (reference_points_.size() >= 2) {
      const auto marker = MakeRibbon(
        reference_points_, 2, reference_trail_width_m_, 1.0, 0.72, 0.05, 0.8);
      if (!transport_node_.Request(marker_service_, marker)) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Gazebo marker service [%s] is not available yet", marker_service_.c_str());
      }
    }
  }

  gz::transport::Node transport_node_;
  std::string marker_service_;
  double sample_period_s_{0.1};
  double minimum_distance_m_{0.01};
  double actual_trail_width_m_{0.05};
  double reference_trail_width_m_{0.025};
  std::size_t max_points_{1500};
  double last_actual_sample_s_{-std::numeric_limits<double>::infinity()};
  double last_reference_sample_s_{-std::numeric_limits<double>::infinity()};
  std::deque<gz::msgs::Vector3d> actual_points_;
  std::deque<gz::msgs::Vector3d> reference_points_;
  rclcpp::Subscription<drone_interfaces::msg::State13>::SharedPtr actual_subscription_;
  rclcpp::Subscription<drone_interfaces::msg::TrajectoryPoint>::SharedPtr
    reference_subscription_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TrajectoryVisualizer>());
  rclcpp::shutdown();
  return 0;
}
