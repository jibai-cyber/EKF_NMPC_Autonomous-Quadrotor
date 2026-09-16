#include <memory>

#include <drone_interfaces/msg/rotor_thrusts.hpp>
#include <gz/msgs/actuators.pb.h>
#include <gz/transport/Node.hh>
#include <rclcpp/rclcpp.hpp>

class ActuatorBridge final : public rclcpp::Node
{
public:
  ActuatorBridge()
  : Node("actuator_bridge")
  {
    const std::string gz_topic = declare_parameter<std::string>(
      "gz_topic", "/drone/rotor_thrusts_gz");
    publisher_ = gz_node_.Advertise<gz::msgs::Actuators>(gz_topic);
    subscription_ = create_subscription<drone_interfaces::msg::RotorThrusts>(
      "/drone/rotor_thrusts", rclcpp::SensorDataQoS(),
      [this](const drone_interfaces::msg::RotorThrusts::SharedPtr message) {
        gz::msgs::Actuators output;
        for (const double thrust : message->thrusts_n) {
          output.add_velocity(thrust);
        }
        if (!publisher_.Publish(output)) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Failed to publish rotor thrusts to Gazebo Transport.");
        }
      });
  }

private:
  gz::transport::Node gz_node_;
  gz::transport::Node::Publisher publisher_;
  rclcpp::Subscription<drone_interfaces::msg::RotorThrusts>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ActuatorBridge>());
  rclcpp::shutdown();
  return 0;
}

