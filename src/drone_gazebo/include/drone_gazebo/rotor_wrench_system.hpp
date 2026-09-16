#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Entity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace drone_gazebo
{

class RotorWrenchSystem final:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(
    const gz::sim::Entity & entity,
    const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm,
    gz::sim::EventManager & event_mgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info,
    gz::sim::EntityComponentManager & ecm) override;

private:
  void OnCommand(const gz::msgs::Actuators & message);

  gz::sim::Model model_{gz::sim::kNullEntity};
  gz::sim::Entity link_entity_{gz::sim::kNullEntity};
  gz::transport::Node transport_node_;

  std::string command_topic_{"/drone/rotor_thrusts_gz"};
  double dx_{0.225};
  double dy_{0.225};
  double moment_ratio_{0.015};
  double min_thrust_{0.2};
  double max_thrust_{5.5};
  double command_timeout_s_{0.2};

  std::mutex command_mutex_;
  std::array<double, 4> thrusts_n_{{0.0, 0.0, 0.0, 0.0}};
  std::chrono::steady_clock::time_point last_command_time_{};
  bool command_received_{false};
};

}  // namespace drone_gazebo
