#include "drone_gazebo/rotor_wrench_system.hpp"

#include <algorithm>
#include <utility>

#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>

namespace drone_gazebo
{

void RotorWrenchSystem::Configure(
  const gz::sim::Entity & entity,
  const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm,
  gz::sim::EventManager &)
{
  model_ = gz::sim::Model(entity);
  if (!model_.Valid(ecm)) {
    gzerr << "RotorWrenchSystem must be attached to a model." << std::endl;
    return;
  }

  const std::string link_name = sdf->Get<std::string>("link_name", "base_link").first;
  link_entity_ = model_.LinkByName(ecm, link_name);
  if (link_entity_ == gz::sim::kNullEntity) {
    gzerr << "RotorWrenchSystem could not find link [" << link_name << "]." << std::endl;
    return;
  }

  command_topic_ = sdf->Get<std::string>("command_topic", command_topic_).first;
  dx_ = sdf->Get<double>("dx", dx_).first;
  dy_ = sdf->Get<double>("dy", dy_).first;
  moment_ratio_ = sdf->Get<double>("moment_ratio", moment_ratio_).first;
  min_thrust_ = sdf->Get<double>("min_thrust", min_thrust_).first;
  max_thrust_ = sdf->Get<double>("max_thrust", max_thrust_).first;
  command_timeout_s_ = sdf->Get<double>("command_timeout", command_timeout_s_).first;

  if (!transport_node_.Subscribe(command_topic_, &RotorWrenchSystem::OnCommand, this)) {
    gzerr << "Failed to subscribe to [" << command_topic_ << "]." << std::endl;
  } else {
    gzmsg << "RotorWrenchSystem listening on [" << command_topic_ << "]." << std::endl;
  }
}

void RotorWrenchSystem::OnCommand(const gz::msgs::Actuators & message)
{
  if (message.velocity_size() != 4) {
    gzwarn << "Expected exactly four rotor thrust values, received "
           << message.velocity_size() << "." << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(command_mutex_);
  for (std::size_t index = 0; index < thrusts_n_.size(); ++index) {
    thrusts_n_[index] = std::clamp(message.velocity(static_cast<int>(index)),
      min_thrust_, max_thrust_);
  }
  last_command_time_ = std::chrono::steady_clock::now();
  command_received_ = true;
}

void RotorWrenchSystem::PreUpdate(
  const gz::sim::UpdateInfo & info,
  gz::sim::EntityComponentManager & ecm)
{
  if (info.paused || link_entity_ == gz::sim::kNullEntity) {
    return;
  }

  std::array<double, 4> thrusts{};
  {
    std::lock_guard<std::mutex> lock(command_mutex_);
    if (!command_received_) {
      return;
    }
    const double age_s = std::chrono::duration<double>(
      std::chrono::steady_clock::now() - last_command_time_).count();
    if (age_s > command_timeout_s_) {
      return;
    }
    thrusts = thrusts_n_;
  }

  const double t0 = thrusts[0];
  const double t1 = thrusts[1];
  const double t2 = thrusts[2];
  const double t3 = thrusts[3];
  const double total_thrust = t0 + t1 + t2 + t3;

  // Equations from project.pdf, originally expressed in NED / body FRD.
  const double tau_x_frd = dy_ * (-t0 - t1 + t2 + t3);
  const double tau_y_frd = dx_ * (-t0 + t1 + t2 - t3);
  const double tau_z_frd = moment_ratio_ * (-t0 + t1 - t2 + t3);

  // Gazebo's body convention is FLU. FRD -> FLU is a pi rotation about x.
  const gz::math::Vector3d force_body_flu(0.0, 0.0, total_thrust);
  const gz::math::Vector3d torque_body_flu(tau_x_frd, -tau_y_frd, -tau_z_frd);

  const gz::math::Pose3d pose_world = gz::sim::worldPose(link_entity_, ecm);
  const gz::math::Vector3d force_world = pose_world.Rot().RotateVector(force_body_flu);
  const gz::math::Vector3d torque_world = pose_world.Rot().RotateVector(torque_body_flu);

  gz::sim::Link(link_entity_).AddWorldWrench(ecm, force_world, torque_world);
}

}  // namespace drone_gazebo

GZ_ADD_PLUGIN(
  drone_gazebo::RotorWrenchSystem,
  gz::sim::System,
  drone_gazebo::RotorWrenchSystem::ISystemConfigure,
  drone_gazebo::RotorWrenchSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(
  drone_gazebo::RotorWrenchSystem,
  "drone_gazebo::RotorWrenchSystem")

