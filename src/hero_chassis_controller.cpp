#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>

namespace hero_chassis_controller {

bool HeroChassisController::init(
    hardware_interface::EffortJointInterface* effort_joint_interface,
    ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
  front_left_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_joint_ =
      effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");
  return true;
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  const double torque = 0.2;
  static const double commands[6][4] = {
      {torque, torque, torque, torque},
      {-2 * torque, -2 * torque, -2 * torque, -2 * torque},
      {-torque, torque, torque, -torque},
      {2 * torque, -2 * torque, -2 * torque, 2 * torque},
      {2 * torque, -2 * torque, 2 * torque, -2 * torque},
      {-torque, torque, -torque, torque}};

  if ((time - last_change_).toSec() > 2) {
    state_ = (state_ + 1) % 6;
    last_change_ = time;
  }
  front_left_joint_.setCommand(commands[state_][0]);
  front_right_joint_.setCommand(commands[state_][1]);
  back_left_joint_.setCommand(commands[state_][2]);
  back_right_joint_.setCommand(commands[state_][3]);
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
