#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <boost/bind.hpp>

namespace hero_chassis_controller {

bool initPid(control_toolbox::Pid& pid, const ros::NodeHandle& node) {
  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double i_max = 0.0;
  double i_min = 0.0;
  bool antiwindup = false;
  node.param("p", p, p);
  node.param("i", i, i);
  node.param("d", d, d);
  node.param("i_clamp_max", i_max, i_max);
  node.param("i_clamp_min", i_min, -i_max);
  node.param("antiwindup", antiwindup, antiwindup);
  pid.initPid(p, i, d, i_max, i_min, antiwindup);
  return true;
}

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

  ros::NodeHandle front_left_pid_nh(controller_nh, "pid/front_left");
  ros::NodeHandle front_right_pid_nh(controller_nh, "pid/front_right");
  ros::NodeHandle back_left_pid_nh(controller_nh, "pid/back_left");
  ros::NodeHandle back_right_pid_nh(controller_nh, "pid/back_right");

  if (!initPid(front_left_pid_, front_left_pid_nh) ||
      !initPid(front_right_pid_, front_right_pid_nh) ||
      !initPid(back_left_pid_, back_left_pid_nh) ||
      !initPid(back_right_pid_, back_right_pid_nh)) {
    ROS_ERROR("Failed to initialize wheel velocity PID controllers.");
    return false;
  }

  front_left_pid_server_ = std::make_unique<PidServer>(front_left_pid_nh);
  front_left_pid_server_->setCallback(
      boost::bind(&HeroChassisController::frontLeftPidCallback, this, _1, _2));
  front_right_pid_server_ = std::make_unique<PidServer>(front_right_pid_nh);
  front_right_pid_server_->setCallback(
      boost::bind(&HeroChassisController::frontRightPidCallback, this, _1, _2));
  back_left_pid_server_ = std::make_unique<PidServer>(back_left_pid_nh);
  back_left_pid_server_->setCallback(
      boost::bind(&HeroChassisController::backLeftPidCallback, this, _1, _2));
  back_right_pid_server_ = std::make_unique<PidServer>(back_right_pid_nh);
  back_right_pid_server_->setCallback(
      boost::bind(&HeroChassisController::backRightPidCallback, this, _1, _2));

  return true;
}


void HeroChassisController::frontLeftPidCallback(PidConfig& config, uint32_t) {
  front_left_pid_.setGains(config.p, config.i, config.d, config.i_clamp_max,
                           config.i_clamp_min, config.antiwindup);
}

void HeroChassisController::frontRightPidCallback(PidConfig& config, uint32_t) {
  front_right_pid_.setGains(config.p, config.i, config.d, config.i_clamp_max,
                            config.i_clamp_min, config.antiwindup);
}

void HeroChassisController::backLeftPidCallback(PidConfig& config, uint32_t) {
  back_left_pid_.setGains(config.p, config.i, config.d, config.i_clamp_max,
                          config.i_clamp_min, config.antiwindup);
}

void HeroChassisController::backRightPidCallback(PidConfig& config, uint32_t) {
  back_right_pid_.setGains(config.p, config.i, config.d, config.i_clamp_max,
                           config.i_clamp_min, config.antiwindup);
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  static const double target_speeds[6][4] = {
      {1.0, 1.0, 1.0, 1.0},
      {-2.0, -2.0, -2.0, -2.0},
      {-1.0, 1.0, 1.0, -1.0},
      {2.0, -2.0, -2.0, 2.0},
      {2.0, -2.0, 2.0, -2.0},
      {-1.0, 1.0, -1.0, 1.0}};

  if ((time - last_change_).toSec() > 2) {
    state_ = (state_ + 1) % 6;
    last_change_ = time;
  }

  const double front_left_error =
      target_speeds[state_][0] - front_left_joint_.getVelocity();
  const double front_right_error =
      target_speeds[state_][1] - front_right_joint_.getVelocity();
  const double back_left_error =
      target_speeds[state_][2] - back_left_joint_.getVelocity();
  const double back_right_error =
      target_speeds[state_][3] - back_right_joint_.getVelocity();

  front_left_joint_.setCommand(
      front_left_pid_.computeCommand(front_left_error, period));
  front_right_joint_.setCommand(
      front_right_pid_.computeCommand(front_right_error, period));
  back_left_joint_.setCommand(
      back_left_pid_.computeCommand(back_left_error, period));
  back_right_joint_.setCommand(
      back_right_pid_.computeCommand(back_right_error, period));
}

PLUGINLIB_EXPORT_CLASS(hero_chassis_controller::HeroChassisController,
                       controller_interface::ControllerBase)

}  // namespace hero_chassis_controller
