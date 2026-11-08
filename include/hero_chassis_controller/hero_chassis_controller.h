#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <control_toolbox/ParametersConfig.h>
#include <control_toolbox/pid.h>
#include <dynamic_reconfigure/server.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

#include <memory>

namespace hero_chassis_controller {

class HeroChassisController
    : public controller_interface::Controller<
          hardware_interface::EffortJointInterface> {
 public:
  HeroChassisController() = default;
  ~HeroChassisController() override = default;

  bool init(
      hardware_interface::EffortJointInterface* effort_joint_interface,
      ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
  void update(const ros::Time& time, const ros::Duration& period) override;

 private:
  using PidConfig = control_toolbox::ParametersConfig;
  using PidServer = dynamic_reconfigure::Server<PidConfig>;

  void frontLeftPidCallback(PidConfig& config, uint32_t level);
  void frontRightPidCallback(PidConfig& config, uint32_t level);
  void backLeftPidCallback(PidConfig& config, uint32_t level);
  void backRightPidCallback(PidConfig& config, uint32_t level);

  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;
  int state_{};
  ros::Time last_change_;
  control_toolbox::Pid front_left_pid_;
  control_toolbox::Pid front_right_pid_;
  control_toolbox::Pid back_left_pid_;
  control_toolbox::Pid back_right_pid_;
  std::unique_ptr<PidServer> front_left_pid_server_;
  std::unique_ptr<PidServer> front_right_pid_server_;
  std::unique_ptr<PidServer> back_left_pid_server_;
  std::unique_ptr<PidServer> back_right_pid_server_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
