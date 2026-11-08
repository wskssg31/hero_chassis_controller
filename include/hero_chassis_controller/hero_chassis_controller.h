#ifndef HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
#define HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H

#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>

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
  hardware_interface::JointHandle front_left_joint_;
  hardware_interface::JointHandle front_right_joint_;
  hardware_interface::JointHandle back_left_joint_;
  hardware_interface::JointHandle back_right_joint_;
  int state_{};
  ros::Time last_change_;
};

}  // namespace hero_chassis_controller

#endif  // HERO_CHASSIS_CONTROLLER_HERO_CHASSIS_CONTROLLER_H
