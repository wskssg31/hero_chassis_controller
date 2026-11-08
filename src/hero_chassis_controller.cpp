#include "hero_chassis_controller/hero_chassis_controller.h"

#include <pluginlib/class_list_macros.hpp>

#include <boost/bind.hpp>

#include <algorithm>
#include <cmath>

#include <tf/transform_datatypes.h>

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

  controller_nh.param("wheel_radius", wheel_radius_, 0.07625);
  controller_nh.param("chassis_length", chassis_length_, 0.4);
  controller_nh.param("chassis_width", chassis_width_, 0.4);
  controller_nh.param("max_linear_acceleration", max_linear_acceleration_, 1.0);
  controller_nh.param("max_angular_acceleration", max_angular_acceleration_, 2.0);
  controller_nh.param("velocity_mode", velocity_mode_, std::string("body"));
  controller_nh.param("odom_frame", odom_frame_, std::string("odom"));
  controller_nh.param("base_frame", base_frame_, std::string("base_link"));
  if (velocity_mode_ != "body" && velocity_mode_ != "world") {
    ROS_WARN("Unknown velocity_mode '%s'; using body mode.", velocity_mode_.c_str());
    velocity_mode_ = "body";
  }
  odom_publisher_ = controller_nh.advertise<nav_msgs::Odometry>("/odom", 10);
  std::array<double, 4> zero_speeds{{0.0, 0.0, 0.0, 0.0}};
  target_wheel_speeds_.writeFromNonRT(zero_speeds);
  cmd_vel_subscriber_ = controller_nh.subscribe(
      "/cmd_vel", 1, &HeroChassisController::cmdVelCallback, this);

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

void HeroChassisController::cmdVelCallback(const geometry_msgs::Twist& command) {
  const ros::Time now = ros::Time::now();
  const double dt = last_command_time_.isZero()
                        ? 0.0
                        : (now - last_command_time_).toSec();
  last_command_time_ = now;
  const double linear_step = max_linear_acceleration_ * std::max(0.0, dt);
  const double angular_step = max_angular_acceleration_ * std::max(0.0, dt);
  const auto limit_step = [](double target, double current, double step) {
    return std::max(current - step, std::min(current + step, target));
  };
  limited_vx_ = limit_step(command.linear.x, limited_vx_, linear_step);
  limited_vy_ = limit_step(command.linear.y, limited_vy_, linear_step);
  limited_wz_ = limit_step(command.angular.z, limited_wz_, angular_step);
  double vx = limited_vx_;
  double vy = limited_vy_;

  if (velocity_mode_ == "world") {
    try {
      tf::Stamped<tf::Vector3> world_velocity(
          tf::Vector3(command.linear.x, command.linear.y, 0.0), ros::Time(0),
          odom_frame_);
      tf::Stamped<tf::Vector3> body_velocity;
      tf_listener_.transformVector(base_frame_, world_velocity, body_velocity);
      vx = body_velocity.x();
      vy = body_velocity.y();
    } catch (const tf::TransformException& exception) {
      ROS_WARN_THROTTLE(1.0, "Failed to transform /cmd_vel from %s to %s: %s",
                        odom_frame_.c_str(), base_frame_.c_str(),
                        exception.what());
      target_wheel_speeds_.writeFromNonRT(
          std::array<double, 4>{{0.0, 0.0, 0.0, 0.0}});
      return;
    }
  }

  const double half_sum = 0.5 * (chassis_length_ + chassis_width_);
  const double scale = 1.0 / wheel_radius_;
  std::array<double, 4> target_speeds{{
      scale * (vx - vy - half_sum * command.angular.z),
      scale * (vx + vy + half_sum * command.angular.z),
      scale * (vx + vy - half_sum * command.angular.z),
      scale * (vx - vy + half_sum * command.angular.z)}};
  target_wheel_speeds_.writeFromNonRT(target_speeds);
}

void HeroChassisController::update(const ros::Time& time,
                                   const ros::Duration& period) {
  const std::array<double, 4>* target_speeds = target_wheel_speeds_.readFromRT();
  const double front_left_error =
      (*target_speeds)[0] - front_left_joint_.getVelocity();
  const double front_right_error =
      (*target_speeds)[1] - front_right_joint_.getVelocity();
  const double back_left_error =
      (*target_speeds)[2] - back_left_joint_.getVelocity();
  const double back_right_error =
      (*target_speeds)[3] - back_right_joint_.getVelocity();

  const double half_sum = 0.5 * (chassis_length_ + chassis_width_);
  const double front_left_velocity = front_left_joint_.getVelocity();
  const double front_right_velocity = front_right_joint_.getVelocity();
  const double back_left_velocity = back_left_joint_.getVelocity();
  const double back_right_velocity = back_right_joint_.getVelocity();
  const double vx = wheel_radius_ *
                    (front_left_velocity + front_right_velocity +
                     back_left_velocity + back_right_velocity) / 4.0;
  const double vy = wheel_radius_ *
                    (-front_left_velocity + front_right_velocity +
                     back_left_velocity - back_right_velocity) / 4.0;
  const double wz = wheel_radius_ *
                    (-front_left_velocity + front_right_velocity -
                     back_left_velocity + back_right_velocity) /
                    (4.0 * half_sum);

  if (!odom_initialized_) {
    odom_initialized_ = true;
  } else {
    const double dt = period.toSec();
    x_ += (vx * std::cos(yaw_) - vy * std::sin(yaw_)) * dt;
    y_ += (vx * std::sin(yaw_) + vy * std::cos(yaw_)) * dt;
    yaw_ += wz * dt;
  }

  nav_msgs::Odometry odom;
  odom.header.stamp = time;
  odom.header.frame_id = odom_frame_;
  odom.child_frame_id = base_frame_;
  odom.pose.pose.position.x = x_;
  odom.pose.pose.position.y = y_;
  odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);
  odom.twist.twist.linear.x = vx;
  odom.twist.twist.linear.y = vy;
  odom.twist.twist.angular.z = wz;
  odom.pose.covariance[0] = 1e-3;
  odom.pose.covariance[7] = 1e-3;
  odom.pose.covariance[14] = 1e6;
  odom.pose.covariance[21] = 1e6;
  odom.pose.covariance[28] = 1e6;
  odom.pose.covariance[35] = 1e-3;
  odom.twist.covariance[0] = 1e-3;
  odom.twist.covariance[7] = 1e-3;
  odom.twist.covariance[14] = 1e6;
  odom.twist.covariance[21] = 1e6;
  odom.twist.covariance[28] = 1e6;
  odom.twist.covariance[35] = 1e-3;
  odom_publisher_.publish(odom);

  geometry_msgs::TransformStamped transform;
  transform.header.stamp = time;
  transform.header.frame_id = odom_frame_;
  transform.child_frame_id = base_frame_;
  transform.transform.translation.x = x_;
  transform.transform.translation.y = y_;
  transform.transform.rotation = odom.pose.pose.orientation;
  tf_broadcaster_.sendTransform(transform);

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
