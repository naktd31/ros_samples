#include <ros/ros.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <std_msgs/Float64MultiArray.h>
#include <vector>

class SamplebotHW : public hardware_interface::RobotHW
{
public:
  SamplebotHW(ros::NodeHandle &nh);

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time, ros::Duration);
  void write(ros::Time, ros::Duration);

protected:
  hardware_interface::JointStateInterface jnt_state_interface_;
  hardware_interface::VelocityJointInterface jnt_vel_interface_;

  ros::Subscriber enc_sub_;
  ros::Publisher motor_pub_;
  void encoderCb(std_msgs::Float64MultiArray msg);

  double cmd_[1];
  double pos_[1];
  double vel_[1];
  double eff_[1];

};
