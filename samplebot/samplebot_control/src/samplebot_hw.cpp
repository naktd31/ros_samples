#include <ros/ros.h>
#include <ros/package.h>
#include <angles/angles.h>
#include <samplebot_control/samplebot_hw.h>

SamplebotHW::SamplebotHW(ros::NodeHandle &nh)
{
  // connect and register the joint state interface
  hardware_interface::JointStateHandle state_handle_1("wheel_joint", &pos_[0], &vel_[0], &eff_[0]);
  jnt_state_interface_.registerHandle(state_handle_1);
  registerInterface(&jnt_state_interface_);

  // connect and register the joint velocity interface
  hardware_interface::JointHandle vel_handle_1(jnt_state_interface_.getHandle("wheel_joint"), &cmd_[0]);
  jnt_vel_interface_.registerHandle(vel_handle_1);
  registerInterface(&jnt_vel_interface_);

  enc_sub_ = nh.subscribe("encoder", 10, &SamplebotHW::encoderCb, this);
  motor_pub_ = nh.advertise<std_msgs::Float64MultiArray>("motor_cmd", 10);
}

void SamplebotHW::read(ros::Time time, ros::Duration period)
{
  // read real robot state
  pos_[0] += vel_[0] * period.toSec();
}

void SamplebotHW::write(ros::Time time, ros::Duration period)
{
  // send control operation
  std_msgs::Float64MultiArray msg;
  msg.data.push_back(cmd_[0]);
  motor_pub_.publish(msg);
}

void SamplebotHW::encoderCb(std_msgs::Float64MultiArray msg)
{
  for(size_t i=0; i<msg.data.size(); i++){ pos_[i] = msg.data[i]; }
}
