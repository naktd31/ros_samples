#ifndef CR2016_HARDWARE_GAZEBO_H
#define CR2016_HARDWARE_GAZEBO_H

#include <vector>
#include <string>

#include <control_toolbox/pid.h>

#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>

#include <gazebo_ros_control/robot_hw_sim.h>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>


namespace cr2016_hardware_gazebo
{

class cr2016HardwareGazebo : public gazebo_ros_control::RobotHWSim
{
  public:
    cr2016HardwareGazebo();
  
    bool initSim(const std::string &robot_namespace,
        ros::NodeHandle model_nh,
        gazebo::physics::ModelPtr parent_model,
        const urdf::Model* const urdf_model,
        std::vector<transmission_interface::TransmissionInfo> transmissions);

    void readSim(ros::Time time, ros::Duration period);
    void writeSim(ros::Time time, ros::Duration period);
  private:
    unsigned int n_dof_;

    std::vector<double> jnt_pos_;
    std::vector<double> jnt_vel_;
    std::vector<double> jnt_eff_;

    std::vector<double> jnt_pos_cmd_;

    std::vector<gazebo::physics::JointPtr> sim_joints_;

    hardware_interface::JointStateInterface jnt_state_interface_;
    hardware_interface::EffortJointInterface jnt_eff_cmd_interface_;

    joint_limits_interface::PositionJointSoftLimitsInterface jnt_limits_interface_;

    std::vector<control_toolbox::Pid> pids_;

    template <class T>
    std::string containerToString(const T& cont, const std::string& prefix)
    {
      std::stringstream ss;
      ss << prefix;
      std::copy(cont.begin(), --cont.end(), std::ostream_iterator<typename T::value_type>(ss, prefix.c_str()));
      ss << *(--cont.end());
      return ss.str();
    }
};

}
#endif
