#pragma once

// ros_control
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_mode_interface.h>
#include <hardware_interface/robot_hw.h>

#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>

// Gazebo
#include <gazebo/common/common.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/gazebo.hh>

// ROS
#include <ros/ros.h>
#include <angles/angles.h>
#include <pluginlib/class_list_macros.h>

// gazebo_hw_plugin
#include <gazebo_hw_plugin/robot_hw_sim.h>
#include <gazebo_hw_plugin/joint_data_group.hpp>

// URDF
#include <urdf/model.h>



namespace gazebo_hw_plugin
{

using mode_t = hardware_interface::JointCommandModes;
const std::map<mode_t, std::string> mapModeToString{{mode_t::ERROR, "ERROR"},
                                                    {mode_t::EMERGENCY_STOP, "EMERGENCY_STOP"},
                                                    {mode_t::NOMODE, "NOMODE"},
                                                    {mode_t::MODE_POSITION, "MODE_POSITION"},
                                                    {mode_t::MODE_VELOCITY, "MODE_VELOCITY"},
                                                    {mode_t::MODE_EFFORT, "MODE_EFFORT"}};

class MultiInterfaceRobotHWSim : public gazebo_hw_plugin::RobotHWSim
{
public:

  virtual bool initSim(
    const std::string& robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions,
    bool perfect_pos_vel_crtl=false);

  virtual void readSim(ros::Time time, ros::Duration period);

  virtual void writeSim(ros::Time time, ros::Duration period);

  virtual void eStopActive(const bool active);

protected:

  JointDataGroup joints_;
  std::vector<gazebo::physics::JointPtr> sim_joints_;
  std::vector<double> joint_initial_position_at_switch_;


  std::string physics_type_;

  // e_stop_active_ is true if the emergency stop is active.
  bool e_stop_active_, last_e_stop_active_;
  bool perfect_posvel_ctrl_ = false;
};

typedef boost::shared_ptr<MultiInterfaceRobotHWSim> MultiInterfaceRobotHWSimPtr;

}
