/*!
 * @file     Joint.h
 * @author   Giuseppe Rizzi
 * @date     15.04.2020
 * @version  1.0
 * @brief    description
 */
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

// URDF
#include <urdf/model.h>

using namespace hardware_interface;
using namespace joint_limits_interface;

namespace gazebo_hw_plugin {

class JointData {
 public:
  JointData(const std::string& name);
  ~JointData();

 private:
  std::string name_;
  int type_;

  // limits
  double lower_limit_;
  double upper_limit_;
  double effort_limit_;
  EffortJointSoftLimitsHandle effort_limits_handle_;
  PositionJointSoftLimitsHandle position_limits_handle_;
  VelocityJointSoftLimitsHandle velocity_limits_handle_;
  EffortJointSaturationHandle effort_sat_handle_;
  PositionJointSaturationHandle position_sat_handle_;
  VelocityJointSaturationHandle velocity_sat_handle_;

  // mode
  std::vector<JointCommandModes> modes_;
  JointCommandModes current_mode_;

  // sim
  double position_;
  double velocity_;
  double effort_;
  gazebo::physics::JointPtr sim_joint_;

  // control
  control_toolbox::Pid pid_;
  double position_cmd_;
  double velocity_cmd_;
  double effort_cmd_;
  hardware_interface::JointHandle& position_handle_;
  hardware_interface::JointHandle& velocity_handle_;
  hardware_interface::JointHandle& effort_handle_;


 public:

  /*
   *
   */
  bool initPid();

  /*
   *
   */
  bool hasMode(const hardware_interface::JointCommandModes& mode) const;

  void registerPositionLimits(const urdf::Model *const urdf_model);
  void registerVelocityLimits(const urdf::Model *const urdf_model);
  void registerEffortLimits(const urdf::Model *const urdf_model);

  double computeError(const double& joint_desired_position) const;

  /*
   * Getters & Setters
   */
  inline const std::string &GetName() const { return name_; }
  inline void setName(const std::string &name) { name_ = name; }

  inline int getType() const { return type_; }
  inline void setType(int type) { type_ = type; }

  inline double getLowerLimit() const { return lower_limit_; }
  inline void setLowerLimit(double lower_limit) { lower_limit_ = lower_limit; }

  inline double getUpperLimit() const { return upper_limit_; }
  inline void setUpperLimit(double upper_limit) { upper_limit_ = upper_limit; }

  inline double getEffortLimit() const { return effort_limit_; }
  inline void setEffortLimit(double effort_limit) { effort_limit_ = effort_limit; }

  inline const std::vector<JointCommandModes> &getModes() const { return modes_; }
  inline void setModes(const std::vector<JointCommandModes> &modes) { modes_ = modes; }

  inline hardware_interface::JointCommandModes getCurrentMode() const { return current_mode_; }
  inline void SetCurrentMode(hardware_interface::JointCommandModes current_mode) { current_mode_ = current_mode; }

  inline const gazebo::physics::JointPtr &getSimJoint() const { return sim_joint_; }
  inline void setSimJoint(const gazebo::physics::JointPtr &sim_joint) { sim_joint_ = sim_joint; }

  inline const control_toolbox::Pid &getPid() const { return pid_; }
  inline void setPid(const control_toolbox::Pid &pid) { pid_ = pid; }

  inline double getPosition() const { return position_; }
  inline double& getPosition() { return position_; }
  inline void setPosition(double pos_cmd) { position_ = pos_cmd; }

  inline double getVelocity() const { return velocity_; }
  inline double& getVelocity() { return velocity_; }
  inline void setVelocity(double vel_cmd) { velocity_ = vel_cmd; }

  inline double getEffort() const { return effort_; }
  inline double& getEffort() { return effort_; }
  inline void setEffort(double eff_cmd) { effort_ = eff_cmd; }

  inline double getPositionCmd() const { return position_cmd_; }
  inline double& getPositionCmd() { return position_cmd_; }
  inline void setPositionCmd(double pos_cmd) { position_cmd_ = pos_cmd; }

  inline double getVelocityCmd() const { return velocity_cmd_; }
  inline double& getVelocityCmd() { return velocity_cmd_; }
  inline void setVelocityCmd(double vel_cmd) { velocity_cmd_ = vel_cmd; }

  inline double getEffortCmd() const { return effort_cmd_; }
  inline double& getEffortCmd() { return effort_cmd_; }
  inline void setEffortCmd(double eff_cmd) { effort_cmd_ = eff_cmd; }
};

using JointDataVector = std::vector<JointData>;

}
