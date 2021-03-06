/*!
 * @file     Joint.h
 * @author   Giuseppe Rizzi
 * @date     15.04.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

// std
#include <memory>

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

using mode_t = JointCommandModes;

const std::map<mode_t, std::string> mapModeToString{{mode_t::ERROR, "ERROR"},
                                                    {mode_t::EMERGENCY_STOP, "EMERGENCY_STOP"},
                                                    {mode_t::NOMODE, "NOMODE"},
                                                    {mode_t::MODE_POSITION, "MODE_POSITION"},
                                                    {mode_t::MODE_VELOCITY, "MODE_VELOCITY"},
                                                    {mode_t::MODE_EFFORT, "MODE_EFFORT"}};

class JointData {
 public:
  using handle_ptr = std::unique_ptr<JointHandle>;

  using state_handle_ptr = std::unique_ptr<JointStateHandle>;
  using mode_handle_ptr = std::unique_ptr<JointModeHandle>;

  using eff_soft_handle_ptr = std::unique_ptr<EffortJointSoftLimitsHandle>;
  using pos_soft_handle_ptr = std::unique_ptr<PositionJointSoftLimitsHandle>;
  using vel_soft_handle_ptr = std::unique_ptr<VelocityJointSoftLimitsHandle>;

  using eff_sat_handle_ptr = std::unique_ptr<EffortJointSaturationHandle>;
  using pos_sat_handle_ptr = std::unique_ptr<PositionJointSaturationHandle>;
  using vel_sat_handle_ptr = std::unique_ptr<VelocityJointSaturationHandle>;


  explicit JointData(const std::string& name);
  ~JointData() = default;

 private:
  std::string name_;
  int type_;
  state_handle_ptr state_handle_;
  mode_handle_ptr mode_handle_;

  // limits
  bool has_limits_;
  bool has_soft_limits_;
  double lower_limit_;
  double upper_limit_;
  double velocity_limit_;
  double effort_limit_;
  eff_soft_handle_ptr effort_limits_handle_;
  pos_soft_handle_ptr position_limits_handle_;
  vel_soft_handle_ptr velocity_limits_handle_;
  eff_sat_handle_ptr effort_sat_handle_;
  pos_sat_handle_ptr position_sat_handle_;
  vel_sat_handle_ptr velocity_sat_handle_;

  // mode
  JointCommandModes current_mode_;
  JointCommandModes mode_;

  // state
  double position_;
  double velocity_;
  double effort_;

  // control
  double position_cmd_;
  double velocity_cmd_;
  double effort_cmd_;
  handle_ptr position_handle_;
  handle_ptr velocity_handle_;
  handle_ptr effort_handle_;
  control_toolbox::Pid pid_controller_;


 private:
  /**
   * @brief Init all handles
   */
   void initHandles();
 public:

  /**
   * @brief Init the PID controller from ROS param server
   * The controller looks for the PID parameters under the namespace
   * "/gazebo_ros_control/pid_gains/<joint_name>". If these are not found
   * initialization fails. The gains are reconfigurable parameters and can be changed using dynamic_reconfigure
   * e.g. rosrun rqt_reconfigure rqt_reconfigure
   * @return false if the controller failed to initialize
   */
  bool initPid();

  //! @brief Init limits from the urdf model if found, otherwise from the parameter server
  // The soft and hard limits are looked for in the urdf model first. If not found, they
  // are then searched in the "gazebo_ros_control/limits/joint_limits/limits" namespace/
  //! @param urdf_model The urdf model of the robot
  void initLimits(const urdf::Model* urdf_model);


  /**
   * @brief Check for joint limits
   * @return true if the joint has hard limits
   */
  inline bool hasLimits(){ return has_limits_; }

  /**
   * @brief Check for soft joint limits
   * @return true if the joint has soft joint limits
   */
  inline bool hasSoftLimits() {return has_soft_limits_; }

  /**
   * @brief Compute the effort command given a desired position
   * The internal pid is used to compute the effort given the error and the control period
   * which is used to compute the error derivative and integral term.
   * @param position_desired The desired joint position
   * @param dt The control interval
   * @return The effort control input
   */
  double computeEffortCommand(const double position_desired, const ros::Duration& dt);

  /**
   * @brief Enforce all saturation limits
   * @param dt: used to compute position/velocity limits given velocity/acceleration limits
   */
  void enforceSaturationLimits(ros::Duration& dt);

  /**
   * @brief Enforce all soft limits
   * @param dt: used to compute position/velocity limits given velocity acceleration limits
   */
  void enforceSoftLimits(ros::Duration& dt);

  /**
   * @brief Reset commands
   * @param position
   * @param velocity
   * @param effort
   */
  void resetCommands(const double position, const double velocity, const double effort);
  /*
   * Getters & Setters
   */
  inline const std::string &getName() const { return name_; }
  inline void setName(const std::string &name) { name_ = name; }

  inline int getType() const { return type_; }
  inline void setType(int type) { type_ = type; }

  inline double getLowerLimit() const { return lower_limit_; }
  inline void setLowerLimit(double lower_limit) { lower_limit_ = lower_limit; }

  inline double getUpperLimit() const { return upper_limit_; }
  inline void setUpperLimit(double upper_limit) { upper_limit_ = upper_limit; }

  inline double getEffortLimit() const { return effort_limit_; }
  inline void setEffortLimit(double effort_limit) { effort_limit_ = effort_limit; }

  inline double getVelocityLimit() const { return velocity_limit_; }
  inline void setVelocityLimit(double velocity_limit) { velocity_limit_ = velocity_limit; }

  inline const JointCommandModes &getMode() const { return mode_; }
  inline void setMode(const JointCommandModes &mode) { mode_ = mode; }

  inline hardware_interface::JointCommandModes getCurrentMode() const { return current_mode_; }
  inline void setCurrentMode(hardware_interface::JointCommandModes current_mode) { current_mode_ = current_mode; }

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

  inline JointStateHandle& getStateHandle() { return *state_handle_; }
  inline JointModeHandle& getModeHandle() { return *mode_handle_; }
  inline JointHandle& getEffortHandle(){ return *effort_handle_; }
  inline JointHandle& getPositionHandle(){return *position_handle_;}
  inline JointHandle& getVelocityHandle(){return *velocity_handle_;}
  inline EffortJointSoftLimitsHandle& getEffortLimitsHandle() { return *effort_limits_handle_; }
  inline PositionJointSoftLimitsHandle& getPositionLimitsHandle() { return *position_limits_handle_; }
  inline VelocityJointSoftLimitsHandle& getVelocityLimitsHandle() { return *velocity_limits_handle_; }
  inline EffortJointSaturationHandle& getEffortSatHandle() { return *effort_sat_handle_; }
  inline PositionJointSaturationHandle& getPositionSatHandle() { return *position_sat_handle_; }
  inline VelocityJointSaturationHandle& getVelocitySatHandle() { return *velocity_sat_handle_; }
};

using joint_data_ptr = std::shared_ptr<JointData>;
}
