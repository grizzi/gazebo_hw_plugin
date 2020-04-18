/*!
 * @file     joint_data.cpp
 * @author   Giuseppe Rizzi
 * @date     15.04.2020
 * @version  1.0
 * @brief    description
 */

#include "gazebo_hw_plugin/joint_data.hpp"

using namespace gazebo_hw_plugin;

JointData::JointData(const std::string &name) :
    name_(name) {
  setPosition(0.0);
  setVelocity(0.0);
  setEffort(0.0);
  setPositionCmd(0.0);
  setVelocityCmd(0.0);
  setEffortCmd(0.0);
  setCurrentMode(JointCommandModes::NOMODE);

  initHandles();
}

void JointData::initHandles() {

  state_handle_ = std::make_unique<JointStateHandle>(name_, &position_, &velocity_, &effort_);
  mode_handle_ = std::make_unique<JointModeHandle>(name_, &mode_);

  position_handle_ = std::make_unique<JointHandle>(*state_handle_, &position_cmd_);
  velocity_handle_ = std::make_unique<JointHandle>(*state_handle_, &velocity_cmd_);
  effort_handle_ = std::make_unique<JointHandle>(*state_handle_, &effort_cmd_);
}

bool JointData::initPid() {
  const ros::NodeHandle nh("/gazebo_ros_control/pid_gains/" + getName());
  if (!pid_controller_.init(nh, true)) {
    ROS_ERROR_STREAM("Failed to initialize pid controller for joint" << getName());
    return false;
  }
}

double JointData::computeEffortCommand(const double position_desired, const ros::Duration &dt) {
  double error;
  switch (type_) {
    case urdf::Joint::REVOLUTE:
      angles::shortest_angular_distance_with_limits(position_,
                                                    position_desired,
                                                    lower_limit_,
                                                    upper_limit_,
                                                    error);
      break;
    case urdf::Joint::CONTINUOUS:
      error = angles::shortest_angular_distance(position_,
                                                position_desired);
      break;
    default:error = position_desired - position_;
  }
  double effort = std::min(std::max(pid_controller_.computeCommand(error, dt), -effort_limit_), effort_limit_);
  return effort;
}

void JointData::initLimits(const urdf::Model *urdf_model) {
  type_ = urdf::Joint::UNKNOWN;
  lower_limit_ = -std::numeric_limits<double>::max();
  upper_limit_ = std::numeric_limits<double>::max();
  effort_limit_ = std::numeric_limits<double>::max();

  has_limits_ = false;
  has_soft_limits_ = false;
  joint_limits_interface::JointLimits limits;
  joint_limits_interface::SoftJointLimits soft_limits;

  if (urdf_model) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(name_);
    if (urdf_joint) {
      type_ = urdf_joint->type;
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits_ = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits_ = true;
    }
  }

  // Get limits from the parameter server.
  const ros::NodeHandle nh("/gazebo_ros_control/limits");
  if (joint_limits_interface::getJointLimits(name_, nh, limits)) has_limits_ = true;
  if (!has_limits_) return;

  // Infer the joint type.
  if (type_ == urdf::Joint::UNKNOWN) {
    if (limits.has_position_limits) {
      type_ = urdf::Joint::REVOLUTE;
    } else {
      if (limits.angle_wraparound)
        type_ = urdf::Joint::CONTINUOUS;
      else
        type_ = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits) {
    lower_limit_ = limits.min_position;
    upper_limit_ = limits.max_position;
  }
  if (limits.has_effort_limits)
    effort_limit_ = limits.max_effort;

  effort_limits_handle_ = std::make_unique<EffortJointSoftLimitsHandle>(*effort_handle_, limits, soft_limits);
  position_limits_handle_ = std::make_unique<PositionJointSoftLimitsHandle>(*position_handle_, limits, soft_limits);
  velocity_limits_handle_ = std::make_unique<VelocityJointSoftLimitsHandle>(*velocity_handle_, limits, soft_limits);

  effort_sat_handle_ = std::make_unique<EffortJointSaturationHandle>(*effort_handle_, limits);
  position_sat_handle_ = std::make_unique<PositionJointSaturationHandle>(*position_handle_, limits);
  velocity_sat_handle_ = std::make_unique<VelocityJointSaturationHandle>(*velocity_handle_, limits);
}
