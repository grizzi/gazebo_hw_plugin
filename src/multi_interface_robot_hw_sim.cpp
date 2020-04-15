/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Open Source Robotics Foundation
 *  Copyright (c) 2013, The Johns Hopkins University
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Open Source Robotics Foundation
 *     nor the names of its contributors may be
 *     used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Dave Coleman, Jonathan Bohren
   Desc:   Hardware Interface for any simulated robot in Gazebo
*/


#include <gazebo_hw_plugin/multi_interface_robot_hw_sim.h>
#include <urdf/model.h>

namespace {

double clamp(const double val, const double min_val, const double max_val) {
  return std::min(std::max(val, min_val), max_val);
}

}

namespace gazebo_hw_plugin {

bool MultiInterfaceRobotHWSim::initSim(
    const std::string &robot_namespace,
    ros::NodeHandle model_nh,
    gazebo::physics::ModelPtr parent_model,
    const urdf::Model *const urdf_model,
    std::vector<transmission_interface::TransmissionInfo> transmissions,
    bool perfect_posvel_ctrl) {

  // getJointLimits() searches joint_limit_nh for joint limit parameters. The format of each
  // parameter's name is "joint_limits/<joint name>". An example is "joint_limits/axle_joint".
  const ros::NodeHandle joint_limit_nh(model_nh);

  // Resize vectors to our DOF
  n_dof_ = transmissions.size();
  joint_names_.resize(n_dof_);
  joint_types_.resize(n_dof_);
  joint_lower_limits_.resize(n_dof_);
  joint_upper_limits_.resize(n_dof_);
  joint_effort_limits_.resize(n_dof_);
  joint_modes_.resize(n_dof_);
  joint_modes_current_.resize(n_dof_);
  joint_control_methods_.resize(n_dof_);
  pid_controllers_.resize(n_dof_);
  joint_position_.resize(n_dof_);
  joint_velocity_.resize(n_dof_);
  joint_effort_.resize(n_dof_);
  joint_effort_command_.resize(n_dof_);
  joint_position_command_.resize(n_dof_);
  joint_velocity_command_.resize(n_dof_);
  joint_initial_position_at_switch_.resize(n_dof_);

  // Initialize values
  perfect_posvel_ctrl_ = perfect_posvel_ctrl;
  if (perfect_posvel_ctrl_)
    ROS_INFO("\n\n\n\n\nPerfect position/velocity control enabled: EFFORT control will NOT work\n\n\n\n\n");

  for (unsigned int j = 0; j < n_dof_; j++) {
    // Check that this transmission has one joint
    if (transmissions[j].joints_.empty()) {
      ROS_ERROR_STREAM_NAMED("default_robot_hw_sim", "Transmission " << transmissions[j].name_
                                                                     << " has no associated joints.");
      return false;
    }

    std::vector<std::string> joint_interfaces = transmissions[j].joints_[0].hardware_interfaces_;
    if (joint_interfaces.empty() &&
        !(transmissions[j].actuators_.empty()) &&
        !(transmissions[j].actuators_[0].hardware_interfaces_.empty())) {
      // TODO: Deprecate HW interface specification in actuators in ROS J
      joint_interfaces = transmissions[j].actuators_[0].hardware_interfaces_;
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "The <hardware_interface> element of tranmission " <<
                                                                                                       transmissions[j].name_
                                                                                                       << " should be nested inside the <joint> element, not <actuator>. "
                                                                                                       <<
                                                                                                       "The transmission will be properly loaded, but please update "
                                                                                                       <<
                                                                                                       "your robot model to remain compatible with future versions of the plugin.");
    }

    if (joint_interfaces.empty()) {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "Joint " << transmissions[j].joints_[0].name_ <<
                                                             " of transmission " << transmissions[j].name_
                                                             << " does not specify any hardware interface. " <<
                                                             "Not adding it to the robot hardware simulation.");
      continue;
    }

    // Add data from transmission
    joint_names_[j] = transmissions[j].joints_[0].name_;
    joint_position_[j] = 1.0;
    joint_velocity_[j] = 0.0;
    joint_effort_[j] = 1.0;  // N/m for continuous joints
    joint_effort_command_[j] = 0.0;
    joint_position_command_[j] = 0.0;
    joint_velocity_command_[j] = 0.0;

    ROS_INFO_STREAM("Registering STATE handle for joint " << joint_names_[j]);
    js_interface_.registerHandle(hardware_interface::JointStateHandle(
        joint_names_[j], &joint_position_[j], &joint_velocity_[j], &joint_effort_[j]));

    ROS_INFO_STREAM("Registering MODE handle for joint " << joint_names_[j]);
    hardware_interface::JointModeHandle mode_handle;
    joint_modes_[j] = hardware_interface::JointCommandModes::NOMODE;
    joint_modes_current_[j] = hardware_interface::JointCommandModes::NOMODE;

    mode_handle = hardware_interface::JointModeHandle(joint_names_[j], &joint_modes_[j]);
    jm_interface_.registerHandle(mode_handle);

    // Command Handles
    bool cmd_position_velocity_found = false;

    for (const auto &hardware_interface : joint_interfaces) {
      bool cmd_handle_found = false;

      if (hardware_interface == "EffortJointInterface"
          || hardware_interface == "hardware_interface/EffortJointInterface") {
        ROS_INFO_STREAM("Registering EFFORT handle for joint " << joint_names_[j]);
        cmd_handle_found = true;
        hardware_interface::JointHandle eff_handle;
        eff_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_effort_command_[j]);
        ej_interface_.registerHandle(eff_handle);
        registerJointLimits(joint_names_[j], eff_handle, EFFORT,
                            joint_limit_nh, urdf_model,
                            &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                            &joint_effort_limits_[j]);
      }

      if (hardware_interface == "PositionJointInterface"
          || hardware_interface == "hardware_interface/PositionJointInterface") {
        ROS_INFO_STREAM("Registering POSITION handle for joint " << joint_names_[j]);
        cmd_handle_found = true;
        cmd_position_velocity_found = true;
        hardware_interface::JointHandle pos_handle;
        pos_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_position_command_[j]);
        pj_interface_.registerHandle(pos_handle);
        registerJointLimits(joint_names_[j], pos_handle, POSITION,
                            joint_limit_nh, urdf_model,
                            &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                            &joint_effort_limits_[j]);
      }

      if (hardware_interface == "VelocityJointInterface"
          || hardware_interface == "hardware_interface/VelocityJointInterface") {
        ROS_INFO_STREAM("Registering VELOCITY handle for joint " << joint_names_[j]);
        cmd_handle_found = true;
        cmd_position_velocity_found = true;
        hardware_interface::JointHandle vel_handle;
        vel_handle = hardware_interface::JointHandle(js_interface_.getHandle(joint_names_[j]),
                                                     &joint_velocity_command_[j]);
        vj_interface_.registerHandle(vel_handle);
        registerJointLimits(joint_names_[j], vel_handle, VELOCITY,
                            joint_limit_nh, urdf_model,
                            &joint_types_[j], &joint_lower_limits_[j], &joint_upper_limits_[j],
                            &joint_effort_limits_[j]);
      }

      if (!cmd_handle_found) {
        ROS_FATAL_STREAM_NAMED("default_robot_hw_sim", "No matching hardware interface found for '"
            << hardware_interface << "' while loading interfaces for " << joint_names_[j]);
        return false;
      }

      if (hardware_interface == "EffortJointInterface" || hardware_interface == "PositionJointInterface"
          || hardware_interface == "VelocityJointInterface") {
        ROS_WARN_STREAM("Deprecated syntax, please prepend 'hardware_interface/' to '" << hardware_interface
                                                                                       << "' within the <hardwareInterface> tag in joint '"
                                                                                       << joint_names_[j] << "'.");
      }
    }

    const ros::NodeHandle nh("/gazebo_ros_control/pid_gains/" + joint_names_[j]);
    if (cmd_position_velocity_found) {
      if (!pid_controllers_[j].init(nh, true)) {
        ROS_ERROR_STREAM("Failed to initialize pid controller for joint" << joint_names_[j]);
        return false;
      }
    }

    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_names_[j]);
    if (!joint) {
      ROS_ERROR_STREAM_NAMED("default_robot_hw", "This robot has a joint named \"" << joint_names_[j]
                                                                                   << "\" which is not in the gazebo model.");
      return false;
    }
    sim_joints_.push_back(joint);

    // get physics engine type
#if GAZEBO_MAJOR_VERSION >= 8
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->Physics();
#else
    gazebo::physics::PhysicsEnginePtr physics = gazebo::physics::get_world()->GetPhysicsEngine();
#endif
    physics_type_ = physics->GetType();
    if (physics_type_.empty()) {
      ROS_WARN_STREAM_NAMED("default_robot_hw_sim", "No physics type found.");
    }

    /*
     joint->SetParam("fmax") must be called if joint->SetAngle() or joint->SetParam("vel") are
     going to be called. joint->SetParam("fmax") must *not* be called if joint->SetForce() is
     going to be called.
     */
#if GAZEBO_MAJOR_VERSION > 2
    if (perfect_posvel_ctrl_) {
      joint->SetParam("fmax", 0, joint_effort_limits_[j]);
#else
      joint->SetMaxForce(0, joint_effort_limits_[j]);
#endif
    }
  }

  // Register interfaces
  registerInterface(&js_interface_);
  registerInterface(&jm_interface_);
  registerInterface(&ej_interface_);
  registerInterface(&pj_interface_);
  registerInterface(&vj_interface_);

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void MultiInterfaceRobotHWSim::readSim(ros::Time time, ros::Duration period) {
  for (unsigned int j = 0; j < n_dof_; j++) {
    // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
    double position = sim_joints_[j]->Position(0);
#else
    double position = sim_joints_[j]->GetAngle(0).Radian();
#endif
    if (joint_types_[j] == urdf::Joint::PRISMATIC) {
      joint_position_[j] = position;
    } else {
      joint_position_[j] += angles::shortest_angular_distance(joint_position_[j],
                                                              position);
    }
    joint_velocity_[j] = sim_joints_[j]->GetVelocity(0);
    joint_effort_[j] = sim_joints_[j]->GetForce((unsigned int) (0));
  }
}

void MultiInterfaceRobotHWSim::writeSim(ros::Time time, ros::Duration period) {
  // If the E-stop is active, joints controlled by position commands will maintain their positions.
  if (e_stop_active_) {
    if (!last_e_stop_active_) {
      last_joint_position_command_ = joint_position_;
      last_e_stop_active_ = true;
    }
    joint_position_command_ = last_joint_position_command_;
  } else {
    last_e_stop_active_ = false;
  }

  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);

  static size_t velocity_idx = 0;
  for (unsigned int j = 0; j < n_dof_; j++) {

    if (joint_modes_[j] != joint_modes_current_[j]) {
      ROS_INFO_STREAM(
          "Joint " << joint_names_[j] << " switched to [" << mapModeToString.at(joint_modes_[j]) << "] mode.");
      joint_modes_current_[j] = joint_modes_[j];
      joint_initial_position_at_switch_[j] = joint_position_[j];
    }

    const auto &mode = joint_modes_current_[j];
    switch (mode) {
      case hardware_interface::JointCommandModes::NOMODE:
      case hardware_interface::JointCommandModes::EMERGENCY_STOP:
      case hardware_interface::JointCommandModes::ERROR: {
        trackPositionCommand(joint_initial_position_at_switch_[j], j, period);
        break;
      }

      case hardware_interface::JointCommandModes::MODE_POSITION: {
        trackPositionCommand(joint_position_command_[j], j, period);
        break;
      }

      case hardware_interface::JointCommandModes::MODE_VELOCITY: {
        if (perfect_posvel_ctrl_){
          if (physics_type_.compare("ode") == 0)
            sim_joints_[j]->SetParam("vel", 0, joint_velocity_command_[j]);
          else
            sim_joints_[j]->SetVelocity(0, joint_velocity_command_[j]);
          break;
        }
        double joint_position_desired = joint_initial_position_at_switch_[j] += period.toSec() * joint_velocity_command_[j];
        trackPositionCommand(joint_position_desired, j, period);
        break;
      }

      case hardware_interface::JointCommandModes::MODE_EFFORT: {
        if (perfect_posvel_ctrl_){
          ROS_WARN_STREAM_THROTTLE(1.0, "EFFORT control not available when perfect position/velocity tracking. Freezing");
          trackPositionCommand(joint_position_[j], j, period);
          break;
        }
        sim_joints_[j]->SetForce(0, joint_effort_command_[j]);
        break;
      }

      default:ROS_ERROR_STREAM("Mode: " << (int) joint_modes_[j] << " not handled/unrecognized");
        sim_joints_[j]->SetForce(0, 0);
    }
  }
}

void MultiInterfaceRobotHWSim::eStopActive(const bool active) {
  e_stop_active_ = active;
}

// Register the limits of the joint specified by joint_name and joint_handle. The limits are
// retrieved from joint_limit_nh. If urdf_model is not NULL, limits are retrieved from it also.
// Return the joint's type, lower position limit, upper position limit, and effort limit.
void MultiInterfaceRobotHWSim::registerJointLimits(const std::string &joint_name,
                                                   const hardware_interface::JointHandle &joint_handle,
                                                   const ControlMethod ctrl_method,
                                                   const ros::NodeHandle &joint_limit_nh,
                                                   const urdf::Model *const urdf_model,
                                                   int *const joint_type, double *const lower_limit,
                                                   double *const upper_limit, double *const effort_limit) {
  *joint_type = urdf::Joint::UNKNOWN;
  *lower_limit = -std::numeric_limits<double>::max();
  *upper_limit = std::numeric_limits<double>::max();
  *effort_limit = std::numeric_limits<double>::max();

  joint_limits_interface::JointLimits limits;
  bool has_limits = false;
  joint_limits_interface::SoftJointLimits soft_limits;
  bool has_soft_limits = false;

  if (urdf_model != NULL) {
    const urdf::JointConstSharedPtr urdf_joint = urdf_model->getJoint(joint_name);
    if (urdf_joint != NULL) {
      *joint_type = urdf_joint->type;
      // Get limits from the URDF file.
      if (joint_limits_interface::getJointLimits(urdf_joint, limits))
        has_limits = true;
      if (joint_limits_interface::getSoftJointLimits(urdf_joint, soft_limits))
        has_soft_limits = true;
    }
  }
  // Get limits from the parameter server.
  if (joint_limits_interface::getJointLimits(joint_name, joint_limit_nh, limits))
    has_limits = true;

  if (!has_limits)
    return;

  if (*joint_type == urdf::Joint::UNKNOWN) {
    // Infer the joint type.

    if (limits.has_position_limits) {
      *joint_type = urdf::Joint::REVOLUTE;
    } else {
      if (limits.angle_wraparound)
        *joint_type = urdf::Joint::CONTINUOUS;
      else
        *joint_type = urdf::Joint::PRISMATIC;
    }
  }

  if (limits.has_position_limits) {
    *lower_limit = limits.min_position;
    *upper_limit = limits.max_position;
  }
  if (limits.has_effort_limits)
    *effort_limit = limits.max_effort;

  if (has_soft_limits) {
    switch (ctrl_method) {
      case EFFORT: {
        const joint_limits_interface::EffortJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        ej_limits_interface_.registerHandle(limits_handle);
      }
        break;
      case POSITION: {
        const joint_limits_interface::PositionJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        pj_limits_interface_.registerHandle(limits_handle);
      }
        break;
      case VELOCITY: {
        const joint_limits_interface::VelocityJointSoftLimitsHandle
            limits_handle(joint_handle, limits, soft_limits);
        vj_limits_interface_.registerHandle(limits_handle);
      }
        break;
    }
  } else {
    switch (ctrl_method) {
      case EFFORT: {
        const joint_limits_interface::EffortJointSaturationHandle
            sat_handle(joint_handle, limits);
        ej_sat_interface_.registerHandle(sat_handle);
      }
        break;
      case POSITION: {
        const joint_limits_interface::PositionJointSaturationHandle
            sat_handle(joint_handle, limits);
        pj_sat_interface_.registerHandle(sat_handle);
      }
        break;
      case VELOCITY: {
        const joint_limits_interface::VelocityJointSaturationHandle
            sat_handle(joint_handle, limits);
        vj_sat_interface_.registerHandle(sat_handle);
      }
        break;
    }
  }
}


void MultiInterfaceRobotHWSim::trackPositionCommand(const double joint_position_desired, const int joint_index, const ros::Duration& dt){
  if (perfect_posvel_ctrl_) {
#if GAZEBO_MAJOR_VERSION >= 9
    sim_joints_[joint_index]->SetPosition(0, joint_position_desired, true);
#else
    ROS_WARN_ONCE("The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.");
        ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
        ROS_WARN_ONCE("Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.");
        ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
        sim_joints_[j]->SetPosition(0, joint_position_desired);
#endif
  } else {
    double error = computePositionError(joint_position_desired, joint_index);
    const double effort_limit = joint_effort_limits_[joint_index];
    const double effort = clamp(pid_controllers_[joint_index].computeCommand(error, dt),
                                -effort_limit, effort_limit);
    sim_joints_[joint_index]->SetForce(0, effort);
  }
}

double MultiInterfaceRobotHWSim::computePositionError(const double reference, const int joint_index) {
  double error;
  switch (joint_types_[joint_index]) {
    case urdf::Joint::REVOLUTE:
      angles::shortest_angular_distance_with_limits(joint_position_[joint_index],
                                                    reference,
                                                    joint_lower_limits_[joint_index],
                                                    joint_upper_limits_[joint_index],
                                                    error);
      break;
    case urdf::Joint::CONTINUOUS:
      error = angles::shortest_angular_distance(joint_position_[joint_index],
                                                reference);
      break;
    default:error = joint_position_command_[joint_index] - joint_position_[joint_index];
  }
  return error;
}

}

PLUGINLIB_EXPORT_CLASS(gazebo_hw_plugin::MultiInterfaceRobotHWSim, gazebo_hw_plugin::RobotHWSim)
