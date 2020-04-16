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

  std::vector<std::string> names;
  for (size_t i = 0; i < transmissions.size(); i++) {
    names.push_back(transmissions[i].joints_[0].name_);
  }

  joints_ = JointDataGroup(*urdf_model);
  if (!joints_.initFromNames(names)) {
    ROS_ERROR("Failed to initialize joints.");
    return false;
  };

  // Register interfaces
  registerInterface(&joints_.getStateInterface());
  registerInterface(&joints_.getModeInterface());
  registerInterface(&joints_.getEffortInterface());
  registerInterface(&joints_.getPositionInterface());
  registerInterface(&joints_.getVelocityInterface());

  joint_initial_position_at_switch_.resize(names.size());

  // Initialize values
  perfect_posvel_ctrl_ = perfect_posvel_ctrl;
  if (perfect_posvel_ctrl_)
    ROS_INFO("\n\n\n\n\nPerfect position/velocity control enabled: EFFORT control will NOT work\n\n\n\n\n");


  // init simulation joints
  for (auto& joint_data : joints_.getJoints()) {
    gazebo::physics::JointPtr joint = parent_model->GetJoint(joint_data.getName());
    if (!joint) {
      ROS_ERROR_STREAM("Joint named " << joint_data.getName() << "not in the gazebo model.");
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
      joint->SetParam("fmax", 0, joint_data.getEffortLimit());
#else
      joint->SetMaxForce(0, joint_data.getEffortLimit());
#endif
    }
  }

  // Initialize the emergency stop code.
  e_stop_active_ = false;
  last_e_stop_active_ = false;

  return true;
}

void MultiInterfaceRobotHWSim::readSim(ros::Time time, ros::Duration period) {
  size_t i=0;
  for (auto& joint_data : joints_.getJoints()) {
    // Gazebo has an interesting API...
#if GAZEBO_MAJOR_VERSION >= 8
    double position = sim_joints_[i]->Position(0);
#else
    double position = sim_joints_[i]->GetAngle(0).Radian();
#endif
    if (joint_data.getType() == urdf::Joint::PRISMATIC) {
      joint_data.setPosition(position);
    } else {
      joint_data.getPosition() += angles::shortest_angular_distance(joint_data.getPosition(),
                                                              position);
    }
    joint_data.setVelocity(sim_joints_[i]->GetVelocity(0));
    joint_data.setEffort(sim_joints_[i]->GetForce((unsigned int) (0)));
  }
}

void MultiInterfaceRobotHWSim::writeSim(ros::Time time, ros::Duration period) {

//  if (e_stop_active_) {
//    if (!last_e_stop_active_) {
//      last_joint_position_command_ = joint_position_;
//      last_e_stop_active_ = true;
//    }
//    joint_position_command_ = last_joint_position_command_;
//  } else {
//    last_e_stop_active_ = false;
//  }

  joints_.enforceLimits(period);

  // TODO cleaner solution to this
  static size_t velocity_idx = 0;
  size_t j = 0;
  for (auto& joint : joints_.getJoints()) {

    const auto mode = joint.getMode();
    if (joint.getCurrentMode() != mode) {
      ROS_INFO_STREAM( "Joint " << joint.getName() << " switched to [" << mapModeToString.at(mode) << "] mode.");
      joint.setCurrentMode(mode);
      joint_initial_position_at_switch_[j] = joint.getPosition();
    }

    switch (mode) {
      case hardware_interface::JointCommandModes::NOMODE:
      case hardware_interface::JointCommandModes::EMERGENCY_STOP:
      case hardware_interface::JointCommandModes::ERROR: {
        if (perfect_posvel_ctrl_)
          setPositionCommand(joint_initial_position_at_switch_[j], sim_joints_[j]);
        else{
          double effort = joint.computeEffortCommand(joint_initial_position_at_switch_[j], period);
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
      }

      case hardware_interface::JointCommandModes::MODE_POSITION: {
        if (perfect_posvel_ctrl_)
          setPositionCommand(joint.getPositionCmd(), sim_joints_[j]);
        else{
          double effort = joint.computeEffortCommand(joint.getPositionCmd(), period);
          sim_joints_[j]->SetForce(0, effort);
        }
        break;
      }

      case hardware_interface::JointCommandModes::MODE_VELOCITY: {
        if (perfect_posvel_ctrl_) {
          if (physics_type_.compare("ode") == 0)
            sim_joints_[j]->SetParam("vel", 0, joint.getVelocityCmd());
          else
            sim_joints_[j]->SetVelocity(0, joint.getVelocityCmd());
          break;
        }
        double joint_position_desired = joint_initial_position_at_switch_[j] +=
                                            period.toSec() * joint.getVelocityCmd();
        double effort = joint.computeEffortCommand(joint_position_desired, period);
        sim_joints_[j]->SetForce(0, effort);
        break;
      }

      case hardware_interface::JointCommandModes::MODE_EFFORT: {
        if (perfect_posvel_ctrl_) {
          ROS_WARN_STREAM_THROTTLE(1.0,
                                   "EFFORT control not available when perfect position/velocity tracking. Freezing");
          setPositionCommand(joint.getPosition(), sim_joints_[j]);
          break;
        }
        sim_joints_[j]->SetForce(0, joint.getEffortCmd());
        break;
      }

      default:
        ROS_ERROR_STREAM("Mode: " << (int) joint.getMode() << " not handled/unrecognized");
        sim_joints_[j]->SetForce(0, 0);
    }
  }
}

void MultiInterfaceRobotHWSim::eStopActive(const bool active) {
  e_stop_active_ = active;
}


void MultiInterfaceRobotHWSim::setPositionCommand(const double position_desired, gazebo::physics::JointPtr sim_joint) {
#if GAZEBO_MAJOR_VERSION >= 9
    sim_joint_->SetPosition(0, position_desired, true);
#else
    ROS_WARN_ONCE("The default_robot_hw_sim plugin is using the Joint::SetPosition method without preserving the link velocity.");
        ROS_WARN_ONCE("As a result, gravity will not be simulated correctly for your model.");
        ROS_WARN_ONCE("Please set gazebo_pid parameters, switch to the VelocityJointInterface or EffortJointInterface, or upgrade to Gazebo 9.");
        ROS_WARN_ONCE("For details, see https://github.com/ros-simulation/gazebo_ros_pkgs/issues/612");
        sim_joint_->SetPosition(0, joint_position_desired);
#endif
}
}

PLUGINLIB_EXPORT_CLASS(gazebo_hw_plugin::MultiInterfaceRobotHWSim, gazebo_hw_plugin::RobotHWSim)
