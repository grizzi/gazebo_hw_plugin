/*!
 * @file     joint_data_vector.h
 * @author   Giuseppe Rizzi
 * @date     16.04.2020
 * @version  1.0
 * @brief    description
 */
#pragma once

#include "gazebo_hw_plugin/joint_data.hpp"

using namespace hardware_interface;
using namespace joint_limits_interface;

namespace gazebo_hw_plugin {

struct JointDataGroup {
 public:
  JointDataGroup() = default;
  JointDataGroup(const urdf::Model &urdf_model) :
      urdf_model_(urdf_model) {}

  bool initFromNames(std::vector<std::string> &joint_names);

 private:
  //! @brief Init all the joints limits interfaces
  void initLimits();

  //! @brief Init all the pid controllers, one for each joint
  //! @return false if one or more failed to initialize
  bool initPid();

  //! @brief Register all the handles under the corresponding interfaces
  void registerHandles();

 public:
  // Getters & Setters
  std::vector<joint_data_ptr> &getJoints() { return joints_; }

  JointStateInterface &getStateInterface() { return js_interface_; };
  JointModeInterface &getModeInterface() { return jm_interface_; }
  EffortJointInterface &getEffortInterface() { return ej_interface_; }
  PositionJointInterface &getPositionInterface() { return pj_interface_; }
  VelocityJointInterface &getVelocityInterface() { return vj_interface_; }

  void enforceLimits(const ros::Duration &period);

 private:
  urdf::Model urdf_model_;
  std::vector<joint_data_ptr> joints_;

  JointStateInterface js_interface_;
  JointModeInterface jm_interface_;
  EffortJointInterface ej_interface_;
  PositionJointInterface pj_interface_;
  VelocityJointInterface vj_interface_;

  EffortJointSoftLimitsInterface ej_limits_interface_;
  PositionJointSoftLimitsInterface pj_limits_interface_;
  VelocityJointSoftLimitsInterface vj_limits_interface_;
  EffortJointSaturationInterface ej_sat_interface_;
  PositionJointSaturationInterface pj_sat_interface_;
  VelocityJointSaturationInterface vj_sat_interface_;
};

}
