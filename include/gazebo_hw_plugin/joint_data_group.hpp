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

  bool initFromNames(std::vector<std::string> &joint_names) {
    if (joint_names.empty()) return false;

    joints_.reserve(joint_names.size());
    for (const auto &name : joint_names)
      joints_.push_back(std::make_shared<JointData>(name));
    initLimits();
    registerHandles();
    if (!initPid()) return false;
  }

  void initLimits() {
    for (auto &joint : joints_)
      joint->initLimits(&urdf_model_);
  }

  bool initPid() {
    for (auto &joint : joints_) {
      if (!joint->initPid()) return false;
    }
    return true;
  }

  void registerHandles() {
    for (auto &joint : joints_) {
      js_interface_.registerHandle(joint->getStateHandle());
      jm_interface_.registerHandle(joint->getModeHandle());
      ej_interface_.registerHandle(joint->getEffortHandle());
      pj_interface_.registerHandle(joint->getPositionHandle());
      vj_interface_.registerHandle(joint->getVelocityHandle());
      if (joint->hasSoftLimits()) {
        ej_limits_interface_.registerHandle(joint->getEffortLimitsHandle());
        pj_limits_interface_.registerHandle(joint->getPositionLimitsHandle());
        vj_limits_interface_.registerHandle(joint->getVelocityLimitsHandle());
      }
      if (joint->hasLimits()) {
        ej_sat_interface_.registerHandle(joint->getEffortSatHandle());
        pj_sat_interface_.registerHandle(joint->getPositionSatHandle());
        vj_sat_interface_.registerHandle(joint->getVelocitySatHandle());
      }
    }
  }

  std::vector<joint_data_ptr> &getJoints() { return joints_; }

  JointStateInterface &getStateInterface() { return js_interface_; };
  JointModeInterface &getModeInterface() { return jm_interface_; }
  EffortJointInterface &getEffortInterface() { return ej_interface_; }
  PositionJointInterface &getPositionInterface() { return pj_interface_; }
  VelocityJointInterface &getVelocityInterface() { return vj_interface_; }

  void enforceLimits(const ros::Duration &period) {
    ej_sat_interface_.enforceLimits(period);
    ej_limits_interface_.enforceLimits(period);
    pj_sat_interface_.enforceLimits(period);
    pj_limits_interface_.enforceLimits(period);
    vj_sat_interface_.enforceLimits(period);
    vj_limits_interface_.enforceLimits(period);
  }

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
