/*!
 * @file     joint_data_group.cpp
 * @author   Giuseppe Rizzi
 * @date     18.04.2020
 * @version  1.0
 * @brief    description
 */

#include "gazebo_hw_plugin/joint_data_group.hpp"

using namespace hardware_interface;
using namespace joint_limits_interface;
using namespace gazebo_hw_plugin;

bool JointDataGroup::initFromNames(std::vector<std::string> &joint_names) {
  if (joint_names.empty()) return false;

  joints_.reserve(joint_names.size());
  for (const auto &name : joint_names)
    joints_.push_back(std::make_shared<JointData>(name));
  initLimits();
  registerHandles();
  if (!initPid()) return false;

  return true;
}

bool JointDataGroup::initFromModel(const urdf::Model &model) {
  for (const auto& joint_urdf : model.joints_){
    if (joint_urdf.second->type == urdf::Joint::FIXED)
      continue;

    if (joint_urdf.second->type == urdf::Joint::UNKNOWN)
      continue;

    std::string joint_name = joint_urdf.first;
    joints_.push_back(std::make_shared<JointData>(joint_name));
  }
  initLimits();
  registerHandles();
  if (!initPid()) return false;
}

void JointDataGroup::initLimits() {
  for (auto &joint : joints_)
    joint->initLimits(&urdf_model_);
}

bool JointDataGroup::initPid() {
  for (auto &joint : joints_) {
    if (!joint->initPid()) return false;
  }
  return true;
}

void JointDataGroup::registerHandles() {
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

void JointDataGroup::enforceLimits(const ros::Duration &period) {
  ej_sat_interface_.enforceLimits(period);
  ej_limits_interface_.enforceLimits(period);
  pj_sat_interface_.enforceLimits(period);
  pj_limits_interface_.enforceLimits(period);
  vj_sat_interface_.enforceLimits(period);
  vj_limits_interface_.enforceLimits(period);
}
