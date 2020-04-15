/*!
 * @file     joint_data.cpp
 * @author   Giuseppe Rizzi
 * @date     15.04.2020
 * @version  1.0
 * @brief    description
 */

#include "gazebo_hw_plugin/joint_data.hpp"

// TODO fill with remaining functions

using namespace gazebo_hw_plugin;

JointData::JointData(const std::string& name) : name_(name){
  setPosition(0.0);
  setVelocity(0.0);
  setEffort(0.0);
  setPositionCmd(0.0);
  setVelocityCmd(0.0);
  setEffortCmd(0.0);
}