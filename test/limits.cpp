/*!
 * @file     limits.cpp
 * @author   Giuseppe Rizzi
 * @date     18.04.2020
 * @version  1.0
 * @brief    description
 */

#include <urdf/model.h>
#include <ros/ros.h>

#include "gazebo_hw_plugin/joint_data.hpp"
#include "gazebo_hw_plugin/joint_data_group.hpp"
#include <gtest/gtest.h>
#include <ros/package.h>

// check this http://docs.ros.org/melodic/api/joint_limits_interface/html/c++/index.html

TEST(LimitsTest, hardLimitsJointUrdf){
  urdf::Model model;
//  <limit effort="30" velocity="1.0" lower="-2.2" upper="0.7" />
  ASSERT_TRUE(model.initParam("/robot_description"));

  gazebo_hw_plugin::JointData joint("joint1");
  joint.initLimits(&model);
  ASSERT_TRUE(joint.hasLimits());

  EXPECT_EQ(30.0, joint.getEffortLimit());
  EXPECT_EQ(0.7, joint.getUpperLimit());
  EXPECT_EQ(-2.2, joint.getLowerLimit());
  EXPECT_EQ(1.0, joint.getVelocityLimit());


  ros::Duration dt(0.01);
  joint.setPositionCmd(std::numeric_limits<double>::max());
  joint.setVelocityCmd(std::numeric_limits<double>::max());
  joint.setEffortCmd(std::numeric_limits<double>::max());
  joint.enforceSaturationLimits(dt);
  double position_upper = joint.getPosition() + dt.toSec() * joint.getVelocityLimit();
  EXPECT_EQ(position_upper, joint.getPositionCmd());
  EXPECT_EQ(1.0, joint.getVelocityCmd());
  EXPECT_EQ(30.0, joint.getEffortCmd());

  joint.resetCommands(0.0, 0.0, 0.0);
  joint.setPositionCmd(std::numeric_limits<double>::lowest());
  joint.setVelocityCmd(std::numeric_limits<double>::lowest());
  joint.setEffortCmd(std::numeric_limits<double>::lowest());
  joint.enforceSaturationLimits(dt);

  // The position command is limited in velocity using the previous command and period
  // to estimate the position reference velocity and clamp it with the velocity effort
  // In other words, the difference between two successive position commands cannot be
  // larger than dt*velocity_limit
  double position_lower = position_upper - dt.toSec() * joint.getVelocityLimit();
  EXPECT_EQ(position_lower, joint.getPositionCmd());
  EXPECT_EQ(-1.0, joint.getVelocityCmd());
  EXPECT_EQ(-30.0, joint.getEffortCmd());
};


TEST(LimitsTest, hardLimitsJointGroupUrdf){
  urdf::Model model;
  ASSERT_TRUE(model.initParam("/robot_description"));

  gazebo_hw_plugin::JointDataGroup joint_data_group(model);
  ASSERT_TRUE(joint_data_group.initFromModel(model));

  auto joint = joint_data_group.getJoints().front();
  ASSERT_TRUE(joint->hasLimits());
  EXPECT_EQ(30.0, joint->getEffortLimit());
  EXPECT_EQ(0.7, joint->getUpperLimit());
  EXPECT_EQ(-2.2, joint->getLowerLimit());
  EXPECT_EQ(1.0, joint->getVelocityLimit());

  ros::Duration dt(0.01);
  joint->setPositionCmd(std::numeric_limits<double>::max());
  joint->setVelocityCmd(std::numeric_limits<double>::max());
  joint->setEffortCmd(std::numeric_limits<double>::max());
  joint->enforceSaturationLimits(dt);
  double position_upper = joint->getPosition() + dt.toSec() * joint->getVelocityLimit();
  EXPECT_EQ(position_upper, joint->getPositionCmd());
  EXPECT_EQ(1.0, joint->getVelocityCmd());
  EXPECT_EQ(30.0, joint->getEffortCmd());


  joint->resetCommands(0.0, 0.0, 0.0);
  joint->setPositionCmd(-std::numeric_limits<double>::max());
  joint->setVelocityCmd(-std::numeric_limits<double>::max());
  joint->setEffortCmd(-std::numeric_limits<double>::max());
  joint->enforceSaturationLimits(dt);
  // The position command is limited in velocity using the previous command and period
  // to estimate the position reference velocity and clamp it with the velocity effort
  // In other words, the difference between two successive position commands cannot be
  // larger than dt*velocity_limit
  double position_lower = position_upper - dt.toSec() * joint->getVelocityLimit();
  EXPECT_EQ(position_lower, joint->getPositionCmd());
  EXPECT_EQ(-1.0, joint->getVelocityCmd());
  EXPECT_EQ(-30.0, joint->getEffortCmd());
};

TEST(LimitsTest, softLimitsUrdf){
  urdf::Model model;
  ASSERT_TRUE(model.initParam("/robot_description"));

  gazebo_hw_plugin::JointData joint("joint1");
  joint.initLimits(&model);
  ASSERT_TRUE(joint.hasSoftLimits());
};

TEST(LimitsTest, hardLimitsFromParam){
  urdf::Model model;
  ASSERT_TRUE(model.initParam("/robot_description"));

  gazebo_hw_plugin::JointData joint("joint2");
  joint.initLimits(&model);
  ASSERT_TRUE(joint.hasLimits());

  EXPECT_EQ(25.0, joint.getEffortLimit());
  EXPECT_EQ(1.0, joint.getUpperLimit());
  EXPECT_EQ(-1.0, joint.getLowerLimit());
  EXPECT_EQ(2.0, joint.getVelocityLimit());
};



int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}