/*!
 * @file     limits.cpp
 * @author   Giuseppe Rizzi
 * @date     18.04.2020
 * @version  1.0
 * @brief    description
 */

#include <urdf/model.h>

#include "gazebo_hw_plugin/joint_data.hpp"
#include "gazebo_hw_plugin/joint_data_group.hpp"
#include <gtest/gtest.h>
#include <ros/package.h>

// check this http://docs.ros.org/melodic/api/joint_limits_interface/html/c++/index.html

TEST(LimitsTest, hardLimitsUrdf){
  urdf::Model model;
  std::string path = ros::package::getPath("gazebo_hw_plugin");
  path += "/resources/model.urdf";
  model.initFile(path);

  gazebo_hw_plugin::JointData joint("joint1");
  ASSERT_TRUE(joint.hasLimits());

  EXPECT_EQ(0.0, joint.getEffortLimit());
  EXPECT_EQ(0.0, joint.getUpperLimit());
  EXPECT_EQ(0.0, joint.getLowerLimit());

  joint.setPositionCmd(std::numeric_limits<double>::max());
  joint.setVelocityCmd(std::numeric_limits<double>::max());
  joint.setEffortCmd(std::numeric_limits<double>::max());

  EXPECT_EQ(0.0, joint.getPositionCmd());
  EXPECT_EQ(0.0, joint.getVelocityCmd());
  EXPECT_EQ(0.0, joint.getEffortCmd());

  joint.setPositionCmd(-std::numeric_limits<double>::max());
  joint.setVelocityCmd(-std::numeric_limits<double>::max());
  joint.setEffortCmd(-std::numeric_limits<double>::max());

  EXPECT_EQ(0.0, joint.getPositionCmd());
  EXPECT_EQ(0.0, joint.getVelocityCmd());
  EXPECT_EQ(0.0, joint.getEffortCmd());

};

TEST(LimitsTest, softLimitsUrdf){

};


int main(int argc, char **argv){
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "tester");
  return RUN_ALL_TESTS();
}