cmake_minimum_required(VERSION 2.8.3)
project(gazebo_hw_plugin)

add_definitions(-std=c++17)

set(PACKAGE_DEPENDENCIES
    message_logger
    gazebo_dev
    roscpp
    std_msgs
    control_toolbox
    controller_manager
    hardware_interface
    transmission_interface
    pluginlib
    joint_limits_interface
    urdf
    angles
)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  ${PACKAGE_DEPENDENCIES}
)

catkin_package(
  CATKIN_DEPENDS ${PACKAGE_DEPENDENCIES}
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME} multi_interface_robot_hw_sim
)

link_directories(
  ${catkin_LIBRARY_DIRS}
)

include_directories(include
  ${Boost_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

## Libraries
add_library(${PROJECT_NAME} src/gazebo_hw_plugin.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_library(multi_interface_robot_hw_sim
        src/joint_data.cpp
        src/joint_data_group.cpp
        src/multi_interface_robot_hw_sim.cpp)
target_link_libraries(multi_interface_robot_hw_sim ${catkin_LIBRARIES})

## Test
#catkin_add_gtest(limits_test test/limits.cpp)
#target_link_libraries(limits_test multi_interface_robot_hw_sim ${catkin_LIBRARIES})
#add_dependencies(limits_test multi_interface_robot_hw_sim ${catkin_EXPORTED_TARGETS})


## Install
install(TARGETS ${PROJECT_NAME} multi_interface_robot_hw_sim
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(FILES multi_interface_sim_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
