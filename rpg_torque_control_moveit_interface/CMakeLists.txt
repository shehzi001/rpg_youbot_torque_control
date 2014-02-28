cmake_minimum_required(VERSION 2.8.3)
project(rpg_torque_control_moveit_interface)

find_package(catkin REQUIRED COMPONENTS
  brics_actuator
  control_msgs
  trajectory_msgs
  message_generation
  roscpp
  rpg_youbot_common
  torque_control
  trajectory_generator
  trajectory_msgs
  actionlib
  roscpp
  message_generation
  control_msgs
  actionlib_msgs
  roslib
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    trajectory_msgs
    control_msgs
    actionlib_msgs
    message_runtime
)

include_directories(
  /src/rpg_torque_control_moveit_interface.hpp
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)


add_executable(rpg_torque_control_moveit_interface src/rpg_torque_control_moveit_interface.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
add_dependencies(rpg_torque_control_moveit_interface ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(rpg_torque_control_moveit_interface
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)
