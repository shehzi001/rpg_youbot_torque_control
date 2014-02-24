cmake_minimum_required(VERSION 2.8.3)
project(torque_example)

find_package(catkin REQUIRED
  COMPONENTS
    brics_actuator
    tf_conversions
    youbot_description
    roscpp
    ik_solver_service
    brics_actuator
    trajectory_msgs
    control_msgs
	rpg_youbot_common
	torque_control
	trajectory_generator
	message_generation
)

find_package(Eigen REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    brics_actuator
    trajectory_msgs
    control_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}  
)


add_executable(example_circle src/example_circle.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
target_link_libraries(example_circle
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(example_JStoJS src/example_JStoJS.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
target_link_libraries(example_JStoJS
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

add_executable(example_CStoCS src/example_CStoCS.cpp ../rpg_youbot_common/src/rpg_youbot_common.cpp)
target_link_libraries(example_CStoCS
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
  ${Boost_LIBRARIES}
)

