/*
 * torque.hpp
 *
 *  Created on: Feb 27, 2014
 *      Author: Shehzad
 */

#ifndef RPG_TORQUE_CONTROL_MOVEIT_INTERFACE_HPP_
#define RPG_TORQUE_CONTROL_MOVEIT_INTERFACE_HPP_

#include <ros/ros.h>

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

#include <iostream>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>

#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>

#include <trajectory_msgs/JointTrajectory.h>
#include <trajectory_msgs/JointTrajectoryPoint.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <torque_control/torque_trajectoryAction.h>

#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>

#include "rpg_youbot_common.h"
#include "trajectory_generator/JStoJS.h"

class RPGTorqueControlMoveitInterface
{
public:
  //Constructor
  RPGTorqueControlMoveitInterface(ros::NodeHandle& nh, std::string name);
  //define functions
  bool initialize();
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> traj_denser;
  std::string action_name;
  control_msgs::FollowJointTrajectoryResult result_controlmsgs;
  //control_msgs::FollowJointTrajectoryFeedback feedback_controlmsgs;
  ros::ServiceClient js2js_client;
  
private:
  void generateDenseTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &trajectory);
  void sendTrajectoryToTorqueController(trajectory_msgs::JointTrajectory trajectory);
  std::vector<brics_actuator::JointPositions> joint_points;
  std::vector<brics_actuator::JointVelocities> joint_vels;
  trajectory_generator::JStoJS js2js;
  
  double lr;
  double max_vel;
  double max_acc;
  int DOF;

};
#endif /* RPG_TORQUE_CONTROL_MOVEIT_INTERFACE_HPP_ */
