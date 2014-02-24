/*
 * torque.hpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#ifndef TORQUE_HPP_
#define TORQUE_HPP_

#include <iostream>

#include <ros/ros.h>
#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotArmFKin.hpp"
#include "YoubotJoints.hpp"

#include <boost/units/systems/si.hpp>
#include <boost/units/io.hpp>

#include <brics_actuator/CartesianWrench.h>
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointPositions.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/Pose.h>
#include <std_msgs/String.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <actionlib/server/simple_action_server.h>
#include <torque_control/torque_trajectoryAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <torque_control/step.h>
#include <std_srvs/Empty.h>
#include <Eigen/Dense>

class TorqueController
{
public:
  //Constructor
  TorqueController(ros::NodeHandle& nh, std::string name);
  //define functions
  bool initialize();

  brics_actuator::JointTorques generate_joint_torque_msg(Eigen::VectorXd arr);

  int limitTorques(Eigen::VectorXd & torques);

  void writeToFile(std::ofstream & file, Eigen::VectorXd & vect);

  bool pointToEigen(Eigen::VectorXd & pos, Eigen::VectorXd & vel, Eigen::VectorXd & acc, trajectory_msgs::JointTrajectoryPoint & point, std::vector<std::string> & joint_names);

  void stepInput();

  void gravityCompensation();
  //define publisher
  ros::Publisher torque_command_pub;
  ros::Publisher pos_command_pub;

  //define subscribers
  ros::Subscriber joint_state_sub;

  actionlib::SimpleActionServer<torque_control::torque_trajectoryAction> traj;
  actionlib::SimpleActionServer<control_msgs::FollowJointTrajectoryAction> traj_follow;
  std::string action_name;
  torque_control::torque_trajectoryResult result;
  control_msgs::FollowJointTrajectoryResult result_controlmsgs;
  torque_control::torque_trajectoryFeedback feedback;
	control_msgs::FollowJointTrajectoryFeedback feedback_controlmsgs;

  ros::ServiceServer srv_step;
  ros::ServiceServer srv_grav_on;
  ros::ServiceServer srv_grav_off;

  std::string mode;

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);

  void followTrajectory(const torque_control::torque_trajectoryGoalConstPtr & trajectory);
  
  void followTrajectory_controlmsgs(const control_msgs::FollowJointTrajectoryGoalConstPtr & trajectory);

  bool stepCallback(torque_control::step::Request &req, torque_control::step::Response &res);

  bool gravityOnCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

  bool gravityOffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

  bool brics2eigen(brics_actuator::JointPositions jpos, Eigen::VectorXd & pos);

  int DOF;
  double duration;
  Eigen::VectorXd m_q, m_qdot, m_qdotdot, q_tra, qdot_tra, qdotdot_tra, m_torques, eff_torques;
  Eigen::MatrixXd Kp, Kv;

  geometry_msgs::Pose m_tar_pos;
  sensor_msgs::JointState m_joint_state;
  brics_actuator::JointPositions stop;
  double lr;

  std::ofstream rp;
  std::ofstream ap;
  std::ofstream rv;
  std::ofstream av;
  std::ofstream ra;
  std::ofstream fft;
  std::ofstream pdt;
  std::ofstream eft;
};
#endif /* TORQUE_HPP_ */
