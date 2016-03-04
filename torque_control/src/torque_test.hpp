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
#include <brics_actuator/JointTorques.h>
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/String.h>

#include "YoubotArmDynamicsSymbolic.hpp"
#include "YoubotJoints.hpp"
#include <torque_control/step.h>
#include <std_srvs/Empty.h>

#include <boost/units/systems/si.hpp>
//#include <boost/units/io.hpp>

class TorqueController
{
public:
  //Constructor
  TorqueController(ros::NodeHandle& nh, std::string name);
  //define functions
  bool initialize();

  brics_actuator::JointTorques generate_joint_torque_msg(Eigen::VectorXd arr);

  int limitTorques(Eigen::VectorXd & torques);

  void pdControlWithGravityCompensation();

  //define publisher
  ros::Publisher torque_command_pub;
  ros::Publisher pos_command_pub;

  //define subscribers
  ros::Subscriber joint_state_sub;

  ros::ServiceServer srv_grav_on;
  ros::ServiceServer srv_grav_off;

  std::string mode;

private:
  void jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  
  //bool checkJointLimits(trajectory_msgs::JointTrajectory &j_traj);

  bool gravityOnCallback(torque_control::step::Request &req, torque_control::step::Response &res);

  bool gravityOffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res);

  bool brics2eigen(brics_actuator::JointPositions jpos, Eigen::VectorXd & pos);

  int DOF;
  double duration;
  Eigen::VectorXd m_q, m_qdot, m_qdotdot, q_tra, qdot_tra, qdotdot_tra, m_torques, eff_torques;
  Eigen::MatrixXd Kp, Kv;

  sensor_msgs::JointState m_joint_state;
  brics_actuator::JointPositions stop;

  double lr;
  double max_vel;
	double max_acc;
};
#endif /* TORQUE_HPP_ */
