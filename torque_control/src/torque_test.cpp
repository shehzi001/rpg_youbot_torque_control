/*
 * torque.cpp
 *
 *  Created on: May 2, 2013
 *      Author: keiserb
 */

#include "torque_test.hpp"
#include <time.h>
#include <ros/package.h>

using namespace std;

TorqueController::TorqueController(ros::NodeHandle& nh, std::string name) :
    DOF(5), duration(0)

{
  // arm joints (always 5)
  m_joint_state.name.assign(5, "0        10       20"); // presized string
  m_joint_state.position.resize(5);
  m_joint_state.velocity.resize(5);
  m_joint_state.effort.resize(5);
  
  //max velocity and max acceleration for trajectory generator
  max_vel = 0.05;
  max_acc = 0.5;

  mode = "idle";

  nh.param("/youBotDriverCycleFrequencyInHz",lr,50.0);
  
  ROS_INFO("Youbot driver frequency : %f", lr);

  joint_state_sub = nh.subscribe("joint_states", 1, &TorqueController::jointstateCallback, this);
 
  srv_grav_on = nh.advertiseService("turn_gravity_compensation_on", &TorqueController::gravityOnCallback, this);
  srv_grav_off = nh.advertiseService("turn_gravity_compensation_off", &TorqueController::gravityOffCallback, this);

  torque_command_pub = nh.advertise<brics_actuator::JointTorques>("torques_command", 1);
  pos_command_pub = nh.advertise<brics_actuator::JointPositions>("position_command", 1);
}

bool TorqueController::initialize()
{
  m_q.setZero(DOF);
  m_qdot.setZero(DOF);
  m_qdotdot.setZero(DOF);
  m_torques.setZero(DOF);
  q_tra.setZero(DOF);
  qdot_tra.setZero(DOF);
  qdotdot_tra.setZero(DOF);
  eff_torques.setZero(DOF);
  Kp.setZero(DOF, DOF);
  Kv.setZero(DOF, DOF);
  std::string package_path = ros::package::getPath("torque_control");

  if (!gainMatrices(Kp, Kv, package_path))
  {
    return false;
  }
  cout << "Kp" << endl;
  cout << Kp << endl;
  cout << "Kv" << endl;
  cout << Kv << endl;

  std::stringstream jointName;

  for (int i = 0; i < DOF; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = 0;
    joint.unit = boost::units::to_string(boost::units::si::radian);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    stop.positions.push_back(joint);
  }
  return true;
}

brics_actuator::JointTorques TorqueController::generate_joint_torque_msg(Eigen::VectorXd arr)
{
  brics_actuator::JointTorques m_joint_torques;
  //Ros component negates torque values for joints with negative direction (all joints except joint 3)
  arr[2] = -arr[2];
  std::stringstream jointName;
  m_joint_torques.torques.clear();

  for (int i = 0; i < DOF; i++)
  {
    brics_actuator::JointValue joint;
    joint.value = arr[i];
    joint.unit = boost::units::to_string(boost::units::si::newton_meter);
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    joint.joint_uri = jointName.str();

    m_joint_torques.torques.push_back(joint);
    
    
  }
  return m_joint_torques;
}

void TorqueController::jointstateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  int size = 0;
  for (int j = 0; j < msg->position.size(); j++)
  {
    for (int i = 0; i < YOUBOT_NR_OF_JOINTS; i++)
    {
      if (msg->name[j] == joint_names[i])
      {
        m_joint_state.position[i] = msg->position[j];
        m_joint_state.velocity[i] = msg->velocity[j];
        if (msg->effort.size() >= YOUBOT_NR_OF_JOINTS)
        {
          m_joint_state.effort[i] = msg->effort[j];   //no effort msg in webots
        }
        size++;
      }
    }
  }
  if (size == YOUBOT_NR_OF_JOINTS)
  {
    //translate values into Torque positions
    m_q(0) = m_joint_state.position[0] - joint_offsets[0];
    m_q(1) = m_joint_state.position[1] - joint_offsets[1];
    m_q(2) = joint_offsets[2] - m_joint_state.position[2];
    m_q(3) = m_joint_state.position[3] - joint_offsets[3];
    m_q(4) = m_joint_state.position[4] - joint_offsets[4];
    m_qdot(0) = m_joint_state.velocity[0];
    m_qdot(1) = m_joint_state.velocity[1];
    m_qdot(2) = -m_joint_state.velocity[2];
    m_qdot(3) = m_joint_state.velocity[3];
    m_qdot(4) = m_joint_state.velocity[4];
    eff_torques(0) = m_joint_state.effort[0];
    eff_torques(1) = m_joint_state.effort[1];
    eff_torques(2) = -m_joint_state.effort[2];
    eff_torques(3) = m_joint_state.effort[3];
    eff_torques(4) = m_joint_state.effort[4];
    for (int i = 0; i < DOF; i++)
    {
      stop.positions[i].value = m_joint_state.position[i];
    }
  }
  else
  {
    //ROS_INFO("NO JOINT STATES FOR YOUBOT ARM RECEIVED");
  }
}


bool TorqueController::gravityOnCallback(torque_control::step::Request &req, torque_control::step::Response &res)
{
  ROS_INFO("Gravity mode on");
  Eigen::VectorXd pos(5);
  if (!brics2eigen(req.position, pos))
  {
    ROS_ERROR("INVALID STEP POSITION");
    return false;
  }
  q_tra = youbot2torque(pos);
  qdot_tra.setZero(DOF);
  qdotdot_tra.setZero(DOF);
  duration = req.duration;
  mode = "pdgravity";
  return true;
}

bool TorqueController::gravityOffCallback(std_srvs::Empty::Request &req, std_srvs::Empty::Request &res)
{
  ROS_INFO("turning gravity compensation off");
  pos_command_pub.publish(stop);
  ros::spinOnce();
  mode = "idle";
  return true;
}


void TorqueController::pdControlWithGravityCompensation()
{
  ros::Rate loop_rate(lr);
  int rval;
  int k = 0;
  Eigen::VectorXd pos_err(5), vel_err(5), d_pos(5), a_pos(5);
  {
    ros::spinOnce();
    calcGravityTorques(m_q,m_torques);

    pos_err = m_q - q_tra;
    vel_err = m_qdot - qdot_tra;

    pd_controller_with_gravity_Torques(Kp, Kv, m_q, pos_err, vel_err, m_torques);

    rval = limitTorques(m_torques);

    d_pos = torque2youbot(q_tra);
    a_pos = torque2youbot(m_q);
    m_qdot(2)=-m_qdot(2);

    if (rval != 0)
    {
      pos_command_pub.publish(stop);
      pos_command_pub.publish(stop);
      pos_command_pub.publish(stop);
      mode = "idle";
      return;
      //ros::shutdown();
    }
    torque_command_pub.publish(generate_joint_torque_msg(m_torques));
  }
}

int TorqueController::limitTorques(Eigen::VectorXd & torques)
{
  //Implement smart torque limiting feature here based on position and velocity
  Eigen::VectorXd br_vel, br_acc;
  br_vel.resize(5);
  br_acc.resize(5);
  br_vel = m_qdot;
  br_acc = m_qdotdot;
  for (int i = 0; i < 5; i++)
  {
    if (m_q(i) > 0.95 * joint_max_angles[i])
    {
      if (m_qdot(i) > 0.2)
      {
        torque2youbot(m_q);
        ROS_ERROR("CRITICAL POSITION %f AND VELOCITY %f ON JOINT %i", (float )m_q(i), (float )m_qdot(i), i + 1);
        pos_command_pub.publish(stop);
        return -1;
      }
    }
    else if (m_q(i) < 0.95 * joint_min_angles[i])
    {
      if (m_qdot(i) < -0.2)
      {
        torque2youbot(m_q);
        ROS_ERROR("CRITICAL POSITION %f AND VELOCITY %f ON JOINT %i", (float )m_q(i), (float )m_qdot(i), i + 1);
        pos_command_pub.publish(stop);
        return -1;
      }
    }
    if (isnan((double)torques[i]))
    {
      torques(i) = 0;
      ROS_ERROR("NAN TORQUES");
      pos_command_pub.publish(stop);
      return -1;
    }
    if (fabs((double)torques[i]) > 1.0 * joint_torque_max[i])
    {
      //ROS_INFO("Torque value on joint %i too high, rescaled to max torque", i + 1);
      if (torques(i) < 0)
        torques(i) = -1 * joint_torque_max[i];
      else
        torques(i) = 1 * joint_torque_max[i];
    }
  }
  return 0;
}

bool TorqueController::brics2eigen(brics_actuator::JointPositions jpos, Eigen::VectorXd & pos)
{
  Eigen::VectorXd temp(5);
  temp = torque2youbot(m_q);
  ROS_INFO("New Target Position received");
  pos = temp;
  int length = jpos.positions.size();
  const std::string unit = boost::units::to_string(boost::units::si::radian);
  bool valid = true;
  for (int i = 0; i < length; i++)
  {
    int j;
    for (j = 0; j < DOF; j++)
    {
      if (jpos.positions[i].joint_uri == joint_names[j])
      {
        pos(j) = jpos.positions[i].value;
        // Check for correct Unit
        if (unit != jpos.positions[i].unit)
        {
          ROS_WARN("Unit incompatibility for %s position. Are you sure you want to command %s instead of %s ?",
                   joint_names[j].c_str(), jpos.positions[i].unit.c_str(), unit.c_str());
          pos(j) = temp(j);
          valid = false;
          return valid;
        }
        // Check for correct value range
        if (j == 2)
        {
          if (pos(j) > 0 || pos(j) < -(fabs(joint_max_angles[j]) + fabs(joint_min_angles[j])))
          {
            ROS_WARN("Desired joint angle for %s out of range (%f). Should be within [%f, %f].", joint_names[j].c_str(),
                     jpos.positions[j].value, -(fabs(joint_max_angles[j]) + fabs(joint_min_angles[j])), 0.0);
            pos(j) = temp(j);
            valid = false;
            return valid;
          }
        }
        else
        {
          if (pos(j) < 0 || pos(j) > fabs(joint_max_angles[j]) + fabs(joint_min_angles[j]))
          {
            ROS_WARN("Desired joint angle for %s out of range (%f). Should be within [%f, %f].", joint_names[j].c_str(),
                     jpos.positions[j].value, 0.0, fabs(joint_max_angles[j]) + fabs(joint_min_angles[j]));
            pos(j) = temp(j);
            valid = false;
            return valid;
          }
        }
        break;
      }
    }
    if (j > 4)
    {
      ROS_WARN("%s is not a valid joint name.", jpos.positions[i].joint_uri.c_str());
      valid = false;
      return valid;
    }
  }
  if (!valid)
  {
    pos = temp;
    return false;
  }
  return true;
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "torque_control");
  ros::NodeHandle nh("~");
  double loop;
  nh.param("/youBotDriverCycleFrequencyInHz", loop, 50.0);
  
  ROS_INFO("Youbot driver frequency : %f", loop);
  ros::Rate loop_rate(loop);
  TorqueController torque(nh, "follow_joint_trajectory");
  if (!torque.initialize())
  {
    return -1;
  }
  ROS_INFO("Torque Controller Running");
  while (ros::ok())
  {
    if (torque.mode == "pdgravity")
    {
      torque.pdControlWithGravityCompensation();
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
