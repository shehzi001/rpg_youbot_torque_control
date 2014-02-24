/*
 * tester.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: keiserb
 */

#include "ros/ros.h"
#include "trajectory_generator/JStoJS.h"
#include "trajectory_msgs/JointTrajectoryPoint.h"
#include <brics_actuator/JointPositions.h>
#include <brics_actuator/JointVelocities.h>
#include <brics_actuator/JointValue.h>
#include <iostream>
#include <string>
#include <boost/units/systems/si.hpp>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <actionlib/client/simple_action_client.h>
#include "rpg_youbot_common.h"

double pre_grasp[5] = {2.93836, 2.020597, -1.88253, 3.36243, 3.01283};
double candle[5] = {2.9496, 1.13446, -2.54818, 1.78896, 2.93075};

std::string joint_names[] = 
{
		"arm_joint_1",
		"arm_joint_2",
		"arm_joint_3",
		"arm_joint_4",
		"arm_joint_5",
		"gripper_finger_joint_l",
		"gripper_finger_joint_r"
};

using namespace std;
int main(int argc, char **argv)
{
		ros::init(argc, argv, "traj_tester");
		ros::NodeHandle nh;
		ros::Rate lr(10);
		ros::ServiceClient js2js_client = nh.serviceClient<trajectory_generator::JStoJS>("From_JS_to_JS");
		ros::Publisher arm_pub_pos = nh.advertise<brics_actuator::JointPositions>("/arm_1/arm_controller/position_command",1);
		actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("torque_control", true);

		double max_vel = 0.05;
		double max_acc = 0.5;
		
    ROS_INFO("max_vel: %f \t max_acc :%f", max_vel,max_acc);
    double first_pt[5];
    bool feasible = true;
    trajectory_msgs::JointTrajectory traj, temp;
    trajectory_msgs::JointTrajectoryPoint point;
 
    brics_actuator::JointPositions start_p, end_p;
    brics_actuator::JointVelocities start_v,end_v;
    brics_actuator::JointValue positions[5];
    brics_actuator::JointValue velocities[5];
    trajectory_generator::JStoJS js2js;
    
    start_p.positions.resize(5);
    start_v.velocities.resize(5);
    
    end_p.positions.resize(5);
    end_v.velocities.resize(5);
    
    
    for(int i = 0;i < 5;i++)
    {
     start_p.positions[i].joint_uri = joint_names[i];
     start_v.velocities[i].joint_uri = joint_names[i];
     start_p.positions[i].unit = "rad";
     start_v.velocities[i].unit = "s^-1 rad";
     start_p.positions[i].value = candle[i];
     start_v.velocities[i].value = 0.0;
     //start_p.positions[i] = positions[i];
     //start_v.velocities[i] = velocities[i];
    }
    
    
    for(int i = 0;i < 5;i++)
    {
     end_p.positions[i].joint_uri = joint_names[i];
     end_v.velocities[i].joint_uri = joint_names[i];
     end_p.positions[i].unit = "rad";
     end_v.velocities[i].unit = "s^-1 rad";
     end_p.positions[i].value = pre_grasp[i];
     end_v.velocities[i].value = 0.0;
     //start_p.positions[i] = positions[i];
     //start_v.velocities[i] = velocities[i];
    }
    
    js2js.request.start_pos = start_p;
    js2js.request.end_pos = end_p;
    js2js.request.start_vel = start_v;
    js2js.request.end_vel = end_v;
    js2js.request.max_vel = max_vel;
    js2js.request.max_acc = max_acc;
    
    
     if (js2js_client.call(js2js))
    {
      if (js2js.response.feasible)
      {
        cout << "feasible" << endl;
        temp = js2js.response.trajectory;
        while (!temp.points.empty())
        {
          point = temp.points.back();
          temp.points.pop_back();
          traj.points.insert(traj.points.begin(), point);
          
        }
        traj.joint_names = temp.joint_names;
      }
      else
      {
        cout << "Second Half Not Feasible" << endl;
        feasible = false;
      }
    }
  
   if (feasible)
    {
      point = traj.points.back();
      int i = 0;
      while (!point.positions.empty())
      {
        first_pt[i] = point.positions.back();
        i++;
        point.positions.pop_back();
      }
      cout << "Publishing arm cmd" << endl;
      arm_pub_pos.publish(rpg_youbot_common::generate_joint_position_msg(first_pt));
      ros::spinOnce();
      sleep(0.5);
      arm_pub_pos.publish(rpg_youbot_common::generate_joint_position_msg(first_pt));
      ros::spinOnce();
      sleep(0.5);
      arm_pub_pos.publish(rpg_youbot_common::generate_joint_position_msg(first_pt));
      ros::spinOnce();
      sleep(1);

      ROS_INFO("READY FOR TORQUE?");
      int x;
      cin >> x;
      ac.waitForServer(); //will wait for infinite time
      ROS_INFO("Action server started, sending goal.");

      //torque_control::torque_trajectoryGoal goal;
      control_msgs::FollowJointTrajectoryGoal goal;
      goal.trajectory = traj;

      ac.sendGoal(goal);

      //wait for the action to return
      bool finished_before_timeout = ac.waitForResult(ros::Duration(60.0));

      if (finished_before_timeout)
      {
        actionlib::SimpleClientGoalState state = ac.getState();
        ROS_INFO("Action finished: %s", state.toString().c_str());
      }
      else
      {
        ROS_INFO("Action did not finish before the time out.");
      }
    }
  return 0;
}

