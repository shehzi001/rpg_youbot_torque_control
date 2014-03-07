/*
 * rpg_torque_control_moveit_interface.cpp
 *
 *  Created on: Feb 27,2014
 *      Author: Shehzad
 */
 
#include "rpg_torque_control_moveit_interface.hpp"
#include <time.h>

using namespace std;

RPGTorqueControlMoveitInterface::RPGTorqueControlMoveitInterface(ros::NodeHandle& nh, std::string name) :
    DOF(5),traj_denser(nh, name, boost::bind(&RPGTorqueControlMoveitInterface::generateDenseTrajectory, this, _1),false)
{
  max_vel = 0.05;
  max_acc = 0.5;
  
  action_name = name;

  nh.param("/youBotDriverCycleFrequencyInHz",lr,50.0);
 
  js2js_client = nh.serviceClient<trajectory_generator::JStoJS>("/From_JS_to_JS");
  
 
  traj_denser.start();
}

bool RPGTorqueControlMoveitInterface::initialize()
{
 return true;
}


void RPGTorqueControlMoveitInterface::generateDenseTrajectory(const control_msgs::FollowJointTrajectoryGoalConstPtr &trajectory)
{
 
  trajectory_msgs::JointTrajectory trajectory_in = trajectory->trajectory;
 
  /*
   * Fill in the vectors,joint_points and joint_vels, of trajectory points to send the request to trajectory generator to generate more way points
   */
  
  double vel = 0.0;
  
  joint_points.resize(trajectory_in.points.size());
  joint_vels.resize(trajectory_in.points.size());
  
  
  int iterator = 0;
 
  while (iterator < (trajectory_in.points.size()))
  { 
    joint_points[iterator].positions.resize(5);
    joint_vels[iterator].velocities.resize(5);
     
    for(int j = 0;j < DOF;j++)
    {
     joint_points[iterator].positions[j].joint_uri = trajectory_in.joint_names[j];
     joint_points[iterator].positions[j].unit = "rad";
     joint_points[iterator].positions[j].value = trajectory_in.points[iterator].positions[j];
          
     joint_vels[iterator].velocities[j].joint_uri = trajectory_in.joint_names[j];
     joint_vels[iterator].velocities[j].unit = "s^-1 rad";
     joint_vels[iterator].velocities[j].value = vel;//trajectory_in.points[iterator].velocities[j];
    }
    
    // checking for last trajectory point to set the velocity at zero.
    if((trajectory_in.points.size() - iterator) == 2)
    {
        vel = 0.0;
    }
    else
    {
       vel = max_vel;
    }
    
    iterator++;
  }
  
  trajectory_msgs::JointTrajectory trajectory_out , trajectory_temp;
  trajectory_msgs::JointTrajectoryPoint point;
  bool trajecotry_feasible = false;
     
  
  /*
   *Iterating over the input trajectory
   *sending request to trajectory generator to generate denser trajectory
   */
   
  iterator = 0;
  while (iterator < (trajectory_in.points.size() - 1)) 
   {
    js2js.request.start_pos = joint_points[iterator];
    js2js.request.start_vel = joint_vels[iterator];
    
    iterator++;
    
    js2js.request.end_pos = joint_points[iterator];
    js2js.request.end_vel = joint_vels[iterator];
    
    js2js.request.max_vel = max_vel;
    js2js.request.max_acc = max_acc;
     
    if (js2js_client.call(js2js))
    {
      if (js2js.response.feasible)
      {
        trajectory_temp = js2js.response.trajectory;
        
        //cout << "Trajectory :\n" << trajectory_temp;
        
        while (!trajectory_temp.points.empty())
        {
          point = trajectory_temp.points.back();
          
          trajectory_temp.points.pop_back();
          
          trajectory_out.points.insert(trajectory_out.points.begin(), point);
        }
        
        trajectory_out.joint_names = trajectory_temp.joint_names;
        
        trajecotry_feasible = true;
      }
      else
      {
        trajecotry_feasible = false; 
        break;
      }
     }
   }
   
   if(trajecotry_feasible == true)
   {     
     traj_denser.setSucceeded(result_controlmsgs);
     ROS_INFO("Dense Trajectory generated");
     sendTrajectoryToTorqueController(trajectory_out);
     //cout << "Trajectory :\n" << trajectory_out;
   }
   else 
   {
     cout << "Trajectory can not be executed:";
     traj_denser.setAborted(result_controlmsgs);
   }
}

void RPGTorqueControlMoveitInterface::sendTrajectoryToTorqueController(trajectory_msgs::JointTrajectory traj)
{
   ROS_INFO("Sending goal to torque controller");
   actionlib::SimpleActionClient<torque_control::torque_trajectoryAction> ac("/arm_1/arm_controller/torque_control/follow_joint_trajectory", true);
   
   ROS_INFO("Waiting for action server to connect.");
   ac.waitForServer();
   
   torque_control::torque_trajectoryGoal goal;
   
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


int main(int argc, char* argv[])
{
   ros::init(argc, argv, "rpg_torque_control_moveit_interface");
   ros::NodeHandle nh("~");
   double loop;
   nh.param("/youBotDriverCycleFrequencyInHz", loop, 50.0);
   
   ros::Rate loop_rate(loop);
  
   RPGTorqueControlMoveitInterface rpgmoveitinterface(nh, "follow_joint_trajectory");
   if (!rpgmoveitinterface.initialize())
   {
    return -1;
   }
   ROS_INFO("RPG Torque Control Moveit interaface Running");
  
   while (ros::ok())
   {
     ros::spinOnce();
     loop_rate.sleep();
   }
   return 0;
}
