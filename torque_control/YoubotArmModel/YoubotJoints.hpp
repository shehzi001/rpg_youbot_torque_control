#ifndef YOUBOTJOINTS_HPP_
#define YOUBOTJOINTS_HPP_

#define YOUBOT_NR_OF_WHEELS 4
#define YOUBOT_NR_OF_JOINTS 5

const std::string joint_names[5] = {"arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5"};
const double joint_min_angles[5] = {-2.9496,-1.1344,-2.6354,-1.7889,-2.9234};
const double joint_max_angles[5] = {2.9496,1.5707,2.5481,1.7889,2.9234};
const double joint_min_vel[5] = {-1.0,-1.0,-1.0,-1.0,-1.0};
const double joint_max_vel[5] = {1.0,1.0,1.0,1.0,1.0};
const double joint_offsets[5] = {2.9496, 1.1344, -2.5481, 1.7889, 2.9234};
const double joint_torque_max[5] =  {7.5,7.5,4.0,2.0,1.0};

#endif
