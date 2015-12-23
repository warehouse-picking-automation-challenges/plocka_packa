/*
 * print_joint_states.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: sven
 */

#include <ros/ros.h>

#include <ros/ros.h>

#include "apc_robot/apc_robot_moveit.h"
int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_testing_node",ros::init_options::AnonymousName);

	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();

	RobotMoveit baxter_moveit;

	std::string tmp;

	while(ros::ok())
	{

		ROS_INFO("Press [enter] to get joint state:");
		std::getline(std::cin, tmp);

		baxter_moveit.printJointStatesSRDF();

	}

	ros::shutdown();

	return 0;
}

