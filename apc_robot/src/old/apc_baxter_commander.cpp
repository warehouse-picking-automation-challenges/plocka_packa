/*!
 * 	\name 	   BaxterCommander
 *  \brief     Controls Baxter through MoveIt and Gripper interface
 *  \details   Controls Baxter through MoveIt and Gripper interface
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Jan 2, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */
/*
 * apc_baxter_commander.cpp
 *
 *  Created on: Jan 2, 2015
 *      Author: sven
 */



#include <apc_baxter/apc_baxter_commander.h>


BaxterCommander::BaxterCommander()
{

	// Object recognition
	//obj_recog_client_ = nh.serviceClient<geometry_msgs::PoseMSG>("apc/obj_recogntion_node");

	// Robot state
	robot_enable_pub_ = nh.advertise<std_msgs::Bool>("/robot/set_super_enable",1);
	robot_reset_pub_  = nh.advertise<std_msgs::Empty>("/robot/set_super_reset",1);
	robot_stop_pub_   = nh.advertise<std_msgs::Empty>("/robot/set_super_stop",1);

	// ROS messages
	true_msg_.data  = true;
	false_msg_.data	= false;

	ROS_INFO("*** Initialized BaxterCommander class ***");
}

BaxterCommander::~BaxterCommander()
{

}

bool BaxterCommander::updateRobotState()
{

	baxter_core_msgs::AssemblyStateConstPtr msg = ros::topic::waitForMessage<baxter_core_msgs::AssemblyState>("/robot/state",ros::Duration(1,0));

	if (msg == NULL)
	{
		ROS_ERROR("updateRobotState: No robot state message received!");
		return false;
	}

	robot_state = *msg;
	robot_state_timestamp = ros::Time::now();

	return true;
}


bool BaxterCommander::enableRobot()
{

	// Check status
	if(!updateRobotState())
	{
		return false;
	}

	if(robot_state.stopped == true)
	{
		ROS_ERROR("enableRobot: Robot is stopped");
		return false;
	}
	if(robot_state.error == true)
	{
		ROS_ERROR("enableRobot: Robot has an error");
		return false;
	}
	if(robot_state.enabled == true)
	{
		ROS_WARN("enableRobot: Robot is already enabled");
		return true;
	}

	// Reset robot
	if(!resetRobot())
		return false;

	// Enable robot
	robot_enable_pub_.publish(true_msg_);
	ros::Time start_time;
	while(ros::ok() && ros::Time::now() > start_time + ros::Duration(1.0) && robot_state.enabled == false)
	{
		if(!updateRobotState())
		{
			return false;
		}
		ROS_INFO("enableRobot: Waiting for robot to be enabled ...");
		ros::Duration(0.1).sleep();
	}

	return robot_state.enabled;
}

bool BaxterCommander::disableRobot()
{

	if(!updateRobotState())
	{
		return false;
	}

	// Disable robot
	robot_enable_pub_.publish(false_msg_);
	ros::Time start_time;
	while(ros::ok() && ros::Time::now() > start_time + ros::Duration(1.0) && robot_state.enabled == true)
	{
		if(!updateRobotState())
		{
			return false;
		}
		ROS_INFO("disableRobot: Waiting for robot to be disabled ...");
		ros::Duration(0.1).sleep();
	}

	return !robot_state.enabled;
}

bool BaxterCommander::stopRobot()
{
	if(!updateRobotState())
	{
		return false;
	}

	robot_stop_pub_.publish(empty_msg_);
	ros::Time start_time;
	while(ros::ok() && ros::Time::now() > start_time + ros::Duration(1.0) && robot_state.stopped == false)
	{
		if(!updateRobotState())
		{
			return false;
		}
		ROS_INFO("stopRobot: Waiting for robot to be stopped ...");
		ros::Duration(0.1).sleep();
	}

	return robot_state.stopped;

}

bool BaxterCommander::resetRobot()
{

	if(!updateRobotState())
	{
		return false;
	}

	robot_reset_pub_.publish(empty_msg_);
	ros::Duration(0.1).sleep();

	return true;
}

bool BaxterCommander::robotInit()
{

	/* Enable robot */
	if(!enableRobot())
	{
		ROS_ERROR("robotInit: Robot could not be enabled!");
		return false;
	}

	sleep(1.0);

	/* Move to start pose */
//	if(!baxter_movit.moveToPose(BaxterMoveit::RIGHT,"right_neutral"))
//	{
//		return false;
//	}
//	if(!baxter_movit.moveToPose(BaxterMoveit::LEFT,"left_neutral"))
//	{
//		return false;
//	}

	/* TODO Open grippers */

	return true;
}

bool BaxterCommander::pick()
{

	RobotMoveit::WhichArm arm = RobotMoveit::RIGHT;
	Gripper::WhichArm gripper = Gripper::RIGHT;

	/* Get object location */
	if(!getObjectPose())
	{
		return false;
	}

	baxter_gripper.open(gripper);

    /* Execute Cartesian goals */
	if(!baxter_movit.executeCarteGoal(arm, object_location))
	{
		return false;
	}

	baxter_gripper.close(gripper);

	return true;
}

bool BaxterCommander::place()
{
	return true;
}

bool BaxterCommander::getObjectPose()
{

//	srv.request.name = "coke";
//
//	if (client.call(srv))
//	{
//		ROS_INFO("Recieve object pose");
//		point_msg_ = srv.response.point;
//	}
//	else
//	{
//		ROS_ERROR("Failed to call service");
//		return false;
//	}

	return true;
}
