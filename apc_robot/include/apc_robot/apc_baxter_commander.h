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
 * apc_baxter_commander.h
 *
 *  Created on: Jan 2, 2015
 *      Author: sven
 */

#ifndef APC_BAXTER_COMMANDER_H_
#define APC_BAXTER_COMMANDER_H_

// ROS
#include <ros/ros.h>

// Messages
#include <std_msgs/Empty.h>
#include <std_msgs/Bool.h>
#include <baxter_core_msgs/AssemblyState.h>
#include <geometry_msgs/Pose.h>

#include "apc_robot_grippers.h"
// Baxter classes
#include "apc_robot_moveit.h"

/*! \class BaxterCommander apc_baxter_commander.h "include/apc_baxter_commander.h"
 *  \brief Controls Baxter through MoveIt and Gripper interface
 */
class BaxterCommander
{
  private:
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;
	/*!
      Moveit Client interface
	 */
	RobotMoveit baxter_movit;
	/*!
      Gripper Client interface
	 */
	Gripper baxter_gripper;
	/*!
      ROS Publisher
      Enable publishing to robot
	 */
	ros::Publisher robot_enable_pub_;
	/*!
      ROS Publisher
      Reset publishing to robot
	 */
	ros::Publisher robot_reset_pub_;
	/*!
      ROS Publisher
      Stop publishing to robot
	 */
	ros::Publisher robot_stop_pub_;
	/*!
      Robot state
	 */
	baxter_core_msgs::AssemblyState robot_state;
	/*!
      Time stamp
	 */
	ros::Time robot_state_timestamp;
	//! Updates the robot state
	/*!
	 \return boolean
	 */
	bool updateRobotState();
	/*!
      Boolean message
	 */
	std_msgs::Bool true_msg_;
	/*!
      Boolean message
	 */
	std_msgs::Bool false_msg_;
	/*!
      Empty message
	 */
	std_msgs::Empty empty_msg_;
	/*!
      Object recognition ROS service client
	 */
	ros::ServiceClient obj_recog_client_;
	/*!
      Object location
	 */
	geometry_msgs::Pose object_location;
  public:
	/*!
	 Constructor
	 */
	BaxterCommander();
	/*!
	 Deconstructer
	 */
	~BaxterCommander();

	// Robot state
    //! Enable Robot
	/*!
	  \return boolean
	  \sa disableRobot(), stopRobot(),resetRobot()
	 */
	bool enableRobot();
    //! Disable Robot
	/*!
	  \return boolean
	  \sa enableRobot(), stopRobot(),resetRobot()
	 */
	bool disableRobot();
    //! Stop Robot
	/*!
	  \return boolean
	  \sa enableRobot(), disableRobot(),resetRobot()
	 */
	bool stopRobot();
    //! Reset Robot
	/*!
	  \return boolean
	  \sa enableRobot(), disableRobot(),stopRobot()
	 */
	bool resetRobot();
	// Pick & Place
    //! Initialize Robot
	/*!
	  \return boolean
	  \sa pick(), place(),getObjectPose()
	 */
	bool robotInit();
    //! Pick up an object
	/*!
	  \return boolean
	  \sa robotInit(), place(),getObjectPose()
	 */
	bool pick();
    //! Place an object
	/*!
	  \return boolean
	  \sa robotInit(), pick(),getObjectPose()
	 */
	bool place();
    //! Get object position
	/*!
	  \return boolean
	  \sa robotInit(), pick(),place()
	 */
	bool getObjectPose();
};
#endif /* APC_BAXTER_COMMANDER_H_ */
