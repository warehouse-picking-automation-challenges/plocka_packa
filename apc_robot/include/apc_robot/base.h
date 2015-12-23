/*!
 * 	\name 	   Base
 *  \brief     Controls PR2 base movement through the navigation stack
 *  \details   Controls PR2 base movement through the navigation stack
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      July 15, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef BASE_H_
#define BASE_H_

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

/*! \class Base base.h "include/base.h"
    \brief Controls PR2 base movement through the navigation stack
 */
class Base
{
private:
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;
	/*!
      MoveBaseClient navigation client
	 */
	MoveBaseClient* action_client;
	/*!
      Command veloctiy
	 */
	geometry_msgs::Twist base_cmd;
	/*!
      Navigational base goal
	 */
	move_base_msgs::MoveBaseGoal base_goal;
	/*!
      Base motion is in progress
	 */
	bool motionInProgress;

public:
	/*!
	 Constructor
	 */
	Base();
	/*!
	 Deconstructer
	 */
	~Base();
	//! Sends position goal to action client given a tf frame
	/*!
	 \param x a double
	 \param y a double
	 \param w a double
	 \param frame a std::string
	 \sa sendBaseGoal()
	 */
	void sendBaseGoal(double x, double y, double w, std::string frame);
	//! Sends position goal from a geometry_msgs::Pose message to action client given a tf frame
	/*!
	 \param target a geometry_msgs::Pose
	 \param frame a std::string
	 \sa sendBaseGoal()
	 */
	void sendBaseGoal(geometry_msgs::Pose target, std::string frame);
	//! Sends position goal from a geometry_msgs::Pose message to action client in the map tf frame
	/*!
	 \param target a geometry_msgs::Pose
	 \sa sendBaseGoal_MapFrame(), sendBaseGoal_RobotFrame(), sendBaseGoal_OdomFrame()
	 */
	void sendBaseGoal_MapFrame(geometry_msgs::Pose target);
	//! Sends position goal to action client in the map tf frame
	/*!
	 \param x a double
	 \param y a double
	 \param w a double
	 \sa sendBaseGoal_MapFrame(), sendBaseGoal_RobotFrame(), sendBaseGoal_OdomFrame()
	 */
	void sendBaseGoal_MapFrame(double x, double y, double w);
	//! Sends position goal to action client in the robot tf frame
	/*!
	 \param x a double
	 \param y a double
	 \param w a double
	 \sa sendBaseGoal_MapFrame(), sendBaseGoal_MapFrame(), sendBaseGoal_OdomFrame()
	 */
	void sendBaseGoal_RobotFrame(double x, double y, double w);
	//! Sends position goal to action client in the odometry tf frame
	/*!
	 \param x a double
	 \param y a double
	 \param w a double
	 \param target a geometry_msgs::Pose
	 \sa sendBaseGoal_MapFrame(), sendBaseGoal_MapFrame(), sendBaseGoal_RobotFrame()
	 */
	void sendBaseGoal_OdomFrame(double x, double y, double w);
	//! Sends an orientation goal to action client
	/*!
	 \param w a double
	 */
	void sendTurnGoal(double w);
	//void sendTurnCmd(double w, double duration);
	//! Checks if goal has been completed
	/*!
	 \return bool true motion is completed
	 \sa cancelMotion()
	 */
	bool motionComplete();
	//! Cancel current goal
	/*!
	 \sa motionComplete()
	 */
	void cancelMotion();
};

#endif /* BASE_H_ */
