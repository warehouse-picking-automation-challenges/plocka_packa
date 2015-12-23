/*!
 * 	\name 	   BaseMovement
 *  \brief     Allows for base movement using action server through zone definition
 *  \details   Allows for base movement using action server through zone definition
 *  \author    Rommel Alonzo
 *  \version   1.0
 *  \date      May 13, 2015
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

#ifndef BASEMOVEMENT_H_
#define BASEMOVEMENT_H_

/****
 *
 * Zone 0:	(-1,1)
 * Zone 1:	(0,1)
 * Zone 2:	(1,1)
 * Zone 3:	(-1,0)
 * Zone 4:	(0.4,0.9,0) - Center
 * Zone 5:	(1,0)
 *
 * Map:
 * 0	1	2
 * 3	4	5
 *
 *	(x position, y position, rotation)
 *
 * (-1,1,0)		(0,1,0)		(1,1,0)
 * (-1,0,-1)	(0,0,0)		(1,0,1)
 */

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <string.h>
#include <vector>
#include <apc_msgs/SetBaseDistance.h>
#include <apc_msgs/SetBaseRotation.h>
#include <apc_msgs/SetBaseLocation.h>
#include <apc_msgs/SetBaseParameter.h>
#include <std_msgs/String.h>
#include <cmath>

struct Zone
{
	std::string name;
	double x;
	double y;
	double w;
};

/*! \class BaseMovement base_movement.h "include/base_movement.h"
 *  \brief Allows for base movement using command velocities
 */
class BaseMovement 
{
private:
	ros::NodeHandle nh_;

	ros::ServiceClient drive_base;
	ros::ServiceClient turn_base;
	ros::ServiceClient drive_to_xy  ;
	ros::ServiceClient drive_to_w  ;
	
	std::string current_zone;
	double current_x;
	double current_y;
	double current_w;
	std::vector<Zone> zones;
	ros::NodeHandle n;

	void loadZoneParameters();

	void updateCurrentPose();
	Zone findZone(std::string);

	// TODO add service clients for apc_base_driver

public:
	/*!
	 Constructor
	 */
	BaseMovement();
	/*!
	 Deconstructor
	 */
	~BaseMovement();
	//double* requestNextPose(std::string next_pose);
	//void odometry_CB(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg);
	//void move_base_cmd_CB(std_msgs::String::ConstPtr msg);
	//! Send base command based on several zone definition
	/*!
	\return True upon successfully base movement command being sent
	\sa motionComplete(), cancelMotion()
	 */
	bool move(std::string zone);
	//! Checks if motion has been completed
	/*!
	\return bool true motion is completed
	\sa move(), cancelMotion()
	 */
	bool motionComplete();
	//! Cancel current motion
	/*!
	\sa move(), cancelMotion()
	 */
	bool cancelMotion();
};

#endif /* BASEMOVEMENT_H_ */
