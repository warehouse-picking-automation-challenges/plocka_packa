/*!
 * 	\name 	   APCBaseDriver
 *  \brief     Controls PR2 base movement through command velocities
 *  \details   Using command velocities and tranformation frames, this class allows for base movement through distance and rotation
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Apr 19, 2015
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


#ifndef APC_BASE_DRIVER_H_
#define APC_BASE_DRIVER_H_

#include <iostream>

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>

#include <apc_msgs/SetBaseDistance.h>
#include <apc_msgs/SetBaseRotation.h>
#include <apc_msgs/SetBaseLocation.h>
#include <apc_msgs/SetBaseParameter.h>

/*! \class APCBaseDriver apc_base_driver.h "include/apc_base_driver.h"
 *  \brief Controls PR2 base movement through command velocities
 */
class APCBaseDriver
{
private:

	//!ROS Handle
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh_;
	//!ROS Publisher
	/*!
      ROS Publisher
	 */
	ros::Publisher cmd_vel_pub_;
	//!TransformListener
	/*!
      TransformListener
	 */
	tf::TransformListener listener_;
	//!Provides message encapsulation
	/*!
      Provides message encapsulation
	 */
	geometry_msgs::Twist base_cmd;
	//!Provides default linear velocity movement
	/*!
      Provides default linear velocity movement
	 */
	double vel_lin;
	//!Provides default angular velocity movement
	/*!
      Provides default angular velocity movement
	 */
	double vel_rot;
	//! Current odometry data from the TF tree
	/*!
      Current odometry data from the TF tree
	 */
	tf::StampedTransform current_transform;
	//! Initial odometry data from the TF tree
	/*!
      Initial odometry data from the TF tree
	 */
	tf::StampedTransform start_transform;
	//! Desired odometry data from the TF tree
	/*!
      Desired odometry data from the TF tree
	 */
	tf::StampedTransform goal_transform;
	//! Name of the robot frame in the TF tree
	/*!
      Name of the robot frame in the TF tree
	 */
	std::string robot_frame;
	//! Name of the world frame in the TF tree
	/*!
      Name of the world frame in the TF tree
	 */
	std::string world_frame;
	//! Amount of time to wait for base movements to complete
	/*!
      Amount of time to wait for base movements to complete
	 */
	double waitTime;
	//! Minimal linear distance the base can be away from the goal to be considered successful
	/*!
      Minimal linear distance the base can be away from the goal to be considered successful
	 */
	double goalTol;
	//! Minimal angular distance the base can be away from the goal to be considered successful
	/*!
      Minimal angular distance the base can be away from the goal to be considered successful
	 */
	double goalTolRot;
	//! updatePose
	/*!
      updatePose
	 */
	bool updatePose;
	///!Services
	//! Robot is moving
	/*!
      Robot is moving
	 */
	bool robotIsMoving;
	//! ROS Service Server offering linear base movement given displacement
	/*!
     	 ROS Service Server offering linear base movement given displacement
	 */
	ros::ServiceServer srv_driveOdom_;
	//! ROS Service Server offering angular base movement given displacement
	/*!
     	 ROS Service Server offering angular base movement given displacement
	 */
	ros::ServiceServer srv_turnOdom_;
	//! ROS Service Server offering linear and angular base movement given location and orientation
	/*!
     	 ROS Service Server offering linear and angular base movement given location and orientation
	 */
	ros::ServiceServer srv_driveToLocation_;
	//! ROS Service Server offering linear base movement given location
	/*!
     	 ROS Service Server offering linear base movement given location
	 */
	ros::ServiceServer srv_driveToXY_;
	//! ROS Service Server offering angular base movement given orientation
	/*!
     	 ROS Service Server offering angular base movement given orientation
	 */
	ros::ServiceServer srv_driveToW_;
	//! ROS Service Server offering loading of base parameters into the class
	/*!
     	 ROS Service Server offering loading of base parameters into the class
	 */
	ros::ServiceServer srv_baseParameter_;
	//! ROS service function offering linear base movement
	/*!
	\param req an apc_msgs::SetBaseDistance::Request
	\param res an apc_msgs::SetBaseDistance::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), turnOdomCB(), driveToLocationCB(), baseParameterCB()
	 */
	bool driveOdomCB(apc_msgs::SetBaseDistance::Request &req, apc_msgs::SetBaseDistance::Response &res);			// TODO: make them non-blocking?
	//! ROS service function offering angular base movement
	/*!
	\param req an apc_msgs::SetBaseRotation::Request
	\param res an apc_msgs::SetBaseRotation::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), driveOdomCB(), driveToLocationCB(), baseParameterCB()
	 */
	bool turnOdomCB(apc_msgs::SetBaseRotation::Request &req, apc_msgs::SetBaseRotation::Response &res);
	//! ROS service function offering linear base movement given position
	/*!
	\param req an apc_msgs::SetBaseLocation::Request
	\param res an apc_msgs::SetBaseLocation::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), driveOdomCB(), turnOdomCB(), baseParameterCB()
	 */
	bool driveToLocationCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res);
	//! ROS service function loading base parameters
	/*!
	\param req an apc_msgs::SetBaseParameter::Request
	\param res an apc_msgs::SetBaseParameter::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), driveOdomCB(), turnOdomCB(), driveToLocationCB()
	 */
	bool baseParameterCB(apc_msgs::SetBaseParameter::Request &req, apc_msgs::SetBaseParameter::Response &res);
	// TODO add cancel service
	// TODO set velocity via service
	//! Move base to specified x and y location
	/*!
	\param x a double
	\param y a doble
	\return True upon successful base movement
	\sa APCBaseDriver(), driveToW(), driveToXYCB(), driveToWCB()
	 */
	bool driveToXY(double x, double y);
	//! Rotate base to specified w orientation
	/*!
	\param w a double
	\return True upon successful base movement
	\sa APCBaseDriver(), driveToXY(), driveToXYCB(), driveToWCB()
	 */
	bool driveToW(double w);
	//! ROS service callback function moving base to specified location
	/*!
	\param req an apc_msgs::SetBaseLocation::Request
	\param res an apc_msgs::SetBaseLocation::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), driveToXY(), driveToW(), driveToWCB()
	 */
	bool driveToXYCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res);
	//! ROS service callback function rotating base to specified orientation
	/*!
	\param req an apc_msgs::SetBaseLocation::Request
	\param res an apc_msgs::SetBaseLocation::Response
	\return True upon successful base movement
	\sa APCBaseDriver(), driveToXY(), driveToW(), driveToXYCB()
	 */
	bool driveToWCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res) ;
public:
	//! An enumerated type
	/*! Allows for easy description of a directional movement throughout the class. */
	typedef enum
	{
		FORWARD,		/*!< Enum value FORWARD. */
		BACKWARD,		/*!< Enum value BACKWARD. */
		LEFT,			/*!< Enum value LEFT. */
		RIGHT			/*!< Enum value RIGHT. */
	}MoveType;
	//!Constructer
	/*!
	 Constructor
	 */
	APCBaseDriver(ros::NodeHandle &nh);
	//! Drive a specified distance in a specified direction
	/*!
	 \param distance a double
	 \param t an enumerated type
	 \return True if movement succeeded
	 \sa turnOdom(), driveToLocation()
	 */
	bool driveOdom(double distance, APCBaseDriver::MoveType t);
	//! Drive a specified distance in a specified direction
	/*!
	 \param radians a double
	 \param clockwise is a boolean
	 \return True if movement succeeded
	 \sa driveOdom(), driveToLocation()
	 */
	bool turnOdom(double radians, bool clockwise);
	//!Drive to a specified location and orientation
	/*!
	 \param x a double
	 \param y a double
	 \param w a double
	 \return True if movement succeeded
	 \sa driveOdom(), driveToLocation()
	 */
	bool driveToLocation(double x, double y, double w);
};



#endif /* APC_BASE_DRIVER_H_ */
