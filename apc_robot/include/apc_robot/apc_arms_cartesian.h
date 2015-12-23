/*!
 * 	\name 	   ArmsCartesian
 *  \brief     Controls arm movement through cartesian control
 *  \details   Using a PR2 action client, cartesian arm coordinates are sent through public functions
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      May 21, 2015
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


#ifndef APC_ARMS_CARTESIAN_H_
#define APC_ARMS_CARTESIAN_H_


// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <apc_msgs/JTCartesianControllerState.h>
#include <tf/transform_listener.h>

//Services
#include <apc_msgs/SetGain.h>

struct GainValue{
	std::string name;

	double t_p;
	double t_d;
	double r_p;
	double r_d;
};

/*! \class ArmsCartesian apc_arms_cartesian.h "include/apc_arms_cartesian.h"
 *  \brief Controls arm movement through cartesian control
 */
class ArmsCartesian
{

public:
	//! An enumerated type
	/*! Allows for easy description of arm postures throughout the class. */
	typedef enum
	{
		off,		/*!< Enum value off. */
		mantis,		/*!< Enum value mantis. */
		elbowupr,	/*!< Enum value elbowupr. */
		elbowupl,	/*!< Enum value elbowupl. */
		elbowdownr,	/*!< Enum value elbowdownr. */
		elbowdownl	/*!< Enum value elbowdownl. */
	}Posture;

	//! An enumerated type
	/*! Allows for easy description of gripper orientation throughout the class. */
	typedef enum
	{
		front,		/*!< Enum value front. */
		side,		/*!< Enum value side. */
		degree45,	/*!< Enum value degree45. */
		degree90,	/*!< Enum value degree90. */
		degree35,	/*!< Enum value degree35. */
		degree25	/*!< Enum value degree25. */
	}GripPose;

	//! An enumerated type
	/*! Allows for easy description of a cartesian arm position throughout the class. */
	typedef enum
	{
		homeLeft,	/*!< Enum value homeLeft. */
		homeRight,	/*!< Enum value homeRight. */
		binHigh,	/*!< Enum value binHigh. */
		binMid,		/*!< Enum value binMid. */
		binLow,		/*!< Enum value binLow. */
		toolVacuum,	/*!< Enum value toolVacuum. */
		toolScoop	/*!< Enum value toolScoop. */
	}CartPose;

private:

	//!ROS Handle
	/*!
	 ROS Handle
	 */
	ros::NodeHandle nh;

	//!TransformListener
	/*!
	 TransformListener
	 */
	tf::TransformListener listener;

	std::string robot_frame;

	//!Geometry Pose Messages
	/*!
	 Geometry Pose Messages
	 */
	geometry_msgs::Pose r_pose ;
	geometry_msgs::Pose l_pose ;

	//!ROS Publisher
	/*!
	 ROS Publishers
	 */
	ros::Publisher rCartPub;
	ros::Publisher lCartPub;
	ros::Publisher lgainPub;
	ros::Publisher rgainPub;
	ros::Publisher rposturePub;
	ros::Publisher lposturePub;

	/// \brief Helper variables
	double timeOut;
	bool leftMotionInProgress;
	bool rightMotionInProgress;
	std::vector<GainValue> gains;

	//!JT Cartesian Controller
	/*!
	 JT Cartesian Controller for PR2
	 */
	apc_msgs::JTCartesianControllerState leftControllerState;
	apc_msgs::JTCartesianControllerState rightControllerState;

public:

	//! An enumerated type
	/*! Allows for easy description of a arm selection throughout the class. */
	typedef enum
	{
		LEFT, 		/*!< Enum value LEFT. */
		RIGHT, 		/*!< Enum value RIGHT. */
		BOTH 		/*!< Enum value BOTH. */
	} WhichArm;

	//! An enumerated type
	/*! Allows for easy description of a Cartesian direction throughout the class. */
	typedef enum
	{
		X, 			/*!< Enum value X. */
		Y, 			/*!< Enum value Y. */
		Z 			/*!< Enum value Z. */
	} Direction;
	//!Constructer
	/*!
	 Constructor
	 */
	ArmsCartesian();
	//!Deconstructer
	/*!
	 Deconstructor
	 */
	~ArmsCartesian();
	//! Loads arm gain parameters set from the parameter server
	/*!
	\sa ArmsCartesian(), ~ArmsCartesian()
	 */
	void loadGainParameters();
	//! Moves specified arm into a given Cartesian position
	/*!
	\param arm an enumerated type
	\param cmd is a geometry_msgs::Pose
	\param waitForMotion is a boolean
	\return True upon successful arm movement to position
	\sa ArmsCartesian(), ~ArmsCartesian(), moveInDirection(), getCurrentPose()
	 */
	bool moveToPose(ArmsCartesian::WhichArm arm, geometry_msgs::Pose, bool waitForMotion = true);
	//! Moves specified arm into a given Cartesian position, Cartesian direction, distance, increment of movement and in the amount of time given
	/*!
	\param arm an enumerated type
	\param gripperPose is a geometry_msgs::Pose
	\param d an enumerated type
	\param distance is a double
	\param dx is a double
	\param dt is a double
	\return True upon successful arm movement to position
	\sa ArmsCartesian(), ~ArmsCartesian(), moveToPose(), getCurrentPose()
	 */
	bool moveInDirection(ArmsCartesian::WhichArm arm, geometry_msgs::Pose& gripperPose, ArmsCartesian::Direction d, double distance, double dx, double dt);
	//! Moves specified arm into a given Cartesian position, Cartesian direction, distance, increment of movement and in the amount of time given
	/*!
	\param arm an enumerated type
	\param gripperPose is a geometry_msgs::PoseStamped
	\return True upon successful arm movement to position
	\sa ArmsCartesian(), ~ArmsCartesian(), moveToPose(), getCurrentPose()
	 */
	bool moveInDirection(ArmsCartesian::WhichArm arm, geometry_msgs::PoseStamped& gripperPose, ArmsCartesian::Direction d, double distance, double dx, double dt);
	//! Gets the current position of the specified arm
	/*!
	\param arm an enumerated type
	\param result is a geometry_msgs::PoseStamped
	\return True upon successful retrieval of arm position
	\sa ArmsCartesian(), ~ArmsCartesian(), moveToPose(), moveInDirection()
	 */
	bool getCurrentPose(ArmsCartesian::WhichArm a, geometry_msgs::PoseStamped& result);
	//! Moves both arms into specified joint positions
	/*!
	\param posture_position a std::vector of doubles
	\return True upon successful arm movement to position
	\sa ArmsCartesian(), ~ArmsCartesian(), setPosture(), loadPosture()
	 */
	bool setPostureBothArms(std::vector<double>& posture_position);//Just an idea
	//! Moves a specified arm into specified joint positions
	/*!
	\param arm an enumerated type
	\param posture_position a std::vector of doubles
	\return True upon successful arm movement to position
	\sa ArmsCartesian(), ~ArmsCartesian(), setPostureBothArms(), loadPosture()
	 */
	bool setPosture(ArmsCartesian::WhichArm arm,std::vector<double>& posture_position);
	//! Moves a specified arm into pre-loaded posture position
	/*!
	\param arm an enumerated type
	\param p is an enumerated type
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), setPostureBothArms(), loadPosture()
	 */
	bool loadPosture(ArmsCartesian::WhichArm arm, ArmsCartesian::Posture p);
	//! Load predefined cartesian positions
	/*!
	\param arm an enumerated type
	\param p is an enumerated type
	\param result is a std:vector of doubles
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), loadOrientation()
	 */
	bool loadCartPose(ArmsCartesian::WhichArm arm, ArmsCartesian::CartPose p, geometry_msgs::PoseStamped& result);
	//! Load predefined euler orientation
	/*!
	\param arm an enumerated type
	\param p is an enumerated type
	\param result is a std:vector of doubles
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), loadOrientation()
	 */
	bool loadOrientation(ArmsCartesian::WhichArm arm, ArmsCartesian::GripPose p, geometry_msgs::PoseStamped& result);
	//! Moves a specified arm into cartesian position specified as an enumerated type and geometry_msgs::PoseStamped
	/*!
	\param arm an enumerated type
	\param p is an enumerated type
	\param result is a std:vector of doubles
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), sendGoal()
	 */
	bool sendGoal(ArmsCartesian::WhichArm arm, ArmsCartesian::CartPose p, geometry_msgs::PoseStamped& result);
	//! Moves a specified arm into cartesian position specified as an enumerated type and geometry_msgs::PoseStamped
	/*!
	\param arm an enumerated type
	\param result is a std:vector of doubles
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), sendGoal()
	 */
	bool sendGoal(ArmsCartesian::WhichArm arm, geometry_msgs::PoseStamped pose);
	///! Status
	//! Determines if any arm is still in motion
	/*!
	\return True upon successful arm movement to posture position
	\sa ArmsCartesian(), ~ArmsCartesian(), cancelMotion(), updateState(), waitForMotion()
	 */
	bool motionComplete();
	//! Cancels any arm motions
	/*!
	\return True upon successful cancellation of arm movement
	\sa ArmsCartesian(), ~ArmsCartesian(), motionComplete(), updateState(), waitForMotion()
	 */
	bool cancelMotion();
	//! Updates the current arm controllers
	/*!
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), motionComplete(), cancelMotion(), waitForMotion()
	 */
	bool updateState();
	//! A blocking function that waits for arm motions to complete within a specified amount of time
	/*!
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), motionComplete(), cancelMotion(), updateState()
	 */
	bool waitForMotion(double max_duration);
	//!Overloaded setGain functions
	//! Sets gain values to a specifed arm
	/*!
	\param gain_name is a std::string
	\param arm is a std::string
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), setGains()
	 */
	bool setGains(std::string gain_name, std::string arm);
	//! Sets gain values to a specifed arm
	/*!
	\param gain_name is a std::string
	\param arm is an enumerated type
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), setGains()
	 */
	bool setGains(std::string gain_name, ArmsCartesian::WhichArm arm);
	//! Sets specified gain values to a specifed arm
	/*!
	\param gains is a std::vector of doubles
	\param arm is an enumerated type
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), setGains()
	 */
	bool setGains(std::vector<double>& gains, std::string arm);
	//! Sets specified gain values to a specifed arm
	/*!
	\param req is a apc_msgs::SetGain::Request
	\param res is a apc_msgs::SetGain::Response
	\return True upon successful update
	\sa ArmsCartesian(), ~ArmsCartesian(), setGains()
	 */
	bool setGains(apc_msgs::SetGain::Request &req, apc_msgs::SetGain::Response &res);
};


#endif /* APC_ARMS_CARTESIAN_H_ */
