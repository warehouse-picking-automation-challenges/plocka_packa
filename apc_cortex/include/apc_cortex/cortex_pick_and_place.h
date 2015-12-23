/*!
 * 	\name 	   Cortex
 *  \brief     Main cortex class
 *  \details   Implments state machine for the development of different autonomous tasks
 *  \author    Sven Cremer
 *  \author	   Rommel Alonzo
 *  \version   1.0
 *  \date      Sept 21, 2016
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

#ifndef CORTEX_PICK_AND_PLACE_H_
#define CORTEX_PICK_AND_PLACE_H_

#include <ros/ros.h>
#include <apc_robot/apc_robot_grippers.h>
#include <apc_robot/pr2_head.h>
#include <apc_robot/pr2_torso.h>
#include <apc_robot/apc_arms_cartesian.h>
#include <apc_robot/apc_arms_joint.h>
#include <apc_robot/base.h>
//#include <apc_robot/apc_robot_moveit.h>

#include <apc_msgs/ToolPos.h>

#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_common_action_msgs/TuckArmsAction.h>
#include <actionlib/client/simple_action_client.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

typedef actionlib::SimpleActionClient<pr2_common_action_msgs::TuckArmsAction> TuckArmsClient;

/*! \class Cortex cortex_pick_and_place.h "include/cortex_pick_and_place.h"
 *  \brief Main cortex class
 */
class Cortex
{
private:
	ros::NodeHandle nh;

	std::vector<geometry_msgs::PoseStamped> locationsVector;
	geometry_msgs::Pose locationPose;

	std::vector<geometry_msgs::PoseStamped> armVector;
	geometry_msgs::Pose armPose;

	ros::ServiceClient 	switch_controllers_service;
	std::vector<std::string> arm_controllers_default;
	std::vector<std::string> arm_controllers_JTcart;

	ros::Publisher initialPose_pub;

	ros::ServiceClient marker_client;
	apc_msgs::ToolPos marker_msg;
	int marker_number;

	bool executingRun;

	TuckArmsClient* tuck_ac_;
	actionlib::SimpleClientGoalState* ac_state;
	pr2_common_action_msgs::TuckArmsGoal armActionGoal;

	ros::Time storedTime;

	std::vector<double> l_untuck_joints;
	std::vector<double> l_object_tuck;
	std::vector<double> l_untuck_placing_joints;
	double dt;
	double x_step;
	double distance;

public:
	/******Enums******/
	//! An enumerated type
	/*! Allows for easy description of state machine throughout the class. */
	typedef enum
	{
		START,							/*!< Enum value START. */
		TUCK,							/*!< Enum value TUCK. */
		UNTUCK,							/*!< Enum value UNTUCK. */
		WAIT_FOR_TUCK,					/*!< Enum value WAIT_FOR_TUCK. */
		MOVE_TO_START_LOCATION,			/*!< Enum value MOVE_TO_START_LOCATION. */
		PREPARE_ARMS_1,					/*!< Enum value PREPARE_ARMS_1. */
		PREPARE_ARMS_2,					/*!< Enum value PREPARE_ARMS_2. */
		MOVE_TO_PICKING_LOCATION,		/*!< Enum value MOVE_TO_PICKING_LOCATION. */
		WAIT_FOR_BASE_MOTION,			/*!< Enum value WAIT_FOR_BASE_MOTION. */
		START_PICKING,					/*!< Enum value START_PICKING. */
			GET_OBJECT_LOCATION,		/*!< Enum value GET_OBJECT_LOCATION. */
			MOVE_TO_OBJECT,				/*!< Enum value MOVE_TO_OBJECT. */
			PICK_OBJECT,				/*!< Enum value PICK_OBJECT. */
		TUCK_OBJECT,					/*!< Enum value TUCK_OBJECT. */
		START_PLACING,					/*!< Enum value START_PLACING. */
			MOVE_TO_DROP_LOCATION,		/*!< Enum value MOVE_TO_DROP_LOCATION. */
			PLACE_OBJECT,				/*!< Enum value PLACE_OBJECT. */
		STOP_PLACING,					/*!< Enum value STOP_PLACING. */
		MOVE_BACK,						/*!< Enum value MOVE_BACK. */
		DONE							/*!< Enum value DONE. */
	}RunState;

	struct StateInfo{

	};
	//!Constructer
	/*!
	 Constructor
	 */
	Cortex();
	//!Deconstructer
	/*!
	 Deconstructor
	 */
	~Cortex();
	//! Number of trials
	/*!
      Number of trials to run. Implicitly, it also represents the number of objects to be picked up
	 */
	int number_trails;

	/****************************************Helper Functions****************************************/
	//! Initializes initial position for the ROS navigation stack
	/*!
	\sa switchControllers(), checkIfMotionsComplete(), loadNavParameters(), loadArmParameters(), getLocationPose(), getArmPose(), getMarker()
	 */
	void initPose();
	//! Switch PR2 arm controllers
	/*!
	\param start_controllers a std::vector of std::string
	\param stop controllers a std::vector of std::string
	\sa initPose(), checkIfMotionsComplete(), loadNavParameters(), loadArmParameters(), getLocationPose(), getArmPose(), getMarker()
	 */
	void switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers);
	//! Checks if robotic motions have stopped
	/*!
	\return True upon successful check of motion completion. AKA, motions have stopped.
	\sa initPose(), switchControllers(), loadNavParameters(), loadArmParameters(), getLocationPose(), getArmPose(), getMarker()
	 */
	bool checkIfMotionsComplete();
	//! Load in navigation parameters and navigation zones
	/*!
	\return True upon successfully loading in navigation parameters and zones
	\sa initPose(), switchControllers(), checkIfMotionsComplete(), loadArmParameters(), getLocationPose(), getArmPose(), getMarker()
	 */
	bool loadNavParameters();
	//! Load in arm parameters
	/*!
	\return True upon successfully loading in arm parameters
	\sa initPose(), switchControllers(), checkIfMotionsComplete(), loadNavParameters(), getLocationPose(), getArmPose(), getMarker()
	 */
	bool loadArmParameters();
	//! Retrieve location given a navigation zone name
	/*!
	\param name a std::string
	\param result a geometry_msgs::Pose
	\return True upon successful retrieval of zone position
	\sa initPose(), switchControllers(), checkIfMotionsComplete(), loadNavParameters(), loadArmParameters(), getArmPose(), getMarker()
	 */
	bool getLocationPose(std::string name, geometry_msgs::Pose* result);		// TODO: generalize
	//! Retrieve arm location
	/*!
	\param name a std::string
	\param result a geometry_msgs::Pose
	\return True upon successful retrieval of arm position
	\sa initPose(), switchControllers(), checkIfMotionsComplete(), loadNavParameters(), loadArmParameters(), getLocationPose(), getMarker()
	 */
	bool getArmPose(std::string name, geometry_msgs::Pose* result);
	//! Retrieve marker location
	/*!
	\param marker_number a integer
	\return True upon successful retrieval of marker position
	\sa initPose(), switchControllers(), checkIfMotionsComplete(), loadNavParameters(), loadArmParameters(), getLocationPose(), getArmPose()
	 */
	bool getMarker(int marker_number);
	//! State Machine
	/*!
	\param start an enumerated type
	\param stop an enumerated type
	\return True upon successful execution of state
	 */
	bool runSM(Cortex::RunState start, Cortex::RunState stop);

	/******Motion clients******/
	/*!
	 Gripper client class
	 */
	Gripper* 	    grippers;
	/*!
	 Torso client class
	 */
	Torso*			torso;
	/*!
	 Head client class
	 */
	Head*			head;
	/*!
	 Base client class
	 */
	Base* 			base;
	/*!
	 Arm Cartesian client class
	 */
	ArmsCartesian*  arms;
	/*!
	 Arm Joint client class
	 */
	ArmsJoint*  	armsJoint;
	/******Motion flags******/
	/*!
	 True if head is in motion
	 */
	bool movingHead;
	/*!
	 True if grippers are in motion
	 */
	bool movingGrippers;
	/*!
	 True if torso is in motion
	 */
	bool movingTorso;
	/*!
	 True if arms are in motion
	 */
	bool movingArms;
	/*!
	 True if any part of the PR2 is in motion
	 */
	bool motionsComplete;

	/******Other variables******/
	/*!
	 Current run state
	 */
	RunState  currentRunState;
	/*!
	 Next run state
	 */
	RunState  nextRunState;

};

#endif /* CORTEX_PICK_AND_PLACE_H_ */
