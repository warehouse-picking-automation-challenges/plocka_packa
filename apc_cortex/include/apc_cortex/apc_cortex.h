/*!
 * 	\name 	   ApcCortex
 *  \brief     Main cortex class
 *  \details   Implments state machine for the development of different autonomous tasks
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Mar 10, 2015
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

#ifndef APC_CORTEX_H_
#define APC_CORTEX_H_

#include <ros/ros.h>

#include <apc_msgs/GetWorkOrder.h>
#include <apc_msgs/WorkOrder.h>
#include <apc_msgs/GetBinContents.h>
#include <apc_msgs/SetBin.h>
#include <apc_msgs/Empty.h>
#include <apc_msgs/GetObjectPose.h>
#include <apc_msgs/SetShelfOrigin.h>
#include <apc_msgs/GetBinOrigin.h>
#include <apc_msgs/SetBaseParameter.h>
#include <apc_msgs/GetBinInfo.h>
#include <apc_msgs/GetGraspStrategy.h>
#include <apc_msgs/SetGain.h>
#include <apc_msgs/GetBool.h>
#include <apc_msgs/GetItemFromWorkOrder.h>
#include <apc_msgs/ToolPos.h>

//#include <apc_robot/apc_robot_commander.h>
#include <apc_robot/apc_robot_moveit.h>
#include <apc_robot/apc_robot_grippers.h>
#include <apc_robot/pr2_head.h>
#include <apc_robot/pr2_torso.h>
#include <apc_robot/apc_arms_cartesian.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>

#include <apc_cortex/BaseMovement.h>

/*! \class ApcCortex apc_cortex.h "include/apc_cortex.h"
 *  \brief Main cortex class
 */
class ApcCortex
{
public:
	//! An enumerated type
	/*! Allows for easy description of state machine throughout the class. */
	typedef enum
	{
		START,						/*!< Enum value START. */
		WAIT_FOR_MOTION,			/*!< Enum value WAIT_FOR_MOTION. */
		WAIT_FOR_JSON,				/*!< Enum value WAIT_FOR_JSON. */
		SORT_WORK_ORDER,			/*!< Enum value SORT_WORK_ORDER. */
		     START_PICKING,			/*!< Enum value START_PICKING. */
			 GET_OBJECT_LOCATION, 	/*!< Enum value GET_OBJECT_LOCATION. */
			 DETERMINE_STRATEGY,	/*!< Enum value DETERMINE_STRATEGY. */
			 MOVE_TO_TOOL,			/*!< Enum value MOVE_TO_TOOL. */
			 PICK_TOOL,				/*!< Enum value PICK_TOOL. */
			 MOVE_TO_CENTER,		/*!< Enum value MOVE_TO_CENTER. */
			 MOVE_ARM,				/*!< Enum value MOVE_ARM. */
			 MOVE_TOWARDS_BIN,		/*!< Enum value MOVE_TOWARDS_BIN. */
			 GET_OBJECT_LOCATION_2,	/*!< Enum value GET_OBJECT_LOCATION_2. */
			 PICK_ITEM,				/*!< Enum value PICK_ITEM. */
			 MOVE_TO_CENTER_2,		/*!< Enum value MOVE_TO_CENTER_2. */
			 MOVE_TO_ORDER_BIN,		/*!< Enum value MOVE_TO_ORDER_BIN. */
			 PLACE_ITEM,			/*!< Enum value PLACE_ITEM. */
			 MOVE_TO_HOME,			/*!< Enum value MOVE_TO_HOME. */
			 DONE_PICKING,			/*!< Enum value DONE_PICKING. */
		SELECT_NEXT_ITEM,			/*!< Enum value SELECT_NEXT_ITEM. */
		DONE						/*!< Enum value DONE. */
	}RunState;
	//! An enumerated type
	/*! Allows for easy description of grasping strategies throughout the class. */
	typedef enum
	{
		//FRONT__CENTER,
		//LIFT_ANGLED__CENTER
		FRONT,						/*!< Enum value FRONT. */
		LIFT,						/*!< Enum value LIFT. */
		ANGLEDLIFT,					/*!< Enum value ANGLEDLIFT. */
		VACUUMFRONT,				/*!< Enum value VACUUMFRONT. */
		UNKNOWN						/*!< Enum value UNKNOWN. */
	}GraspStrategy;
	//! An enumerated type
	/*! Allows for easy description of grasping tools throughout the class. */
	typedef enum
	{
		ELECTRIC,					/*!< Enum value ELECTRIC. */
		VACUUM,						/*!< Enum value VACUUM. */
		SCOOP,						/*!< Enum value SCOOP. */
		TEST = 8					/*!< Enum value TEST. */
	}Tool;
	//! An enumerated type
	/*! Allows for easy description of arm selection throughout the class. */
	typedef enum
	{
		LEFT,						/*!< Enum value LEFT. */
		RIGHT,						/*!< Enum value RIGHT. */
		BOTH						/*!< Enum value BOTH. */
	}WhichArm;
	//! An enumerated type
	/*! Allows for easy description of arm configuration throughout the class. */
	typedef enum
	{
		L_HOME_01,					/*!< Enum value L_HOME_01. */
		R_HOME_01					/*!< Enum value R_HOME_01. */
	}ArmConfigurations;
	//! An enumerated type
	/*! Allows for easy description of bin points throughout the class. */
	typedef enum
	{
		CENTER,						/*!< Enum value CENTER. */
		A,							/*!< Enum value A. */
		B,							/*!< Enum value B. */
		C,							/*!< Enum value C. */
		O							/*!< Enum value O. */
	}BinPoints;

	/****** Structs ******/
	//! A struct
	/*! Allows for easy abstraction of different gripper positions for individual arms */
	struct GripperPose{
		/*@{*/
		WhichArm arm;				/**< Arm Selection */
		geometry_msgs::Pose pose;	/**< Cartesian Coordinate */
		/*@{*/
	};
	//! A struct
	/*! Allows for easy abstraction of the current state of the cortex*/
	struct StateInfo{
		/*@{*/
		Tool currentToolLeft;					/**< Current tool on the left arm */
		Tool currentToolRight;					/**< Current tool on the right arm */
		Tool requiredToolLeft;					/**< Required tool for the next item to pick on the left arm */
		Tool requiredToolRight;					/**< Required tool for the next item to pick on the right arm */
		Tool requiredTool;						/**< Required tool for the next item to pick */
		  /*@}*/
		  /**
		   * @name Group 2
		   */
		  /*@{*/
		int currentBin;							/**< Current bin that item that is currently being picked is located in */
		std::string currentObject;				/**< Current item being picked */
		apc_msgs::GetBinInfo currentBinInfo;	/**< General bin information */
		  /*@}*/
		  /**
		   * @name Group 3
		   */
		  /*@{*/
		GraspStrategy requiredStrategy;						/**< Required strategy to pick the current item */
		WhichArm requiredArm;								/**< Arm required to pick object */
		Gripper::WhichArm requiredArmGripper;				/**< Arm required to pick object. Different enum type */
		ArmsCartesian::WhichArm requiredArmCartesian;		/**< Arm required to pick object. Different enum type  */
		  /*@}*/
		  /**
		   * @name Group 4
		   */
		  /*@{*/
		std::string currentObjectOrienation;					/**< Current object's orientation  */
		geometry_msgs::Pose currentObjPose;						/**< Current object's position  */
		geometry_msgs::PoseStamped currentGripperPose;			/**< Current gripper's position  */
		geometry_msgs::PoseStamped currentGripperPoseLeft;		/**< Current gripper's position for the left arm  */
		geometry_msgs::PoseStamped currentGripperPoseRight;		/**< Current gripper's position for the right arm  */
		geometry_msgs::Point orderBinLocation;					/**< Current gripper's position for the left arm  */
		  /*@}*/
		  /**
		   * @name Group 5
		   */
		  /*@{*/
		int itemsLeftToPick;									/**< Number of items left to pick */
		int objectsPicked;										/**< Number of objects picked*/
		int currentItemNumber;									/**< Number of item number being picked */
		int totalNumberOfItems;									/**< Total number of items */
//		bool leftArmHome;
//		bool rightArmHome;
//		int r_gripper_bin_location; 	// -1 if not in front of a bin
	};
	//! A struct
	/*! Allows for easy abstraction of the gripping strategy needed to pick up an object */
	struct objectOrder{
		int bin;												/**< Bin number */
		std::string name;										/**< Name of object */
		ApcCortex::WhichArm requiredArm;						/**< Arm required to pick up object */
		ArmsCartesian::WhichArm requiredArmCartesian;			/**< Arm required to pick up object */
		Gripper::WhichArm requiredArmGripper;					/**< Arm required to pick up object */
		ApcCortex::GraspStrategy requiredStrategy;				/**< Gripping strategy required to pick up an object */
		ApcCortex::Tool requiredTool;							/**< Required tool to pick up an object */
	};
	//! A struct
	/*! Allows for easy abstraction of the list of objects needing different tools */
	struct sortedWorkOrder{
		std::vector<ApcCortex::objectOrder> vacuumLift;			/**< Items requiring the vacuum lift to pick*/
		std::vector<ApcCortex::objectOrder> electricFront;		/**< Items requiring the electric front to pick*/
		std::vector<ApcCortex::objectOrder> vacuumFront;		/**< Items requiring the vacuum front to pick*/
		std::vector<ApcCortex::objectOrder> electricAngledLift; /**< Items requiring the electric angled lift to pick*/
	};
	//! A struct
	/*! Allows for easy abstraction of the number of objects needing different tools */
	struct sortedWorkOrderItemsLeft{
		int vacuumLift;											/**< Number of items requiring the vacuum lift to pick*/
		int electricFront;										/**< Number of items requiring the electric front to pick*/
		int vacuumFront;										/**< Number of items requiring the vacuum front to pick*/
		int electricAngledLift;									/**< Number of items requiring the electric angled lift to pick*/
	};

private:
	ros::NodeHandle nh;

	geometry_msgs::Pose tmp;

	StateInfo stateVariables;
	RunState  currentRunState;
	RunState  nextRunState;

	double loopRateRunSM;

	bool executingRun;
	bool receivedJSON;
	bool waitingStarted;
	bool switchLeftTool;
	bool switchRightTool;

	ros::Time time_now;
	ros::Time time_start;
	ros::Time time_stop;
	double max_execution_time;
	double max_arm_execution_time;

	/* Service msgs */
	apc_msgs::GetWorkOrder 			srv_msg_GetWorkOrder;
	apc_msgs::GetItemFromWorkOrder 	srv_msg_GetItemFromWorkOrder;
	apc_msgs::GetGraspStrategy 		srv_msg_GetGraspStrategy;
	apc_msgs::GetBool       		srv_msg_GetBool;
	apc_msgs::SetBin 				srv_msg_SetBin;
	apc_msgs::Empty 				srv_msg_Empty;
	apc_msgs::GetObjectPose 		srv_msg_GetObjectPose;

	sortedWorkOrder work_order;
	sortedWorkOrderItemsLeft items_left;
	std::vector<ApcCortex::objectOrder>::const_iterator orderIter;

//	GripperPose l_arm_home;
//	GripperPose r_arm_home;
	std::string robot_frame;	// frame for cartesian gripper poses

	/* Motion clients */

	//RobotCommander* robot_commander;
	//RobotMoveit* 	robot_moveit;
	Gripper* 	    grippers;
	Torso*			torso;
	Head*			head;
	BaseMovement* 	base;
	ArmsCartesian*  arms;

	bool movingHead;
	bool movingGrippers;
	bool movingTorso;
	bool movingArms;
	bool motionsComplete;

	/* Service clients */

	// apc_json/scripts/apc_parser.py
	ros::ServiceClient srvClient_get_status;
	ros::ServiceClient srvClient_get_work_order;
	ros::ServiceClient srvClient_get_bin_contents;
	ros::ServiceClient srvClient_work_order_request;

	// apc_object_detecion/src/apc_ork_manager.cpp
	ros::ServiceClient srvClient_startORK;
	ros::ServiceClient srvClient_stopORK;

	// apc_object_detecion/src/apc_object_detecion.cpp
	ros::ServiceClient srvClient_get_object_pose;

	// apc_shelf/src/shelf_properties.cpp
	ros::ServiceClient srvClient_set_shelf_origin;
	ros::ServiceClient srvClient_get_bin_origin;
	ros::ServiceClient srvClient_get_bin_info;

	//apc_robot/src/apc_base_driver.cpp
	ros::ServiceClient srvClient_base_parameters;

	//apc_json/scripts/json_reader.py
	ros::ServiceClient srvClient_get_grasping_strategy;

	//apc_robot/src/apc_arms_cartesian.cpp
	ros::ServiceClient srvClient_set_arm_gain;

	//apc_robot/src/apc_marker.cpp
	ros::ServiceClient srvClient_toolPose;

public:
	//!Constructer
	/*!
	 Constructor
	 */
	ApcCortex();
	//!Deconstructer
	/*!
	 Deconstructor
	 */
	~ApcCortex();

	//! State Machine
	/*!
	\param start an enumerated type
	\param stop an enumerated type
	\param waitForMotion is a boolean
	\return True upon successful execution of state
	 */
	bool runSM(ApcCortex::RunState start, ApcCortex::RunState stop);

	//! updates the grasping strategy based on an given item, bin location and object orientation
	/*!
	\param item a string
	\param bin a integer
	\param orientation a string
	\return True upon successfully updating grasping strategy
	\sa updateGraspingStrategy(), getBinOrigin(), getBinInfo(), getBinColumn()
	 */
	bool updateGraspingStrategy(std::string item, int bin, std::string orientation);
	//! set shelf origin
	/*!
	\param p a geometry_msgs::Point
	\return boolean upon successful setting of shelf origin
	\sa updateGraspingStrategy(), getBinOrigin(), getBinInfo(), getBinColumn()
	 */
	bool setShelfOrigin(geometry_msgs::Point p);
	//! Get bin origin
	/*!
	\param bin_nr a integer
	\param p a geometry_msgs::Point
	\return True upon successful retrieval of the bin origin
	\sa updateGraspingStrategy(), setShelfOrigin(), getBinInfo(), getBinColumn()
	 */
	bool getBinOrigin(int bin_nr, geometry_msgs::Point& p);
	//! Get bin information
	/*!
	\param bin_nr a integer
	\param result a apc_msgs::GetBinInfo
	\return True upon successful retrieval of bin inforamtion
	\sa updateGraspingStrategy(), setShelfOrigin(), getBinOrigin(), getBinColumn()
	 */
	bool getBinInfo(int bin_nr, apc_msgs::GetBinInfo& result);
	//! Get bin column
	/*!
	\param bin_nr a integer
	\param result a apc_msgs::GetBinInfo
	\return True upon successful retrieval of bin column
	\sa updateGraspingStrategy(), setShelfOrigin(), getBinOrigin(), getBinColumn()
	 */
	int getBinColumn(int bin_number);

	/* Arm movement */
	//! Moves a specified arm to bin
	/*!
	\param bin_number a integer
	\param a an enumerated type
	\return True upon successful movement of specified arm to bin
	\sa moveToBinPlane(), moveInsideBin(), getArmPose()
	 */
	bool moveToBin(int bin_number, ApcCortex::WhichArm a);
	//! Moves a specified arm to bin plane
	/*!
	\param bin_number a integer
	\param a an enumerated type
	\return True upon successful movement of specified arm to bin
	\sa moveToBin(), moveInsideBin(), getArmPose()
	 */
	bool moveToBinPlane(int bin_number, ApcCortex::WhichArm a, double distance_from_robot);
	//! Moves a specified arm inside of a bin
	/*!
	\param distance a double
	\return True upon successful movement of specified arm into bin
	\sa moveToBin(), moveToBinPlane(), getArmPose()
	 */
	bool moveInsideBin(double distance);
	//! Gets current arm position
	/*!
	\param a a structure
	\param p a geometry_msgs::Pose
	\return True upon successful retreival of arm information
	\sa moveToBin(), moveToBinPlane(), moveInsideBin()
	 */
	bool getArmPose(ApcCortex::ArmConfigurations a, geometry_msgs::PoseStamped& p);
	//! Arm grasps object given a position
	/*!
	\param g an enumerated type
	\param a an enumerated type
	\param gripperPose a geometry_msgs::PoseStamped
	\param objPose a geometry_msgs::PoseStamped
	\param binInfo a structure
	\return True upon successful arm grasp movement
	\sa pickObject()
	 */
	bool grasp(ApcCortex::GraspStrategy g, ApcCortex::WhichArm a,geometry_msgs::PoseStamped& gripperPose, geometry_msgs::Pose objPose, apc_msgs::GetBinInfo binInfo);
	/****************************************Head Movement Functions****************************************/
	//! Picks up object given object name and bin number
	/*!
	\param bin_nr a integer
	\param object_nr a std::string
	\return True upon successful arm grasp movement
	\sa grasp()
	 */
	bool pickObject(int bin_nr, std::string object_nr);
	//! Moves head to look at a specific bin
	/*!
	\param bin_nr a integer
	\param p a enumerated type
	\return True upon successful head movement
	\sa lookAtOrderBin()
	 */
	bool lookAtBin(int bin_nr, ApcCortex::BinPoints p);
	//! Moves head to look at a specific bin given current object
	/*!
	\return True upon successful head movement
	\sa lookAtBin()
	 */
	bool lookAtOrderBin();
	/****************************************Helper Functions****************************************/
	//! Checks to see if robotic movements have completed
	/*!
	\return True upon successful robotic movements have stopped
	\sa getGripperArm(), getCartesianArm(), determineArmFromBin(), sortWorkOrder()
	 */
	bool checkIfMotionsComplete();
	//! Returns proper enumerated type based on arm input
	/*!
	\param a an enumerated type
	\return Proper enumerated type based on input
	\sa checkIfMotionsComplete(), getCartesianArm(), determineArmFromBin(), sortWorkOrder()
	 */
	Gripper::WhichArm getGripperArm(ApcCortex::WhichArm a);
	//! Returns proper enumerated type based on arm input
	/*!
	\param a an enumerated type
	\return Proper enumerated type based on input
	\sa getGripperArm(), getGripperArm(), determineArmFromBin(), sortWorkOrder()
	 */
	ArmsCartesian::WhichArm getCartesianArm(ApcCortex::WhichArm a);
	//! Determines which arm needs to move based on given bin number
	/*!
	\param bin_number a integer
	\return Arm that should be moved given a bin number
	\sa getGripperArm(), getGripperArm(), getCartesianArm(), sortWorkOrder()
	 */
	ApcCortex::WhichArm determineArmFromBin(int bin_number);
	//! Sorts given work order
	/*!
	\param order a vector of apc_msgs::WorkOrder
	\return Successful sorting of work order
	\sa getGripperArm(), getGripperArm(), getCartesianArm(), determineArmFromBin()
	 */
	bool sortWorkOrder(std::vector<apc_msgs::WorkOrder> order);
	/****************************************Testing Functions****************************************/
	//! Tests arm movements toward specific bin
	/*!
	\return True upon successful run of test
	\sa testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingMoveToBin();
	//! Tests Cartesian coordinate retrieval given bin numbers
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingCartesian();
	//! Tests arm movements toward specific Cartesian coordinate
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingCartPose();
	//! Tests arm and gripper movements toward specific Cartesian coordinate
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingGripPose();
	//! Tests cartesian arm movement towards a specified direction
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingCartesianMoveInDirection();
	//! Tests retrieval of cartesian arm position
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	void testingCartesianGetCurrentPose();
	//! Tests grasping process
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingCartesianArmGains(), testingToolPickUp(), testingGripperTilt()
	 */
	bool testingGrasping();
	//! Tests arm movements based on different arm gains
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingToolPickUp(), testingGripperTilt()
	 */
	void testingCartesianArmGains();
	//! Tests arm movements for pickin up tools
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingGripperTilt()
	 */
	bool testingToolPickUp();
	//! Tests tilted pick up gripping strategy
	/*!
	\return True upon successful run of test
	\sa testingMoveToBin(), testingCartesian(), testingCartPose(), testingGripPose(), testingCartesianMoveInDirection(), testingCartesianGetCurrentPose(), testingGrasping(), testingCartesianArmGains(), testingToolPickUp()
	 */
	bool testingGripperTilt();

};

#endif /* _APC_CORTEX_H_ */
