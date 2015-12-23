/*!
 * 	\name 	   RobotMoveit
 *  \brief     Controls PR2 arm movements through MoveIt
 *  \details   Arm movements are implmented through the MoveIt API
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Dec 14, 2014
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

#ifndef APC_ROBOT_MOVEIT_H_
#define APC_ROBOT_MOVEIT_H_

// ROS
#include <ros/ros.h>

// Messages
#include <geometry_msgs/Pose.h>
#include <sensor_msgs/JointState.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>

#include <moveit/trajectory_processing/iterative_time_parameterization.h>

// TODO JointStateGroup has been moved into RobotState
// TODO use collision avoidance
/*! \class RobotMoveit apc_robot_moveit.h "include/apc_robot_moveit.h"
 *  \brief Controls PR2 arm movements through MoveIt
 */

/*! \class RobotMoveit apc_robot_moveit.h "include/apc_robot_moveit.h"
 *  \brief Provides interface features that simplify commands to the MoveIt API
 */
class RobotMoveit
{
private:

	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;
	/*!
      std::vector<double> containing current joint positions for the left arm
	 */
	std::vector<double> joint_pos_left_arm;
	/*!
      std::vector<double> containing current joint positions for the right arm
	 */
	std::vector<double> joint_pos_right_arm;
	/*!
      RobotModel pointer containing the structure of the entire structure of the arm
	 */
	robot_model::RobotModelPtr kinematic_model;
	/*!
      RobotState pointer containing the state of the arm's configuration
	 */
	robot_state::RobotStatePtr kinematic_state;
	/*!
      An array of strings representing the name of every individual joint for the left arm
	 */
	std::vector<std::string> joint_names_right_arm;
	/*!
      An array of strings representing the name of every individual joint for the right arm
	 */
	std::vector<std::string> joint_names_left_arm;
	/*!
      roveit::planing_interface::MoveGroup::Plan
	 */
	moveit::planning_interface::MoveGroup::Plan my_plan;
	/*!
      roveit::planing_interface::MoveGroup for the right arm
	 */
	moveit::planning_interface::MoveGroup* group_right_arm;
	/*!
      roveit::planing_interface::MoveGroup for the left arm
	 */
	moveit::planning_interface::MoveGroup* group_left_arm;
	//moveit::planning_interface::MoveGroup* group_both_arms;

	// Planning
	//moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
	// Rviz
	//moveit_msgs::DisplayTrajectory* display_trajectory;
	/*!
      Represents left arm motion
      true: arm is in motion
      false: arm is not in motion
	 */
	bool leftMotionInProgress;
	/*!
      Represents right arm motion
      true: arm is in motion
      false: arm is not in motion
	 */
	bool rightMotionInProgress;
	// Groovy
	/*!
	  Moveit class representing the current state of the right arm
	 */
	robot_state::JointStateGroup* joint_state_group_right_arm;
	/*!
	  Moveit class representing the current state of the left arm
	 */
	robot_state::JointStateGroup* joint_state_group_left_arm;
public:
	//! An enumerated type
	/*! Allows for easy description of arm selection throughout the class. */
	typedef enum
	{
		LEFT, 		/*!< Enum value LEFT. */
		RIGHT, 		/*!< Enum value RIGHT. */
		BOTH 		/*!< Enum value BOTH. */
	} WhichArm;
	/*!
		 Constructor
	 */
	RobotMoveit();
	/*!
		 Deconstructor
	 */
	~RobotMoveit();
	//! Sets group pointer to desired arm. Parameter group must be passed by reference, i.e. "&group".
	/*!
	 \param arm an enumerated type
	 \param group an moveit::planning_interface::MoveGroup
	 \return True if arm selection is successful
	 */
	bool selectArmGroup(RobotMoveit::WhichArm arm, moveit::planning_interface::MoveGroup** group);
	//! Updates class joint states
	/*!
	 \return True if joint update is successful
	 \sa printJointStates(), printJointStatesSRDF(), getEndeffectorPose()
	 */
	bool updateJointStates();
	//! Outputs joint states on screen
	/*!
	 \return True if joint update is successful
	 \sa updateJointStates(), printJointStatesSRDF(), getEndeffectorPose()
	 */
	void printJointStates();
	//! Outputs joint states on screen
	/*!
	 \return True if joint update is successful
	 \sa printJointStates(), printJointStatesSRDF(), getEndeffectorPose()
	 */
	void printJointStatesSRDF();
	//! Gets the end effector pose
	/*!
	 \param arm is an enumerated type
	 \param current_pose is geometry_msgs::Pose
	 \return True if retrieval of end effector pose is successful
	 \sa printJointStates(), printJointStatesSRDF(), printJointStatesSRDF()
	 */
	bool getEndeffectorPose(RobotMoveit::WhichArm arm, geometry_msgs::Pose* current_pose);
	//! Move arm to group state specified inside baxter.srdf, which is loaded by planning_context.launch
	/*!
	 \param arm is an enumerated type
	 \param pose_name a std::string
	 \param waitForMotion a boolean
	 \return True if arm moving to current_pose is successful
	 */
	bool moveToPose(RobotMoveit::WhichArm arm, std::string pose_name, bool waitForMotion = true);
	// Kinematics
	//! Gets joint states from right and left arms through MoveIt for the Kinematic model
	/*!
	 \param joint_values_left_arm is a std::vector of doubles
	 \param joint_values_right_arm is a std::vector of doubles
	 \return True if arm update is successful
	 \sa setKinematicModelJointValues(), kinematicModelFK(), kinematicModelIK(), kinematicModelJacobian()
	 */
	bool getKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm);	// note: this is for the kinematic model, not the actual robot
	//! Set joint states from right and left arms through MoveIt for the Kinematic model
	/*!
	 \param joint_values_left_arm is a std::vector of doubles
	 \param joint_values_right_arm is a std::vector of doubles
	 \return True if arm update is successful
	 \sa getKinematicModelJointValues(), kinematicModelFK(), kinematicModelIK(), kinematicModelJacobian()
	 */
	bool setKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm);
	//! NA
	/*!
	 \sa getKinematicModelJointValues(), setKinematicModelJointValues(), kinematicModelIK(), kinematicModelJacobian()
	 */
	bool kinematicModelFK();
	//! NA
	/*!
	 \sa getKinematicModelJointValues(), setKinematicModelJointValues(), kinematicModelFK(), kinematicModelJacobian()
	 */
	bool kinematicModelIK();
	//! NA
	/*!
	 \sa getKinematicModelJointValues(), setKinematicModelJointValues(), kinematicModelFK(), kinematicModelIK()
	 */
	bool kinematicModelJacobian();

	// Arm control
	//! Moves arm to specified joint states values
	/*!
	 \param arm an enumerated type
	 \param joint_values is a std::vector of doubles
	 \return True if arm movement is successful
	 \sa executeCarteGoal(), executeCarteGoalWithConstraint()
	 */
	bool executeJointGoal(RobotMoveit::WhichArm arm, std::vector<double> joint_values);
	//! Moves arm to specified cartesian space pose
	/*!
	 \param arm an enumerated type
	 \param target a geometry_msgs::Pose
	 \param waitForMotion a boolean
	 \return True if arm movement is successful
	 \sa executeJointGoal(), executeCarteGoalWithConstraint()
	 */
	bool executeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, bool waitForMotion = true);
	//! Moves arm to specified cartesian space pose with constraints
	/*!
	 \param arm an enumerated type
	 \param target a geometry_msgs::Pose
	 \return True if arm movement is successful
	 \sa executeJointGoal(), executeCarteGoal()
	 */
	bool executeCarteGoalWithConstraint(RobotMoveit::WhichArm arm, geometry_msgs::Pose target);
	//! Moves arm though specified joint state path
	/*!
	 \param arm an enumerated type
	 \param joint_waypoint is a std::vector of std::vector of doubles
	 \return True if arm movement is successful
	 \sa executeCartePath()
	 */
	bool executeJointPath(RobotMoveit::WhichArm arm, std::vector< std::vector<double> > joint_waypoints);
	//! Moves arm though specified cartesian space path
	/*!
	 \param arm an enumerated type
	 \param waypoints a std::vector of geometry_msgs::Pose
	 \param dx a double
	 \param fractionThreshold a double
	 \return True if arm movement is successful
	 \sa executeJointPath()
	 */
	bool executeCartePath(RobotMoveit::WhichArm arm, std::vector<geometry_msgs::Pose> waypoints, double dx = 0.01, double fractionThreshold = 1);
	//! Safely reach cartesian goal with constraints to prevent self-collision
	/*!
	 \param arm an enumerated type
	 \param target a geometry_msgs::Pose
	 \param nHold a integer
	 \param dx a double
	 \param fractionThreshold a double
	 \return True if arm movement is successful
	 */
	bool safeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, int nHold = 5, double dx = 0.01, double fractionThreshold = 1); // Avoids unnecessary joint rotations
	// Status
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
	bool cancelMotion();
};



#endif /* APC_ROBOT_MOVEIT_H_ */
