/*!
 * 	\name 	   ArmsJoint
 *  \brief     Controls arm movement through joint control
 *  \details   Using a PR2 action client, joint arm positions are sent through public functions
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Sept 16, 2013
 *  \pre       First initialize the system.
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#ifndef APC_ARMS_JOINT_H_
#define APC_ARMS_JOINT_H_

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
// Messages
#include <control_msgs/JointTrajectoryAction.h>
#include <control_msgs/JointTrajectoryControllerState.h>

typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;


/*! \class ArmsJoint apc_arms_joint.h "include/apc_arms_joint.h"
 *  \brief Controls arm movement through joint control
 */
class ArmsJoint
{
private:
	//!ROS Handle
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;

	//! Left Arm Action Client
	/*!
      Provides ROS services to ArmJoint class. Sends left arm commands to PR2.
	 */
	TrajClient* left_client;

	//! Right Arm Action Client
	/*!
      Provides ROS services to ArmJoint class. Sends right arm commands to PR2.
	 */
	TrajClient* right_client;

	/// \brief Helper variables
	std::string arm_prefix;
	bool leftMotionInProgress;
	bool rightMotionInProgress;

public:
    //! An enumerated type
    /*! Allows for easy description of arm selection throughout the class. */
	typedef enum {
		LEFT, 	/*!< Enum value LEFT. */
		RIGHT,	/*!< Enum value RIGHT. */
		BOTH	/*!< Enum value BOTH. */
	} WhichArm;

	//! A constructor.
	/*!
    Initializes action client and waits for server
	 */
	ArmsJoint();
	//! A deconstructor.
	/*!
    Deconstructs action clients
	 */
	~ArmsJoint();

	//! Sends joint positions to be sent to the arm controller determined by the user
	/*!
    \param joints a vector of doubles
    \param a a enumerated type
    \return The test results
    \sa ArmsJoint(), ~ArmsJoint(), cancelMotion(),get_left_joint_angles(), get_left_joint_angles() and motionComplete()
	 */
	bool sendGoal(std::vector<double> joints, ArmsJoint::WhichArm a);

	//! Check if goal has been completed
	/*!
    \return Boolean representing if current state of motion. True:Motion Succeeded. False: Motion in progress or failed
    \sa ArmsJoint(), ~ArmsJoint(), sendGoal(), cancelMotion(),get_left_joint_angles() and get_left_joint_angles()
	 */
	bool motionComplete();

	//! Cancels any current motion
	/*!
    \sa ArmsJoint(), ~ArmsJoint(), sendGoal(),get_left_joint_angles(), get_left_joint_angles() and motionComplete()
	 */
	void cancelMotion();

	//! Get current right arm angles from the state being broadcast by the r(l)_arm_controller, by listening for a single message.
	/*!
    \param current_angles an integer array argument.
    \sa ArmsJoint(), ~ArmsJoint(), sendGoal(), cancelMotion(), get_left_joint_angles() and motionComplete()
	 */
	void get_right_joint_angles(double current_angles[7]);

	//! Get current left arm angles from the state being broadcast by the l(l)_arm_controller, by listening for a single message.
	/*!
    \param current_angles an integer array argument.
    \sa ArmsJoint(), ~ArmsJoint(), sendGoal(), cancelMotion(), get_right_joint_angles() and motionComplete()
	 */
	void get_left_joint_angles(double current_angles[7]);
};


#endif /* APC_ARMS_JOINT_H_ */
