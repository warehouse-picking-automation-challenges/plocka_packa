/*!
 * 	\name 	   APCBaseDriver
 *  \brief     Controls PR2 grippers through action clients
 *  \details   Controls PR2 grippers through action clients
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Dec 22, 2014
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


/* TODO implement additional electric/suction functionalities:

        if self._type == 'electric':
            self._prm['dead_zone'] = self._dyn.config[self._ee + '_goal']
            self._prm['velocity'] = self._dyn.config[self._ee + '_velocity']
            self._prm['moving_force'] = self._dyn.config[self._ee +'_moving_force']
            self._prm['holding_force'] = self._dyn.config[self._ee +'_holding_force']
        elif self._type == 'suction':
            self._prm['vacuum_sensor_threshold'] = self._dyn.config[self._ee + '_vacuum_threshold']
            self._prm['blow_off_seconds'] = self._dyn.config[self._ee + '_blow_off']
*/

#ifndef APC_BAXTER_GRIPPERS_H_
#define APC_BAXTER_GRIPPERS_H_

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// Messages
#include <control_msgs/GripperCommandAction.h>
#include <control_msgs/GripperCommandGoal.h>
#include <apc_msgs/ReturnJointStates.h>


typedef actionlib::SimpleActionClient<control_msgs::GripperCommandAction> GripperClient;
/*! \class Gripper apc_robot_gripper.h "include/apc_robot_gripper.h"
 *  \brief Controls PR2 grippers through action clients
 */
class Gripper
{
  private:
	//!ROS Handle
	/*!
      ROS Handle
	 */
	ros::NodeHandle nh;
	//!Left hand Gripper Clients
	/*!
      Left hand Gripper Clients
	 */
	GripperClient* left_client;
	//!Right hand Gripper Clients
	/*!
      Right hand Gripper Clients
	 */
	GripperClient* right_client;
	//!Joint state client
	/*!
      Joint state client
	 */
	ros::ServiceClient joint_state_client;


	bool leftMotionInProgress;
	bool rightMotionInProgress;

	std::string gripper_left_topic_name;
	std::string gripper_right_topic_name;

	apc_msgs::ReturnJointStates joint_states;

  public:
	//! An enumerated type
	/*! Allows for easy description of arm selection throughout the class. */
	typedef enum
	{
		LEFT,	/*!< Enum value LEFT. */
		RIGHT,	/*!< Enum value RIGHT. */
		BOTH	/*!< Enum value BOTH. */
	} WhichArm;
	//!Constructer
	/*!
	 Constructor
	 */
	Gripper();
	//!Deconstructer
	/*!
	 Deconstructer
	 */
	~Gripper();
	//! Sends position goal to action client and waits for it to finish
	/*!
	 \param angle a double
	 \param effort a double
	 \param a an enumerated type
	 */
	void position(double angle, double effort = -1.0, Gripper::WhichArm a = Gripper::BOTH);
	//! Sends position goal to action client (non-blocking)
	/*!
	 \param angle a double
	 \param effort a double
	 \param a an enumerated type
	 \sa open(), close(), closeGently()
	 */
	void sendGoal(double angle, double effort = -1.0, Gripper::WhichArm a = Gripper::BOTH);
	//! Fully opens grippers
	/*!
	 \param a an enumerated type
	 \sa sendGoal(), close(), closeGently()
	 */
	void open( Gripper::WhichArm a = Gripper::BOTH);
	//! Fully closes grippers
	/*!
	 \param a an enumerated type
	 \sa sendGoal(), open(), closeGently()
	 */
	void close(Gripper::WhichArm a = Gripper::BOTH);
	//! Gently closes grippers
	/*!
	 \param a an enumerated type
	 \sa sendGoal(), open(), close()
	 */
	void closeGently(Gripper::WhichArm a = Gripper::BOTH);
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
	//! Gets current gripper position from joint state server
	/*!
	 \return bool if successfully received gripper position
	 \param a an enumerated type
	 \param result a double
	 */
	bool getPosition(Gripper::WhichArm a, double &result);
};




#endif /* APC_BAXTER_GRIPPERS_H_ */
