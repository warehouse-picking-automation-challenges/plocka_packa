/*!
 * 	\name 	   Torso
 *  \brief     Simple action client for moving the PR2's torso. Adapted from:http://wiki.ros.org/pr2_controllers/Tutorials
 *  \details   Simple action client for moving the PR2's torso. Adapted from:http://wiki.ros.org/pr2_controllers/Tutorials
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      May 17, 2015
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

/*********************************************************************
FILENAME:   pr2_torso.cpp
AUTHORS:    Sven Cremer

DESCRIPTION:
Simple action client for moving the PR2's torso. Adapted from:
http://wiki.ros.org/pr2_controllers/Tutorials

PUBLISHES:  torso_controller/position_joint_action
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2015.05.17  SC     original file creation
*********************************************************************/

#ifndef TORSO_H_
#define TORSO_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/SingleJointPositionAction.h>
typedef actionlib::SimpleActionClient<control_msgs::SingleJointPositionAction> TorsoClient;

/*! \class Torso pr2_torso.h "include/pr2_torso.h"
 *  \brief Simple action client for moving the PR2's torso
 */
class Torso
{
  private: 
	/*!
  	  ROS Handle
	 */
	ros::NodeHandle nh;
	/*!
      PR2 TorsoClient
	 */
	TorsoClient* torso_client;  
	/*!
      Arm goal message
	 */
	control_msgs::SingleJointPositionGoal goal;
	/*!
      Boolean representing arm motions
	 */
	bool motionInProgress;
  public:
	//! Constructor initializes action client and waits for server
	Torso();
	//! Destructor
	~Torso();
	//! Sends position goal to action client and waits for it to finish (blocking)
	/*!
	 \param height a double
	 \sa sendGoal()
	 */
	void position(double height);
	//! Sends position goal to action client (non-blocking)
	/*!
	 \param height a double
	 \sa position()
	 */
	void sendGoal(double height);
	//! Move the torso all the way up (blocking)
	/*!
	 \sa down()
	 */
	void up();
	//! Move the torso all the down (blocking)
	/*!
	 \sa up()
	 */
	void down();
    //! Check if goal has been reached
    /*!
	 \return bool
	 \sa cancelMotion()
	 */
	bool motionComplete();
	//! Cancel motion
    /*!
	 \sa motionComplete()
	 */
	void cancelMotion();
};


#endif /* TORSO_H_ */
