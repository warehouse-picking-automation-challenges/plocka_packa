/*!
 * 	\name 	   Head
 *  \brief     Simple action client for moving the PR2's head
 *  \details   Simple action client for moving the PR2's head
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      May 17, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */
/*********************************************************************
FILENAME:   pr2_head.cpp
AUTHORS:    Sven Cremer

DESCRIPTION:
Simple action client for moving the PR2's head

PUBLISHES:  /head_traj_controller/point_head_action
			/head_traj_controller/joint_trajectory_action
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2015.05.17  SC     original file creation
*********************************************************************/
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

#ifndef HEAD_H_
#define HEAD_H_

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <control_msgs/PointHeadAction.h>
#include <control_msgs/JointTrajectoryAction.h>
typedef actionlib::SimpleActionClient<control_msgs::JointTrajectoryAction> TrajClient;
typedef actionlib::SimpleActionClient<control_msgs::PointHeadAction> HeadClient;

//#include <arm_navigation_msgs/FilterJointTrajectory.h>
/*! \class Head pr2_head.h "include/pr2_head.h"
 *  \brief Simple action client for moving the PR2's head
 */
class Head
{
  private:
		/*!
      	  ROS Handle
		 */
        ros::NodeHandle nh;
    	/*!
          PR2 HeadClient
    	 */
        HeadClient* point_client;
    	/*!
          PR2 TrajClient
    	 */
        TrajClient* action_client;
    	/*!
          Head Message
    	 */
        control_msgs::PointHeadGoal goal;
    	/*!
          Point Message
    	 */
        geometry_msgs::PointStamped point;
    	/*!
          Boolean representing motions in progress
    	 */
        bool motionInProgress;
    	/*!
          Point Message
    	 */
        bool point_motionInProgress;
  public:
    	/*!
    	 Constructor
    	 */
        Head();
    	/*!
    	 Deconstructer
    	 */
        ~Head();
        //! Shake the head from left to right n times
    	/*!
    	  \param n a integer
    	  \sa sendGoal(), sendGoalCart()
    	 */
        void shakeHead(int n);
        //! Points the high-def camera frame at a point in a given frame
    	/*!
    	  \param frame_id a std::string
	      \param x a double
	      \param y a double
	      \param z a double
	      \param w a double
	      \param duration a double
	      \sa sendGoal(), shakeHead()
    	 */
        void sendGoalCart(std::string frame_id, double x, double y, double z, double duration);
    	//! Sends position goal to action client (non-blocking)
        /*!
    	 \param pan a double
    	 \param tilt a double
    	 \param pan_dot a double
    	 \param tilt_dot a double
    	 \sa shakeHead(), sendGoalCart()
    	 */
        void sendGoal(double pan, double tilt, double pan_dot, double tilt_dot);
    	//! Sends position goal to action client (non-blocking)
        /*!
    	 \param pan a double
    	 \param tilt a double
    	 \param pan_dot a double
    	 \param tilt_dot a double
    	 \sa shakeHead(), sendGoalCart()
    	 */
        void sendGoal(control_msgs::JointTrajectoryGoal &goal);
        //Apply trajectory filters for smoother motions
        //bool applyTrajectoryFilter(control_msgs::JointTrajectoryGoal &goal);

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

#endif /* HEAD_H_ */
