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

#include <apc_robot/pr2_torso.h>

/***********************************************************************************************************************
Constructor initializes action client and waits for server
***********************************************************************************************************************/
Torso::Torso()
{
	torso_client = new TorsoClient("torso_controller/position_joint_action", true);

	while(!torso_client->waitForServer(ros::Duration(3.0)))
	{
		ROS_INFO(" Waiting for the torso_client server...");
	}

	ROS_INFO(" Connected to torso_client server!");

	motionInProgress = false;
}
/***********************************************************************************************************************
Destructor
***********************************************************************************************************************/
Torso::~Torso()
{
	delete torso_client;
}
/***********************************************************************************************************************
Sends position goal to action client and waits for it to finish (blocking)
***********************************************************************************************************************/
void Torso::position(double height)
{
    double max_execution_time = 30.0;

    goal.position = height;
    goal.min_duration = ros::Duration(2.0);
    goal.max_velocity = 1.0;

        if (nh.ok())
        {
                bool finished_within_time = false;

                ROS_INFO("Sending torso goal");
                torso_client->sendGoal(goal);

                //ros::Duration(0.1).sleep(); // finished_within_time is always false if not waiting

                finished_within_time = torso_client->waitForResult(ros::Duration(max_execution_time));
                if (!finished_within_time)
                {
                        torso_client->cancelGoal();
                        ROS_INFO("Timed out achieving torso goal");
                }
                else
                {
                        actionlib::SimpleClientGoalState state = torso_client->getState();
                        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                                ROS_INFO("Torso action finished: %s",state.toString().c_str());
                        else
                                ROS_INFO("Torso action failed: %s",state.toString().c_str());
                }
        }
}
/***********************************************************************************************************************
Sends position goal to action client (non-blocking)
***********************************************************************************************************************/
void Torso::sendGoal(double height)
{

    goal.position = height;
    goal.min_duration = ros::Duration(2.0);
    goal.max_velocity = 1.0;
			
    if (nh.ok())
    {
      cancelMotion();
      ROS_INFO("Sending torso goal ...");
      torso_client->sendGoal(goal);
      motionInProgress = true;
    }
    else
    {
      ROS_INFO("ERROR: Shutdown already in progress. Cannot send torso goal.");
    }
}
/***********************************************************************************************************************
Check if goal has been reached
Various states the goal can be in:
StateEnum { PENDING, ACTIVE, RECALLED, REJECTED,PREEMPTED, ABORTED, SUCCEEDED, LOST}
***********************************************************************************************************************/
bool Torso::motionComplete()
{

 if(motionInProgress)
 {
   actionlib::SimpleClientGoalState state = torso_client->getState();
   if(state.isDone())
   {
     if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       ROS_INFO("Torso action finished: %s",state.toString().c_str());
     }
     else
     {
       ROS_INFO("Torso action failed: %s",state.toString().c_str());
     }
     motionInProgress = false;
   }
 }

 if(motionInProgress)
   return false;

 return true;
}
/***********************************************************************************************************************
Cancel current goal
***********************************************************************************************************************/
void Torso::cancelMotion()
{
  if(motionInProgress)
  {
    actionlib::SimpleClientGoalState state = torso_client->getState();
    ROS_INFO("Canceling torso motion (Current state: %s)",state.toString().c_str());
    torso_client->cancelGoal();
    motionInProgress = false;
  }
}

/***********************************************************************************************************************
Move the torso all the way up (blocking)
***********************************************************************************************************************/
void Torso::up()
{
	position(0.295);	// All the way up is 0.3
}
/***********************************************************************************************************************
Move the torso all the down (blocking)
***********************************************************************************************************************/
void Torso::down()
{
	position(0.005);	// All the way down is 0.0
}
/***********************************************************************************************************************
Program entry point, demonstrates use of torso class
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// Initialize the ROS node
	ros::init(argc, argv, "torso_test");

	// Create object
	Torso torso;
  
	// Send torso commands
	torso.up();
	torso.down();

	ROS_INFO("Running two position commands sequentially ...");
	torso.position(0.3);
	torso.position(0.2);

	ros::Duration(5.0).sleep();

	ROS_INFO("Sending two overlapping position commands ...");
	torso.sendGoal(0.1);
	ros::Duration(0.1).sleep();
	torso.sendGoal(0.3);

	ros::shutdown();

	return 0;
}

