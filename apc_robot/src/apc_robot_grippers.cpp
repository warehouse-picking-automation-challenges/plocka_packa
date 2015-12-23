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

#include "apc_robot/apc_robot_grippers.h"

/***********************************************************************************************************************
initializes action client and waits for server
***********************************************************************************************************************/
Gripper::Gripper()
{
	ROS_INFO("Gripper::Gripper - Initializing");

	std::string robot;
	if(!(nh.getParam("/apc_robot/robot", robot))) { ROS_ERROR("APC robot parameter not found."); }

	std::string param_left  = "/apc_robot/"+robot+"/gripper_left";
	std::string param_right = "/apc_robot/"+robot+"/gripper_right";

	if(!(nh.getParam(param_left,  gripper_left_topic_name)))  { ROS_ERROR("Gripper topic name parameter not found."); }
	if(!(nh.getParam(param_right, gripper_right_topic_name))) { ROS_ERROR("Gripper topic name parameter not found."); }

	ROS_INFO(" left client:  %s", gripper_left_topic_name.c_str());
	ROS_INFO(" right client: %s", gripper_right_topic_name.c_str());

	// Tell the action client that we want to spin a thread by default
	left_client  = new GripperClient(gripper_left_topic_name, true);
	right_client = new GripperClient(gripper_right_topic_name, true);

	// Wait for action server to come up
	if(!left_client->waitForServer(ros::Duration(1.0)))
		ROS_ERROR("Failed to connect to left gripper_action servers!");

	if(!right_client->waitForServer(ros::Duration(1.0)))
		ROS_ERROR("Failed to connect to right gripper_action servers!");

	leftMotionInProgress = false;
	rightMotionInProgress = false;

	// Create client for getting gripper positions
	ros::ServiceClient joint_state_client = nh.serviceClient<apc_msgs::ReturnJointStates>("return_joint_states");
	if( !joint_state_client.waitForExistence(ros::Duration(1.0) ) )
		ROS_ERROR("Joint state listener not running! Please run \n  rosrun apc_robot joint_states_listener.py");

	joint_states.request.name.resize(0);
	joint_states.request.name.push_back("l_gripper_joint");
	joint_states.request.name.push_back("r_gripper_joint");

	ROS_INFO("Gripper::Gripper - Done!");
}

/***********************************************************************************************************************
clean up the action client
***********************************************************************************************************************/
Gripper::~Gripper()
{
	  if(right_client)
	    delete right_client;
	  if(left_client)
	    delete left_client;
}

void Gripper::position(double angle, double effort, Gripper::WhichArm a)
{

	GripperClient* action_client;

	switch(a)
	{
	case Gripper::LEFT:
		action_client = left_client;
		break;
	case Gripper::RIGHT:
		action_client = right_client;
		break;
	case Gripper::BOTH:
		position(angle, effort, Gripper::LEFT);
		position(angle, effort, Gripper::RIGHT);
		break;
	default:
		ROS_ERROR("Received faulty gripper argument in Gripper::position() function!");
		exit(-1);
	}

	if(a != Gripper::BOTH)
	{
        double max_execution_time = 15.0;

        control_msgs::GripperCommandGoal goal;
        goal.command.position = angle;
        goal.command.max_effort = effort;

        if (nh.ok())
        {
                bool finished_within_time = false;

                ROS_INFO("Sending gripper goal ...");
                action_client->sendGoal(goal);

                ros::Duration(0.1).sleep(); // finished_within_time is always false if not waiting

                finished_within_time = action_client->waitForResult(ros::Duration(max_execution_time));
                if (!finished_within_time)
                {
                	action_client->cancelGoal();
                        ROS_INFO("Timed out achieving gripper goal!");
                }
                else
                {
                        actionlib::SimpleClientGoalState state = action_client->getState();
                        if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
                                ROS_INFO("Gripper action finished: %s",state.toString().c_str());
                        else
                                ROS_INFO("Gripper action failed (?): %s",state.toString().c_str());
                }
        }
	}
}

void Gripper::sendGoal(double angle, double effort, Gripper::WhichArm a)
{

	control_msgs::GripperCommandGoal goal;
    goal.command.position = angle;
    goal.command.max_effort = effort;

  if (nh.ok())
  {
    switch(a)
	{
	case Gripper::LEFT:
		left_client->sendGoal(goal);
		ROS_INFO("Sending LEFT gripper goal ...");
		leftMotionInProgress = true;
		break;
	case Gripper::RIGHT:
		right_client->sendGoal(goal);
		ROS_INFO("Sending RIGHT gripper goal ...");
		rightMotionInProgress = true;
		break;
	case Gripper::BOTH:
		sendGoal(angle, effort, Gripper::LEFT);
		sendGoal(angle, effort, Gripper::RIGHT);
		break;
	default:
		ROS_ERROR("Received faulty gripper argument in Gripper::position() function!");
		exit(-1);
	}
  }
  else
  {
    ROS_INFO("ERROR: Shutdown already in progress. Cannot send gripper goal.");
  }
}

// TODO check if Position to 0:100 == close:open
/*
        # Apply max effort if specified < 0
        if effort == -1.0:
            effort = 100.0
 */
void Gripper::open(Gripper::WhichArm a)
{
	sendGoal(100,-1.0, a);
}
void Gripper::close(Gripper::WhichArm a)
{
	sendGoal(0.0,-1.0, a);
}

void Gripper::closeGently(Gripper::WhichArm a)
{
	sendGoal(0.0, 20.0, a);
}

/***********************************************************************************************************************
check if goal has been completed

Various states the goal can be in:
StateEnum { PENDING, ACTIVE, RECALLED, REJECTED,
            PREEMPTED, ABORTED, SUCCEEDED, LOST}
***********************************************************************************************************************/
bool Gripper::motionComplete()
{

	return true;// FIXME Comment out later when gripper_action servers are connected properly
				// [ERROR] [1432419266.555697458]: Failed to connect to gripper_action servers!

	  if(leftMotionInProgress)
	  {
	    actionlib::SimpleClientGoalState state = left_client->getState();
	    if(state.isDone())																// For some reason the state is set to aborted within 1-2 sec and the state is set to done despite that the gripper is still moving
	    {
	      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	      {
	        ROS_INFO("Left gripper action finished: %s",state.toString().c_str());
	      }
	      else
	      {
	        ROS_INFO("Left gripper action failed: %s",state.toString().c_str());      // ABORTED seems to be OK
	      }
	      leftMotionInProgress = false;
	    }
	  }

	  if(rightMotionInProgress)
	  {
	    actionlib::SimpleClientGoalState state = right_client->getState();
	    if(state.isDone())
	    {
	      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
	      {
	        ROS_INFO("Right gripper action finished: %s",state.toString().c_str());
	      }
	      else
	      {
	        ROS_INFO("Right gripper action failed: %s",state.toString().c_str());
	      }
	      rightMotionInProgress = false;
	    }
	  }


 if(leftMotionInProgress || rightMotionInProgress)
   return false;

 return true;

}

void Gripper::cancelMotion()
{

	  if(leftMotionInProgress)
	  {
	    actionlib::SimpleClientGoalState state = left_client->getState();
	    ROS_INFO("Canceling left motion (Current state: %s)",state.toString().c_str());
	    left_client->cancelGoal();
	    leftMotionInProgress = false;
	  }
	  if(rightMotionInProgress)
	  {
	    actionlib::SimpleClientGoalState state = right_client->getState();
	    ROS_INFO("Canceling right motion (Current state: %s)",state.toString().c_str());
	    right_client->cancelGoal();
	    rightMotionInProgress = false;
	  }

}


bool Gripper::getPosition(Gripper::WhichArm a, double &result)
{
//	ros::service::waitForService("return_joint_states");

	if(joint_state_client.call(joint_states))		// FIXME this always fails
	{
		switch(a)
		{
		case Gripper::LEFT:
			result = joint_states.response.position[0];
			break;
		case Gripper::RIGHT:
			result = joint_states.response.position[1];
			break;
		default:
			ROS_ERROR("Please specify which gripper position to get!");
			return false;
		}
		return true;
	}
	ROS_ERROR("Failed to call joint state server!");
	return false;
}

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of gripper class
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// init the ROS node
	ros::init(argc, argv, "gripper_test");

	double r_result=-1;
	double l_result=-1;

	// create objects
	Gripper gripper;

	// send gripper commands
	while(true)											// FIXME remove after testing
	{
		gripper.getPosition(Gripper::RIGHT,r_result);
		sleep(1);
		gripper.getPosition(Gripper::LEFT,l_result);
		std::cout<<"{Left,Right} gripper pose: {"<<l_result<<","<<r_result<<"}\n";
		sleep(1);
	}

	/* ----------------------------------------- */
	ROS_INFO("=== Cmd 1 ===");
	gripper.position(0.0,50, Gripper::BOTH);

	while(!gripper.motionComplete())
	{
		std::cout<<".";
		ros::Duration(1.0).sleep();
	}

	gripper.getPosition(Gripper::RIGHT,r_result);
	gripper.getPosition(Gripper::LEFT,l_result);
	std::cout<<"\n{Left,Right} gripper pose: {"<<l_result<<","<<r_result<<"} compared to 0.0\n";

	/* ----------------------------------------- */
	ROS_INFO("=== Cmd 2 ===");
	gripper.position(100,50, Gripper::BOTH);

	while(!gripper.motionComplete())
	{
		std::cout<<".";
		ros::Duration(1.0).sleep();
	}

	gripper.getPosition(Gripper::RIGHT,r_result);
	gripper.getPosition(Gripper::LEFT,l_result);
	std::cout<<"{Left,Right} gripper pose: {"<<l_result<<","<<r_result<<"} compared to 100\n";

	/* ----------------------------------------- */
	ROS_INFO("=== Cmd 3 ===");
	gripper.sendGoal(0.0,10, Gripper::BOTH);

	sleep(3);

	gripper.getPosition(Gripper::RIGHT,r_result);
	gripper.getPosition(Gripper::LEFT,l_result);
	std::cout<<"{Left,Right} gripper pose: {"<<l_result<<","<<r_result<<"} compared to 0.0\n";

	/* ----------------------------------------- */
	ROS_INFO("=== Cmd 4 ===");
	gripper.sendGoal(100,10, Gripper::BOTH);

	sleep(3);

	gripper.getPosition(Gripper::RIGHT,r_result);
	gripper.getPosition(Gripper::LEFT,l_result);
	std::cout<<"{Left,Right} gripper pose: {"<<l_result<<","<<r_result<<"} compared to 100\n";

//	gripper.open();
//
//	while(!gripper.motionComplete())
//	{
//		ROS_INFO("Waiting for motion to complete ...");
//		ros::Duration(0.5).sleep();
//	}
//
//
//	gripper.close(PosGripper::LEFT);
//
//	while(!gripper.motionComplete())
//	{
//		ROS_INFO("Waiting for motion to complete ...");
//		ros::Duration(0.5).sleep();
//	}
//
//
//	gripper.close(PosGripper::RIGHT);
//
//	while(!gripper.motionComplete())
//	{
//		ROS_INFO("Wainting for motion to complete ...");
//		ros::Duration(0.5).sleep();
//	}


	ros::shutdown();

	return 0;
}

