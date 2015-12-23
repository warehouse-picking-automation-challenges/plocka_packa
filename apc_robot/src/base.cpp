/*!
 * 	\name 	   Base
 *  \brief     Controls PR2 base movement through the navigation stack
 *  \details   Controls PR2 base movement through the navigation stack
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      July 15, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include <apc_robot/base.h>

/***********************************************************************************************************************
Initializes class
***********************************************************************************************************************/
Base::Base()
{
	ROS_INFO("Base::Base - Initializing");
	//set up the publisher for the cmd_vel topic
	//cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/base_controller/command", 1);

    action_client = new MoveBaseClient("/move_base", true);

    // Wait for action server to come up
    if( !action_client->waitForServer(ros::Duration(1.0)) )
    	ROS_ERROR(" Failed connecting to move_base server!");
    else
    	ROS_INFO(" Connected to move_base server!");

    ROS_INFO("Base::Base - Done!");
}
/***********************************************************************************************************************
Destructor
***********************************************************************************************************************/
Base::~Base()
{
	delete action_client;
}
/***********************************************************************************************************************
Send twist command for turning
***********************************************************************************************************************/
//void Base::sendTurnCmd(double radiansPerSecond, double duration)
//{
//	// Reset values
//	base_cmd.linear.x = base_cmd.linear.y = 0;
//	base_cmd.angular.z = radiansPerSecond;
//
//	ros::Time start_time = ros::Time::now();
//
//	while(ros::Time::now() < start_time + ros::Duration(duration) )
//	{
//		cmd_vel_pub.publish(base_cmd);
//		ros::Duration(0.01).sleep();				// Todo: check if update rate is OK
//	}
//
//	cmd_vel_pub.publish(geometry_msgs::Twist());	// Reset topic values
//}
/***********************************************************************************************************************
Send base command for turning
***********************************************************************************************************************/
void Base::sendBaseGoal(geometry_msgs::Pose target, std::string frame)
{

	base_goal.target_pose.header.frame_id 	= frame;				// Reference frame
	base_goal.target_pose.header.stamp 		= ros::Time::now();

	base_goal.target_pose.pose = target;

	  if (nh.ok())
	  {
	    //cancelMotion();
	    ROS_INFO("Sending move base goal ...");
	    action_client->sendGoal(base_goal);
	    motionInProgress = true;
	  }
	  else
	  {
	    ROS_INFO("ERROR: Shutdown already in progress. Cannot send gripper goal.");
	  }

}
void Base::sendBaseGoal_MapFrame(geometry_msgs::Pose target)
{
	sendBaseGoal(target, "map");			// Use map as reference
}

void Base::sendBaseGoal(double x, double y, double w, std::string frame)
{

	base_goal.target_pose.header.frame_id 	= frame;				// Reference frame
	base_goal.target_pose.header.stamp 		= ros::Time::now();

	base_goal.target_pose.pose.position.x	= x;
	base_goal.target_pose.pose.position.y	= y;
	base_goal.target_pose.pose.position.z	= 0.0;

	tf::Quaternion quat = tf::createQuaternionFromYaw(w);	// Convert from euler angle to quaternion representation
	tf::quaternionTFToMsg(quat, base_goal.target_pose.pose.orientation);

	  if (nh.ok())
	  {
	    //cancelMotion();
	    ROS_INFO("Sending move base goal ...");
	    action_client->sendGoal(base_goal);
	    motionInProgress = true;
	  }
	  else
	  {
	    ROS_INFO("ERROR: Shutdown already in progress. Cannot send gripper goal.");
	  }

}
void Base::sendBaseGoal_RobotFrame(double x, double y, double w)
{
	sendBaseGoal(x, y, w, "base_link");		// Use robot base as reference
}
void Base::sendBaseGoal_MapFrame(double x, double y, double w)
{
	sendBaseGoal(x, y, w, "map");			// Use map as reference
}
void Base::sendBaseGoal_OdomFrame(double x, double y, double w)
{
	sendBaseGoal(x, y, w, "odom_combined");	// Use odometry as reference
}
/***********************************************************************************************************************
Send base command for turning
***********************************************************************************************************************/
void Base::sendTurnGoal(double w)
{
	sendBaseGoal_RobotFrame(0.0, 0.0, w);
}
/***********************************************************************************************************************
Check if goal has been reached
Various states the goal can be in:
StateEnum { PENDING, ACTIVE, RECALLED, REJECTED,PREEMPTED, ABORTED, SUCCEEDED, LOST}
***********************************************************************************************************************/
bool Base::motionComplete()
{

 if(motionInProgress)
 {
   actionlib::SimpleClientGoalState state = action_client->getState();
   if(state.isDone())
   {
     if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
     {
       ROS_INFO("Base action finished: %s",state.toString().c_str());
     }
     else
     {
       ROS_INFO("Base action failed (?): %s",state.toString().c_str());
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
void Base::cancelMotion()
{
  if(motionInProgress)
  {
    actionlib::SimpleClientGoalState state = action_client->getState();
    ROS_INFO("Canceling Base motion (Current state: %s)",state.toString().c_str());
    action_client->cancelGoal();
    motionInProgress = false;
  }
}
/***********************************************************************************************************************
Program entry point, demonstrates use of Base class
***********************************************************************************************************************/
int main(int argc, char **argv)
{
        // Initialize the ROS node
        ros::init(argc, argv, "base_test");

        // Create object
        Base base;

//        ROS_INFO("### Testing base cmd ###");
//        base.sendTurnCmd(0.2,5);
//        while(!base.motionComplete())
//        {
//        	ROS_INFO("...");
//        	ros::Duration(5.0).sleep(); // sleep for x seconds
//        }

        ROS_INFO("### Testing base goal ###");
        base.sendTurnGoal(0.0);

        while(!base.motionComplete())
        {
        	ROS_INFO("...");
        	ros::Duration(5.0).sleep(); // sleep for x seconds
        }

//        base.sendTurnGoal(1.57);
//
//        while(!base.motionComplete())
//        {
//        	ROS_INFO("...");
//        	ros::Duration(5.0).sleep(); // sleep for x seconds
//        }

        base.sendTurnGoal(3.14);

        while(!base.motionComplete())
        {
        	ROS_INFO("...");
        	ros::Duration(5.0).sleep(); // sleep for x seconds
        }

        ros::shutdown();
        return 0;
}
