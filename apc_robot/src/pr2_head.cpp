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

#include <apc_robot/pr2_head.h>
/***********************************************************************************************************************
Constructor initializes action client and waits for server
***********************************************************************************************************************/
Head::Head()
{
	ROS_INFO("Head::Head - Initializing");
	point_client = new HeadClient("/head_traj_controller/point_head_action", true);
	action_client = new TrajClient("/head_traj_controller/joint_trajectory_action", true);

	// Wait for action server to come up
	while(!point_client->waitForServer(ros::Duration(1.0)) && !action_client->waitForServer(ros::Duration(1.0)))
	{
		ROS_INFO(" Waiting for the head_traj_controller servers...");
	}
	ROS_INFO(" Connected to head_traj_controller server!");


//	// Wait 3 seconds for trajectory filter
//	if(ros::service::waitForService("trajectory_filter/filter_trajectory",ros::Duration(3.0)))
//		ROS_INFO("Connected to joint_trajectory_action server!");
//	else
//		ROS_WARN("Failed to connect to joint_trajectory_action server!");	// Todo: set flag to prevent execution of certain functions


	motionInProgress = false;
	point_motionInProgress = false;
	ROS_INFO("Head::Head - Done!");
}
/***********************************************************************************************************************
Destructor
***********************************************************************************************************************/
Head::~Head()
{
	delete action_client;
	delete point_client;
}
/***********************************************************************************************************************
Points the high-def camera frame at a point in a given frame
***********************************************************************************************************************/
void Head::sendGoalCart(std::string frame_id, double x, double y, double z, double duration)
{

	point.header.frame_id = frame_id;                             // frame in which x, y, z was specified
	point.point.x = x; point.point.y = y; point.point.z = z;
	goal.target = point;

	//we want the X axis of the camera frame to be pointing at the target
	goal.pointing_frame = "high_def_frame";
	goal.pointing_axis.x = 1;
	goal.pointing_axis.y = 0;
	goal.pointing_axis.z = 0;

	//take at least X seconds to get there
	goal.min_duration = ros::Duration(duration);

	//and go no faster than 1 rad/s
	goal.max_velocity = 1.0;                                      // What is recommended?

	if (nh.ok())
	{
		//cancelMotion();														// Todo: make a sendNewGoalCart
		ROS_INFO("Sending Point Head goal ...");
		point_client->sendGoal(goal);                                       // TODO: ADD TO motionComplte/ cancel Goals
		point_motionInProgress = true;
	}
	else
	{
		ROS_INFO("ERROR: Shutdown already in progress. Cannot send Head goal.");
	}

}
///***********************************************************************************************************************
//Trajectory filter for smooth motions. Computes joints velocities at each waypoint.
//***********************************************************************************************************************/
//bool Head::applyTrajectoryFilter(pr2_controllers_msgs::JointTrajectoryGoal &goal)
//{
//
//  ros::ServiceClient filter_trajectory_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter/filter_trajectory");
//
//  arm_navigation_msgs::FilterJointTrajectory::Request req;
//  arm_navigation_msgs::FilterJointTrajectory::Response res;
//
//  req.trajectory = goal.trajectory;                             // The trajectory to filter. Copies joint_names, points, and time_from_start.
//  req.allowed_time = ros::Duration(2.0);                        // Max computation time for filtering (usually takes less than 0.2 seconds).
//  //req.start_state =                                           // Starting state updates. If certain joints should be considered at positions other than the current ones, these positions should be set here
//  //req.limits =                                                // Custom position, velocity, and acceleration limits for specific joints
//
//  if(filter_trajectory_client_.call(req,res))
//  {
//    if(res.error_code.val == res.error_code.SUCCESS)
//    {
//      ROS_INFO("Requested HEAD trajectory was filtered.");
//    }
//    else
//    {
//      ROS_ERROR("Requested HEAD trajectory was not filtered. Error code: %d",res.error_code.val);
//      return false;
//    }
//  }
//  else
//  {
//    ROS_ERROR("Service call to filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
//    return false;
//  }
//
//
//  goal.trajectory = res.trajectory;                                     // Filtering result
//  for(unsigned int i=0; i < goal.trajectory.points.size(); i++)         // Start time of first point is always zero. This is bad if the current state is not identical.
//  {
//    goal.trajectory.points[i].time_from_start += ros::Duration(2.5);    // Add 2.5 seconds to all points. Use joint_durations[i] in future?
//  }
//
//  // Print results:
////  for (int i = 0 ; i < nr_traj; ++i)
////  {
////    ROS_INFO("--> Traj %i :",i);
////    for (int  j = 0; j < 7; ++j)
////    {
////      ROS_INFO("%f ",req.trajectory.points[i].velocities[j] );
////    }
////    ROS_INFO("Time from start: %f ",req.trajectory.points[i].time_from_start.toSec());
////  }
//
//  return true;
//}
/***********************************************************************************************************************
Execute given trajectory
 ***********************************************************************************************************************/
/*
void Head::execute(JointSpaceTrajectory *pTraj)                //
{
	if(pTraj==NULL)		// Check for valid value
		return;

  pr2_controllers_msgs::JointTrajectoryGoal temp;

  if(pTraj->getJointHeadGoal(temp))
    execute(temp);

}
 */
/***********************************************************************************************************************
Sends position goal to action client (non-blocking)
***********************************************************************************************************************/
void Head::sendGoal(double pan, double tilt, double pan_dot, double tilt_dot)
{

	double time_from_start = 3.0;				// TODO estimate or use trajectory filter

	control_msgs::JointTrajectoryGoal goal;

	goal.trajectory.joint_names.push_back("head_pan_joint");
	goal.trajectory.joint_names.push_back("head_tilt_joint");

	goal.trajectory.points.resize(1);



	goal.trajectory.points[0].positions.resize(2);
	goal.trajectory.points[0].velocities.resize(2);

	goal.trajectory.points[0].positions[0] = pan;
	goal.trajectory.points[0].positions[1] = tilt;

	goal.trajectory.points[0].velocities[0] = pan_dot;
	goal.trajectory.points[0].velocities[1] = tilt_dot;

	//	for (int  j = 0; j < 2; ++j)                                                        // Two head joints
	//	{
	//		goal.trajectory.points[0].velocities[j] = 0.0;                                    // Zero velocity by default at joint position
	//	}
	goal.trajectory.points[0].time_from_start = ros::Duration(time_from_start);         // To be reached X seconds after start

	goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);             // When to start the trajectory: X sec from now


	sendGoal(goal);

}
/***********************************************************************************************************************
Sends position goal to action client (non-blocking)
***********************************************************************************************************************/
void Head::sendGoal(control_msgs::JointTrajectoryGoal &goal)
{

	//  pr2_controllers_msgs::JointTrajectoryGoal goal;
	//
	//  std::vector< std::vector<double> > tvect = t.GetHeadTrajectoryVector();
	//  std::vector<double> joint_durations = t.GetHeadDurations();
	//  int numPoints   = tvect.size();       // Number of waypoints


	unsigned int numPoints = goal.trajectory.points.size();           // Number of waypoints


	if(numPoints < 1)
	{
		ROS_WARN("Head::execute - received vector is empty");
		return;
	}
	//
	//  goal.trajectory.joint_names.push_back("head_pan_joint");
	//  goal.trajectory.joint_names.push_back("head_tilt_joint");
	//
	//  goal.trajectory.points.resize(numPoints);
	//
	//  double time_from_start = 0;
	//
	//  for (int i = 0 ; i < numPoints; ++i)
	//  {
	//    //ROS_INFO("--> Traj %i :", i);
	//    goal.trajectory.points[i].positions.resize(2);
	//    goal.trajectory.points[i].velocities.resize(2);
	//    for (int  j = 0; j < 2; ++j)                                                        // Two head joints
	//    {
	//      //ROS_INFO("%f", tvect[i][j]);
	//      goal.trajectory.points[i].positions[j] = tvect[i][j];
	//      goal.trajectory.points[i].velocities[j] = 0.0;                                    // Zero velocity by default at joint position
	//    }
	//    time_from_start += joint_durations[i];
	//    goal.trajectory.points[i].time_from_start = ros::Duration(time_from_start);         // To be reached X seconds after start
	//
	//  }
	//  goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);             // When to start the trajectory: X sec from now


	//  if(numPoints > 1)                                                                     // Perform filtering if goal contains several waypoints
	//   {
	//     applyTrajectoryFilter(goal);
	//   }

	//   ROS_INFO("Sending head  goal with %i point(s) and %.1f sec duration.",
	//            numPoints, goal.trajectory.points[numPoints-1].time_from_start.toSec()/*+joint_durations[numPoints-1]*/);

	if (nh.ok())
	{
		//cancelMotion();
		action_client->sendGoal(goal);
		motionInProgress = true;
	}
	else
	{
		ROS_INFO("ERROR: Shutdown already in progress. Cannot send Head goal.");
	}
}
/***********************************************************************************************************************
Check if goal has been reached
Various states the goal can be in:
StateEnum { PENDING, ACTIVE, RECALLED, REJECTED,PREEMPTED, ABORTED, SUCCEEDED, LOST}
***********************************************************************************************************************/
bool Head::motionComplete()
{

	if(motionInProgress)
	{
		actionlib::SimpleClientGoalState state = action_client->getState();
		if(state.isDone())
		{
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Head action finished: %s",state.toString().c_str());
			}
			else
			{
				ROS_INFO("Head action failed (?): %s",state.toString().c_str());
			}
			motionInProgress = false;
		}
	}

	if(point_motionInProgress)
	{
		actionlib::SimpleClientGoalState state = point_client->getState();
		if(state.isDone())
		{
			if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_INFO("Head action finished: %s",state.toString().c_str());
			}
			else
			{
				ROS_INFO("Head action failed (?): %s",state.toString().c_str());
			}
			point_motionInProgress = false;
		}
	}


	if(motionInProgress || point_motionInProgress)
		return false;

	return true;
}
/***********************************************************************************************************************
Cancel current goal
***********************************************************************************************************************/
void Head::cancelMotion()
{
	if(motionInProgress)
	{
		actionlib::SimpleClientGoalState state = action_client->getState();
		ROS_INFO("Canceling Head motion (Current state: %s)",state.toString().c_str());
		action_client->cancelGoal();
		motionInProgress = false;
	}

	if(point_motionInProgress)
	{
		actionlib::SimpleClientGoalState state = point_client->getState();
		ROS_INFO("Canceling Head motion (Current state: %s)",state.toString().c_str());
		point_client->cancelGoal();
		point_motionInProgress = false;
	}

}
/***********************************************************************************************************************
Shake the head from left to right n times
***********************************************************************************************************************/
void Head::shakeHead(int n)//, RobotHead head)
{
	int count = 0;
	while (ros::ok() && ++count <= n )
	{
		//Looks at a point forward (x=5m), slightly left (y=1m), and 1.2m up
		sendGoalCart("base_link", 4.0, 3, 1.4, 1.3);
		while(!motionComplete() && ros::ok())                  // wait until done
		{
			ros::Duration(0.2).sleep();
		}
		sendGoalCart("base_link", 4.0, -3, 1.4, 1.3);
		while(!motionComplete() && ros::ok())
		{
			ros::Duration(0.2).sleep();
		}
	}
}
/***********************************************************************************************************************
Program entry point, demonstrates use of Head class
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// init the ROS nodeint main(int argc, char **argv)
	ros::init(argc, argv, "Head_test");

	// create object
	Head head;

	head.shakeHead(2);

	// send Head commands
	ROS_INFO("### Testing Head pointing ###");
	head.sendGoalCart("odom_combined", 2, 0, 1.5, 5.0);
	sleep(5.0);
	head.sendGoalCart("odom_combined", 2, 0, 1.0, 5.0);
	sleep(5.0);

	ROS_INFO("### Testing Head trajectory ###");
	head.sendGoal(0.1, -0.2, 0.0, 0.0);

	while(!head.motionComplete())
	{
		ROS_INFO("...");
		ros::Duration(1.0).sleep(); // sleep for x seconds
	}

	head.sendGoal(0.1, 0.6, 0.0, 0.0);

	while(!head.motionComplete())
	{
		ROS_INFO("...");
		ros::Duration(1.0).sleep(); // sleep for x seconds
	}

	//        Trajectory t("test");
	//
	//        double pos1[] = {0.3, -0.3};
	//        double pos2[] = {0.3, -0.1};
	//        double pos3[] = {0.2, -0.3};
	//        double pos4[] = {0.0, -0.0};
	//
	//        t.AddHeadPosition(JointHeadPosition(pos1));
	//        t.AddHeadPosition(JointHeadPosition(pos2));
	//        t.AddHeadPosition(JointHeadPosition(pos3));
	//        t.AddHeadPosition(JointHeadPosition(pos4));
	//
	//        head.execute(t);

	ros::shutdown();

	return 0;
}


