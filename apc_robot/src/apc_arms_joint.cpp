/*!
 * 	\name 	   ArmsJoint
 *  \brief     Controls arm movement through joint control
 *  \details   Using a PR2 action client, joint arm positions are sent through public functions
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Sept 16, 2013
 *  \pre       First initialize the system.
 *  \warning   Improper use can crash your application
 *  \copyright GNU Public License.
 */
/***********************************************************************************************************************
FILENAME:   arm_joint_traj.h
AUTHORS:    Sven Cremer

DESCRIPTION:
Simple action client for controlling the PR2's grippers. Adapted from: http://wiki.ros.org/pr2_controllers/Tutorials

PUBLISHES:  r(l)_arm_controller/joint_trajectory_action
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2013.09.16  SC     original file creation
2013.09.22  SC     formating
***********************************************************************************************************************/


#include <apc_robot/apc_arms_joint.h>

/***********************************************************************************************************************
Initializes action client and waits for server
***********************************************************************************************************************/
ArmsJoint::ArmsJoint()
{
  ROS_INFO("ArmsJoint::ArmsJoint - Initializing");

  // Tell the action client that we want to spin a thread by default
  left_client  = new TrajClient("l_arm_controller/joint_trajectory_action",true);
  right_client = new TrajClient("r_arm_controller/joint_trajectory_action",true);

  // Wait for action server to come up
  if(!left_client->waitForServer(ros::Duration(1.0)))
	  ROS_ERROR(" Failed to connect to left joint_trajectory_action servers!");

  if(!right_client->waitForServer(ros::Duration(1.0)))
	  ROS_ERROR(" Failed to connect to right joint_trajectory_action servers!");

//  // Wait 30 seconds for trajectory filter
//  if(ros::service::waitForService("trajectory_filter/filter_trajectory",ros::Duration(30.0)))
//    ROS_INFO("Connected to joint_trajectory_action server!");
//  else
//    ROS_WARN("Failed to connect to joint_trajectory_action server!");

  leftMotionInProgress = false;
  rightMotionInProgress = false;

  ROS_INFO("ArmsJoint::ArmsJoint - Done!");
}

/***********************************************************************************************************************
clean up the action client
***********************************************************************************************************************/
ArmsJoint::~ArmsJoint()
{
  if(right_client)
    delete right_client;
  if(left_client)
    delete left_client;
}

/***********************************************************************************************************************
Figure out where the arm is now
Get current right arm angles from the state being broadcast by the r(l)_arm_controller, by listening for a single message.
***********************************************************************************************************************/
void ArmsJoint::get_right_joint_angles(double current_angles[7])
{
  //get a single message from the topic 'r_arm_controller/state'
	control_msgs::JointTrajectoryControllerStateConstPtr state_msg =ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("r_arm_controller/state");


  //extract the joint angles from it
  for(int i=0; i<7; i++)
    current_angles[i] = state_msg->actual.positions[i];
}


/***********************************************************************************************************************
Figure out where the arm is now
Get current lower arm angles from the state being broadcast by the l(l)_arm_controller, by listening for a single message.
***********************************************************************************************************************/
void ArmsJoint::get_left_joint_angles(double current_angles[7])
{
  //get a single message from the topic 'l_arm_controller/state'
	control_msgs::JointTrajectoryControllerStateConstPtr state_msg =
      ros::topic::waitForMessage<control_msgs::JointTrajectoryControllerState>("l_arm_controller/state");

  //extract the joint angles from it
  for(int i=0; i<7; i++)
    current_angles[i] = state_msg->actual.positions[i];
}

/***********************************************************************************************************************
Trajectory filter for smooth motions. Computes joints velocities at each waypoint.
***********************************************************************************************************************/
/*
bool ArmsJoint::applyTrajectoryFilter(pr2_controllers_msgs::JointTrajectoryGoal &goal)
{

  ros::ServiceClient filter_trajectory_client_ = nh.serviceClient<arm_navigation_msgs::FilterJointTrajectory>("trajectory_filter/filter_trajectory");

  arm_navigation_msgs::FilterJointTrajectory::Request req;
  arm_navigation_msgs::FilterJointTrajectory::Response res;

  req.trajectory = goal.trajectory;                             // The trajectory to filter. Copies joint_names, points, and time_from_start.
  req.allowed_time = ros::Duration(2.0);                        // Max computation time for filtering (usually takes less than 0.2 seconds).
  //req.start_state =                                           // Starting state updates. If certain joints should be considered at positions other than the current ones, these positions should be set here
  //req.limits =                                                // Custom position, velocity, and acceleration limits for specific joints

  if(filter_trajectory_client_.call(req,res))
  {
    if(res.error_code.val == res.error_code.SUCCESS)
    {
      ROS_INFO("Requested ARM trajectory was filtered.");
      //        for(unsigned int i=0; i < res.trajectory.points.size(); i++)
      //        {
      //          ROS_INFO_STREAM(res.trajectory.points[i].velocities[0] << ", " << res.trajectory.points[i].velocities[1] << ", "
      //                          << res.trajectory.points[i].velocities[2] << "," << res.trajectory.points[i].velocities[3] << ", "
      //                          << res.trajectory.points[i].velocities[4] << "," << res.trajectory.points[i].velocities[5] << ", " << res.trajectory.points[i].velocities[5] << ", " <<endl
      //                          << res.trajectory.points[i].time_from_start.toSec());
      //        }
    }
    else
    {
      ROS_ERROR("Requested ARM trajectory was not filtered. Error code: %d",res.error_code.val);
      return false;
    }
  }
  else
  {
    ROS_ERROR("Service call to filter trajectory failed %s",filter_trajectory_client_.getService().c_str());
    return false;
  }


  goal.trajectory = res.trajectory;                                     // Filtering result
  for(unsigned int i=0; i < goal.trajectory.points.size(); i++)         // Start time of first point is always zero. This is bad if the current state is not identical.
  {
    goal.trajectory.points[i].time_from_start += ros::Duration(2.5);    // Add 2.5 seconds to all points. Use joint_durations[i] in future?
  }

  // Print results:
//  for (int i = 0 ; i < nr_traj; ++i)
//  {
//    ROS_INFO("--> Traj %i :",i);
//    for (int  j = 0; j < 7; ++j)
//    {
//      ROS_INFO("%f ",req.trajectory.points[i].velocities[j] );
//    }
//    ROS_INFO("Time from start: %f ",req.trajectory.points[i].time_from_start.toSec());
//  }

  return true;
}
*/
/***********************************************************************************************************************
void ArmTrajJoint::execute(std::vector< std::vector<double> > &joint_states)
Execute given trajectory.
***********************************************************************************************************************/
/*
void ArmsJoint::execute(Trajectory t)
{
  TrajClient* action_client;
  pr2_controllers_msgs::JointTrajectoryGoal goal;
  std::vector< std::vector<double> > tvect = t.GetTrajectoryVector();
  std::vector<double> joint_durations = t.GetJointDurations();

  //t.Print();

  unsigned int numPoints = tvect.size();           // Number of waypoints

  if(numPoints < 1)
  {
    ROS_INFO("ArmTrajJoint::execute - received vector is empty");
    return;
  }

  if(t.getArmName() == Trajectory::LEFT)
  {
    arm_prefix = "l";
    action_client = left_client;
  }
  else
  {
    arm_prefix = "r";
    action_client = right_client;
  }

  goal.trajectory.joint_names.push_back(arm_prefix+"_shoulder_pan_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_shoulder_lift_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_upper_arm_roll_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_elbow_flex_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_forearm_roll_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_wrist_flex_joint");
  goal.trajectory.joint_names.push_back(arm_prefix+"_wrist_roll_joint");

  goal.trajectory.points.resize(numPoints);

  double time_from_start = 0.0;

  for (unsigned int i = 0 ; i < numPoints; ++i)
  {
    //ROS_INFO("--> Traj %i :", i);
    goal.trajectory.points[i].positions.resize(7);
    goal.trajectory.points[i].velocities.resize(7);
    for (int  j = 0; j < 7; ++j)
    {
      //ROS_INFO("%f", tvect[i][j]);
      goal.trajectory.points[i].positions[j] = tvect[i][j];
      goal.trajectory.points[i].velocities[j] = 0.0;                                                  // Arm has no velocity at joint position
    }
    time_from_start += joint_durations[i];
    goal.trajectory.points[i].time_from_start = ros::Duration(time_from_start);			      // To be reached X seconds after start
  }
  goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(1.0);                           // When to start the trajectory: X sec from now


  if(numPoints > 1)                                                                                   // Perform filtering if goal contains several waypoints
  {
    applyTrajectoryFilter(goal);
  }

  ROS_INFO("Sending %s arm goal with %i point(s) and %.1f sec duration.",
           arm_prefix.c_str(), numPoints, goal.trajectory.points[numPoints-1].time_from_start.toSec()+joint_durations[numPoints-1]);

  if (nh.ok())
  {
    action_client->sendGoal(goal);

    if(arm_prefix.compare("l") == 0)
      leftMotionInProgress = true;
    else
      rightMotionInProgress = true;
  }
  else
  {
    ROS_ERROR("Shutdown already in progress. Cannot send arm goal.");
  }

}
*/
/***********************************************************************************************************************
void ArmTrajJoint::execute(std::vector< std::vector<double> > &joint_states)
Execute given trajectory
***********************************************************************************************************************/
/*
void ArmsJoint::executeAndWait(Trajectory t, double max_wait_time)
{
  TrajClient* action_client;
  execute(t);

  if((int)t.getArmName() == (int)Trajectory::LEFT)
    action_client = left_client;
  else
    action_client = right_client;

  bool finished_within_time = action_client->waitForResult(ros::Duration(max_wait_time));
   if (!finished_within_time)
   {
     action_client->cancelGoal();
     ROS_INFO("Timed out achieving joint goal.");
   }
   else
   {
     actionlib::SimpleClientGoalState state = action_client->getState();
     bool success = (state == actionlib::SimpleClientGoalState::SUCCEEDED);
     if(success)
       ROS_INFO("Action finished: %s",state.toString().c_str());
     else
       ROS_INFO("Action failed: %s",state.toString().c_str());
   }
}
*/

/***********************************************************************************************************************
bool ArmsJoint::sendGoal(std::vector<double> joints, ArmsJoint::WhichArm a)
Moves arm a to desired joint position
***********************************************************************************************************************/
bool ArmsJoint::sendGoal(std::vector<double> joints, ArmsJoint::WhichArm a)
{

	TrajClient* action_client;

	switch(a)
	{
	case ArmsJoint::LEFT:
	    arm_prefix = "l";
		action_client = left_client;
		break;
	case ArmsJoint::RIGHT:
	    arm_prefix = "r";
		action_client = right_client;
		break;
//	case Gripper::BOTH:
//		break;
	default:
		ROS_ERROR("Received faulty gripper argument in ArmsJoint::sendGoal() function!");
		return false;
	}

	//our goal variable
	control_msgs::JointTrajectoryGoal goal;

	// First, the joint names, which apply to all waypoints
	goal.trajectory.joint_names.push_back(arm_prefix+"_shoulder_pan_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_shoulder_lift_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_upper_arm_roll_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_elbow_flex_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_forearm_roll_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_wrist_flex_joint");
	goal.trajectory.joint_names.push_back(arm_prefix+"_wrist_roll_joint");

	// We will have two waypoints in this goal trajectory
	goal.trajectory.points.resize(1);

	// First trajectory point
	// Positions
	int ind = 0;
	goal.trajectory.points[ind].positions.resize(7);
	for(int i=0;i<7;i++)
	{
		goal.trajectory.points[ind].positions[i] = joints[i];		// TODO check size
	}
	// Velocities
	goal.trajectory.points[ind].velocities.resize(7);
	for (size_t j = 0; j < 7; ++j)
	{
		goal.trajectory.points[ind].velocities[j] = 0.0;
	}
	// To be reached 1 second after starting along the trajectory
	goal.trajectory.points[ind].time_from_start = ros::Duration(2.0);

	goal.trajectory.header.stamp = ros::Time::now();

	ROS_INFO("Sending arm joint goal ...");

	  if (nh.ok())
	  {
	    action_client->sendGoal(goal);

	    if(arm_prefix.compare("l") == 0)
	      leftMotionInProgress = true;
	    else
	      rightMotionInProgress = true;
	  }
	  else
	  {
	    ROS_ERROR("Shutdown already in progress. Cannot send arm goal.");
	  }

	return true;
}


/***********************************************************************************************************************
check if goal has been completed

Various states the goal can be in:
StateEnum { PENDING, ACTIVE, RECALLED, REJECTED,
            PREEMPTED, ABORTED, SUCCEEDED, LOST}
***********************************************************************************************************************/
bool ArmsJoint::motionComplete()
{

  if(leftMotionInProgress)
  {
    actionlib::SimpleClientGoalState state = left_client->getState();
    if(state.isDone())
    {
      if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Left action finished: %s",state.toString().c_str());
      }
      else
      {
        ROS_INFO("Left action failed: %s",state.toString().c_str());
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
        ROS_INFO("Right action finished: %s",state.toString().c_str());
      }
      else
      {
        ROS_INFO("Right action failed: %s",state.toString().c_str());
      }
      rightMotionInProgress = false;
    }
  }

  if(leftMotionInProgress || rightMotionInProgress)
    return false;

  return true;

}


void ArmsJoint::cancelMotion()
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

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of ArmTrajJoint class
***********************************************************************************************************************/
int main(int argc, char **argv)
{
  // Init the ROS node
  ros::init(argc, argv, "apc_arm_joint");

  ArmsJoint arms;
  double current_angles[7];

  arms.get_left_joint_angles(current_angles);

  for(int i=0;i<7;i++)
	  std::cout<<current_angles[i]<<" ";
  std::cout<<"\n";

  std::vector<double> joints;
  joints.push_back(0.341943);
  joints.push_back(0.512964);
  joints.push_back(0.840619);
  joints.push_back(-1.73563);
  joints.push_back(3.22009);
  joints.push_back(-1.08709);
  joints.push_back(2.35645);


  //arms.sendGoal(joints, ArmsJoint::LEFT);


  ros::shutdown();

  return 0;
}

