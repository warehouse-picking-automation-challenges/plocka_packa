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

#include "apc_robot/apc_robot_moveit.h"

RobotMoveit::RobotMoveit()
{
	// Kinematics
	ROS_INFO("RobotMoveit - kinematic_state init");
	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	kinematic_model = robot_model::RobotModelPtr(robot_model_loader.getModel());

	kinematic_state = robot_state::RobotStatePtr(new robot_state::RobotState(kinematic_model));
	kinematic_state->setToDefaultValues();

	// Indigo
	//joint_names_right_arm = kinematic_state->getJointModelGroup("right_arm")->getActiveJointModelNames();
	//joint_names_left_arm  = kinematic_state->getJointModelGroup("left_arm")->getActiveJointModelNames();
	
	// Groovy
	joint_state_group_right_arm = kinematic_state->getJointStateGroup("right_arm");
	joint_state_group_left_arm = kinematic_state->getJointStateGroup("left_arm");
	joint_names_right_arm = joint_state_group_right_arm->getJointModelGroup()->getJointModelNames();
	joint_names_left_arm = joint_state_group_left_arm->getJointModelGroup()->getJointModelNames();

	// Move Group Interface
	ROS_INFO("RobotMoveit - planning_interface init");
	group_left_arm  = new moveit::planning_interface::MoveGroup("left_arm");		// TODO Gets stuck here without move_group.launch
	group_right_arm = new moveit::planning_interface::MoveGroup("right_arm");

	// Set planners
	group_left_arm->setPlannerId("RRTConnectkConfigDefault");
	group_left_arm->setPlanningTime(15);

	group_right_arm->setPlannerId("RRTConnectkConfigDefault");
	group_right_arm->setPlanningTime(15);
//
	group_left_arm->setGoalOrientationTolerance(0.1);
//
	group_right_arm->setGoalOrientationTolerance(0.1);
	group_right_arm->setGoalJointTolerance(0.1);
	group_right_arm->setGoalPositionTolerance(0.1);
	group_right_arm->setGoalTolerance(0.1);

	joint_pos_right_arm.resize(joint_names_right_arm.size());
	joint_pos_left_arm.resize(joint_names_left_arm.size());

	leftMotionInProgress = false;
	rightMotionInProgress = false;

	// Check if /joint_states is remapped to /robot/joint_states
	ROS_INFO("RobotMoveit - checking if /joint_states is publishing");
	sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/joint_states",ros::Duration(1,0));
	if (msg == NULL)
	{
		ROS_ERROR("Remap /joint_states to /robot/joint_states for getCurrentJointValues() to work!");
		ROS_ERROR("Do this inside a launch file or pass '/joint_states:=/robot/joint_states' as a function argument.");
	}

	ROS_INFO("*** Initialized RobotMoveit class ***");
}

RobotMoveit::~RobotMoveit()
{
	if(group_left_arm)
		delete group_left_arm;
	if(group_right_arm)
		delete group_right_arm;
}

bool RobotMoveit::updateJointStates()	// This function is meant to be called occasionally (it is not optimized)
{

	joint_pos_right_arm = group_right_arm->getCurrentJointValues();
	joint_pos_left_arm  = group_left_arm->getCurrentJointValues();

	/*
	sensor_msgs::JointStateConstPtr msg = ros::topic::waitForMessage<sensor_msgs::JointState>("/robot/joint_states",ros::Duration(1,0));

	if (msg == NULL)
	{
		ROS_ERROR("No robot joint state message received!");
		return false;
	}

	// Update right arm
	for(unsigned int i = 0; i < joint_names_right_arm.size(); i++)
	{
		for (int k = 0; k < msg->name.size(); k++)
		{
			if (joint_names_right_arm[i] == msg->name[k])
			{
				joint_pos_right_arm[i] = msg->position[k]; 		// Update joint position
				k = msg->name.size();
			}
		}
	}

	// Update left arm
	for(unsigned int i = 0; i < joint_names_left_arm.size(); i++)
	{
		for (int k = 0; k < msg->name.size(); k++)
		{
			if (joint_names_left_arm[i] == msg->name[k])
			{
				joint_pos_left_arm[i] = msg->position[k]; 		// Update joint position
				k = msg->name.size();
			}
		}
	}
	*/
	return true;
}

void RobotMoveit::printJointStates()
{

	updateJointStates();

	for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	{
		ROS_INFO(" Joint %s: %f", joint_names_right_arm[i].c_str(), joint_pos_right_arm[i]);
	}

	for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	{
		ROS_INFO(" Joint %s: %f", joint_names_left_arm[i].c_str(), joint_pos_left_arm[i]);
	}

}

void RobotMoveit::printJointStatesSRDF()
{

	updateJointStates();

	// Print right arm postion
	std::cout << "    <group_state name=\"right_\" group=\"right_arm\">\n";
	for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	{
		std::cout << "        <joint name=\"" << joint_names_right_arm[i].c_str() <<"\" value=\"" << joint_pos_right_arm[i] << "\" />\n";
	}
	std::cout << "    </group_state>\n\n";

	// Print left arm postion
	std::cout << "    <group_state name=\"left_\" group=\"left_arm\">\n";
	for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	{
		std::cout << "        <joint name=\"" << joint_names_left_arm[i].c_str() <<"\" value=\"" << joint_pos_left_arm[i] << "\" />\n";
	}
	std::cout << "    </group_state>\n\n";

}


bool RobotMoveit::getEndeffectorPose(RobotMoveit::WhichArm arm, geometry_msgs::Pose* current_pose)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	*current_pose = group->getCurrentPose().pose;


	return true;
}


bool RobotMoveit::selectArmGroup(RobotMoveit::WhichArm arm, moveit::planning_interface::MoveGroup** group)
{
	switch(arm)
	{
	case RobotMoveit::LEFT:
		*group = group_left_arm;
		break;
	case RobotMoveit::RIGHT:
		*group = group_right_arm;
		break;
	case RobotMoveit::BOTH:
		//group = group_both_arms;
		//break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	return true;

}

bool RobotMoveit::moveToPose(RobotMoveit::WhichArm arm, std::string pose_name, bool waitForMotion)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setNamedTarget(pose_name);
	group->setPlanningTime(10);

	ROS_INFO_STREAM("moveToPose: Moving arm to pose " << pose_name );

	bool success;

	if(waitForMotion)
	{
		success = group->move();
	}
	else
	{
		success = group->asyncMove();
		if(arm == RobotMoveit::LEFT)
			leftMotionInProgress = true;
		else
			rightMotionInProgress = true;
	}

	if(!success)
		ROS_ERROR("moveToPose: Failed to move to pose");

	return success;
}

bool RobotMoveit::getKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm)
{

      // Indigo
	  //kinematic_state->copyJointGroupPositions("right_arm", joint_values_right_arm);
	  //kinematic_state->copyJointGroupPositions("left_arm", joint_values_left_arm);
	  
	  // Groovy
      joint_state_group_right_arm->getVariableValues(joint_values_right_arm);
	  joint_state_group_left_arm->getVariableValues(joint_values_left_arm);


	  /* Print joint names and values */
	  for(std::size_t i = 0; i < joint_names_right_arm.size(); ++i)
	  {
	    ROS_INFO("Joint %s: %f", joint_names_right_arm[i].c_str(), joint_values_right_arm[i]);
	  }

	  for(std::size_t i = 0; i < joint_names_left_arm.size(); ++i)
	  {
	    ROS_INFO("Joint %s: %f", joint_names_left_arm[i].c_str(), joint_values_left_arm[i]);
	  }

	  return true;
}

bool RobotMoveit::setKinematicModelJointValues(std::vector<double> joint_values_left_arm, std::vector<double> joint_values_right_arm)
{

    // Indigo
	//kinematic_state->setJointGroupPositions("right_arm", joint_values_right_arm);
	//kinematic_state->setJointGroupPositions("left_arm", joint_values_left_arm);
	
	// Groovy
	joint_state_group_right_arm->setVariableValues(joint_values_right_arm);
	joint_state_group_left_arm->setVariableValues(joint_values_left_arm);


	// Check joint limits
	if(kinematic_state->satisfiesBounds())
	{
		ROS_INFO("Current kinematic model state is valid");
	}
	else
	{
		ROS_WARN("Current kinematic model state is not valid");
	}

	// Enforce joint limits
	kinematic_state->enforceBounds();
	// and check again
	if(kinematic_state->satisfiesBounds())
	{
		ROS_INFO("Current kinematic model state is valid");
	}
	else
	{
		ROS_WARN("Current kinematic model state is not valid");
		return false;
	}

	return true;
}

bool RobotMoveit::kinematicModelFK() {return true;}
bool RobotMoveit::kinematicModelIK() {return true;}
bool RobotMoveit::kinematicModelJacobian() {return true;}


bool RobotMoveit::executeJointGoal(RobotMoveit::WhichArm arm, std::vector<double> joint_values)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setJointValueTarget(joint_values);
	bool success = group->plan(my_plan);

	ROS_INFO("Planning Joint goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		group->move();
		return true;
	}

	return false;
}

bool RobotMoveit::executeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, bool waitForMotion)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	group->setPoseTarget(target);

	bool success = true;//group->plan(my_plan);		// FIXME gets stuck here, missing a launch file?

	ROS_INFO("Planning Cartesian goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		if(waitForMotion)
		{
			group->move();
		}
		else
		{
			group->asyncMove();
			if(arm == RobotMoveit::LEFT)
				leftMotionInProgress = true;
			else
				rightMotionInProgress = true;
		}
		return true;
	}

	return false;
}

bool RobotMoveit::safeCarteGoal(RobotMoveit::WhichArm arm, geometry_msgs::Pose target, int nHold, double dx, double fractionThreshold)
{

	  std::vector<geometry_msgs::Pose> waypoints;
	  geometry_msgs::Pose start_pose;

	  // Get starting position
//	  getEndeffectorPose(BaxterMoveit::RIGHT,&start_pose);
//	  for(int i=0;i<nHold;i++)
//		  waypoints.push_back(start_pose);

//	  for(int i=0;i<nHold;i++)			// Hold final position for better accuracy
		  waypoints.push_back(target);

	  return executeCartePath(arm, waypoints, dx, fractionThreshold);

}

bool RobotMoveit::executeCarteGoalWithConstraint(RobotMoveit::WhichArm arm, geometry_msgs::Pose target)
{

	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	std::string arm_name;
	switch(arm)
	{
	case RobotMoveit::LEFT:
		arm_name = "left";
		break;
	case RobotMoveit::RIGHT:
		arm_name = "right";
		break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	moveit_msgs::OrientationConstraint ocm;
	ocm.link_name = arm_name+"_gripper";//"_lower_forearm";
	ocm.header.frame_id = "base";//"torso";
//	ocm.orientation.x = -0.6768 ;
//	ocm.orientation.y = -0.2047  ;
//	ocm.orientation.z = -0.2047;
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.1;
	ocm.absolute_y_axis_tolerance = 0.1;
	ocm.absolute_z_axis_tolerance = 0.1;
	ocm.weight = 1.0;

	moveit_msgs::Constraints test_constraints;
	test_constraints.orientation_constraints.push_back(ocm);
	group->setPathConstraints(test_constraints);


	// will only work if the current state already satisfies the path constraints
	robot_state::RobotState start_state(*group->getCurrentState());
	geometry_msgs::Pose start_pose2;
	start_pose2.orientation.w = 1.0;
	start_pose2.position.x = 0.55;
	start_pose2.position.y = -0.05;
	start_pose2.position.z = 0.8;
//	const robot_state::JointModelGroup *joint_model_group = start_state.getJointModelGroup(group->getName());
//	start_state.setFromIK(joint_model_group, start_pose2);
	group->setStartState(start_state);

	group->setPoseTarget(target);

	bool success = group->plan(my_plan);

	ROS_INFO("Planning Cartesian goal %s",success?"SUCCEEDED":"FAILED");

	// TODO: Publish to RVIZ
//	  ROS_INFO("Visualizing plan 1 (again)");
//	  display_trajectory.trajectory_start = my_plan.start_state_;
//	  display_trajectory.trajectory.push_back(my_plan.trajectory_);
//	  display_publisher.publish(display_trajectory);
//	  /* Sleep to give Rviz time to visualize the plan. */
//	  sleep(5.0);

	if(success)
	{
		group->move();
		group->clearPathConstraints();
		return true;
	}

	group->clearPathConstraints();
	return false;
}

bool RobotMoveit::executeJointPath(RobotMoveit::WhichArm arm, std::vector< std::vector<double> > joint_waypoints)
{

	return true;
}

bool RobotMoveit::executeCartePath(RobotMoveit::WhichArm arm, std::vector<geometry_msgs::Pose> waypoints, double dx, double fractionThreshold)
{
/*
 * http://answers.ros.org/question/74776/cartesian-controller-for-ros/
 * https://groups.google.com/forum/#!msg/moveit-users/x5FwalM5ruk/9OpXslS8x2YJ
 * https://groups.google.com/forum/#!msg/moveit-users/MOoFxy2exT4/JKFeSGASPMMJ
 *
 * call move_group->ComputeCartesianPath()executeCartePath with your sequence of cartesian points

    NOTE: this function accepts a std::vector<Pose>, so you'll need to convert your cartesian positions using tf::poseEigenToMsg()
    this returns a RobotTrajectory.

 * send your trajectory to MoveIt

    the easiest way I've found is to use the move_group node's ExecuteKnownTrajectory service, which is available on my setup at "/execute_kinematic_path".
    alternatively, you could probably send it to the TrajectoryExecutionManager object directly, as shown here, but that seems messy
 */


	moveit::planning_interface::MoveGroup* group;

	if(!selectArmGroup(arm,&group))
	{
		return false;
	}

	if(dx > 0.15)
	{
		ROS_ERROR("BaxterMoveit::executeCartePath -> dt step size is too large.");
		return false;
	}


	moveit_msgs::RobotTrajectory trajectory;
	group->setPlanningTime(5.0);

	double fraction = group->computeCartesianPath(waypoints,
	                                              dx,  		    // eef_step (step size in meters)
	                                              0.0,   		// jump_threshold
	                                              trajectory,	// result
												  true);		// collision avoidance

	//std::cout<<trajectory<<"\n\n";

	if(fraction == -1)
	{
		ROS_ERROR("Computing the Cartesian path failed.");
		return false;
	}
	if(fraction < fractionThreshold)
	{
		ROS_ERROR("The computed Cartesian path achieved %.2f%% of the given path.",fraction * 100.0);
		return false;
	}
	if(0 <= fraction && fraction < 1)
	{
		ROS_WARN("The computed Cartesian path achieved %.2f%% of the given path.",fraction * 100.0);
	}

	std::string arm_group;
	switch(arm)
	{
	case RobotMoveit::LEFT:
		arm_group = "left_arm";
		break;
	case RobotMoveit::RIGHT:
		arm_group = "right_arm";
		break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}

	// Modify trajectory to include velocities
	// 1) create a RobotTrajectory object
	robot_trajectory::RobotTrajectory rt(group->getCurrentState()->getRobotModel(), arm_group);

	// 2) get a RobotTrajectory from trajectory
	rt.setRobotTrajectoryMsg(*group->getCurrentState(), trajectory);
/*
	// 3) create a IterativeParabolicTimeParameterization object
	trajectory_processing::IterativeParabolicTimeParameterization iptp;

	// 4) compute computeTimeStamps
	bool success = iptp.computeTimeStamps(rt);
	ROS_INFO("Computed time stamp %s",success?"SUCCEDED":"FAILED");

	// Get RobotTrajectory_msg from RobotTrajectory
	rt.getRobotTrajectoryMsg(trajectory);
*/
	// Plan and execute the trajectory
	my_plan.trajectory_ = trajectory;

	if(!group->execute(my_plan))
	{
		return false;
	}

	return true;
}


bool RobotMoveit::motionComplete()
{

	return true;
}

bool RobotMoveit::cancelMotion()
{

	return true;
}
