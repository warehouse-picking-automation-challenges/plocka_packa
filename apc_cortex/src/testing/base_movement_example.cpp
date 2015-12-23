/*
 * apc_baxter_node.cpp
 *
 *  Created on: Mar 8, 2015
 *      Author: Sven Cremer
 */


#include <apc_cortex/apc_cortex.h>

//#include <apc_baxter/apc_baxter_commander.h>
#include <apc_robot/apc_robot_moveit.h>
#include <apc_robot/apc_robot_grippers.h>
#include <apc_cortex/BaseMovement.h>

//#include <apc_object_detection/GetObject.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


/* Functions:
 * - shelf detection
 * - load approach strategy for a bin (joint postions: neutral -> approach -> close -> leave -> neutral)
 * - find object with ASUS scanning
 * - visual servoing
 * - etc.
 */

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of BaxterMoveit class
 ***********************************************************************************************************************/
int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_core_node",ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();


	BaseMovement* base_movement = new BaseMovement;

	ros::spinOnce();
	base_movement->move("center");

		//ROS_INFO_STREAM("Delta Pose(" << delta_pose[0] << "," << delta_pose[1] << "," << delta_pose[2] << ")");

	ROS_INFO("Done!");
	ros::shutdown();

	return 0;

}


