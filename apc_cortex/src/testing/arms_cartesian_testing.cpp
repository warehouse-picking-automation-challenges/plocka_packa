/*
 * arms_cartesian_testing.cpp
 *
 *  Created on: May 21, 2015
 *      Author: sven
 */


#include <apc_cortex/apc_cortex.h>

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
 ***********************************************************************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_core_node",ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	ApcCortex cortex;

	int rate = 10;
	ros::Rate r(rate);
//	while( ros::ok() )
//	{
//		ros::spinOnce();
//		r.sleep();
//	}

	/* Arm testing */
//	cortex.testingCartesian();
//	cortex.testingCartesianMoveInDirection();
//	cortex.testingCartesianGetCurrentPose();
//	cortex.testingCartesianArmGains();
//	cortex.testingToolPickUp();
	cortex.testingGripperTilt();
//	cortex.testingGripPose();

	ROS_INFO("Done!");
	ros::shutdown();

	return 0;

}


