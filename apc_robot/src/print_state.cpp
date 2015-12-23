/*
 * print_state.cpp
 *
 *  Created on: May 22, 2015
 *      Author: sven
 */

#include <ros/ros.h>

#include <tf/transform_listener.h>
#include <apc_msgs/ReturnJointStates.h>
#include <geometry_msgs/PoseStamped.h>

tf::TransformListener* listenerPtr;

typedef enum { LEFT, RIGHT } WhichArm;

bool getTF(std::string target, std::string source, tf::StampedTransform& result)
{

	try
	{
		listenerPtr->lookupTransform(source, target, ros::Time(0), result);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		return false;
	}
	return true;
}

void printTF(tf::StampedTransform t, WhichArm a)
{
	std::string p;
	if(a==RIGHT)
	{
		std::cout<< "#Right gripper:\n";
		p="r";
	}
	else
	{
		std::cout<< "#Left gripper:\n";
		p="l";
	}

	std::cout<< "p.position.x     = " << t.getOrigin().getX()    << ";\n"
			 << "p.position.y     = " << t.getOrigin().getY()    << ";\n"
			 << "p.position.z     = " << t.getOrigin().getZ()    << ";\n"
			 << "p.orientation.x  = " << t.getRotation().getX()  << ";\n"
			 << "p.orientation.y  = " << t.getRotation().getY()  << ";\n"
			 << "p.orientation.z  = " << t.getRotation().getZ()  << ";\n"
			 << "p.orientation.w  = " << t.getRotation().getW()  << ";\n";

	geometry_msgs::PoseStamped tmp;
	tmp.pose.position.x 	= t.getOrigin().getX()  ;
	tmp.pose.position.y     = t.getOrigin().getY()  ;
	tmp.pose.position.z     = t.getOrigin().getZ()  ;
	tmp.pose.orientation.x 	= t.getRotation().getX();
	tmp.pose.orientation.y  = t.getRotation().getY();
	tmp.pose.orientation.z  = t.getRotation().getZ();
	tmp.pose.orientation.w  = t.getRotation().getW();
	std::cout<< "    pose:\n"
			 << "      position:\n"
			 << "        x: "			<< tmp.pose.position.x << "\n"
			 << "        y: "			<< tmp.pose.position.y << "\n"
			 << "        z: "			<< tmp.pose.position.z << "\n"
			 << "      orientation:\n"
			 << "        x: "			<< tmp.pose.orientation.x << "\n"
			 << "        y: "			<< tmp.pose.orientation.y << "\n"
			 << "        z: "			<< tmp.pose.orientation.z << "\n"
			 << "        w: "			<< tmp.pose.orientation.w << "\n\n";
//	std::cout<<tmp<<"\n";
}


apc_msgs::ReturnJointStates joints_right;
apc_msgs::ReturnJointStates joints_left;

int main(int argc, char **argv)
{

	ros::init(argc, argv, "print_state_node");
	ros::NodeHandle n;

	std::string robot_frame = "base_link";
	if (argc > 1)
	{
		robot_frame = argv[1];
	}

	ROS_INFO_STREAM("Frame id is: " << robot_frame );

	ros::ServiceClient srv_get_joints_ = n.serviceClient<apc_msgs::ReturnJointStates>("return_joint_states");

	if( !srv_get_joints_.waitForExistence(ros::Duration(1.0) ) )
		ROS_ERROR("Joint state listener not running! Please run \n  rosrun apc_robot joint_states_listener.py");

	listenerPtr = new tf::TransformListener();

	std::string r_controller_name = "r_arm_controller_loose";
	std::string l_controller_name = "l_arm_controller_loose";

	std::string r_frame = "r_gripper_tool_frame";
	std::string l_frame = "l_gripper_tool_frame";

	joints_right.request.name.resize(0);
	joints_right.request.name.push_back( 	"r_shoulder_pan_joint"     );
	joints_right.request.name.push_back( 	"r_shoulder_lift_joint"    );
	joints_right.request.name.push_back( 	"r_upper_arm_roll_joint"   );
	joints_right.request.name.push_back( 	"r_elbow_flex_joint"       );
	joints_right.request.name.push_back( 	"r_forearm_roll_joint"     );
	joints_right.request.name.push_back( 	"r_wrist_flex_joint"       );
	joints_right.request.name.push_back( 	"r_wrist_roll_joint"       );

	joints_left.request.name.resize(0);
	joints_left.request.name.push_back( 	"l_shoulder_pan_joint"     );
	joints_left.request.name.push_back( 	"l_shoulder_lift_joint"    );
	joints_left.request.name.push_back( 	"l_upper_arm_roll_joint"   );
	joints_left.request.name.push_back( 	"l_elbow_flex_joint"       );
	joints_left.request.name.push_back( 	"l_forearm_roll_joint"     );
	joints_left.request.name.push_back( 	"l_wrist_flex_joint"       );
	joints_left.request.name.push_back( 	"l_wrist_roll_joint"       );


	tf::StampedTransform transform;

	int Hz = 50;
	ros::Rate loopRate(Hz);

	// Wait for the listener to get the first message
	if(! listenerPtr->waitForTransform(l_frame, robot_frame, ros::Time(0), ros::Duration(3.0)) )
		ROS_ERROR("Could not fprintTFind source->target transform!");

	if(! listenerPtr->waitForTransform(r_frame, robot_frame, ros::Time(0), ros::Duration(3.0)) )
		ROS_ERROR("Could not fprintTFind source->target transform!");

	std::string tmp;

	bool done = false;
	while(ros::ok() && !done)
	{

		std::cout<<"Press [enter] to get joint state (q to quit):";
		std::getline(std::cin, tmp);

		if(tmp == "q")
		{
			done=true;
		}
		else
		{
			if( getTF(r_frame, robot_frame, transform) )
				printTF(transform, RIGHT);

			if( getTF(l_frame, robot_frame, transform) )
				printTF(transform, LEFT);
		}
		std::cout<<"-----------------------\n";

		if(srv_get_joints_.call(joints_left))
		{
			std::cout<<"left: =   [";
			for(int k = 0;k<7;k++)
			{
				std::cout<<joints_left.response.position[k];
				if(k<6)
					std::cout<<",";
				else
					std::cout<<"];\n";
			}
		}

		if(srv_get_joints_.call(joints_right))
		{
			std::cout<<"right: =  [";
			for(int k = 0;k<7;k++)
			{
				std::cout<<joints_right.response.position[k];
				if(k<6)
					std::cout<<",";
				else
					std::cout<<"];\n";
			}
		}
		std::cout<<"\n###############################\n";
		ROS_INFO("");
	}

	ROS_INFO("Shutting down ... ");
	ros::shutdown();

	return 0;

}
