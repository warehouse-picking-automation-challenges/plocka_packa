/*
 * origin_tf_publisher.cpp
 *
 *  Created on: Apr 19, 2015
 *      Author: sven
 *
 *  Description: adds reset functionallity to odom_combined tf
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

#include <apc_msgs/SetPose.h>


tf::StampedTransform tf_origin_to_odom;
tf::StampedTransform tf_shelf_to_origin;

double tf_origin_to_odom_x, tf_origin_to_odom_y, tf_origin_to_odom_z, tf_shelf_to_origin_x, tf_shelf_to_origin_y, tf_shelf_to_origin_z;

void printOdomOrigin()
{
	std::cout<<"*** Odom origin ***\n";
	std::cout<<"  position:\n";
	std::cout<<"    x:"<<tf_origin_to_odom.getOrigin().getX()<<"\n";
	std::cout<<"    y:"<<tf_origin_to_odom.getOrigin().getY()<<"\n";
	std::cout<<"    z:"<<tf_origin_to_odom.getOrigin().getZ()<<"\n";
	std::cout<<"  orientation:\n";
	std::cout<<"    x:"<<tf_origin_to_odom.getRotation().getX()<<"\n";
	std::cout<<"    y:"<<tf_origin_to_odom.getRotation().getY()<<"\n";
	std::cout<<"    z:"<<tf_origin_to_odom.getRotation().getZ()<<"\n";
	std::cout<<"    w:"<<tf_origin_to_odom.getRotation().getW()<<"\n";
}

void printShelfOrigin()
{
	std::cout<<"*** Shelf origin ***\n";
	std::cout<<"  position:\n";
	std::cout<<"    x:"<<tf_shelf_to_origin.getOrigin().getX()<<"\n";
	std::cout<<"    y:"<<tf_shelf_to_origin.getOrigin().getY()<<"\n";
	std::cout<<"    z:"<<tf_shelf_to_origin.getOrigin().getZ()<<"\n";
	std::cout<<"  orientation:\n";
	std::cout<<"    x:"<<tf_shelf_to_origin.getRotation().getX()<<"\n";
	std::cout<<"    y:"<<tf_shelf_to_origin.getRotation().getY()<<"\n";
	std::cout<<"    z:"<<tf_shelf_to_origin.getRotation().getZ()<<"\n";
	std::cout<<"    w:"<<tf_shelf_to_origin.getRotation().getW()<<"\n";
}

bool setOdomOriginCB(apc_msgs::SetPose::Request &req, apc_msgs::SetPose::Response &res)
{

	tf_origin_to_odom.setOrigin( tf::Vector3(	req.pose.position.x,
												req.pose.position.y,
												req.pose.position.z) );

	tf_origin_to_odom.setRotation( tf::Quaternion(	req.pose.orientation.x,
													req.pose.orientation.y,
													req.pose.orientation.z,
													req.pose.orientation.w) );
	ROS_INFO("Updated odom_origin location:");
	printOdomOrigin();

	return true;
}

bool setShelfOriginCB(apc_msgs::SetPose::Request &req, apc_msgs::SetPose::Response &res)
{

	tf_shelf_to_origin.setOrigin( tf::Vector3(	req.pose.position.x,
												req.pose.position.y,
												req.pose.position.z) );

	tf_shelf_to_origin.setRotation( tf::Quaternion(	req.pose.orientation.x,
													req.pose.orientation.y,
													req.pose.orientation.z,
													req.pose.orientation.w) );
	ROS_INFO("Updated shelf_origin location:");
	printShelfOrigin();

	return true;
}

int main(int argc, char **argv)
{

	ros::init(argc, argv, "origin_tf_publisher_node");
	ros::NodeHandle n;

	ros::ServiceServer setOdomOrigin_srv  = n.advertiseService("apc/tf/set_odom_origin",setOdomOriginCB);
	ros::ServiceServer setShelfOrigin_srv = n.advertiseService("apc/tf/set_shelf_origin",setShelfOriginCB);

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	int Hz = 50;
	ros::Rate loopRate(Hz);


	// Wait for the listener to get the first message
	if(! listener.waitForTransform("odom_combined","base_footprint", ros::Time(0), ros::Duration(3.0)) )
		ROS_ERROR("origin_tf_publisher_node: Could not find odom_combined->base_footprint transform!");


	// Initialize origin using current location
	/*
	try
	{
		listener.lookupTransform("odom_combined","base_footprint", ros::Time(0), tf_origin_to_odom);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("%s",ex.what());
		ros::Duration(1.0).sleep();
	}
	*/

	if(n.hasParam("apc/tf_origin_to_odom/x") && n.hasParam("apc/tf_origin_to_odom/y") && n.hasParam("apc/tf_origin_to_odom/z") && n.hasParam("apc/tf_shelf_to_origin/x") && n.hasParam("apc/tf_shelf_to_origin/y") && n.hasParam("apc/tf_shelf_to_origin/z"))
	{
		n.getParam("apc/tf_origin_to_odom/x", tf_origin_to_odom_x);
		n.getParam("apc/tf_origin_to_odom/y", tf_origin_to_odom_y);
		n.getParam("apc/tf_origin_to_odom/z", tf_origin_to_odom_z);
		n.getParam("apc/tf_shelf_to_origin/x", tf_shelf_to_origin_x);
		n.getParam("apc/tf_shelf_to_origin/y", tf_shelf_to_origin_y);
		n.getParam("apc/tf_shelf_to_origin/z", tf_shelf_to_origin_z);

		tf_origin_to_odom.setOrigin(tf::Vector3(tf_origin_to_odom_x,tf_origin_to_odom_y,tf_origin_to_odom_z));
		tf_origin_to_odom.setRotation(tf::createQuaternionFromRPY(0,0,0));

		tf_shelf_to_origin.setOrigin(tf::Vector3(tf_shelf_to_origin_x,tf_shelf_to_origin_y,tf_shelf_to_origin_z));
		tf_shelf_to_origin.setRotation(tf::createQuaternionFromRPY(0,0,0));

	}
	else
	{
		tf_origin_to_odom.setOrigin(tf::Vector3(1.32,0.6,0.0));
		tf_origin_to_odom.setRotation(tf::createQuaternionFromRPY(0,0,0));

		tf_shelf_to_origin.setOrigin(tf::Vector3(0.0,0.0,0.86));
		tf_shelf_to_origin.setRotation(tf::createQuaternionFromRPY(0,0,0));
	}
	printOdomOrigin();
	printShelfOrigin();

	while (n.ok())
	{
		broadcaster.sendTransform(tf::StampedTransform(tf_origin_to_odom, ros::Time::now(), "odom_combined", "odom_origin"));

		broadcaster.sendTransform(tf::StampedTransform(tf_shelf_to_origin, ros::Time::now(), "odom_origin", "shelf_origin"));

		ros::spinOnce();

		loopRate.sleep();
	}

	ROS_INFO("Shutting down ... ");
	ros::shutdown();

	return 0;

}




