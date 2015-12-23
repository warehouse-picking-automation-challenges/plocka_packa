/*
 * origin_camera_tf_publisher.cpp
 *
 *  Created on: May 11, 2015
 *      Author: Sven Cremer
 *
 *      Publishes tf between origin and camera
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

#include <geometry_msgs/PointStamped.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "origin_camera_tf_publisher_node");
	ros::NodeHandle n;

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	int Hz = 50;
	ros::Rate loopRate(Hz);

	std::string camera_link = "kinect_camera";
	std::string origin_link = "odom_origin";
	double timeOut=0.5;

	// Wait for the listener to get the first message
	if(! listener.waitForTransform("odom_combined","odom_origin", ros::Time(0), ros::Duration(3.0)) )
		ROS_ERROR("origin_camera_tf_publisher_node: Could not find odom_combined->odom_origin transform!");


	while (n.ok())
	{

		// Compute tf from Origin into Camera frame
		tf::StampedTransform tf_origin_to_camera;

		try
		{
			listener.lookupTransform( origin_link, "head_mount_kinect_rgb_optical_frame", ros::Time(0), tf_origin_to_camera);
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(timeOut).sleep();
			continue;
		}

		tf::Transform tf_origin2kinect;
		tf_origin2kinect.setOrigin( tf::Vector3( tf_origin_to_camera.getOrigin() ) );
		tf_origin2kinect.setRotation( tf_origin_to_camera.getRotation() );

		broadcaster.sendTransform(tf::StampedTransform(tf_origin2kinect, ros::Time::now(), origin_link, camera_link));

		loopRate.sleep();
	}

	ROS_INFO("Shutting down ... ");
	ros::shutdown();

	return 0;

}


