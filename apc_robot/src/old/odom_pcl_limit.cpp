/*
 * odom_pcl_limit.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: sven
 */


#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <apc_msgs/SetLimits.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "odom_pcl_limits_node");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<apc_msgs::SetLimits>("apc/passthrough_filter/setLimits");
	apc_msgs::SetLimits srv_msg;

	std::string camera_link = "kinect_camera";
	std::string origin_link = "odom_origin";
	double timeOut = 0.5;

	tf::TransformListener listener;

	int Hz = 10;
	ros::Rate loopRate(Hz);

	/************************ Initialize Marker ************************/
	ros::Publisher marker_pub_odom   = n.advertise<visualization_msgs::MarkerArray>("apc/shelf_odom", 100);
	ros::Publisher marker_pub_camera = n.advertise<visualization_msgs::MarkerArray>("apc/shelf_camera", 100);

	visualization_msgs::MarkerArray marker_array_msg;
	marker_array_msg.markers.resize(4);

	for ( int i = 0; i < 4; i++)
	{
		marker_array_msg.markers[i].header.frame_id = origin_link;

		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker_array_msg.markers[i].ns = "apc_shelf";
		marker_array_msg.markers[i].id = i;

		// Set the marker type.
		marker_array_msg.markers[i].type = visualization_msgs::Marker::CUBE;

		// Set the marker action.  Options are ADD and DELETE
		marker_array_msg.markers[i].action = visualization_msgs::Marker::ADD;

		// Set the scale of the marker -- 1x1x1 here means 1m on a side
		marker_array_msg.markers[i].scale.x = 0.02;
		marker_array_msg.markers[i].scale.y = 0.02;
		marker_array_msg.markers[i].scale.z = 0.02;

		// Set the color -- be sure to set alpha to something non-zero!
		marker_array_msg.markers[i].color.r = 0.0f;
		marker_array_msg.markers[i].color.g = 1.0f;
		marker_array_msg.markers[i].color.b = 0.0f;
		marker_array_msg.markers[i].color.a = 0.8;

		marker_array_msg.markers[i].lifetime = ros::Duration(3.0/Hz);				//  how long this marker should stick around before being automatically deleted
	}

	// Lower left
	marker_array_msg.markers[0].pose.position.x = 0.0;
	marker_array_msg.markers[0].pose.position.y = 0.0;
	marker_array_msg.markers[0].pose.position.z = 0.82;
	marker_array_msg.markers[0].pose.orientation.x = 1.0;

	// Lower right
	marker_array_msg.markers[1].pose.position.x = 0.0;
	marker_array_msg.markers[1].pose.position.y = -0.81;
	marker_array_msg.markers[1].pose.position.z = 0.82;
	marker_array_msg.markers[1].pose.orientation.x = 1.0;

	// Upper left
	marker_array_msg.markers[2].pose.position.x = 0.0;
	marker_array_msg.markers[2].pose.position.y = 0.0;
	marker_array_msg.markers[2].pose.position.z = 1.77;
	marker_array_msg.markers[2].pose.orientation.x = 1.0;

	// Upper right
	marker_array_msg.markers[3].pose.position.x = 0.0;
	marker_array_msg.markers[3].pose.position.y = -0.81;
	marker_array_msg.markers[3].pose.position.z = 1.77;
	marker_array_msg.markers[3].pose.orientation.x = 1.0;


	visualization_msgs::MarkerArray marker_array_msg_camera_frame = marker_array_msg;
	for ( int i = 0; i < 4; i++)
	{
		marker_array_msg_camera_frame.markers[i].header.frame_id = camera_link;
		marker_array_msg.markers[i].color.r = 1.0f;
		marker_array_msg.markers[i].color.g = 0.0f;
	}

	/**********************************************************************/


	//wait for the listener to get the first message
	ROS_INFO("Waiting for origin->camera transform ...");
	if( !listener.waitForTransform(origin_link,camera_link, ros::Time(0), ros::Duration(5.0)) )
	{
		ROS_WARN("Could not find transform!");
	}
	else
	{
		ROS_INFO("Found transform!");
	}


	while (n.ok())
	{

		/* Publish Markers */
		for ( int i = 0; i < 4; i++)
		{

			geometry_msgs::PoseStamped marker_point;
			marker_point.header.frame_id 	= marker_array_msg.markers[i].header.frame_id;
			marker_point.pose 				= marker_array_msg.markers[i].pose;
			geometry_msgs::PoseStamped transformed_point;

			try
			{
				listener.transformPose("kinect_camera", marker_point, transformed_point);
			}
			catch(tf::TransformException& ex)
			{
				ROS_ERROR("%s",ex.what());
				ros::Duration(timeOut).sleep();
				continue;
			}

			marker_array_msg_camera_frame.markers[i].pose 	= transformed_point.pose;
			marker_array_msg_camera_frame.markers[i].id 			+= 4;
			marker_array_msg_camera_frame.markers[i].id 			%= 100;
		}

		for ( int i = 0; i < 4; i++)
		{
			marker_array_msg_camera_frame.markers[i].header.stamp 	= ros::Time::now();
			marker_array_msg.markers[i].header.stamp 	= ros::Time::now();
		}

		//publish in kinect frame
		marker_pub_camera.publish(marker_array_msg_camera_frame);
		marker_pub_odom.publish(marker_array_msg);

		/* Publish Filter limits */

		geometry_msgs::PointStamped min_point;
		min_point.header.frame_id = "odom_origin";
		min_point.point.x = marker_array_msg.markers[0].pose.position.x - 0.15;
		min_point.point.y = marker_array_msg.markers[1].pose.position.y - 0.15;
		min_point.point.z = marker_array_msg.markers[0].pose.position.x - 0.15;
		geometry_msgs::PointStamped transformed_min_point;

		try
		{
			listener.transformPoint(camera_link, min_point, transformed_min_point);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(timeOut).sleep();
			continue;
		}

		geometry_msgs::PointStamped max_point;
		max_point.header.frame_id = "odom_origin";
		max_point.point.x = marker_array_msg.markers[0].pose.position.x + 0.65;
		max_point.point.y = marker_array_msg.markers[0].pose.position.y + 0.15;
		max_point.point.z = marker_array_msg.markers[2].pose.position.z + 0.15;
		geometry_msgs::PointStamped transformed_max_point;

		try
		{
			listener.transformPoint(camera_link, max_point, transformed_max_point);
		}
		catch(tf::TransformException& ex)
		{
			ROS_ERROR("%s",ex.what());
			ros::Duration(timeOut).sleep();
			continue;
		}

		srv_msg.request.x_min  = transformed_max_point.point.x;
		srv_msg.request.y_min  = transformed_max_point.point.y;
		srv_msg.request.z_min  = transformed_min_point.point.z;

		srv_msg.request.x_max  = transformed_min_point.point.x;
		srv_msg.request.y_max  = transformed_min_point.point.y;
		srv_msg.request.z_max  = transformed_max_point.point.z;


		std::cout<<srv_msg.request.x_min<<", "<<srv_msg.request.x_max<<"\n";
		std::cout<<srv_msg.request.y_min<<", "<<srv_msg.request.y_max<<"\n";
		std::cout<<srv_msg.request.z_min<<", "<<srv_msg.request.z_max<<"\n\n";

		  if (!client.call(srv_msg))
		  {
			  ROS_ERROR("Failed to call service apc/passthrough_filter/setLimits");
		  }


		loopRate.sleep();
		//ros::Duration(10.0).sleep();
	}

	ROS_INFO("Shutting down ... ");
	ros::shutdown();

	return 0;

}

