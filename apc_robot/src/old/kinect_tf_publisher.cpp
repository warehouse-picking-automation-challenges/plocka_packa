/*
 * kinect_tf_publisher.cpp
 *
 *  Created on: Jan 8, 2015
 *      Author: sven
 */

/* The tf published from a static_transform_publisher between the /right_hand and /camera_link
 * seems to lag because of a clock synchronization issue. This seems to be more stable way
 * of publishing the transform.
 */

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <kdl/frames.hpp>
#include <tf_conversions/tf_kdl.h>

int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect_tf_publisher_node");
	ros::NodeHandle n;

	tf::TransformListener listener;
	tf::TransformBroadcaster broadcaster;

	int Hz = 50;
	ros::Rate loopRate(Hz);

	// Fixed transforms between hand and camera
	tf::Vector3 fixedTrans( 0.022, -0.075, -0.021 );		// Position inside right_hand frame

	tf::Quaternion fixedRot ( 0.70711,
							  0.0,
							  0.70711,
							  0.0);

	while (n.ok())
	{

		tf::StampedTransform transform;

		try{
			listener.lookupTransform("base", "right_hand",
					ros::Time(0), transform);
		}
		catch (tf::TransformException &ex) {
			ROS_ERROR("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
		}

		tf::Transform tf_kinect;

		tf_kinect.setOrigin( tf::Vector3( transform.getOrigin() + tf::quatRotate(transform.getRotation(), fixedTrans) ) );

		tf_kinect.setRotation( transform.getRotation()*fixedRot );

		//		    tf_kinect.setOrigin(transform.getOrigin());
		//			tf_kinect.setOrigin(tf::Vector3(transform.getOrigin().x(),
		//											transform.getOrigin().y(),
		//											transform.getOrigin().z()));
		//
		//			tf_kinect.setRotation(tf::Quaternion(transform.getRotation().x(),
		//												 transform.getRotation().y(),
		//												 transform.getRotation().z(),
		//												 transform.getRotation().w()));

		broadcaster.sendTransform(tf::StampedTransform(tf_kinect, ros::Time::now(), "base", "camera_link"));

		loopRate.sleep();
	}

	ROS_INFO("Shutting down ... ");
	ros::shutdown();

	return 0;

}



