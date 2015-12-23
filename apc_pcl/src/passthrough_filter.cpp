/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2015, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/*
 * passthrough_filter.cpp
 *
 *  Created on: Apr 18, 2015
 *      Author: sven
 *
 *      All PCL processing will be done in this node in to avoid sending several PCL msgs over the network
 */


#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <apc_msgs/SetLimits.h>
#include <apc_msgs/SetBool.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>

ros::Publisher pub;
tf::TransformListener *tf_listener;

bool publishPCL;

double x_min;
double x_max;
double y_min;
double y_max;
double z_min;
double z_max;

// rosservice call /apc/passthrough_filter/setLimits "{x_min: -0.5, x_max: 0.7, y_min: -0.5, y_max: 1.8, z_min: 0.7, z_max: 2.2}"
// rosservice call /apc/passthrough_filter/setLimits "{x_min: -10, x_max: 10, y_min: -10, y_max: 10, z_min: -10, z_max: 10}"
// rosservice call /apc/passthrough_filter/setLimits "{x_min: -0.15, x_max: 0.6, y_min: -0.96, y_max: 0.15, z_min: 0.68, z_max: 1.93}"

//double x_min = -10.0;
//double x_max = 10.0;
//double y_min = -10.0;
//double y_max = 10.0;
//double z_min = -10.0;
//double z_max = 10.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	if(!publishPCL)
		return;

	/* Transform into fixed frame */
	sensor_msgs::PointCloud2 tf_cloud;
	if(!pcl_ros::transformPointCloud("odom_origin", *cloud, tf_cloud, *tf_listener))
	{
		ROS_WARN("Point cloud could not be transformed");
		return;
	}
	sensor_msgs::PointCloud2ConstPtr tf_cloud_ptr (new sensor_msgs::PointCloud2(tf_cloud));

	/* Filter cloud */
	sensor_msgs::PointCloud2 cloud_filtered;
	sensor_msgs::PointCloud2 cloud_filtered1;
	sensor_msgs::PointCloud2 cloud_filtered2;

	// Create the filtering objects
	pcl::PassThrough<sensor_msgs::PointCloud2> passX;
	pcl::PassThrough<sensor_msgs::PointCloud2> passY;
	pcl::PassThrough<sensor_msgs::PointCloud2> passZ;

	// Filter x
	passX.setInputCloud (tf_cloud_ptr);
	passX.setFilterFieldName ("x");
	passX.setFilterLimits(x_min, x_max);
	passX.filter (cloud_filtered);

	// Filter y
	sensor_msgs::PointCloud2ConstPtr cloud_filtered_x_ptr (new sensor_msgs::PointCloud2(cloud_filtered));
	passY.setInputCloud (cloud_filtered_x_ptr);
	passY.setFilterFieldName ("y");
	passY.setFilterLimits(y_min, y_max);
	passY.filter (cloud_filtered1);

	// Filter z
	sensor_msgs::PointCloud2ConstPtr cloud_filtered_xy_ptr (new sensor_msgs::PointCloud2(cloud_filtered1));
	passZ.setInputCloud (cloud_filtered_xy_ptr);
	passZ.setFilterFieldName ("z");
	passZ.setFilterLimits(z_min, z_max);
	passZ.filter (cloud_filtered2);

	//pass.setFilterLimitsNegative (true);

	// Publish the data
	pub.publish (cloud_filtered2);					// TODO only publish bin PCL; perfomr shelf detection inside node
}

//void updateParam(ros::NodeHandle& nh)
//{
//	std::string path = "/apc/passthrough_filter/";
//	if(!(nh.getParam(std::string(path+"x/min"), x_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"x/max"), x_max))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"y/min"), y_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"y/max"), y_max))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"z/min"), z_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"z/max"), z_max))){ROS_ERROR("Parameter Not found.");}
//}

bool enablePCL(apc_msgs::SetBool::Request &req, apc_msgs::SetBool::Response &res)
{
	publishPCL = req.variable;

	return true;
}

bool updateLimits(apc_msgs::SetLimits::Request &req, apc_msgs::SetLimits::Response &res)
{
	x_min = req.x_min;
	x_max = req.x_max;
	y_min = req.y_min;
	y_max = req.y_max;
	z_min = req.z_min;
	z_max = req.z_max;

	if(x_max < x_min)
	{
		ROS_WARN("Passthrough filter: x_max < x_min (switching values)");
		x_min = x_max;
		x_max = req.x_min;
	}
	if(y_max < y_min)
	{
		ROS_WARN("Passthrough filter: y_max < y_min (switching values)");
		y_min = y_max;
		y_max = req.y_min;
	}
	if(z_max < z_min)
	{
		ROS_WARN("Passthrough filter: z_max < z_min (switching values)");
		z_min = z_max;
		z_max = req.z_min;
	}

	ROS_INFO("Updated filter limits: x=[%f,%f],y=[%f,%f],z=[%f,%f]",x_min,x_max,y_min,y_max,z_min,z_max);

	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "apc_pcl");
	ros::NodeHandle nh;

//	updateParam(nh);
//	ROS_INFO("Loaded parameters");

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	// TODO pub = nh.advertise<sensor_msgs::PointCloud2> ("bin", 1);

	tf_listener    = new tf::TransformListener();

	//Service
	ros::ServiceServer srv_setLimits = nh.advertiseService("apc/passthrough_filter/setLimits",updateLimits);
	ros::ServiceServer srv_enablePCL = nh.advertiseService("apc/passthrough_filter/enablePCL",enablePCL);

	//ros::spin();

	// Update rate TODO use flag instead
	int rate = 10;
	ros::Rate r(rate);

	publishPCL = true;

	x_min = -0.15;
	x_max = 0.6;
	y_min = 0.15;
	y_max = -0.96;
	z_min = 0.68;
	z_max = 1.93;

	// Main loop
	while (nh.ok())
	{
		//updateParam(nh);

		ros::spinOnce();		// check for incoming messages

		r.sleep();
	}

}

