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
 * opencv_filtering.cpp
 *
 *  Created on: May 22, 2015
 *      Author: Sumit Kumar Das
 */

#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/Point.h>


#include <apc_msgs/SetImageCrop.h>
#include <apc_msgs/SetBool.h>


image_transport::Publisher pub_kinect_rgb_;
image_transport::Publisher pub_kinect_depth_;

int roi_x,roi_y, roi_width, roi_height;
bool pubhlish_cropped_image;


// Publish Marker for Bin Corners
ros::Publisher pub_marker_bin_corner_;
visualization_msgs::MarkerArray markerArray;


bool publishBinCornerMarker(geometry_msgs::Point p, int i)
{



	// Marker identifiers
	markerArray.markers[i].header.frame_id 		=      "/head_mount_kinect_rgb_optical_frame";
	markerArray.markers[i].header.stamp 		=      ros::Time();
	markerArray.markers[i].ns 					=      "shelf_corners";
	markerArray.markers[i].id 					=      (i + 1);

	// Marker shape
	markerArray.markers[i].type 				=      visualization_msgs::Marker::SPHERE;
	markerArray.markers[i].action 				=      visualization_msgs::Marker::ADD;


	// Marker position and orientation
	markerArray.markers[i].pose.position.x 		=      p.x;
	markerArray.markers[i].pose.position.y 		=      p.y;
	markerArray.markers[i].pose.position.z 		=      p.z;
	markerArray.markers[i].pose.orientation.x 	=      0.0;
	markerArray.markers[i].pose.orientation.y 	=      0.0;
	markerArray.markers[i].pose.orientation.z 	=      0.0;
	markerArray.markers[i].pose.orientation.w 	=      1.0;

	// Marker size and color
	markerArray.markers[i].scale.x 				=      0.05;
	markerArray.markers[i].scale.y 				=      0.05;
	markerArray.markers[i].scale.z 				=      0.05;
	markerArray.markers[i].color.a 				=      0.5;
	markerArray.markers[i].color.r 				=      0.0;
	markerArray.markers[i].color.g 				=      0.0;
	markerArray.markers[i].color.b 				=      1.0;

	return true;

}

void kinectRGBImageCB(const sensor_msgs::ImageConstPtr& msg)
{

	if(!pubhlish_cropped_image)
		return;

	cv_bridge::CvImagePtr cv_rgb_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_rgb_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("cv_bridge RGB_Image exception: %s", e.what());
		return;
	}

	cv::Rect myROI(roi_x, roi_y, roi_width, roi_height);
	cv_rgb_ptr->image = cv_rgb_ptr->image(myROI);



	pub_kinect_rgb_.publish(cv_rgb_ptr->toImageMsg());
}

void kinectDepthImageCB(const sensor_msgs::ImageConstPtr& msg)
{
	if(!pubhlish_cropped_image)
		return;

	cv_bridge::CvImagePtr cv_depth_ptr;
	try
	{
		//Always copy, returning a mutable CvImage
		//OpenCV expects color images to use BGR channel order.
		cv_depth_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
	}
	catch (cv_bridge::Exception& e)
	{
		//if there is an error during conversion, display it
		ROS_ERROR("cv_bridge Depth_Image exception: %s", e.what());
		return;
	}

	cv::Rect myROI(roi_x, roi_y, roi_width, roi_height);
	cv_depth_ptr->image = cv_depth_ptr->image(myROI);

	pub_kinect_depth_.publish(cv_depth_ptr->toImageMsg());
}


bool enableFilter(apc_msgs::SetBool::Request &req, apc_msgs::SetBool::Response &res)
{
	pubhlish_cropped_image = req.variable;

	return true;
}

bool setImageCropLimitsCB(apc_msgs::SetImageCrop::Request &req, apc_msgs::SetImageCrop::Response &res)
{
	double 	focalLength_x, 	focalLength_y;
	double 	bin_x_min, 		bin_x_max, 		bin_y_min, 	bin_y_max, 	bin_z_min, 	bin_z_max;
	int   	pix_x_min, 		pix_x_max, 		pix_y_min, 	pix_y_max;

	ROS_INFO("Set Crop Service Called! :D");

	// Calculate Focal Lengths
	focalLength_x 	= 	640 / (2 * (tan((57/2) * (M_PI/180))));
	focalLength_y 	= 	480 / (2 * (tan((43/2) * (M_PI/180))));
//	focalLength_x	= 	610;
//	focalLength_y 	= 	610;

	ROS_INFO("Focal length: %d, %d", (int)focalLength_x, (int)focalLength_x);
	// Get Bin Details

//	double dy 		= 	0.1;
//	double dz 		= 	0.1;
//
//	double w2 		= 	req.width / 2;
//	double h2 		= 	req.height / 2;

//	bin_x_min       =   (req.center.x - w2);
//	bin_y_min       =   (req.center.y + h2);
//	bin_z_min       =   req.center.z  + 0.2;
//
//	bin_x_max       =   (req.center.x + w2);
//	bin_y_max       =   (req.center.y - h2);
//	bin_z_max       =   req.center.z  + 0.5;

	bin_x_min       =   (req.o.x);
	bin_y_min       =   (req.o.y) - .26;	//TODO figure out the reason for the bias in Y direction
	bin_z_min       =   req.o.z  + 0.2;

	bin_x_max       =   (req.b.x);
	bin_y_max       =   (req.b.y) - .26;	//TODO figure out the reason for the bias in Y direction
	bin_z_max       =   req.b.z  + 0.5;

	ROS_INFO("The Bin min coordinate is: %f, %f, %f  and The Bin max coordinate is: %f, %f, %f", bin_x_min, bin_y_min, bin_z_min, bin_x_max, bin_y_max, bin_z_max);

	// Pixel Calculation
	pix_x_min 		= 	((bin_x_min * focalLength_x)/bin_z_min) + 320 - 20;
	pix_y_min 		= 	((bin_y_min * focalLength_y)/bin_z_min) + 240 + 20;

	pix_x_max 		= 	((bin_x_max * focalLength_x)/bin_z_min) + 320 + 20;
	pix_y_max 		= 	((bin_y_max * focalLength_y)/bin_z_min) + 240 - 20;

	ROS_INFO("The Pixels are: %d, %d, %d, %d", pix_x_min, pix_y_min, pix_x_max, pix_y_max);

	// Publishing corner markers
	markerArray.markers.resize(4);

	publishBinCornerMarker(req.o, 0);
	publishBinCornerMarker(req.a, 1);
	publishBinCornerMarker(req.b, 2);
	publishBinCornerMarker(req.c, 3);

	pub_marker_bin_corner_.publish(markerArray);


	// Sanity check
	if((pix_x_min < 0) | (pix_y_min < 0) | ((pix_x_min + abs(pix_x_max - pix_x_min)) > 640) | ((pix_y_min + abs(pix_y_max - pix_y_min))>480))
	{
		return true;
	}
	// Update Filter
	roi_x 	  		= 	pix_x_min;
	roi_y 	  		= 	pix_y_min;
	roi_width 		= 	abs(pix_x_max - pix_x_min);
	roi_height		= 	abs(pix_y_max - pix_y_min);

	ROS_INFO("The Filter is: %d, %d, %d, %d", roi_x, roi_y, roi_width, roi_height);
	// Enable Crop
	//cropped 		= 	req.crop_image;

	res.success 	= 	true;



	return true;

}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "opencv_filtering_node");
	ros::NodeHandle nh;

	// Image Transport
	image_transport::ImageTransport it(nh);

	// Marketr for bin corners
	pub_marker_bin_corner_							=	nh.advertise<visualization_msgs::MarkerArray>("/apc/shelf/marker_bin_corner", 1);

	// Create a ROS subscribers
	image_transport::Subscriber sub_kinect_rgb_ 	=	it.subscribe("head_mount_kinect/rgb/image_color", 1, kinectRGBImageCB);
	image_transport::Subscriber sub_kinect_depth_ 	= 	it.subscribe("head_mount_kinect/depth_registered/image_rect", 	1, kinectDepthImageCB);

	// Create a ROS publisher for the output point cloud
	pub_kinect_rgb_   								=  	it.advertise("head_mount_kinect/rgb/image_color/filtered",             	1);
	pub_kinect_depth_ 								=  	it.advertise("head_mount_kinect/depth_registered/image_rect/filtered", 	1);

	//Service
	ros::ServiceServer 	srv_image_crop_ 			= 	nh.advertiseService("apc/opencv_filter/set_image_crop" , setImageCropLimitsCB);
	ros::ServiceServer srv_enablePCL 				= 	nh.advertiseService("apc/opencv_filter/enableFilter",enableFilter);

	ROS_INFO("Node Started");

	// Update rate TODO use flag instead
	int rate = 10;
	ros::Rate r(rate);

	pubhlish_cropped_image = true;

	 roi_x 			= 	0;
	 roi_y 			= 	0;
	 roi_width		=	640;
	 roi_height		=	480;

	// Main loop
//	while (nh.ok())
//	{
//		ros::spinOnce();		// check for incoming message
//		r.sleep();
//	}

	 ros::spin();

	 ros::shutdown();
	 return 0;

}

