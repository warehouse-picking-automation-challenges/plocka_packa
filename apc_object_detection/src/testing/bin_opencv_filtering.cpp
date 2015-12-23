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
 * bin_opencv_filtering.cpp
 *
 *  Created on: May 22, 2015
 *      Author: Sumit Kumar Das
 */

#include <ros/ros.h>
#include <apc_msgs/SetImageCrop.h>
#include <apc_msgs/GetBinInfo.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_object_detection_filtering_node");
	ros::NodeHandle nh;

	ros::ServiceClient clientSetImageCrop = nh.serviceClient<apc_msgs::SetImageCrop>("/apc/opencv_filter/set_image_crop");
	ros::ServiceClient client_getBinInfo_ = nh.serviceClient<apc_msgs::GetBinInfo>("/apc/shelf/get_bin_info");

//	ApcObjectDectection obj;

	apc_msgs::SetImageCrop image_crop;
	apc_msgs::GetBinInfo msg_getBinInfo;
	std::string camera_topic = "/head_mount_kinect_rgb_optical_frame";

	int rate = 10;
	ros::Rate r(rate);

	int counter = 0;
	int bin = 0;

	// Main loop
	while (nh.ok())
	{
		//obj.print();

		//obj.publishMarker();

		ros::spinOnce();		// check for incoming messages

		if( (counter%30) == 0) // Every 3 seconds, filter a different bin
		{
			ROS_INFO("Filtering bin #%i",bin+1);

			msg_getBinInfo.request.bin_number = (bin+1);
			msg_getBinInfo.request.frame	  = "/head_mount_kinect_rgb_optical_frame";
//			msg_getBinInfo.request.frame	  = "/torso_lift_link";

			if (!client_getBinInfo_.call(msg_getBinInfo))
			{
				ROS_ERROR("ApcObjectDectection: Failed to call service /apc/shelf/get_bin_info");
				return false;
			}

			ROS_INFO("The center of bin %d is: %f, %f, %f, %f, %f", (bin+1), msg_getBinInfo.response.center.x, msg_getBinInfo.response.center.y, msg_getBinInfo.response.center.z, msg_getBinInfo.response.width, msg_getBinInfo.response.height );

			image_crop.request.center 		= 	msg_getBinInfo.response.center;
			image_crop.request.o 			= 	msg_getBinInfo.response.o;
			image_crop.request.a 			= 	msg_getBinInfo.response.a;
			image_crop.request.b 			= 	msg_getBinInfo.response.b;
			image_crop.request.c 			= 	msg_getBinInfo.response.c;

			image_crop.request.width  		= 	msg_getBinInfo.response.width;
			image_crop.request.height  		= 	msg_getBinInfo.response.height;
			image_crop.request.crop_image	= 	1;

			ROS_INFO("Calling");
			if(!(clientSetImageCrop.call(image_crop)))
			{
				ROS_ERROR("Failed to call service apc/object_recognition/set_image_crop");
			}

			bin++;
			bin %= 12;
		}

		counter++;

		r.sleep();
	}

	return 0;
}




