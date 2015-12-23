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
 * apc_object_detection.cpp
 *
 *  Created on: Mar 6, 2015
 *      Author: Sven Cremer
 */


#include <apc_object_detection/apc_object_detection.h>

ApcObjectDectection::ApcObjectDectection()
{

	confidence_threshold = 0.90;

//	// Image Transport
//	image_transport::ImageTransport it(nh);

	// Initialize subscriber
	sub_recognized_obj_ = nh.subscribe<object_recognition_msgs::RecognizedObjectArray>("recognized_object_array", 1, &ApcObjectDectection::recognizedObjCB, this);

	// Kinect Image Subscribers
//	sub_kinect_rgb_ 	= 	it.subscribe("head_mount_kinect/rgb/image_color", 				1, &ApcObjectDectection::kinectRGBImageCB, 		this);
//	sub_kinect_depth_ 	= 	it.subscribe("head_mount_kinect/depth_registered/image_rect", 	1, &ApcObjectDectection::kinectDepthImageCB, 	this);

//	// Image Publishers
//	pub_kinect_rgb_   	=  	it.advertise("head_mount_kinect/rgb/image_color/filtered",             	1);
//	pub_kinect_depth_ 	=  	it.advertise("head_mount_kinect/depth_registered/image_rect/filtered", 	1);

//	// Image Cropped
//	cropped = false;

	// Initialize service
	srv_get_obj_pose_ 	= nh.advertiseService("apc/object_recognition/get_object_pose", &ApcObjectDectection::getObjectPoseCB, 		this);
//	srv_image_crop_ 	= nh.advertiseService("apc/object_recognition/set_image_crop" , &ApcObjectDectection::setImageCropLimitsCB, this);

	client_setLimits_ = nh.serviceClient<apc_msgs::SetLimits>("apc/passthrough_filter/setLimits");
	client_enablePCL_ = nh.serviceClient<apc_msgs::SetBool>("apc/passthrough_filter/enablePCL");
	client_getBinInfo_ = nh.serviceClient<apc_msgs::GetBinInfo>("/apc/shelf/get_bin_info");

	// Initialize publishers
	pub_marker_ = nh.advertise<visualization_msgs::Marker>("apc_object/target_marker", 1);

	ac = new actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>("recognized_objects",true);
	x_min = 0.0;
	y_min = 0.0;
	z_min = 0.0;
	x_max = 0.0;
	y_max = 0.0;
	z_max = 0.0;

	result = "Empty";

	// Check if object IDs have been loaded
	 std::string names[] = {"oreo_mega_stuf",							// TODO read this from a file instead
					        "champion_copper_plus_spark_plug",
					        "expo_dry_erase_board_eraser",
					        "kong_duck_dog_toy",
					        "genuine_joe_plastic_stir_sticks",
					        "munchkin_white_hot_duck_bath_toy",
					        "crayola_64_ct",
					        "mommys_helper_outlet_plugs",
					        "sharpie_accent_tank_style_highlighters",
					        "kong_air_dog_squeakair_tennis_ball",
					        "stanley_66_052",
					        "safety_works_safety_glasses",
					        "dr_browns_bottle_brush",
					        "laugh_out_loud_joke_book",
					        "cheezit_big_original",
					        "paper_mate_12_count_mirado_black_warrior",
					        "feline_greenies_dental_treats",
					        "elmers_washable_no_run_school_glue",
					        "mead_index_cards",
					        "rolodex_jumbo_pencil_cup",
					        "first_years_take_and_toss_straw_cup",
					        "highland_6539_self_stick_notes",
					        "mark_twain_huckleberry_finn",
					        "kyjen_squeakin_eggs_plush_puppies",
					        "kong_sitting_frog_dog_toy"};

	std::vector<std::string> object_names(names,names+25);				// FIXME do not hardcode # of items

	for(std::vector<std::string>::iterator it = object_names.begin(); it != object_names.end(); ++it)
	{
		Object tmp;
		tmp.name = (*it);

		std::string path = "/apc/objects/"+(*it);
		if( !(nh.getParam(path, tmp.key)) )
		{
			ROS_ERROR("ApcObjectDectection: parameter not found for %s",tmp.name.c_str());
			tmp.key  = "unknown";
		}
		objects.push_back(tmp);
	}

	f = boost::bind(&ApcObjectDectection::callback, this, _1, _2);
    dynamicReconfigureServer.setCallback(f);
}

ApcObjectDectection::~ApcObjectDectection()
{

}

void ApcObjectDectection::callback(apc_pcl::BinConfig &config, uint32_t level)
{
  ROS_INFO("Reconfigure Request: %f %f %f %f %f %f",
            config.min_dx,
			config.min_dy,
            config.min_dz,
            config.max_dx,
            config.max_dy,
			config.max_dz );

	min_dx = config.min_dx;
	min_dy = config.min_dy;
	min_dz = config.min_dz;
	max_dx = config.max_dx;
	max_dy = config.max_dy;
	max_dz = config.max_dz;
}

void ApcObjectDectection::getObject()
{

//	tf::StampedTransform transform;
//
//	try{
//		listener.lookupTransform("base", "camera_rgb_optical_frame",
//				ros::Time(0), transform);client_getBinInfo_
//	}
//	catch (tf::TransformException &ex) {
//		ROS_ERROR("%s",ex.what());
//		ros::Duration(1.0).sleep();
//		continue;
//	}
//
//	object_recognition_msgs::ObjectRecognitionGoal goal;
//
//	goal.use_roi = true;
//	goal.filter_limits = [x_min, x_max, y_min, y_max, z_min, z_max];
//
//	ac->sendGoal(goal);
//
//	ros::Duration(0.1).sleep(); // wait for ros
//
//	double max_execution_time = 20;
//	bool finished_within_time = ac->waitForResult(ros::Duration(max_execution_time));
//	if (!finished_within_time)
//	{
//		ac->cancelGoal();
//		ROS_WARN("ApcObjectDectection::getObject -> Timed out achieving action goal!");
//	}
//	else
//	{
//		actionlib::SimpleClientGoalState state = ac->getState();
//		if(state == actionlib::SimpleClientGoalState::SUCCEEDED)
//		{
//			ROS_INFO("Action finished: %s",state.toString().c_str());
//			// STORE RESULT
//			object_recognition_msgs::RecognizedObjectArrayConstPtr result = ac->getResult();
//		}
//		else
//		{
//			ROS_INFO("Action failed (?): %s",state.toString().c_str());
//		}
//	}

}

std::string ApcObjectDectection::getObjectName(std::string key)
{

	for(std::vector<Object>::iterator it = objects.begin(); it != objects.end(); ++it)
	{
		if((*it).key == key)
			return (*it).name;
	}
	ROS_ERROR("ApcObjectDectection: could not find object name for the key %s",key.c_str());
	return "unknown";
}

void ApcObjectDectection::recognizedObjCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg)
{

	// Loop through recognized objects inside message
	for(std::vector<object_recognition_msgs::RecognizedObject>::const_iterator itMsg = msg->objects.begin(); itMsg != msg->objects.end(); ++itMsg)
	{
		bool object_already_detect = false;
		bool replace_data = false;

		RecObject recievedObj;
		recievedObj.key 		= (*itMsg).type.key;
		recievedObj.name		= getObjectName(recievedObj.key);
		recievedObj.confidence 	= (*itMsg).confidence;
		recievedObj.location	= (*itMsg).pose;

		if(recievedObj.confidence > confidence_threshold)
		{
			// Loop through recognized objects already stored
			for(std::vector<RecObject>::iterator it = recognized_objects.begin(); it != recognized_objects.end(); ++it)
			{
				if( (*it).key == recievedObj.key )
				{
					object_already_detect = true;
					//Only replace data if msg confidence is higher						TODO check if locations are approx the same
					if( (*it).confidence <= recievedObj.confidence )
					{
						(*it).confidence = (*itMsg).confidence;
						(*it).location   = (*itMsg).pose;

						//curlOperation((*it).key);
						(*it).name		 = result;
						//FIXME stop loop
					}
				}
			}
			if(object_already_detect==false)
			{
				recognized_objects.push_back(recievedObj);
			}
		}
	}

	last_update = ros::Time::now();

}

bool ApcObjectDectection::getObjectPoseCB(apc_msgs::GetObjectPose::Request &req, apc_msgs::GetObjectPose::Response &res)
{
	res.found_obj = false;

	// Find res.obj_name inside table
	for(std::vector<RecObject>::iterator it = recognized_objects.begin(); it != recognized_objects.end(); ++it)
	{
		int i = 0;
		if( (*it).name == req.obj_name )
		{
			res.found_obj = true;
			res.pose      = (*it).location.pose.pose;
		}
		i++;
	}

	return true;
}

//bool ApcObjectDectection::setImageCropLimitsCB(apc_msgs::SetImageCrop::Request &req, apc_msgs::SetImageCrop::Response &res)
//{
//	double 	focalLength_x, 	focalLength_y;
//	double 	bin_x_min, 		bin_x_max, 		bin_y_min, 	bin_y_max, 	bin_z_min, 	bin_z_max;
//	int   	pix_x_min, 		pix_x_max, 		pix_y_min, 	pix_y_max;
//
//	ROS_INFO("Set Crop Service Called! :D");
//
//	// Calculate Focal Lengths
//	focalLength_x 	= 	640 / (2 * (tan((57/2) * (M_PI/180))));
//	focalLength_y 	= 	480 / (2 * (tan((43/2) * (M_PI/180))));
//	ROS_INFO("Focal length: %d, %d", (int)focalLength_x, (int)focalLength_x);
//	// Get Bin Details
//
//	double dy 		= 	0.1;
//	double dz 		= 	0.1;
//
//	double w2 		= 	req.width / 2;
//	double h2 		= 	req.height / 2;
//
//	bin_x_min  		= 	req.center.x - 0.1;
//	bin_y_min  		= 	req.center.y - (w2 + dy);
//	bin_z_min  		= 	req.center.z - (h2 + dz);
//
//	bin_x_max  		= 	req.center.x + 0.5;
//	bin_y_max  		= 	req.center.y + (w2 + dy);
//	bin_z_max  		= 	req.center.z + (h2 + dz);
//
//	ROS_INFO("The Bin min coordinate is: %f, %f, %f  and The Bin max coordinate is: %f, %f, %f", bin_x_min, bin_y_min, bin_z_min, bin_x_max, bin_y_max, bin_z_max);
//
//	// Pixel Calculation
//	pix_x_min 		= 	((bin_y_min * focalLength_x)/bin_x_max);
//	pix_y_min 		= 	((bin_z_min * focalLength_y)/bin_x_max);
//	pix_x_max 		= 	((bin_y_max * focalLength_x)/bin_x_max);
//	pix_y_max 		= 	((bin_z_max * focalLength_y)/bin_x_max);
//
//	// Sanity check
//	if((pix_x_min < 0) | (pix_y_min < 0) | ((pix_x_min + abs(pix_x_max - pix_x_min)) > 640) | ((pix_y_min + abs(pix_y_max - pix_y_min))>480))
//		return true;
//
//	// Update Filter
//	roi_x 	  		= 	pix_x_min;
//	roi_y 	  		= 	pix_y_min;
//	roi_width 		= 	abs(pix_x_max - pix_x_min);
//	roi_height		= 	abs(pix_y_max - pix_y_min);
//
//	ROS_INFO("The Filter is: %d, %d, %d, %d", roi_x, roi_y, roi_width, roi_height);
//	// Enable Crop
//	cropped 		= 	req.crop_image;
//
//	res.success 	= 	true;
//
//
//	return true;
//
//}

//void ApcObjectDectection::kinectRGBImageCB(const sensor_msgs::ImageConstPtr& msg)
//{
//
//	if(!cropped)
//		return;
//
//	cv_bridge::CvImagePtr cv_rgb_ptr;
//	try
//	{
//		//Always copy, returning a mutable CvImage
//		//OpenCV expects color images to use BGR channel order.
//		cv_rgb_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
//	}
//	catch (cv_bridge::Exception& e)
//	{
//		//if there is an error during conversion, display it
//		ROS_ERROR("cv_bridge RGB_Image exception: %s", e.what());
//		return;
//	}
//
//	cv::Rect myROI(roi_x, roi_y, roi_width, roi_height);
//	cv_rgb_ptr->image = cv_rgb_ptr->image(myROI);
//
//
//
//	pub_kinect_rgb_.publish(cv_rgb_ptr->toImageMsg());
//}
//
//void ApcObjectDectection::kinectDepthImageCB(const sensor_msgs::ImageConstPtr& msg)
//{
//	if(!cropped)
//		return;
//
//	cv_bridge::CvImagePtr cv_depth_ptr;
//	try
//	{
//		//Always copy, returning a mutable CvImage
//		//OpenCV expects color images to use BGR channel order.
//		cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
//	}
//	catch (cv_bridge::Exception& e)
//	{
//		//if there is an error during conversion, display it
//		ROS_ERROR("cv_bridge Depth_Image exception: %s", e.what());
//		return;
//	}
//
//	cv::Rect myROI(roi_x, roi_y, roi_width, roi_height);
//	cv_depth_ptr->image = cv_depth_ptr->image(myROI);
//
//	pub_kinect_depth_.publish(cv_depth_ptr->toImageMsg());
//}

void ApcObjectDectection::print()
{

	int i = 1;
	for(std::vector<RecObject>::const_iterator it = recognized_objects.begin(); it != recognized_objects.end(); ++it)
	{
		std::cout<<"----------\n";
		std::cout<<"Object "<<i<<"\n";
		std::cout<<"   key: "<<(*it).key<<"\n";
		std::cout<<"   confidence: "<<(*it).confidence<<"\n";
		std::cout<<"   x: "<<(*it).location.pose.pose.position.x<<"\n";
		std::cout<<"   y: "<<(*it).location.pose.pose.position.y<<"\n";
		std::cout<<"   z: "<<(*it).location.pose.pose.position.z<<"\n";
		std::cout<<"Name: "<<(*it).name.c_str()<<"\n";
		i++;

	}
	//std::cout<<"===================\n";

}

void ApcObjectDectection::publishMarker()
{
	// Publish object location
	int i = 0;
	for(std::vector<RecObject>::iterator it = recognized_objects.begin(); it != recognized_objects.end(); ++it)
	{
		visualization_msgs::Marker marker;
	
		// Set the namespace and id for this marker.  This serves to create a unique ID
		// Any marker sent with the same namespace and id will overwrite the old one
		marker.ns = "apc_object";
		marker.id = i;
	
		// Set the marker type.
		marker.type = visualization_msgs::Marker::SPHERE;
	
		// Set the marker action.  Options are ADD, DELETE, DELETEALL
		marker.action = visualization_msgs::Marker::ADD;
	
		// Set the scale of the marker (in meters)
		marker.scale.x = 0.05;
		marker.scale.y = 0.05;
		marker.scale.z = 0.05;
	
		// Set the color and transparency
		switch(i)
		{
			case 0:
				marker.color.r = 1.0f;
				marker.color.g = 0.0f;
				marker.color.b = 0.0f;
				break;
			case 1:
				marker.color.r = 0.0f;
				marker.color.g = 1.0f;
				marker.color.b = 0.0f;
				break;
			default:
				marker.color.r = 0.0f;
				marker.color.g = 0.0f;
				marker.color.b = 1.0f;
				break;
		}
	    marker.color.a = 0.5;
			
	    // Set position
	    marker.pose.position.x = (*it).location.pose.pose.position.x;
	    marker.pose.position.y = (*it).location.pose.pose.position.y;
	    marker.pose.position.z = (*it).location.pose.pose.position.z;
	    marker.pose.orientation.x = 0.0;
	    marker.pose.orientation.y = 0.0;
	    marker.pose.orientation.z = 0.0;
	    marker.pose.orientation.w = 1.0;
		
		// Publish marker
		marker.header.frame_id = "/camera_rgb_optical_frame";
		marker.header.stamp = ros::Time::now();
		pub_marker_.publish(marker);
		
		i++;
	}

    // Publish text
	i = 0;
	for(std::vector<RecObject>::const_iterator it = recognized_objects.begin(); it != recognized_objects.end(); ++it)
	{
		visualization_msgs::Marker marker;
	
		marker.ns = "apc_text";
		marker.id = i;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.action = visualization_msgs::Marker::ADD;
		marker.scale.z = 0.05;
		marker.color.r = 1.0f;
		marker.color.g = 1.0f;
		marker.color.b = 1.0f;
	    marker.color.a = 0.9;
	    marker.pose.position.x = (*it).location.pose.pose.position.x;
	    marker.pose.position.y = (*it).location.pose.pose.position.y;
	    marker.pose.position.z = (*it).location.pose.pose.position.z;
		marker.text = (*it).key;	// TODO: Display confidence also
		marker.header.frame_id = "/camera_rgb_optical_frame";
		marker.header.stamp = ros::Time::now();
		
		pub_marker_.publish(marker);
		i++;
	}

}

bool ApcObjectDectection::filterPCL(int bin)
{
	apc_msgs::SetLimits  msg_setLimits;
	apc_msgs::SetBool msg_enablePCL;
	apc_msgs::GetBinInfo msg_getBinInfo;

	msg_getBinInfo.request.bin_number = bin;
	msg_getBinInfo.request.frame	  = "/odom_origin";

	if (!client_getBinInfo_.call(msg_getBinInfo))
	{
		ROS_ERROR("ApcObjectDectection: Failed to call service /apc/shelf/get_bin_info");
		return false;
	}

	double w2 = msg_getBinInfo.response.width / 2;
	double h2 = msg_getBinInfo.response.height / 2;

	msg_setLimits.request.x_min  = msg_getBinInfo.response.center.x -       min_dx ;
	msg_setLimits.request.y_min  = msg_getBinInfo.response.center.y - (w2 + min_dy);
	msg_setLimits.request.z_min  = msg_getBinInfo.response.center.z - (h2 + min_dz);

	msg_setLimits.request.x_max  = msg_getBinInfo.response.center.x +       max_dx ;
	msg_setLimits.request.y_max  = msg_getBinInfo.response.center.y + (w2 + max_dy);
	msg_setLimits.request.z_max  = msg_getBinInfo.response.center.z + (h2 + max_dz);

	if (!client_setLimits_.call(msg_setLimits))
	{
		ROS_ERROR("ApcObjectDectection: Failed to call service apc/passthrough_filter/setLimits");
		return false;
	}

	msg_enablePCL.request.variable = true;

	if (!client_enablePCL_.call(msg_enablePCL))
	{
		ROS_ERROR("ApcObjectDectection: Failed to call service apc/passthrough_filter/enablePCL");
		return false;
	}

	ROS_INFO("PCL Filtering Done!");
	// TODO disable PCL after object is detected

	return true;

}
