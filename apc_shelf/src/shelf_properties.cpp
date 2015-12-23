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
 * shelf_properties.cpp
 *
 *  Created on: Mar 10, 2015
 *      Author: Sven Cremer
 */

#include <apc_shelf/shelf_properties.h>


ShelfProperties::ShelfProperties()
{

	//std::string fname;
	//std::string path = ros::package::getPath("apc_shelf") + "/conf/" + fname;

	if( loadShelfParameters() )
	{
		param_loaded = true;
	}
	else
	{
		param_loaded = false;
	}

	// Initialize services
	srv_get_bin_origin_ 					= 		nh.advertiseService("/apc/shelf/get_bin_origin", &ShelfProperties::getBinOriginCB, this);
	//srv_set_shelf_origin_					= 		nh.advertiseService("/apc/shelf/set_shelf_origin", &ShelfProperties::setShelfOriginCB, this);
	//srv_load_shelf_properties_			= 		nh.advertiseService("/apc_shelf/load_shelf_properties", &ShelfProperties::getLoadShelfPropertiesCB, this);
	srv_get_bin_info_ 						= 		nh.advertiseService("/apc/shelf/get_bin_info", &ShelfProperties::getBinInfoCB, this);

	robot_frame								=		"base_footprint";
	shelf_frame								=		"shelf_origin";

	// Wait for the listener to get the first message
	if(! listener.waitForTransform(shelf_frame,robot_frame, ros::Time(0), ros::Duration(1.0)) )
		ROS_ERROR("ShelfProperties: Could not find base_footprint->shelf_origin transform!");

	updateShelfOrigin();

	// Publish Marker for Bin Center
	pub_marker_bin_center_ 					= 		nh.advertise<visualization_msgs::MarkerArray>("/apc/shelf/marker_bin_center", 1);


	ROS_INFO("ShelfProperties::ShelfProperties() - done initializing!");
}
ShelfProperties::~ShelfProperties()
{

}

void ShelfProperties::updateShelfOrigin()
{
	try
	{
		listener.lookupTransform(shelf_frame,robot_frame, ros::Time(0), tf_robot_to_shelf);
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("ShelfProperties::updateShelfOrigin() - %s",ex.what());
	}

	shelf_origin.x = tf_robot_to_shelf.getOrigin().getX();
	shelf_origin.y = tf_robot_to_shelf.getOrigin().getY();
	shelf_origin.z = tf_robot_to_shelf.getOrigin().getZ();
}

bool ShelfProperties::loadShelfParameters()
{
	// Load properties from parameter server

	Bin tmp;
	std::string corner[] = {"/origin", "/a", "/b", "/c"};

	for(int i = 1; i <= 12; i++)
	{
		//Get bin Number
		std::string bin = "/bin" + boost::lexical_cast<std::string>(i);

		for (int c = 0; c < 4; c++)
		{
			std::string bin_y = bin + corner[c] + "/y";
			std::string bin_z = bin + corner[c] + "/z";

			//std::cout<<"Name: " << bin_x << "\n";

			switch(c)
			{
			case(0):
				{
					if(!(nh.getParam(bin_y, tmp.o.y))){ROS_ERROR("Parameter Not found.");}
					if(!(nh.getParam(bin_z, tmp.o.z))){ROS_ERROR("Parameter Not found.");}
					//std::cout<<tmp.o.x << "  " << tmp.o.z << std::endl;
					break;
				}
			case(1):
				{
					if(!(nh.getParam(bin_y, tmp.a.y))){ROS_ERROR("Parameter Not found.");}
					if(!(nh.getParam(bin_z, tmp.a.z))){ROS_ERROR("Parameter Not found.");}
					//std::cout<<tmp.a.x << "  " << tmp.a.z << std::endl;
					break;
				}
			case(2):
				{
					if(!(nh.getParam(bin_y, tmp.b.y))){ROS_ERROR("Parameter Not found.");}
					if(!(nh.getParam(bin_z, tmp.b.z))){ROS_ERROR("Parameter Not found.");}
					//std::cout<<tmp.b.x << "  " << tmp.b.z << std::endl;
					break;
				}
			case(3):
				{
					if(!(nh.getParam(bin_y, tmp.c.y))){ROS_ERROR("Parameter Not found.");}
					if(!(nh.getParam(bin_z, tmp.c.z))){ROS_ERROR("Parameter Not found.");}
					//std::cout<<tmp.c.x << "  " << tmp.c.z << std::endl;
					break;
				}
			}
		}
		tmp.number = i;
		tmp.height = fabs(tmp.a.z - tmp.o.z);
		tmp.width  = fabs(tmp.b.y - tmp.a.y);

		bin_vec.push_back(tmp);

		//std::cout<<tmp.a.x <<std::endl;
	}

	nBins = bin_vec.size(); // Check if this is 12
	//std::cout<<"Number of bins: "<<nBins<<std::endl;

	return true;
}

void ShelfProperties::print()
{
	int i = 0;
	for(std::vector<Bin>::const_iterator it = bin_vec.begin(); it != bin_vec.end(); ++it)
	{
		//std::cout<<"Bin "<<i+1<<": ("<<(*it).o.x<<","<<(*it).o.y<<","<<(*it).o.z<<")\n";
		std::cout<<"Bin "<<i+1<<": (y,z)=("<<(*it).o.y<<",  "<<(*it).o.z<<")\n";
		i++;
	}

}

//bool ShelfProperties::getShelfTF(std::string frame, geometry_msgs::Point& result)
//{
//
//	if(frame == shelf_frame)
//	{
//		result.x = 0;
//		result.y = 0;
//		result.z = 0;
//		return true;
//	}
//
//	tf::StampedTransform tf_frame_shelf;
//	try
//	{
//		listener.lookupTransform(frame, shelf_frame, ros::Time(0), tf_frame_shelf);
//		result.x = tf_frame_shelf.getOrigin().getX();
//		result.y = tf_frame_shelf.getOrigin().getY();
//		result.z = tf_frame_shelf.getOrigin().getZ();
//		return true;
//	}
//	catch (tf::TransformException &ex)
//	{
//		ROS_ERROR("ShelfProperties::updateShelfOrigin() - %s",ex.what());
//	}
//
//	return false;
//}

bool ShelfProperties::getTF(std::string target_frame, geometry_msgs::Point pin, geometry_msgs::Point& pout)
{
	geometry_msgs::PointStamped input;
	input.header.frame_id = shelf_frame;
//	input.header.stamp = ros::Time::now();		// makes TF fail
	input.point = pin;
	geometry_msgs::PointStamped output;

	try
	{
		listener.transformPoint(target_frame, input, output);
		pout = output.point;
		return true;
	}
	catch (tf::TransformException &ex)
	{
		ROS_ERROR("ShelfProperties::getTF() - %s",ex.what());
	}

	return false;
}

bool ShelfProperties::getBinOriginCB(apc_msgs::GetBinOrigin::Request &req, apc_msgs::GetBinOrigin::Response &res)
{
	geometry_msgs::Point pout;

	if(!param_loaded)
	{
		ROS_ERROR("ShelfProperties::getBinOriginCB -> Parameters not loaded.");
		return false;
	}

	for(std::vector<Bin>::const_iterator it = bin_vec.begin(); it != bin_vec.end(); ++it)
	{
		int i = 0;
		if( (*it).number == req.bin_number)
		{
			geometry_msgs::Point pin = (*it).o;
			if(! getTF(req.frame, pin, pout) )
			{
				ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
				return false;
			}
			res.point = pout;
			return true;
		}
		i++;
	}
	ROS_ERROR("ShelfProperties::getBinOriginCB -> Did not find bin number.");
	return false;
}
/*
bool ShelfProperties::setShelfOriginCB(apc_msgs::SetShelfOrigin::Request &req, apc_msgs::SetShelfOrigin::Response &res)
{
	shelf_origin = req.point;
	return true;
}
*/

bool ShelfProperties::getBinInfoCB(apc_msgs::GetBinInfo::Request &req, apc_msgs::GetBinInfo::Response &res)
{
	geometry_msgs::Point pout, oout, aout, bout, cout;

	if(!param_loaded)
	{
		ROS_ERROR("ShelfProperties::getBinOriginCB -> Parameters not loaded.");
		return false;
	}

	for(std::vector<Bin>::const_iterator it = bin_vec.begin(); it != bin_vec.end(); ++it)
	{
		int i = 0;
		if( (*it).number == req.bin_number)
		{
			double w = (*it).width;
			double h = (*it).height;

			geometry_msgs::Point center = (*it).o;
			geometry_msgs::Point o 		= (*it).o;
			geometry_msgs::Point a 		= (*it).a;
			geometry_msgs::Point b 		= (*it).b;
			geometry_msgs::Point c 		= (*it).c;

			center.y -= w/2;
			center.z += h/2;
			if(! getTF(req.frame, center, pout) )
			{
				ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
				return false;
			}
			if(! getTF(req.frame, o, oout) )
				{
					ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
					return false;
				}
			if(! getTF(req.frame, a, aout) )
				{
					ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
					return false;
				}
			if(! getTF(req.frame, b, bout) )
				{
					ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
					return false;
				}
			if(! getTF(req.frame, c, cout) )
				{
					ROS_ERROR("ShelfProperties::getBinOriginCB -> Failed to get transform from %s to %s", req.frame.c_str(), shelf_frame.c_str());
					return false;
				}

			res.center = pout;
			res.o	   = oout;
			res.a	   = aout;
			res.b	   = bout;
			res.c	   = cout;
			res.height = h;			// TODO verify
			res.width  = w;
			return true;
		}
		i++;
	}
	ROS_ERROR("ShelfProperties::getBinOriginCB -> Did not find bin number.");
	return false;
}
/*
bool ShelfProperties::loadShelfPropertiesCB(apc_msgs::XXX::Request &req, apc_msgs::XXX::Response &res);
{

	return true;
}
*/


bool ShelfProperties::publishBinCenterMarker(std::string target_frame)
{
	visualization_msgs::MarkerArray markerArray;
	apc_msgs::GetBinInfo msg_getBinInfo;
    geometry_msgs::Point center;
    geometry_msgs::Point p;

	// 12 bins present and 12 markers in the array denoting center points
	markerArray.markers.resize(12);

	for ( int i = 0; i < 12; i++)
	{
	    // Marker identifiers
		markerArray.markers[i].header.frame_id 		=      target_frame;
	    markerArray.markers[i].header.stamp 		=      ros::Time();
	    markerArray.markers[i].ns 					=      "shelf_properties";
	    markerArray.markers[i].id 					=      i;

	    // Marker shape
	    markerArray.markers[i].type 				=      visualization_msgs::Marker::SPHERE;
	    markerArray.markers[i].action 				=      visualization_msgs::Marker::ADD;

	    // Get bin details
	    int bin										= 		i+1;
		std::string frame	  						= 		target_frame;

		for(std::vector<Bin>::const_iterator it = bin_vec.begin(); it != bin_vec.end(); ++it)
		{
			if( (*it).number == bin)
			{

				double w							= 		(*it).width;
				double h 							= 		(*it).height;
				geometry_msgs::Point center 		= 		(*it).o;
				center.y 					 	 	-= 	 	w/2;
				center.z 					 	 	+= 	 	h/2;
				getTF(frame, center, p);
			}
		}

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
	    markerArray.markers[i].color.r 				=      1.0;
	    markerArray.markers[i].color.g 				=      0.0;
	    markerArray.markers[i].color.b 				=      0.0;
	}

	pub_marker_bin_center_.publish(markerArray);

}

/***********************************************************************************************************************
Program entry point
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// init the ROS node
	ros::init(argc, argv, "shelf_properties");
	ros::NodeHandle nh;

	// create objects
	ShelfProperties selfPropertiesObject;

	selfPropertiesObject.print();

	int rate = 10;
	ros::Rate r(rate);

	// Main loop
	while (nh.ok())
	{
		//selfPropertiesObject.updateShelfOrigin();
		selfPropertiesObject.publishBinCenterMarker("head_mount_kinect_rgb_optical_frame");
		ros::spinOnce();		// check for incoming messages

		r.sleep();
	}

	ros::shutdown();

	return 0;
}
