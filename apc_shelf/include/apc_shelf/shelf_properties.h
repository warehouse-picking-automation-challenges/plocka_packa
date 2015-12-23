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
 * shelf_properties.h
 *
 *  Created on: Mar 10, 2015
 *      Author: Sven Cremer
 *
 *
 *  The bin origin is located at the lower left corner
 *      z
 *      ^
 *      |	a	b
 *      |
 *      |	o   c
 *      |
 *      L----------> -y
 *
 *      Note that all passed values are offset by the shelf_origin, e.g. x = bin.x + shelf_origin.x
 */

#ifndef APC_SHELF_SHELF_PROPERTIES_H_
#define APC_SHELF_SHELF_PROPERTIES_H_

#include <ros/ros.h>
#include <ros/package.h>
#include "std_msgs/String.h"
#include <string.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/transform_listener.h>

#include <apc_msgs/SetShelfOrigin.h>
#include <apc_msgs/GetBinOrigin.h>
#include <apc_msgs/GetBinInfo.h>

struct Bin {
  int number;
  geometry_msgs::Point o;		// Lower right (origin)
  geometry_msgs::Point a;		// Upper left
  geometry_msgs::Point b;		// Upper right
  geometry_msgs::Point c;		// Lower right
  double width;					// Computed from points
  double height;
} ;

class ShelfProperties
{
private:
	ros::NodeHandle nh;

	tf::TransformListener listener;					// TODO make this a pointer so that nh is garranteed to init before
	tf::StampedTransform tf_robot_to_shelf;
	std::string robot_frame;
	std::string shelf_frame;

	ros::Time last_update;

	// Publish Marker
	ros::Publisher pub_marker_bin_center_;

	geometry_msgs::Point shelf_origin;

	int nBins;
	std::vector<Bin> bin_vec;

	ros::ServiceServer srv_get_bin_origin_;
	ros::ServiceServer srv_get_bin_info_;
	ros::ServiceServer srv_set_shelf_origin_;
//	ros::ServiceServer srv_load_shelf_properties_; 	// TODO implement

	bool getBinOriginCB(apc_msgs::GetBinOrigin::Request &req, apc_msgs::GetBinOrigin::Response &res);				// Create msgs, srvs
	bool getBinInfoCB(apc_msgs::GetBinInfo::Request &req, apc_msgs::GetBinInfo::Response &res);
	//bool setShelfOriginCB(apc_msgs::SetShelfOrigin::Request &req, apc_msgs::SetShelfOrigin::Response &res);
	//bool loadShelfPropertiesCB(apc_msgs::XXX::Request &req, apc_msgs::XXX::Response &res);

	bool param_loaded;
	bool loadShelfParameters();

public:

	ShelfProperties();
	~ShelfProperties();

	void print();
	void updateShelfOrigin();
	//bool getShelfTF(std::string frame, geometry_msgs::Point& result);
	bool getTF(std::string target_frame, geometry_msgs::Point pin, geometry_msgs::Point& pout);
	bool publishBinCenterMarker(std::string target_frame);

};


#endif /* APC_SHELF_SHELF_PROPERTIES_H_ */
