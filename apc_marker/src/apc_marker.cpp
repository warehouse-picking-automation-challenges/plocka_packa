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

#include "ros/ros.h"
#include "ar_track_alvar/AlvarMarker.h"
#include "ar_track_alvar/AlvarMarkers.h"
#include "geometry_msgs/PoseStamped.h"
#include "apc_msgs/ToolPos.h"

struct Marker{

	int marker_id;
	geometry_msgs::PoseStamped marker_pose;
};

std::vector<Marker> MarkerPositions;

void marker_CB(const ar_track_alvar::AlvarMarkers::ConstPtr& msg)		// TODO check for valid data before storing
{
	MarkerPositions.clear();
	MarkerPositions.resize(msg->markers.size());
	for(int i=0; i<msg->markers.size(); i++)
	{
		//Create the pose marker messages
		geometry_msgs::PoseStamped marker_position = msg->markers[i].pose;

		Marker marker;
		marker.marker_id = msg->markers[i].id;
		marker.marker_pose = marker_position;

		//ROS_INFO_STREAM( marker.marker_pose);
		MarkerPositions.push_back(marker);
	}
}


bool tool_posCB(apc_msgs::ToolPos::Request  &req, apc_msgs::ToolPos::Response &res)
{
	for(int i=0; i<MarkerPositions.size(); i++)
	{
		if(MarkerPositions.at(i).marker_id == req.marker_num)
		{
			if( MarkerPositions.at(i).marker_pose.pose.orientation.x != 0 &&
				MarkerPositions.at(i).marker_pose.pose.orientation.y != 0 &&
				MarkerPositions.at(i).marker_pose.pose.orientation.z != 0 &&
				MarkerPositions.at(i).marker_pose.pose.orientation.w != 0    )	// TODO Needs a better fix
			{
				ROS_INFO_STREAM(MarkerPositions.at(i).marker_id);
				ROS_INFO_STREAM(MarkerPositions.at(i).marker_pose);
				res.marker_position = MarkerPositions.at(i).marker_pose;
				res.success = true;
				return true;
			}
		}
	}
	res.success = false;
	return true;
}



int main(int argc, char **argv)
{

	ros::init(argc, argv, "marker_listener");
	ros::NodeHandle n;
	ros::Subscriber marker_sub = n.subscribe("ar_pose_marker", 10, marker_CB);
	ros::ServiceServer marker_srv = n.advertiseService("/apc/get_tool_pos", tool_posCB);
	//marker_pose.resize(5);


	ros::spin();

	return 0;
}
