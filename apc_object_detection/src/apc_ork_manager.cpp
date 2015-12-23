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

#include <ros/ros.h>
#include <stdlib.h>
#include <string>

#include <apc_msgs/SetBin.h>
#include <apc_msgs/Empty.h>

bool ORK_running;

bool start_ORK_CB(apc_msgs::SetBin::Request  &req, apc_msgs::SetBin::Response &res)
{
	if(ORK_running)
	{
		ROS_WARN("ORK detection is still running!");
	}

	std::string bin = "bin_" + boost::lexical_cast<std::string>(req.number) + "_detection.ros.ork";
	std::string command = ("rosrun object_recognition_core detection -c `rospack find apc_object_detection`/config/" + bin + " &");

	int ok = system(command.c_str());

	// TODO check if all is OK
	if(ok < 0)
	{
		ROS_INFO("Failed to execute \n  %s \n",command.c_str());
	}
	else
	{
		ROS_INFO("ORK detection node started!");
		ORK_running = true;
	}

	return true;
}

bool stop_ORK_CB(apc_msgs::Empty::Request  &req, apc_msgs::Empty::Response &res)
{
	if(!ORK_running)
	{
		ROS_WARN("ORK detection is not running!");
	}

	std::string command = ("rosnode kill /object_recognition_server");

	int ok = system(command.c_str());

	// TODO check if all is OK
	if(ok < 0)
	{
		ROS_INFO("Failed to execute \n  %s \n",command.c_str());
	}
	else
	{
		ROS_INFO("ORK detection node stoped!");
		ORK_running = false;
	}

	return true;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_set_bin_object_detection_node");
	ros::NodeHandle nh;
	ORK_running = false;
	ros::ServiceServer s1 = nh.advertiseService("/apc/startORK", start_ORK_CB);
	ros::ServiceServer s2 = nh.advertiseService("/apc/stopORK", stop_ORK_CB);
	ros::spin();
	return 0;
}
