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
 * kinect_tf_publisher.cpp
 *
 *  Created on: Dec 19, 2014
 *      Author: Sven Cremer
 */

#include <ros/ros.h>
#include <gazebo_msgs/GetModelState.h>
#include <tf/transform_broadcaster.h>


int main(int argc, char **argv)
{

	ros::init(argc, argv, "kinect_tf_publisher_node");
	ros::NodeHandle n;

	ros::ServiceClient getModelState_client = n.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");

	gazebo_msgs::GetModelState getmodelstate;

	getmodelstate.request.model_name			= "kinect_1";
	getmodelstate.request.relative_entity_name	= "baxter::base";


	tf::TransformBroadcaster broadcaster;

	int Hz = 50;
	ros::Rate loopRate(Hz);
	int count = 0;


	while (n.ok())
	{


		if(!getModelState_client.call(getmodelstate))
		{
			ROS_ERROR("Failed to get model state");
		}
		else
		{

			tf::Transform tf_kinect;

			tf_kinect.setOrigin(tf::Vector3(getmodelstate.response.pose.position.x,
											getmodelstate.response.pose.position.y,
											getmodelstate.response.pose.position.z));

			tf_kinect.setRotation(tf::Quaternion(0.5,-0.5,0.5,-0.5));	// RPY = 0, 90, -90

			/*
			Get model state returns this instead:
				x: 4.16362733088e-07
				y: -2.39460619187e-06
				z: -0.0144914886547
				w: 0.999894992862
			so the code below doesn't work:
			*/
//			tf_kinect.setRotation(tf::Quaternion(getmodelstate.response.pose.orientation.x,		// FIXME for some reason this gives the wrong orientation
//												 getmodelstate.response.pose.orientation.y,
//												 getmodelstate.response.pose.orientation.z,
//												 getmodelstate.response.pose.orientation.w));

		    broadcaster.sendTransform(tf::StampedTransform(tf_kinect, ros::Time::now(), "base", "base_link"));

		}


		loopRate.sleep();
		++count;
	}

	return 0;

}
