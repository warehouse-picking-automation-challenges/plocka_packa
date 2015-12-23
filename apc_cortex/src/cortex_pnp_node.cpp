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
 * cortex_pnp_node.cpp
 *
 *  Created on: Sept. 23, 2015
 *      Author: Rommel Alonzo
 */


#include <apc_cortex/cortex_pick_and_place.h>
/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point
 ***********************************************************************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "cortex_pnp_node");
	ros::NodeHandle nh;

	Cortex cortex;

	int rate = 10;
	ros::Rate r(rate);

	bool done = false;

	/* State machine testing */
	while( ros::ok() && !done)
	{
		ros::spinOnce();
		done = cortex.runSM(Cortex::START,Cortex::DONE);
		//done = cortex.runSM(Cortex::PREPARE_ARMS_2,Cortex::DONE);
		r.sleep();
		//std::cout<<".";
	}

	ROS_INFO("Done!");
	ros::shutdown();

	return 0;

}


