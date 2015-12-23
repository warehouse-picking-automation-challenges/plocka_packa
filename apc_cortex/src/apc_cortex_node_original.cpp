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
 * apc_cortex_node.cpp
 *
 *  Created on: Mar 8, 2015
 *      Author: Sven Cremer
 */


#include <apc_cortex/apc_cortex.h>

//#include <apc_baxter/apc_baxter_commander.h>
#include <apc_robot/apc_robot_moveit.h>
#include <apc_robot/apc_robot_grippers.h>

#include <apc_object_detection/GetObject.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>


/* Functions:
 * - shelf detection
 * - load approach strategy for a bin (joint postions: neutral -> approach -> close -> leave -> neutral)
 * - find object with ASUS scanning
 * - visual servoing
 * - etc.
 */

/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, demonstrates use of BaxterMoveit class
 ***********************************************************************************************************************/

int main(int argc, char **argv)
{
	ros::init(argc, argv, "apc_core_node",ros::init_options::AnonymousName);
	ros::NodeHandle nh;

	// start a ROS spinning thread
	ros::AsyncSpinner spinner(1);
	spinner.start();

	std::string tmp;

	// Objects
	//BaxterCommander	 baxter_commander;				// FIXME: crashes when baxter is not on? Want to be able to run this without Baxter.
	RobotMoveit 	 baxter_moveit;
	Gripper 	     baxter_grippers;
	ApcCortex        cortex;

	/* -------------------------- */
	/* Test ShelfProperties class */
	/* -------------------------- */

	geometry_msgs::Point shelf_origin;  // This is in the torso frame
	shelf_origin.x = 0.78-0.06;  		// forward
	shelf_origin.y = 0.15-0.08;			// sideways to the left
	shelf_origin.z = 0.01;				// height

	double pokeDist = 0.15+0.07;

	cortex.setShelfOrigin( shelf_origin );
	//cortex.setShelfOrigin( geometry_msgs::Point(1.0,0.5,-0.3) );   <- doesn't work?

	for(int i=1;i<=12;i++)
	{
		geometry_msgs::Point p;
		if(cortex.getBinOrigin(i, p))
		{
			std::cout<<"Bin "<<i<<": ("<<p.x<<","<<p.y<<","<<p.z<<")\n";
		}

	}

	/* -------------------------- */
	/*    Test Baxter class       */
	/* -------------------------- */

	  //~ if(!baxter_commander.enableRobot())
	  //~ {
		  //~ ROS_ERROR("main: Robot could not be enabled!");
	  //~ }
	  //~ {
		  //~ ROS_INFO("Enabled robot!");
	  //~ }

	  baxter_grippers.close(Gripper::BOTH);

	  sleep(0.1);

	  geometry_msgs::Pose target;				// End-effector position (/right_gripper)
	  // (r,p,y) = (180,-90,0)
//	  target.orientation.x = 0.70711 ;
//	  target.orientation.y = 0.0 ;
//	  target.orientation.z = 0.70711 ;
//	  target.orientation.w = 0.0 ;

	  // (r,p,y) = (180,-88,0)
	  target.orientation.x = 0.72537 ;
	  target.orientation.y = 0.0 ;
	  target.orientation.z = 0.68835 ;
	  target.orientation.w = 0.0 ;

	  /*
	  for(int i=1;i<=12;i++)
	  {
		  geometry_msgs::Point p;
		  if(cortex.getBinOrigin(i, p))
		  {
			  std::cout<<"Bin "<<i<<": ("<<p.x<<","<<p.y<<","<<p.z<<")\n";

			  target.position = p;
			  baxter_moveit.executeCarteGoal(BaxterMoveit::RIGHT, target);

			  sleep(0.1);

			  //ROS_INFO("Press [enter] to continue:");
			  //std::getline(std::cin, tmp);
		  }
	  }
	  */

	  // Moving to bins

	  int bin_atempts[12]   = {0,0,0,0,0,0,0,0,0,0,0,0};
	  int bin_successes[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	  int continuous_order[] = {1,2,3,6,5,4,7,8,9,12,11,10,11,12,9,8,7,4,5,6,3,2}; 		//{5,6,9,8,7,4,5,6,3,2,1,4};
	  int length = (sizeof(continuous_order)/sizeof(*continuous_order));
	  int counter = 4;
	  int N = counter+length * 1;

//	  geometry_msgs::Point start;
//	  cortex.getBinOrigin(continuous_order[counter], start);
//	  target.position = start;
//	  baxter_moveit.executeCarteGoal(BaxterMoveit::RIGHT, target);
//	  sleep(0.1);

  	  std::vector<double> joint_start;					// Joint position
	  joint_start.push_back( 0.45712 );
	  joint_start.push_back( 0.73362 );
	  joint_start.push_back( 2.47815 );
	  joint_start.push_back( 2.34047 );
	  joint_start.push_back( -0.2323 );
	  joint_start.push_back( -1.4691 );
	  joint_start.push_back( 0.43756 );
//	  baxter_moveit.executeJointGoal(BaxterMoveit::RIGHT, joint_start);

	  while(counter<N)
	  {
		  int i = continuous_order[counter % length]; // Bin number


		  geometry_msgs::Point p;

		  if(cortex.getBinOrigin(i, p))
		  {
			  std::cout<<"Bin "<<i<<": ("<<p.x<<","<<p.y<<","<<p.z<<")\n";

			  bin_atempts[i-1]++;

			  if( cortex.moveToBin(i) )
			  {
//				  if( cortex.moveInsideBin(pokeDist) )
//				  {
//					  if( cortex.moveInsideBin(-pokeDist) )
//					  {
						  bin_successes[i-1] += 1;
//					  }
//					  else
//					  {
//						  ROS_ERROR("Stuck inside bin! Press [enter] to continue:");
////						  std::getline(std::cin, tmp);
//					  }
//				  }
			  }

			  //sleep(0.1);

			  counter++;
		  }
	  }

	  std::cout<<"## Success rate ## \n";
	  for(int i=0;i<12;i++)
	  {
		  std::cout<<"Bin "<<i+1<<": " << bin_successes[i] << "/" << bin_atempts[i] << "\n";
	  }

/*
	  // Scanning bins - TODO: 10-12 doesnt work

	  cortex.moveToBin(5);

	  for(int i=12;i<=12;i++)
	  {

//		  std::vector<double> joint_start;					// Joint position
//		  joint_start.push_back( -1.4638    );				// Need to start here otherwise cartesian path fails
//		  joint_start.push_back( -0.0763155 );
//		  joint_start.push_back( 2.97362    );
//		  joint_start.push_back( 1.95429    );
//		  joint_start.push_back( 2.32513    );
//		  joint_start.push_back( -1.26515   );
//		  joint_start.push_back( 0.234699   );
//		  baxter_moveit.executeJointGoal(BaxterMoveit::RIGHT, joint_start);

		  std::cout<<"Bin "<<i<<"\n";
		  cortex.scanForItem(i);
		  sleep(1.0);

	  }
*/



	  //	  if(counter>10)
	  //	  {
	  //	    ROS_ERROR("Failed to get object location");
	  //	    return 1;
	  //	  }
	  //
	  //	  int i = 9;
	  //	  if( cortex.moveToBin(i) )
	  //	  {
	  //		  baxter_grippers.open(Gripper::RIGHT);
	  //
	  //		  if( cortex.moveInsideBin(pokeDist) )
	  //		  {
	  //			  baxter_grippers.close(Gripper::RIGHT);
	  //			  if( cortex.moveInsideBin(-pokeDist) )
	  //			  {
	  //				  ROS_INFO("Success!");
	  //			  }
	  //			  else
	  //			  {
	  //				  ROS_ERROR("Stuck inside bin! Press [enter] to continue:");
	  //				  std::getline(std::cin, tmp);
	  //			  }
	  //		  }
	  //	  }

	  //	  cortex.scanForItem(9);


	  // Bin 9, elemers

//	  if(!baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9"))
//	  {
//		    ROS_ERROR("Failed to gmove to right_obj9");
//		    return 1;
//	  }
	  /*
	  if(true)
	  {
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9");

//	  ros::ServiceClient ser_get_obj_ = nh.serviceClient<apc_object_detection::GetObject>("/apc_object/get_object");
//
//	  apc_object_detection::GetObject getObj;
//	  getObj.request.obj_name = "f436484731295f16350a64465c02e811";
//
//	  int counter = 0;
//	  while (!ser_get_obj_.call(getObj) && counter < 10)
//	  {
//	    ROS_INFO("Trying to get object location ...");
//	    sleep(3);
//	    counter++;
//	  }
	  //getObj.response.
	  sleep(3.0);
	  ROS_INFO("Found object f436484731295f16350a64465c02e811");

	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_a0");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_a1");

	  baxter_grippers.open(Gripper::RIGHT);

	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_a2");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_a3");

	  baxter_grippers.close(Gripper::RIGHT);

	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_l1");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_l2");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_d1");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_d2");

	  baxter_grippers.open(Gripper::RIGHT);

	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_obj9_d1");
	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_r1");


	  baxter_grippers.close(Gripper::RIGHT);
	  }

	  // Sticky, bin 7

	  baxter_moveit.moveToPose(BaxterMoveit::RIGHT,"right_7_s1");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_n1");

	  sleep(3.0);
	  ROS_INFO("Found object f436484731295f16350a64465c033eac");

	  baxter_grippers.open(Gripper::LEFT);

	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_a1");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_a2");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_a3");



	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_l1");
	  baxter_grippers.close(Gripper::LEFT);

	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_l2");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_l3");

	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_d1");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_d2");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_d3");

	  baxter_grippers.open(Gripper::LEFT);
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_d2");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_d1");
	  baxter_moveit.moveToPose(BaxterMoveit::LEFT,"left_7_n1");

//	  for(int i=1;i<=12;i++)
//	  {
//		  std::cout<<"Bin "<<i<<"\n";
//		  cortex.pickObject(i, "");
//		  sleep(1.0);
//	  }

*/

	/*
	 *
	 * STATE MACHINE
	 *
	 *
	 */

	  // SERVICES:

	  // apc_json: getWorkOrder
	  // apc_ork_manager: "/apc/startORK", "/apc/stopORK"
	  // apc_object_detection: "/apc/object_recognition/get_object_pose"





	ROS_INFO("Done!");
	ros::shutdown();

	return 0;

}



