/*!
 * 	\name 	   BaseMovement
 *  \brief     Allows for base movement using action server through zone definition
 *  \details   Allows for base movement using action server through zone definition
 *  \author    Rommel Alonzo
 *  \version   1.0
 *  \date      May 13, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include "apc_cortex/BaseMovement.h"


BaseMovement::BaseMovement()
{
	// TODO Auto-generated constructor stub

	//sub = nh_.subscribe("/robot_pose_ekf/odom_combined",1,&BaseMovement::odometry_CB,this);			// TODO remove
	//move_base_cmd_sub = n.subscribe("/apc/robot/base/cmd",1,&BaseMovement::move_base_cmd_CB,this);	// for debugging
	drive_base = nh_.serviceClient<apc_msgs::SetBaseLocation>("/apc/robot/base/location");
	turn_base = nh_.serviceClient<apc_msgs::SetBaseRotation>("/apc/robot/base/turn");

	drive_to_xy = nh_.serviceClient<apc_msgs::SetBaseLocation>("/apc/robot/base/xy");
	drive_to_w = nh_.serviceClient<apc_msgs::SetBaseLocation>("/apc/robot/base/w");

	/**Read in zone definitions**/
	loadZoneParameters();
	ROS_INFO_STREAM(zones.size());

	this->current_zone = "center";

}

BaseMovement::~BaseMovement() 
{
	// TODO Auto-generated destructor stub
}

void BaseMovement::loadZoneParameters()
{
	int NUMBER_OF_ZONES = 6;

	for(int i=0;i<NUMBER_OF_ZONES;i++)
	{
		Zone zone_tmp;
		std::string zone = "/zone" +  boost::lexical_cast<std::string>(i);
		std::string zone_name = zone + "/name";
		std::string zone_x = zone + "/x";
		std::string zone_y = zone + "/y";
		std::string zone_w = zone + "/w";
		if(!(n.getParam(zone_name, zone_tmp.name))){ROS_ERROR("Parameter Not found.");}
		if(!(n.getParam(zone_x, zone_tmp.x))){ROS_ERROR("Parameter Not found.");}
		if(!(n.getParam(zone_y, zone_tmp.y))){ROS_ERROR("Parameter Not found.");}
		if(!(n.getParam(zone_w, zone_tmp.w))){ROS_ERROR("Parameter Not found.");}

		zones.push_back(zone_tmp);
		//ROS_INFO_STREAM("\tZone: " << zone_tmp.name << "(" << zone_tmp.x << "," << zone_tmp.y << ")");
	}
}

//double* BaseMovement::requestNextPose(std::string zone_name)
//{
//
//	double *next_zone_diff = new double[3];
//	Zone next_zone = findZone(zone_name);
//
//	ROS_INFO_STREAM("Current(" << this->current_x << "," << this->current_y << "," << this->current_w << ")");
//	ROS_INFO_STREAM("Next Zone: " << next_zone.name << "(" << next_zone.x << "," << next_zone.y << "," << next_zone.w << ")");
//
//	next_zone_diff[0] = next_zone.x;
//	next_zone_diff[1] = next_zone.y;
//	next_zone_diff[2] = next_zone.w;
//
//	//Apply difference
//	ROS_INFO_STREAM("Diff(" << next_zone_diff[0] << "," << next_zone_diff[1] << "," << next_zone_diff[2] << ")");
//
//	//If succeeded, return true
//	return next_zone_diff;
//}

Zone BaseMovement::findZone(std::string next_zone_name)
{
	for(int i=0;i<this->zones.size();i++)
	{
		Zone temp = (Zone)this->zones[i];
		if(temp.name == next_zone_name){
			return temp;
		}
	}
}

//void BaseMovement::odometry_CB(geometry_msgs::PoseWithCovarianceStamped::ConstPtr msg)
//{
//	this->current_x = msg->pose.pose.position.x;
//	this->current_y = msg->pose.pose.position.y;
//	double roll, pitch,yaw;
//	tf::Quaternion q_t(msg->pose.pose.orientation.x,msg->pose.pose.orientation.y,msg->pose.pose.orientation.z,msg->pose.pose.orientation.w);
//	tf::Matrix3x3(q_t).getRPY(roll,pitch,yaw);
//	this->current_x = yaw;
//}

/*void BaseMovement::move_base_cmd_CB(std_msgs::String::ConstPtr msg)
{
	if(msg->data == "center")
	{
		if(move_base_cmd("center")) ROS_INFO("Moved to center\n");
	}
	else
	{
		if(this->current_zone != "center")
		{
			if(move_base_cmd("center")) ROS_INFO_STREAM("Moved to center");
		}

		if(move_base_cmd(msg->data)) ROS_INFO_STREAM("Moved to desired\n");//<<msg->data);
	}
}*/

bool BaseMovement::move(std::string zone)
{
	ROS_INFO_STREAM("BaseMovement: moving to " << zone);
	apc_msgs::SetBaseLocation pose;
	apc_msgs::SetBaseRotation rot;

	//ROS_INFO_STREAM("BaseMovement: Zone Lookup");
	Zone tmp = findZone(zone);
	//ROS_INFO_STREAM("BaseMovement: Found zone");


	pose.request.x = tmp.x;
	pose.request.y = tmp.y;
    pose.request.w = tmp.w;

    ROS_INFO_STREAM("X: "<< tmp.x << ", Y: " << tmp.y << ", W: " << tmp.w);

	//ROS_INFO_STREAM("BaseMovement: Moving Translation");

    //ROS_INFO_STREAM("BaseMovement: Moving Rotation");
    if(zone == "tool" || zone == "bin")		// FIXME
    {
		drive_to_xy.call(pose);
		drive_to_w.call(pose);
    }
    else
    {
    	drive_to_w.call(pose);
		drive_to_xy.call(pose);
    }

	this->current_zone = zone;

	return true;

}

bool BaseMovement::motionComplete()
{

	return true;
}

bool BaseMovement::cancelMotion()
{

	return true;
}

