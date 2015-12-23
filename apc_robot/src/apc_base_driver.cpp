/*!
 * 	\name 	   ArmsCartesian
 *  \brief     Controls arm movement through cartesian control
 *  \details   Using a PR2 action client, cartesian arm coordinates are sent through public functions
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Apr 19, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include "apc_robot/apc_base_driver.h"

//! ROS node initialization
APCBaseDriver::APCBaseDriver(ros::NodeHandle &nh)
{
	nh_ = nh;
	// Initialize publishers
	cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/base_controller/command", 1);		// TODO cmd_vel:=base_controller/command

	// Initialize services
	srv_driveOdom_ = nh.advertiseService("/apc/robot/base/drive", &APCBaseDriver::driveOdomCB, this);
	srv_turnOdom_  = nh.advertiseService("/apc/robot/base/turn", &APCBaseDriver::turnOdomCB, this);
	srv_driveToLocation_ = nh.advertiseService("/apc/robot/base/location", &APCBaseDriver::driveToLocationCB, this);
	srv_driveToXY_  = nh.advertiseService("/apc/robot/base/xy", &APCBaseDriver::driveToXYCB, this);
	srv_driveToW_  = nh.advertiseService("/apc/robot/base/w", &APCBaseDriver::driveToWCB, this);
	srv_baseParameter_ = nh.advertiseService("/apc/robot/base/parameter", &APCBaseDriver::baseParameterCB, this);

	vel_lin = 0.10;		// meters per second
	vel_rot = 0.15;		// radians per second

	robotIsMoving = false;
	robot_frame = "base_footprint";
	world_frame = "map";

	waitTime = 1.0;
	goalTol = 0.03;			// TODO avoid hardcoding
	goalTolRot = 0.03;

	//wait for the listener to get the first message
	if(! listener_.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(waitTime)) )
	{
		ROS_WARN("APCBaseDriver: Timed out waiting for robot_frame->world_frame transform!");
	}

	//record the starting transform from the world to the robot frame
	try
	{
		listener_.lookupTransform(robot_frame, world_frame, ros::Time(0), current_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("APCBaseDriver: Could not initialize starting transform!");
	}

	start_transform = current_transform;
	goal_transform = current_transform;

	ROS_INFO("APCBaseDriver initialized!");
}

bool APCBaseDriver::baseParameterCB(apc_msgs::SetBaseParameter::Request &req, apc_msgs::SetBaseParameter::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}
	else
	{
		vel_lin = req.vel_lin;		// meters per second
		vel_rot = req.vel_rot;		// radians per second
		waitTime = req.waitTime;
		goalTol = req.goalTol;		// TODO avoid hardcoding
		goalTolRot = req.goalTolR;
		res.success=true;
	}
		
	return true;
}

bool APCBaseDriver::driveOdomCB(apc_msgs::SetBaseDistance::Request &req, apc_msgs::SetBaseDistance::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}

	APCBaseDriver::MoveType t;
	switch(req.direction)
	{
	case 0:
		t=APCBaseDriver::FORWARD;
		break;
	case 1:
		t=APCBaseDriver::BACKWARD;
		break;
	case 2:
		t=APCBaseDriver::LEFT;
		break;
	case 3:
		t=APCBaseDriver::RIGHT;
		break;
	default:
		ROS_ERROR("APCBaseDriver: Received faulty MoveType in driveOdomCB");
		res.success=false;
		return true;
	}

	res.success=driveOdom(req.x, t);

	return true;
}
bool APCBaseDriver::turnOdomCB(apc_msgs::SetBaseRotation::Request &req, apc_msgs::SetBaseRotation::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}
	
	ROS_INFO("Start Turning...");
	res.success=turnOdom(req.w, req.clockwise);
	ROS_INFO("...Turning End");

	return true;
}
bool APCBaseDriver::driveToLocationCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}

	res.success=driveToLocation(req.x, req.y, req.w);

	return true;
}
bool APCBaseDriver::driveToXYCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}

	res.success=driveToXY(req.x, req.y);

	return true;
}
bool APCBaseDriver::driveToWCB(apc_msgs::SetBaseLocation::Request &req, apc_msgs::SetBaseLocation::Response &res)
{
	if(robotIsMoving)
	{
		ROS_ERROR("APCBaseDriver: robot is still moving");
		res.success=false;
		return true;
	}

	res.success=driveToW(req.w);

	return true;
}

bool APCBaseDriver::driveOdom(double distance, APCBaseDriver::MoveType t)
{
	//wait for the listener to get the first message
	if(! listener_.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(waitTime)) )
	{
		ROS_WARN("APCBaseDriver: Timed out waiting for robot_frame->world_frame transform!");
	}

	//record the starting transform from the world to the robot frame
	try
	{
		listener_.lookupTransform(robot_frame, world_frame, ros::Time(0), start_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("APCBaseDriver: Could not initialize starting transform!");
		return false;
	}

	//switch direction if negative
	if(distance<0)
	{
		switch(t)
		{
		case APCBaseDriver::FORWARD:
			t=APCBaseDriver::BACKWARD;
			break;
		case APCBaseDriver::BACKWARD:
			t=APCBaseDriver::FORWARD;
			break;
		case APCBaseDriver::LEFT:
			t=APCBaseDriver::RIGHT;
			break;
		case APCBaseDriver::RIGHT:
			t=APCBaseDriver::LEFT;
			break;
		default:
			ROS_ERROR("APCBaseDriver: Received faulty MoveType");
			return false;
		}
		ROS_INFO("APCBaseDriver: Switched direction since distance is negative.");
	}

	//create twist command
	base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
	switch(t)
	{
	case APCBaseDriver::FORWARD:
		base_cmd.linear.x = vel_lin;
		break;
	case APCBaseDriver::BACKWARD:
		base_cmd.linear.x = -vel_lin;;
		break;
	case APCBaseDriver::LEFT:
		base_cmd.linear.y = vel_lin;
		break;
	case APCBaseDriver::RIGHT:
		base_cmd.linear.y = -vel_lin;;
		break;
	default:
		ROS_ERROR("Received faulty MoveType");
		return false;
	}

	//loop until goal is reached
	ros::Rate rate(10.0);
	robotIsMoving = true;
	bool done = false;
	while (!done && nh_.ok())
	{
		//send the drive command
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		//get the current transform
		try
		{
			listener_.lookupTransform(robot_frame, world_frame,
					ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}
		//see how far we've traveled
		tf::Transform relative_transform = start_transform.inverse() * current_transform;
		double dist_moved = relative_transform.getOrigin().length();

		if(dist_moved > fabs(distance)) done = true;
	}
	robotIsMoving = false;
	if (done) return true;
	return false;
}

bool APCBaseDriver::turnOdom(double radians, bool clockwise)
{
	while(radians < 0) radians += 2*M_PI;
	while(radians > 2*M_PI) radians -= 2*M_PI;

	//wait for the listener to get the first message
	if(! listener_.waitForTransform(robot_frame, world_frame, ros::Time(0), ros::Duration(waitTime)) )
	{
		ROS_WARN("APCBaseDriver: Timed out waiting for robot_frame->world_frame transform!");
	}

	//record the starting transform from the world to the robot frame
	try
	{
		listener_.lookupTransform(robot_frame, world_frame, ros::Time(0), start_transform);
	}
	catch (tf::TransformException ex)
	{
		ROS_ERROR("%s",ex.what());
		ROS_ERROR("APCBaseDriver: Could not initialize starting transform!");
		return false;
	}

	//create twist command
	base_cmd.linear.x = base_cmd.linear.y = 0.0;
	base_cmd.angular.z = vel_rot;
	if (clockwise) base_cmd.angular.z = -base_cmd.angular.z;

	//the axis we want to be rotating by
	tf::Vector3 desired_turn_axis(0,0,1);
	if (!clockwise) desired_turn_axis = -desired_turn_axis;

	ros::Rate rate(10.0);
	robotIsMoving = true;
	bool done = false;
	while (!done && nh_.ok())
	{
		//send the drive command
		cmd_vel_pub_.publish(base_cmd);
		rate.sleep();
		//get the current transform
		try
		{
			listener_.lookupTransform(robot_frame, "odom_combined",
					ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}
		tf::Transform relative_transform =
				start_transform.inverse() * current_transform;
		tf::Vector3 actual_turn_axis =
				relative_transform.getRotation().getAxis();
		double angle_turned = relative_transform.getRotation().getAngle();
		if ( fabs(angle_turned) < 1.0e-2) continue;

		if ( actual_turn_axis.dot( desired_turn_axis ) < 0 )
			angle_turned = 2 * M_PI - angle_turned;

		if (angle_turned > radians) done = true;
	}
	robotIsMoving = false;
	if (done) return true;
	return false;
}


bool APCBaseDriver::driveToLocation(double x, double y, double w)
{
	goal_transform.setOrigin(tf::Vector3(-x,-y,0));
	//goal_transform.setRotation(current_transform.getRotation());		// TODO make sure this is up-to-date!
	goal_transform.setRotation(tf::createQuaternionFromRPY(0,0,w));	// TODO use w

	//loop until goal is reached
	ros::Rate rate(10.0);
	robotIsMoving = true;
	bool done = false;
	
	while (!done && nh_.ok())
	{
		//get the current transform
		try
		{
			listener_.lookupTransform(robot_frame, world_frame,
					ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}

		//see how far is left to travel
		tf::Transform relative_transform = current_transform.inverse() * goal_transform;
		double l = relative_transform.getOrigin().length();
		double dx = relative_transform.getOrigin().getX();
		double dy = relative_transform.getOrigin().getY();
		double dw = tf::getYaw(relative_transform.getRotation());


		//create twist command
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

		double goalTolRot = 0.02;
		if(dw > goalTolRot)
			base_cmd.angular.z = -vel_rot;
		else if (dw < -goalTolRot)
			base_cmd.angular.z = vel_rot;

		// APPROACH 1
		double vx = -(dx/l)*vel_lin;			// FIXME this makes PR2 oscillate in place
		double vy = -(dy/l)*vel_lin;

		//ROS_INFO("l=(%f), dx,dy=(%f, %f), vx,vy=(%f, %f)",l,dx,dy,vx,vy);
		
		base_cmd.linear.x = vx;

		base_cmd.linear.y = vy;
		ROS_INFO_STREAM("l = "<<l<<" dw= "<<dw);


		if( l>goalTol || fabs(dw)>goalTolRot)
		{
			//send the drive command
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
		}
		else
		{
			done = true;
		}
	}
	robotIsMoving = false;
	if (done) return true;
	return false;
}

bool APCBaseDriver::driveToXY(double x, double y)
{
	goal_transform.setOrigin(tf::Vector3(-x,-y,0));
	goal_transform.setRotation(current_transform.getRotation());		// TODO make sure this is up-to-date!

	//loop until goal is reached
	ros::Rate rate(10.0);
	robotIsMoving = true;
	bool done = false;

	while (!done && nh_.ok())
	{
		//get the current transform
		try
		{
			listener_.lookupTransform(robot_frame, world_frame,
					ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}

		//see how far is left to travel
		/* FIXME
		 *
		 * 	x and y given in /odom_origin
		 * If base is already rotated, we need to transform those?
		//tf::Vector3 fixedGoal( -x, -y, 0 );
		//goal_transform.setOrigin( tf::Vector3( current_transform.getOrigin() + tf::quatRotate(current_transform.getRotation(), fixedGoal) ) );
		//goal_transform.setRotation( current_transform.getRotation() );
		*/
		// 1) This computes the difference in the world frame
		tf::Transform relative_transform = current_transform.inverse() * goal_transform;
		double l = relative_transform.getOrigin().length();
		double dx = relative_transform.getOrigin().getX();
		double dy = relative_transform.getOrigin().getY();
		/*
		// 2) Convert the difference or velocity back into robot frame ???
		tf::Vector3 tmp = tf::quatRotate(current_transform.getRotation(), tf::Vector3(dx,dy,0) );
		dx = tmp.getX();
		dy = tmp.getX();
		l = sqrt(dx*dx +dy*dy);
		*/
//		double dx = x - current_transform.getOrigin().getX();
//		double dy = y - current_transform.getOrigin().getY();
//		double l =  sqrt(dx*dx+dy*dy);

		//create twist command
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

		double vx = -(dx/l)*vel_lin;			// FIXME this makes PR2 oscillate in place
		double vy = -(dy/l)*vel_lin;

		base_cmd.linear.x = vx;
		base_cmd.linear.y = vy;

		//ROS_INFO("l=(%f), dx,dy=(%f, %f), vx,vy=(%f, %f)",l,dx,dy,vx,vy);
		//ROS_INFO_STREAM("Goal_transform: " << goal_transform);
		//ROS_INFO_STREAM("Current Transform: " << current_transform);

		if( l>goalTol)
		{
			//send the drive command
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
		}
		else
		{
			done = true;
		}
	}
	robotIsMoving = false;
	if (done) return true;
	return false;
}

bool APCBaseDriver::driveToW(double w)
{
	goal_transform.setOrigin(current_transform.getOrigin());
	goal_transform.setRotation(tf::createQuaternionFromRPY(0,0,w));

	//loop until goal is reached
	ros::Rate rate(10.0);
	robotIsMoving = true;
	bool done = false;

	while (!done && nh_.ok())
	{
		//get the current transform
		try
		{
			listener_.lookupTransform(robot_frame, world_frame,
					ros::Time(0), current_transform);
		}
		catch (tf::TransformException ex)
		{
			ROS_ERROR("%s",ex.what());
			break;
		}

		//see how far is left to travel
		tf::Transform relative_transform = current_transform.inverse() * goal_transform;

		double dw = tf::getYaw(relative_transform.getRotation());

		//create twist command
		base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;

		if(dw > goalTolRot)
			base_cmd.angular.z = -vel_rot;
		else if (dw < -goalTolRot)
			base_cmd.angular.z = vel_rot;

		//ROS_INFO_STREAM(" dw= "<<dw);

		if(fabs(dw)>goalTolRot)
		{
			//send the drive command
			cmd_vel_pub_.publish(base_cmd);
			rate.sleep();
		}
		else
		{
			done = true;
		}
	}
	robotIsMoving = false;
	if (done) return true;
	return false;
}

int main(int argc, char** argv)
{
	//init the ROS node
	ros::init(argc, argv, "robot_driver");
	ros::NodeHandle nh;

	APCBaseDriver driver(nh);


//	sleep(2);
//	driver.driveOdom(0.3, APCBaseDriver::FORWARD);
//	sleep(2);
//	driver.driveOdom(0.3, APCBaseDriver::BACKWARD);

//	sleep(2);
//	driver.driveOdom(0.3, APCBaseDriver::LEFT);
//	sleep(2);
//	driver.driveOdom(0.3, APCBaseDriver::RIGHT);
//
//	sleep(2);
//	driver.turnOdom(0.7,false);
//	sleep(2);
//	driver.turnOdom(0.7,true);

//	int rate = 10;
//	ros::Rate r(rate);
//
//	// Main loop
//	while (nh.ok())
//	{
//		ros::spinOnce();		// check for incoming messages
//
//		r.sleep();
//	}

	ros::spin();

	return 0;
}

