/*!
 * 	\name 	   ArmsCartesian
 *  \brief     Controls arm movement through cartesian control
 *  \details   Using a PR2 action client, cartesian arm coordinates are sent through public functions
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      May 21, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */


#include "apc_robot/apc_arms_cartesian.h"



ArmsCartesian::ArmsCartesian()
{
	ROS_INFO("ArmsCartesian::ArmsCartesian - Initializing");

	 rCartPub = nh.advertise<geometry_msgs::PoseStamped>("/r_cart/command_pose", 1);
	 lCartPub = nh.advertise<geometry_msgs::PoseStamped>("/l_cart/command_pose", 1);
	 rgainPub = nh.advertise<std_msgs::Float64MultiArray>("/r_cart/gains", 1);
	 lgainPub = nh.advertise<std_msgs::Float64MultiArray>("/l_cart/gains", 1);
	 rposturePub = nh.advertise<std_msgs::Float64MultiArray>("/r_cart/command_posture", 1);
	 lposturePub = nh.advertise<std_msgs::Float64MultiArray>("/l_cart/command_posture", 1);

//	srvClient_setGain 	= nh.advertiseService("/apc/arm/set_gains",&ArmsCartesian::setGains, this);

	leftMotionInProgress = false;
	rightMotionInProgress = false;

	loadGainParameters();

	loadPosture(RIGHT,elbowupr);
	loadPosture(LEFT, elbowupl);

	timeOut= 20;
	
	robot_frame = "torso_lift_link";

	ROS_INFO("ArmsCartesian::ArmsCartesian - Done!");
}

ArmsCartesian::~ArmsCartesian()
{

}

void ArmsCartesian::loadGainParameters(){
	int NUMBER_OF_GAINS = 4;

	for(int i=0;i<NUMBER_OF_GAINS;i++)
	{
		GainValue gain_tmp;

		std::string zone = "/gain_" +  boost::lexical_cast<std::string>(i+1);

		if(!(nh.getParam(zone + "/name", gain_tmp.name))){ROS_ERROR("Parameter Not found.");}
		if(!(nh.getParam(zone + "/t_p", gain_tmp.t_p))) {ROS_ERROR("Parameter Not found.");}
		if(!(nh.getParam(zone + "/t_d", gain_tmp.t_d))) {ROS_ERROR("Parameter Not found.");}
		if(!(nh.getParam(zone + "/r_p", gain_tmp.r_p))) {ROS_ERROR("Parameter Not found.");}
		if(!(nh.getParam(zone + "/r_d", gain_tmp.r_d))) {ROS_ERROR("Parameter Not found.");}

		gains.push_back(gain_tmp);
	}
}

bool ArmsCartesian::getCurrentPose(ArmsCartesian::WhichArm a, geometry_msgs::PoseStamped& result)
{
	if( !updateState() )
		return false;

	switch(a)
	{
	case LEFT:
		result = leftControllerState.x;
		break;
	case RIGHT:
		result = rightControllerState.x;
		break;
	default:
		ROS_WARN("grasp: invalid ApcCortex::WhichArm");
		return false;
		break;
	}

	return true;
}

bool ArmsCartesian::waitForMotion(double max_duration)
{

	ROS_INFO("ArmsCartesian: Waiting for motions to complete...");

	ros::Time now = ros::Time::now();
	ros::Time stop = ros::Time::now() + ros::Duration(max_duration);

	while(!motionComplete() && now<stop)
	{
		ros::Duration(0.2).sleep();
		now = ros::Time::now();
	}

	if(!motionComplete() )
	{
		ROS_ERROR("ArmsCartesian::moveToPose - motion timed out!");
		leftMotionInProgress = false;
		rightMotionInProgress = false;
		return false;
	}

	return true;
}

bool ArmsCartesian::motionComplete()
{
	double threshold = 0.13;		// FIXME do not hardcode
	updateState();
	if(this->leftMotionInProgress==true)
	{
		if(fabs(leftControllerState.x_err.linear.x )< threshold &&
		   fabs(leftControllerState.x_err.linear.y )< threshold &&
		   fabs(leftControllerState.x_err.linear.z )< threshold &&
		   fabs(leftControllerState.x_err.angular.x)< threshold &&
		   fabs(leftControllerState.x_err.angular.y)< threshold &&
		   fabs(leftControllerState.x_err.angular.z)< threshold)
		{
			leftMotionInProgress = false;
		}
	}
	if(this->rightMotionInProgress==true)
	{

		if(fabs(rightControllerState.x_err.linear.x )< threshold &&
		   fabs(rightControllerState.x_err.linear.y )< threshold &&
		   fabs(rightControllerState.x_err.linear.z )< threshold &&
		   fabs(rightControllerState.x_err.angular.x)< threshold &&
		   fabs(rightControllerState.x_err.angular.y)< threshold &&
		   fabs(rightControllerState.x_err.angular.z)< threshold)
		{
			rightMotionInProgress = false;
		}
	}

//	std::cout << "\n" << rightControllerState.x_err;
//	std::cout << "\n" << leftControllerState.x_err;

	if(leftMotionInProgress || rightMotionInProgress)
	{
		return false;
	}
	else
	{
		return true;
	}
}

bool ArmsCartesian::updateState()
{
	apc_msgs::JTCartesianControllerStateConstPtr tmp1 = ros::topic::waitForMessage<apc_msgs::JTCartesianControllerState>("/r_cart/state",ros::Duration(0.1));
	apc_msgs::JTCartesianControllerStateConstPtr tmp2 = ros::topic::waitForMessage<apc_msgs::JTCartesianControllerState>("/l_cart/state",ros::Duration(0.1));

	if(tmp1==NULL && tmp2==NULL )
	{
		ROS_ERROR("ArmsCartesian::updateState - Could not update state");
		return false;
	}
	rightControllerState = *tmp1;
	leftControllerState = *tmp2;
	return true;
}

bool ArmsCartesian::setGains(std::string gain_name, ArmsCartesian::WhichArm arm)
{
	if(arm==ArmsCartesian::LEFT){
		return setGains(gain_name,"left");
	}
	if(arm==ArmsCartesian::RIGHT){
		return setGains(gain_name,"right");
	}
	return false;
}

bool ArmsCartesian::setGains(std::string gain_name, std::string arm)
{
	GainValue gain_;
	for(int i=0;i<this->gains.size();i++)
	{
		GainValue temp = (GainValue)this->gains[i];
		if(temp.name == gain_name){
			gain_ = temp;
			i = gains.size();//Just faster way to exit loop
		}
	}

	std::vector<double> gains;
	gains.resize(4);
	gains.at(0) = gain_.t_p;
	gains.at(1) = gain_.r_p;
	gains.at(2) = gain_.t_d;
	gains.at(3) = gain_.r_d;

	setGains(gains,arm);
}

bool ArmsCartesian::setGains(std::vector<double>& gains, std::string arm){
	std_msgs::Float64MultiArray msg;

	msg.data.resize(12);

	ROS_INFO_STREAM("ArmsCartesian::setGains():Publishing new arm gains for " + arm);

	if(gains.size()==4){
		msg.data.at(0) =  gains[0];
		msg.data.at(1) =  gains[0];
		msg.data.at(2) =  gains[0];
		msg.data.at(3) =  gains[1];
		msg.data.at(4) =  gains[1];
		msg.data.at(5) =  gains[1];
		msg.data.at(6) =  gains[2];
		msg.data.at(7) =  gains[2];
		msg.data.at(8) =  gains[2];
		msg.data.at(9) =  gains[3];
		msg.data.at(10) = gains[3];
		msg.data.at(11) = gains[3];
	}else if(gains.size()==12){
		msg.data.at(0) =  gains[0];
		msg.data.at(1) =  gains[1];
		msg.data.at(2) =  gains[2];
		msg.data.at(3) =  gains[3];
		msg.data.at(4) =  gains[4];
		msg.data.at(5) =  gains[5];
		msg.data.at(6) =  gains[6];
		msg.data.at(7) =  gains[7];
		msg.data.at(8) =  gains[8];
		msg.data.at(9) =  gains[9];
		msg.data.at(10) = gains[10];
		msg.data.at(11) = gains[11];
	}else{
		return false;
	}

	if(arm == "left"){
		ROS_INFO_STREAM("Publishing new left arm gains");
		lgainPub.publish(msg);
		return true;
	}
	if(arm == "right"){
		ROS_INFO_STREAM("Publishing new right arm gains");
		rgainPub.publish(msg);
		return true;
	}

	return false;
}

bool ArmsCartesian::setGains(apc_msgs::SetGain::Request &req, apc_msgs::SetGain::Response &res){

	std_msgs::Float64MultiArray msg;

	msg.data.resize(12);

	ROS_INFO_STREAM("ArmsCartesian::setGains():Publishing new arm gains for " + req.arm);

	msg.data[0] = req.p_gain_x;
	msg.data[1] = req.p_gain_y;
	msg.data[2] = req.p_gain_z;
	msg.data[3] = req.p_gain_roll;
	msg.data[4] = req.p_gain_pitch;
	msg.data[5] = req.p_gain_yaw;
	msg.data[6] = req.d_gain_x;
	msg.data[7] = req.d_gain_y;
	msg.data[8] = req.d_gain_z;
	msg.data[9] = req.d_gain_roll;
	msg.data[10] = req.d_gain_pitch;
	msg.data[11] = req.d_gain_yaw;

	if(req.arm == "left"){
		ROS_INFO_STREAM("Publishing new left arm gains");
		lgainPub.publish(msg);
		return true;
	}
	if(req.arm == "right"){
		ROS_INFO_STREAM("Publishing new right arm gains");
		rgainPub.publish(msg);
		return true;
	}


	return false;
}

bool ArmsCartesian::moveToPose(ArmsCartesian::WhichArm arm, geometry_msgs::Pose cmd, bool waitForMotion)
{

	geometry_msgs::PoseStamped cmd_stamped;
	cmd_stamped.header.frame_id = "base_link";
	cmd_stamped.header.stamp = ros::Time::now();
	cmd_stamped.pose = cmd;

	ros::Publisher* pubPtr;
	switch(arm)
	{
	case ArmsCartesian::LEFT:
		leftMotionInProgress = true;
		pubPtr = &lCartPub;
		break;
	case ArmsCartesian::RIGHT:
		rightMotionInProgress = true;
		pubPtr = &rCartPub;
		break;
	default:
		ROS_ERROR("Received faulty arm argument!");
		return false;
	}


	pubPtr->publish(cmd_stamped);

	if(waitForMotion)
	{
		ros::Time now = ros::Time::now();
		ros::Time stop = ros::Time::now() + ros::Duration(timeOut);

		ROS_INFO("Waiting for arm motion...");
		while(!motionComplete() && now<stop)
		{
			ros::Duration(0.2).sleep();
			now = ros::Time::now();
		}

		if(!motionComplete() )
		{
			ROS_ERROR("ArmsCartesian::moveToPose - motion timed out!");
			leftMotionInProgress = false;
			rightMotionInProgress = false;
			return false;
		}

	}

	return true;
}

bool ArmsCartesian::moveInDirection(ArmsCartesian::WhichArm arm, geometry_msgs::Pose& gripperPose, ArmsCartesian::Direction d, double distance, double dx, double dt)
{
	geometry_msgs::PoseStamped tmp;
	tmp.pose = gripperPose;
	tmp.header.frame_id = "base_link";
	bool result = moveInDirection(arm, tmp, d, distance, dx, dt);
	gripperPose = tmp.pose;
	return result;
}
bool ArmsCartesian::moveInDirection(ArmsCartesian::WhichArm arm, geometry_msgs::PoseStamped& gripperPose, ArmsCartesian::Direction d, double distance, double dx, double dt)
{
	//ROS_INFO("ArmsCartesian::moveInDirection - start");
	ros::Publisher* pubPtr;
	switch(arm)
	{
	case ArmsCartesian::LEFT:
		leftMotionInProgress = true;
		pubPtr = &lCartPub;
		break;
	case ArmsCartesian::RIGHT:
		rightMotionInProgress = true;
		pubPtr = &rCartPub;
		break;
	default:
		ROS_ERROR("ArmsCartesian::moveInDirection - Received faulty arm argument!");
		return false;
	}

	// Get current gripper pose	- this introduce an error
//	geometry_msgs::PoseStamped gripperPose;
//	if( !getCurrentPose(arm,gripperPose) )
//	{
//		ROS_ERROR("ArmsCartesian::moveInDirection - Could not get gripper pose!");
//		return false;
//	}

	// Make sure distanceis postive and set the direction (-1 or 1)
	double direction = 1;
	if(distance<0)
	{
		direction = -1;
		distance = - distance;
	}
	double traveled = 0;
	double step = dx;

	// Loop until distance has been traveled
	bool goalReached = false;
	while(!goalReached)
	{
		if( (traveled+step) > distance )
			step = distance-traveled;

		switch(d)
		{
		case ArmsCartesian::X:
			gripperPose.pose.position.x += step*direction;
			break;
		case ArmsCartesian::Y:
			gripperPose.pose.position.y += step*direction;
			break;
		case ArmsCartesian::Z:
			gripperPose.pose.position.z += step*direction;
			break;
		default:
			ROS_ERROR("ArmsCartesian::moveInDirection - Received faulty direction argument!");
			return false;
		}

		traveled += step;
		gripperPose.header.seq = (gripperPose.header.seq+1)%255;
		gripperPose.header.stamp = ros::Time::now();

		pubPtr->publish(gripperPose);
		ros::Duration(dt).sleep();

		if(traveled >= distance)
			goalReached=true;

		//ROS_INFO("Traveled: %f, step: %f, distance: %f",traveled, step*direction, distance);
		//ROS_INFO("x: %f, y: %f, z: %f",gripperPose.pose.position.x, gripperPose.pose.position.y, gripperPose.pose.position.z);
	}

	if(ArmsCartesian::LEFT)
		leftMotionInProgress = false;
	else
		rightMotionInProgress = false;

	//ROS_INFO("ArmsCartesian::moveInDirection - done");
	return true;
}

bool ArmsCartesian::loadCartPose(ArmsCartesian::WhichArm arm, ArmsCartesian::CartPose p, geometry_msgs::PoseStamped& result)
{
	geometry_msgs::PoseStamped pose;
		
	updateState();
	if( arm == LEFT )
	{
		pose = leftControllerState.x;
	}else
	{
		pose = rightControllerState.x;
	}

	// TODO use Poses in torso_lift_link instead since this would not require a transform
	switch(p)
	{
	case homeLeft   :
		pose.header.frame_id = "base_link";
		pose.pose.position.x     = 0.1 ;
		pose.pose.position.y     = 0.4 ;
		pose.pose.position.z     = 1.15;
		pose.pose.orientation.x  = 0   ;
		pose.pose.orientation.y  = 0   ;
		pose.pose.orientation.z  = 0   ;
		pose.pose.orientation.w  = 1   ;
		break;
	case homeRight  :
		pose.header.frame_id = "base_link";
		pose.pose.position.x     = 0.1 ;
		pose.pose.position.y     = -0.4;
		pose.pose.position.z     = 1.15;
		pose.pose.orientation.x  = 0   ;
		pose.pose.orientation.y  = 0   ;
		pose.pose.orientation.z  = 0   ;
		pose.pose.orientation.w  = 1   ;
		break;
	case binHigh    :
		pose.header.frame_id 	 = "odom_origin";
		pose.pose.position.x     =  -0.8 ;
		pose.pose.position.y     =  0.4 ;
		pose.pose.position.z     =  1.35;
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.70711;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.70711;
		break;
	case binMid     :
		pose.header.frame_id 	 = "odom_origin";
		pose.pose.position.x     =  -0.8 ;
		pose.pose.position.y     =  0.4 ;
		pose.pose.position.z     =  1.15;
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.70711;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.70711;
		break;
	case binLow     :
		pose.header.frame_id 	 = "odom_origin";
		pose.pose.position.x     =  -0.8 ;
		pose.pose.position.y     =  0.4 ;
		pose.pose.position.z     =  1.10;
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.70711;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.70711;
		break;
	case toolVacuum :
		pose.header.frame_id 	 = "odom_origin";
		pose.pose.position.x     =  -0.9 ;
		pose.pose.position.y     =  0.45 ;
		pose.pose.position.z     =  0.8;
	case toolScoop  :
		pose.header.frame_id 	 = "odom_origin";
		pose.pose.position.x     =  -0.9 ;
		pose.pose.position.y     =  0.45 ;
		pose.pose.position.z     =  0.8;
		break;
	default:
		break;
	}

	result = pose;
	return true;
	
}

bool ArmsCartesian::loadOrientation(ArmsCartesian::WhichArm arm, ArmsCartesian::GripPose p, geometry_msgs::PoseStamped& result)
{
	geometry_msgs::PoseStamped pose;

	//We get the current pose from outside the function
	/*updateState();

	if( arm == LEFT )
	{
		pose = leftControllerState.x;
	}else
	{
		pose = rightControllerState.x;
	}*/

	switch(p)
	{
	case front      :
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.0;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 1.0;
		break;
	case side       :
		pose.pose.orientation.x  = 0.70711;
		pose.pose.orientation.y  = 0.0;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.70711;
		break;
	case degree45   :
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.38268;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.92388;
		break;
	case degree90   :
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.70711;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.70711;
		break;
	case degree35:
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.30071;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.95372;
		break;
	case degree25:
		pose.pose.orientation.x  = 0.0;
		pose.pose.orientation.y  = 0.21644;
		pose.pose.orientation.z  = 0.0;
		pose.pose.orientation.w  = 0.9763;
		break;
	default:
		break;
	}

	result = pose;

	return true;
}

bool ArmsCartesian::loadPosture(ArmsCartesian::WhichArm arm, ArmsCartesian::Posture p)
{

	std_msgs::Float64MultiArray posture_values;
	posture_values.data.resize(0);

	//std::vector<double> posture_values;

	//double off[];
	double dmantis[] 		= {0, 1, 0,  -1, 3.14, -1, 3.14};
	double delbowupr[] 	= {-0.79,0,-1.6,  9999, 9999, 9999, 9999};
	double delbowupl[] 	= {0.79,0,1.6 , 9999, 9999, 9999, 9999};
	double dold_elbowupr[] = {-0.79,0,-1.6, -0.79,3.14, -0.79,5.49};
	double dold_elbowupl[] = {0.79,0,1.6, -0.79,3.14, -0.79,5.49};
	double delbowdownr[] 	= {-0.028262077316910873, 1.2946342642324222, -0.25785640577652386, -1.5498884526859626, -31.278913849571776, -1.0527644894829107, -1.8127318367654268};
	double delbowdownl[] 	= {-0.0088195719039858515, 1.2834828245284853, 0.20338442004843196, -1.5565279256852611, -0.096340012666916802, -1.0235018652439782, 1.7990893054129216};

	double *joint;

	switch(p)
	{
	case off        :
		break;
	case mantis     :
		joint = dmantis;
		break;
	case elbowupr   :
		joint = delbowupr;
		break;
	case elbowupl   :
		joint = delbowupl;
		break;
	case elbowdownr :
		joint = delbowdownr;
		break;
	case elbowdownl :
		joint = delbowdownl;
		break;
	default:
		break;
	}

	posture_values.data.push_back(joint[0]);
	posture_values.data.push_back(joint[1]);
	posture_values.data.push_back(joint[2]);
	posture_values.data.push_back(joint[3]);
	posture_values.data.push_back(joint[4]);
	posture_values.data.push_back(joint[5]);

	if(arm == LEFT){
		ROS_INFO_STREAM(" Publishing new left arm posture");
		lposturePub.publish(posture_values);
		return true;
	}
	if(arm == RIGHT){
		ROS_INFO_STREAM(" Publishing new right arm posture");
		rposturePub.publish(posture_values);
		return true;
	}
}

bool ArmsCartesian::setPosture(ArmsCartesian::WhichArm arm,std::vector<double>& posture_position){
	std_msgs::Float64MultiArray posture_values;

	posture_values.data.resize(12);

	//Assuming we start from the shoulder
	posture_values.data.at(0) = posture_position.at(0);
	posture_values.data.at(1) = posture_position.at(1);
	posture_values.data.at(2) = posture_position.at(2);
	posture_values.data.at(3) = posture_position.at(3);
	posture_values.data.at(4) = posture_position.at(4);
	posture_values.data.at(5) = posture_position.at(5);

	if(arm == LEFT){
		ROS_INFO_STREAM("Publishing new left arm posture");
		lposturePub.publish(posture_values);
		return true;
	}
	if(arm == RIGHT){
		ROS_INFO_STREAM("Publishing new right arm posture");
		rposturePub.publish(posture_values);
		return true;
	}
}

bool ArmsCartesian::cancelMotion()
{

	return true;
}


bool ArmsCartesian::sendGoal(ArmsCartesian::WhichArm arm, ArmsCartesian::CartPose p, geometry_msgs::PoseStamped& result)
{
	geometry_msgs::PoseStamped pose;
	loadCartPose(arm, p, pose);
	return sendGoal(arm, pose);


}
bool ArmsCartesian::sendGoal(ArmsCartesian::WhichArm arm, geometry_msgs::PoseStamped pose)
{
	
	// Transform it just to be sure
	if(pose.header.frame_id != robot_frame)
	{
		geometry_msgs::PointStamped output;
		geometry_msgs::PointStamped input;
		input.header.frame_id = robot_frame;
		input.point = pose.pose.position;
		try
		{
			listener.transformPoint(robot_frame, input, output);
			pose.header.frame_id = robot_frame;
			pose.pose.position = output.point;
		}
		catch (tf::TransformException &ex)
		{
			ROS_ERROR("ArmsCartesian::sendGoal - %s",ex.what());
			return false;
		}
	}
	
	pose.header.stamp = ros::Time::now();
	
	if(arm == LEFT)
	{
		ROS_INFO_STREAM("Publishing new left arm posture");
		lCartPub.publish(pose);
		leftMotionInProgress = true;							// TODO set this to false at some point?
	}
	if(arm == RIGHT)
	{
		ROS_INFO_STREAM("Publishing new right arm posture");
		rCartPub.publish(pose);
		rightMotionInProgress = true;
	}
	
	return true;
}

