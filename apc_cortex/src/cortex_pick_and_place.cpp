/*!
 * 	\name 	   Cortex
 *  \brief     Main cortex class
 *  \details   Implments state machine for the development of different autonomous tasks
 *  \author    Sven Cremer
 *  \author	   Rommel Alonzo
 *  \version   1.0
 *  \date      Sept 21, 2016
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include <apc_cortex/cortex_pick_and_place.h>

Cortex::Cortex()
{
	ROS_INFO_STREAM("# Loading Cortex parameters");
	if(!loadNavParameters())
		ROS_ERROR("Failed to load some location parameters!");
	if(!loadArmParameters())
		ROS_ERROR("Failed to load some arm parameters!");

	// Objects
	ROS_INFO_STREAM("# Initializing PR2 interfaces");
	grippers 		= new Gripper();
	torso 			= new Torso();
	head 			= new Head();
	base 			= new Base();
	arms			= new ArmsCartesian();
	armsJoint		= new ArmsJoint();
//	armsMoveit 		= new RobotMoveit();

	ROS_INFO_STREAM("# Tuckarm initialization");
	tuck_ac_ = new TuckArmsClient("/tuck_arms", true);
    ROS_INFO("Waiting for the tuck_arms action server to come up ...");
    if( !tuck_ac_->waitForServer(ros::Duration(1.0)) )
    	ROS_ERROR("Failed connecting to tuck_arms server!");
    else
    	ROS_INFO("Connected to tuck_arms server!");

    ROS_INFO_STREAM("# Publishers and Services");

    // Publishers
    initialPose_pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1);

    // Service Clients
	marker_client				= 	nh.serviceClient<apc_msgs::ToolPos>("/apc/get_tool_pos");

	//Switching arms
	switch_controllers_service = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
	arm_controllers_default.push_back("l_arm_controller");
	arm_controllers_default.push_back("r_arm_controller");
	arm_controllers_JTcart.push_back("l_cart");
	arm_controllers_JTcart.push_back("r_cart");


	movingHead 		= false;
	movingGrippers	= false;
	movingTorso		= false;
	movingArms		= false;
	motionsComplete = true;

	storedTime = ros::Time::now();

	l_untuck_joints.push_back(0.341943); 		// TODO: move somewhere else
	l_untuck_joints.push_back(0.512964);
	l_untuck_joints.push_back(0.840619);
	l_untuck_joints.push_back(-1.73563);
	l_untuck_joints.push_back(3.22009);
	l_untuck_joints.push_back(-1.08709);
	l_untuck_joints.push_back(2.35645);

	l_untuck_placing_joints.push_back(0.415647); 		// TODO: move somewhere else
	l_untuck_placing_joints.push_back(0.672848);
	l_untuck_placing_joints.push_back(1.10857);
	l_untuck_placing_joints.push_back(-1.67338);
	l_untuck_placing_joints.push_back(-2.79479);
	l_untuck_placing_joints.push_back(-0.943089);
	l_untuck_placing_joints.push_back(2.07735);

	l_object_tuck.push_back(0.523757); 		// TODO: move somewhere else
	l_object_tuck.push_back(1.14472);
	l_object_tuck.push_back(1.76891);
	l_object_tuck.push_back(-1.47171);
	l_object_tuck.push_back(-3.98563);
	l_object_tuck.push_back(-0.562114);
	l_object_tuck.push_back(2.83661);

	dt = 0.15;
	x_step = 0.01;
	distance = 0.20;

	//initPose();

	executingRun = false;

	ROS_INFO("Initialized Cortex!");
}

Cortex::~Cortex()
{
	if(grippers != NULL)
		delete grippers;
	if(torso != NULL)
		delete torso;
	if(head != NULL)
		delete head;
	if(base != NULL)
		delete base;
	if(arms != NULL)
		delete arms;
	if(armsJoint != NULL)
		delete armsJoint;
//	if(armsMoveit != NULL)
//		delete armsMoveit;
	if(tuck_ac_ != NULL)
		delete tuck_ac_;
}

bool Cortex::loadNavParameters()
{
	bool result=true;
	int number;

	if(!(nh.getParam("/locations/number", number)))
	{
		ROS_ERROR("Parameter Not found.");
		return false;
	}
	locationsVector.clear();
	for(int i=0;i<number;i++)
	{
		geometry_msgs::PoseStamped tmp;
		std::string loc = "/locations/l" + boost::lexical_cast<std::string>(i+1);

		if(!(nh.getParam(std::string(loc+"/name"), tmp.header.frame_id))){ROS_ERROR("Parameter Not found.");result=false;}

		if(!(nh.getParam(std::string(loc+"/pose/position/x"), tmp.pose.position.x))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/position/y"), tmp.pose.position.y))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/position/z"), tmp.pose.position.z))){ROS_ERROR("Parameter Not found.");result=false;}

		if(!(nh.getParam(std::string(loc+"/pose/orientation/x"), tmp.pose.orientation.x))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/y"), tmp.pose.orientation.y))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/z"), tmp.pose.orientation.z))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/w"), tmp.pose.orientation.w))){ROS_ERROR("Parameter Not found.");result=false;}

		//std::cout<<loc<<": \n"<<tmp<<"---\n";

		locationsVector.push_back(tmp);
	}

	return result;
}

bool Cortex::loadArmParameters()
{
	bool result=true;
	int number;

	if(!(nh.getParam("/poses/number", number)))
	{
		ROS_ERROR("Parameter Not found.");
		return false;
	}

	armVector.clear();
	for(int i=0;i<number;i++)
	{
		geometry_msgs::PoseStamped tmp;
		std::string loc = "/poses/p" + boost::lexical_cast<std::string>(i+1);

		if(!(nh.getParam(std::string(loc+"/name"), tmp.header.frame_id))){ROS_ERROR("Parameter Not found.");result=false;}

		if(!(nh.getParam(std::string(loc+"/pose/position/x"), tmp.pose.position.x))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/position/y"), tmp.pose.position.y))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/position/z"), tmp.pose.position.z))){ROS_ERROR("Parameter Not found.");result=false;}

		if(!(nh.getParam(std::string(loc+"/pose/orientation/x"), tmp.pose.orientation.x))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/y"), tmp.pose.orientation.y))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/z"), tmp.pose.orientation.z))){ROS_ERROR("Parameter Not found.");result=false;}
		if(!(nh.getParam(std::string(loc+"/pose/orientation/w"), tmp.pose.orientation.w))){ROS_ERROR("Parameter Not found.");result=false;}

		//std::cout<<loc<<": \n"<<tmp<<"---\n";

		armVector.push_back(tmp);
	}

	return result;
}

bool Cortex::getLocationPose(std::string name, geometry_msgs::Pose* result)
{
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = locationsVector.begin();it != locationsVector.end();it++)
	{
		if((*it).header.frame_id == name)
		{
			*result = (*it).pose;
			return true;
		}
	}
	ROS_WARN("Could not find location %s",name.c_str());
	return false;
}

bool Cortex::getArmPose(std::string name, geometry_msgs::Pose* result)
{
	for(std::vector<geometry_msgs::PoseStamped>::const_iterator it = armVector.begin();it != armVector.end();it++)
	{
		if((*it).header.frame_id == name)
		{
			*result = (*it).pose;
			return true;
		}
	}
	ROS_WARN("Could not find arm pose %s",name.c_str());
	return false;
}

void Cortex::initPose()
{
	geometry_msgs::PoseWithCovarianceStamped p;

	p.header.frame_id = "/map";
	p.header.seq = 0;				// Ensures that position is only initialized once after 2d_nav startup
	p.pose.pose.position.x = 3.3;	// If last seq>0, then this message will be ignored
	p.pose.pose.position.y = -2.0;
	p.pose.pose.position.z = 0.0;
	p.pose.pose.orientation.x = 0.0;
	p.pose.pose.orientation.y = 0.0;
	p.pose.pose.orientation.z = 0.312873220349;
	p.pose.pose.orientation.w = 0.949794897853;
	p.pose.covariance[0] = 0.25;
	p.pose.covariance[7] = 0.25;
	p.pose.covariance[35] = 0.06853891945200942;

	initialPose_pub.publish(p);
}

bool Cortex::getMarker(int marker_number)
{
	marker_msg.request.marker_num = marker_number;

	if (marker_client.call(marker_msg))
	{
		if(marker_msg.response.success)
		{
			ROS_INFO("Received marker pose!");
			return true;
		}
		else
		{
			ROS_WARN("Failed to get marker pose!");
		}
	}
	else
	{
		ROS_ERROR("Failed to call service /apc/get_tool_pos");
	}
	return false;
}

bool Cortex::runSM(Cortex::RunState start, Cortex::RunState stop)
{
	if(!executingRun)
	{
		currentRunState = start;
		number_trails = 0;
		marker_number = 0;
		executingRun = true;
	}
	if(executingRun && currentRunState == stop)
	{
		executingRun = false;
		return true;
	}

		switch(currentRunState)
		{
		case START:
			ROS_INFO("**********START**********");

			torso->sendGoal(0.20);
			head->sendGoal(0,0.45,0,0);

			switchControllers(arm_controllers_default, arm_controllers_JTcart);		// Switch to default arm controllers

			currentRunState = TUCK;
			nextRunState = MOVE_TO_START_LOCATION;
			break;
		// -------------------------------------------------------------------------------------- //
		case TUCK:
			ROS_INFO("**********TUCKING ARM**********");

			armActionGoal.tuck_left 	= 	true;
			armActionGoal.tuck_right	= 	true;
			tuck_ac_->sendGoal(armActionGoal);

			currentRunState = WAIT_FOR_TUCK;
			break;
		case UNTUCK:
			ROS_INFO("**********UNTUCKING ARM**********");

			armActionGoal.tuck_left 	= 	false;
			armActionGoal.tuck_right	= 	true;
			tuck_ac_->sendGoal(armActionGoal);

			currentRunState = WAIT_FOR_TUCK;
			break;
		case WAIT_FOR_TUCK:
			if( (ros::Time::now() - storedTime) > ros::Duration(0.25))	// Check every 0.25 seconds
			{
				//ROS_INFO("**********WAIT_FOR_TUCK**********");

				ac_state = new actionlib::SimpleClientGoalState(tuck_ac_->getState());

				if(ac_state->isDone())
				{
					if(ac_state->state_ == actionlib::SimpleClientGoalState::SUCCEEDED)
					{
						ROS_INFO("Base action finished: %s",ac_state->toString().c_str());
					}
					else
					{
						ROS_INFO("Base action failed (?): %s",ac_state->toString().c_str());
					}
					currentRunState = nextRunState;
				}
				delete ac_state;

				storedTime = ros::Time::now();
			}
			break;
		// -------------------------------------------------------------------------------------- //
		case MOVE_TO_START_LOCATION:
			ROS_INFO("**********MOVE_TO_START_LOCATION**********");

			if(getLocationPose("start",&locationPose))
				base->sendBaseGoal_MapFrame(locationPose);

			currentRunState = WAIT_FOR_BASE_MOTION;
			nextRunState = PREPARE_ARMS_1;
			break;

		case PREPARE_ARMS_1:
			ROS_INFO("**********PREPARE_ARMS_1**********");
			currentRunState = UNTUCK;
			nextRunState = PREPARE_ARMS_2;
			break;

		case PREPARE_ARMS_2:
			ROS_INFO("**********PREPARE_ARMS_2**********");

			armsJoint->sendGoal(l_object_tuck,ArmsJoint::LEFT);
			ros::Duration(1.0).sleep();

			currentRunState = MOVE_TO_PICKING_LOCATION;
			break;

		// -------------------------------------------------------------------------------------- //
		case MOVE_TO_PICKING_LOCATION:
			ROS_INFO("**********MOVE_TO_PICKING_LOCATION**********");

			if(getLocationPose("front_counter",&locationPose))
				base->sendBaseGoal_MapFrame(locationPose);

			currentRunState = WAIT_FOR_BASE_MOTION;
			nextRunState = START_PICKING;
			break;

		case WAIT_FOR_BASE_MOTION:
			if( (ros::Time::now() - storedTime) > ros::Duration(0.25))	// Check every 0.25 seconds
			{
				//ROS_INFO("**********WAIT_FOR_BASE_MOTION**********");
				if(base->motionComplete())
				{
					currentRunState = nextRunState;
				}
				storedTime = ros::Time::now();
			}
			break;
		// -------------------------------------------------------------------------------------- //
		case START_PICKING:
			ROS_INFO("**********START_PICKING**********");
			currentRunState = GET_OBJECT_LOCATION;
			break;

		case GET_OBJECT_LOCATION:
			ROS_INFO("**********GET_OBJECT_LOCATION**********");

			if( getMarker(marker_number) )	// Marker was detected
			{
				std::cout<<marker_msg.response.marker_position;
				currentRunState = MOVE_TO_OBJECT;
			}
			else
			{
				currentRunState = GET_OBJECT_LOCATION;
			}
			break;
		case MOVE_TO_OBJECT:
			ROS_INFO("**********MOVE_TO_OBJECT**********");

			// Adjust base location
			//ros::Duration(1.0).sleep();

			currentRunState = PICK_OBJECT;
			break;

		// -------------------------------------------------------------------------------------- //
		case PICK_OBJECT:
			ROS_INFO("**********PICK_OBJECT**********");		// TODO make this a separate function / grasp class
																// TODO remove any blocking functions, sleep statements
			// Untuck arm using joint positions
			armsJoint->sendGoal(l_untuck_joints,ArmsJoint::LEFT);

			while(!armsJoint->motionComplete())
			{
				std::cout<<".";
				ros::Duration(0.5).sleep();
			}
			std::cout<<"\n";

			// Switch to Cartesian
			switchControllers(arm_controllers_JTcart, arm_controllers_default);
			ros::Duration(0.1).sleep();

			// Adjust arm position depending on marker location
			ROS_INFO("Moving to position");
			if(getArmPose("l_start_picking",&armPose))
			{
				armPose.position.y = marker_msg.response.marker_position.pose.position.y;	// This assumes the PR2 is perfectly parallel to marker
//				armPose.position.z = marker_msg.response.marker_position.pose.position.z;
				std::cout<<armPose;

				arms->moveToPose(ArmsCartesian::LEFT,armPose,true);
			}

			// Open grippers
			grippers->open(Gripper::LEFT);

			ros::Duration(0.5).sleep();

			distance=0.22;

			//Move forward
			ROS_INFO("Moving forward");
			arms->moveInDirection(ArmsCartesian::LEFT, armPose, ArmsCartesian::X, distance, x_step, dt);
			ros::Duration(2.0).sleep();

			grippers->closeGently(Gripper::LEFT);
			ros::Duration(2.0).sleep();

			//Move Backward
			ROS_INFO("Moving Back");
			arms->moveInDirection(ArmsCartesian::LEFT, armPose, ArmsCartesian::X, -distance, x_step, dt);
			ros::Duration(2.0).sleep();

			currentRunState = TUCK_OBJECT;
			break;

		case TUCK_OBJECT:
			ROS_INFO("**********TUCK_OBJECT**********");

			switchControllers(arm_controllers_default, arm_controllers_JTcart);
			armsJoint->sendGoal(l_object_tuck,ArmsJoint::LEFT);

			currentRunState = START_PLACING;
			break;

		// -------------------------------------------------------------------------------------- //
		case START_PLACING:
			ROS_INFO("**********START_PLACING**********");

			currentRunState = MOVE_TO_DROP_LOCATION;
			break;

		case MOVE_TO_DROP_LOCATION:
			ROS_INFO("**********MOVE_TO_DROP_LOCATION**********");

			if(getLocationPose("front_table",&locationPose))
				base->sendBaseGoal_MapFrame(locationPose);

			currentRunState = WAIT_FOR_BASE_MOTION;
			nextRunState = PLACE_OBJECT;
			break;

		case PLACE_OBJECT:
			ROS_INFO("**********PLACE_OBJECT**********");


			// Untuck arm using joint positions
			switchControllers(arm_controllers_default, arm_controllers_JTcart);

			armsJoint->sendGoal(l_untuck_placing_joints,ArmsJoint::LEFT);
			while(!armsJoint->motionComplete())
			{
				std::cout<<".";
				ros::Duration(0.5).sleep();
			}
			std::cout<<"\n";

			// Switch to Cartesian
			switchControllers(arm_controllers_JTcart, arm_controllers_default);
			ros::Duration(0.1).sleep();

			ROS_INFO("Moving to position");
			if(getArmPose("l_start_placing",&armPose))
			{
				std::cout<<armPose;
				arms->moveToPose(ArmsCartesian::LEFT,armPose,true);
			}

			ros::Duration(1.0).sleep();

			distance = 0.2;

			//Move forward
			ROS_INFO("Moving forward");
			arms->moveInDirection(ArmsCartesian::LEFT, armPose, ArmsCartesian::X, distance, x_step, dt);
			ros::Duration(2.0).sleep();

			grippers->open(Gripper::LEFT);
			ros::Duration(2.0).sleep();

			//Move Backward
			ROS_INFO("Moving Back");
			std::cout<<armPose;
			arms->moveInDirection(ArmsCartesian::LEFT, armPose, ArmsCartesian::X, -distance, x_step, dt);
			ros::Duration(2.0).sleep();

			grippers->close(Gripper::LEFT);

			switchControllers(arm_controllers_default, arm_controllers_JTcart);

			currentRunState = STOP_PLACING;
			break;

		case STOP_PLACING:
			ROS_INFO("**********STOP_PLACING**********");

			if(marker_number<2)
			{
				currentRunState = PREPARE_ARMS_2;
				number_trails++;
				marker_number++;
				ROS_INFO("Number of trails: %d", number_trails);
			}
			else
			{
				currentRunState = TUCK;
				nextRunState = MOVE_BACK;
			}
			break;
		// -------------------------------------------------------------------------------------- //
		case MOVE_BACK:
			ROS_INFO("**********MOVE_BACK**********");

			if(getLocationPose("start",&locationPose))
				base->sendBaseGoal_MapFrame(locationPose);

			currentRunState = WAIT_FOR_BASE_MOTION;
			nextRunState = DONE;

		case DONE:
			ROS_INFO("**********DONE**********");

		default:
			ROS_WARN("runSM: invalid state");
			break;
		}

	return false;
}

bool Cortex::checkIfMotionsComplete()
{

	movingHead		= !( head->motionComplete() );
	movingGrippers  = !( grippers->motionComplete()	    );
	movingTorso		= !( torso->motionComplete()		);
	movingArms		= !( arms->motionComplete()		    );

	motionsComplete = !movingHead &&
					  !movingGrippers &&
					  !movingTorso &&
					  !movingArms;

	return motionsComplete;
}



void Cortex::switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers)
{

	pr2_mechanism_msgs::SwitchController::Request req;
	pr2_mechanism_msgs::SwitchController::Response res;
	req.start_controllers = start_controllers;
	req.stop_controllers = stop_controllers;
	for(std::vector<std::string>::const_iterator it = start_controllers.begin();
			it != start_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to start controller " << (*it));
	}
	for(std::vector<std::string>::const_iterator it = stop_controllers.begin();
			it != stop_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to stop controller " << (*it));
	}
	req.strictness =  pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
	if(!switch_controllers_service.call(req,res))
	{
		ROS_ERROR("Call to switch controllers failed entirely");
	}
	if(res.ok != true)
	{
		ROS_ERROR("Call to switch controllers reports not ok");
	}
	else
	{
		ROS_INFO("Call to switch controllers reports ok");
	}
}


