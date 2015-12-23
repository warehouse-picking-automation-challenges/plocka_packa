/*!
 * 	\name 	   ApcCortex
 *  \brief     Main cortex class
 *  \details   Implments state machine for the development of different autonomous tasks
 *  \author    Sven Cremer
 *  \version   1.0
 *  \date      Mar 10, 2015
 *  \pre       First initialize the apc_robot/apc_robot.launch
 *  \warning   Improper use can crash your application
 *  \copyright BSD License
 */

#include <apc_cortex/apc_cortex.h>

ApcCortex::ApcCortex()
{

	// Initialize services
	ROS_INFO_STREAM("Registering to services");
	srvClient_get_status		= nh.serviceClient<apc_msgs::GetBool>("/apc/json/status");
	srvClient_get_work_order 	= nh.serviceClient<apc_msgs::GetWorkOrder>("/apc/json/work_order");
	srvClient_work_order_request= nh.serviceClient<apc_msgs::GetItemFromWorkOrder>("/apc/work_order_request");
	srvClient_get_bin_contents 	= nh.serviceClient<apc_msgs::GetBinContents>("/apc/json/bin_contents");
	srvClient_startORK 			= nh.serviceClient<apc_msgs::SetBin>("/apc/startORK");
	srvClient_stopORK 			= nh.serviceClient<apc_msgs::Empty>("/apc/stopORK");
	srvClient_get_object_pose 	= nh.serviceClient<apc_msgs::GetObjectPose>("apc/object_recognition/get_object_pose");
	srvClient_set_shelf_origin 	= nh.serviceClient<apc_msgs::SetShelfOrigin>("/apc/shelf/set_shelf_origin");
	srvClient_get_bin_origin 	= nh.serviceClient<apc_msgs::GetBinOrigin>("/apc/shelf/get_bin_origin");
	srvClient_get_bin_info 		= nh.serviceClient<apc_msgs::GetBinInfo>("/apc/shelf/get_bin_info");
	srvClient_get_grasping_strategy = nh.serviceClient<apc_msgs::GetGraspStrategy>("/apc/grasp_strategy");
	srvClient_set_arm_gain 		= nh.serviceClient<apc_msgs::SetGain>("/apc/arm/set_gains");
	srvClient_toolPose 			= nh.serviceClient<apc_msgs::ToolPos>("/apc/get_tool_pos");
	ROS_INFO_STREAM("Finished registering to services");

	// Objects
	ROS_INFO_STREAM("Initializing PR2 interfaces");
//	ROS_INFO_STREAM("RobotMoveit");
//	robot_moveit 	= new RobotMoveit();
	ROS_INFO_STREAM("Gripper");
	grippers 		= new Gripper();
	ROS_INFO_STREAM("Torso");
	torso 			= new Torso();
	ROS_INFO_STREAM("Head");
	head 			= new Head();
	ROS_INFO_STREAM("BaseMovement");
	base 			= new BaseMovement(); //TODO Remove NodeHandle from constructor
	ROS_INFO_STREAM("Finished PR2 interface initialization");
	arms			= new ArmsCartesian();
	ROS_INFO_STREAM("Finished PR2 arms initialization");

	movingHead 		= false;
	movingGrippers	= false;
	movingTorso		= false;
	movingArms		= false;
	motionsComplete = true;
	receivedJSON	= false;
	waitingStarted  = false;

	//robot_frame = "base_footprint";
	robot_frame = "base_link";

	// Properties
	executingRun=false;
	max_execution_time = 60;
	max_arm_execution_time = 3.0;
	loopRateRunSM = 0.2;

}

ApcCortex::~ApcCortex()
{
//	if(robot_moveit != NULL)
//	{
//		delete robot_moveit;
//	}
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
}


bool ApcCortex::sortWorkOrder(std::vector<apc_msgs::WorkOrder> order)
{
	ROS_INFO("ApcCortex::sortWorkOrder - INIT");

	apc_msgs::GetGraspStrategy srv_;
	work_order.vacuumLift.resize(0);
	work_order.vacuumFront.resize(0);
	work_order.electricFront.resize(0);
	work_order.electricAngledLift.resize(0);

	// Iterate through WorkOrder
	for(std::vector<apc_msgs::WorkOrder>::const_iterator it = order.begin(); it != order.end(); ++it)
	{

		srv_.request.item 			= (*it).name;
		srv_.request.bin 			= (*it).bin;
		srv_.request.orientation 	= "all";	// Don't care for now

		if(!srvClient_get_grasping_strategy.call(srv_))
		{
			ROS_ERROR("ApcCortex::sortWorkOrder - Failed to call grasping strategy server");
		}
		else
		{
			ApcCortex::objectOrder tmp;
			tmp.bin = srv_.request.bin;
			tmp.name = srv_.request.item;
			tmp.requiredArm = determineArmFromBin(tmp.bin);
			tmp.requiredArmCartesian = getCartesianArm(tmp.requiredArm);
			tmp.requiredArmGripper = getGripperArm(tmp.requiredArm);

			if(srv_.response.tool == "vacuum" && srv_.response.grasp == "lift")
			{
				tmp.requiredStrategy = LIFT;
				tmp.requiredTool = VACUUM;
				work_order.vacuumLift.push_back(tmp);
			}
			else if(srv_.response.tool == "electric" && srv_.response.grasp == "front")
			{
				tmp.requiredStrategy = FRONT;
				tmp.requiredTool = ELECTRIC;
				work_order.electricFront.push_back(tmp);
			}
			else if(srv_.response.tool == "vacuum" && srv_.response.grasp == "front")
			{
				tmp.requiredStrategy = FRONT;
				tmp.requiredTool = VACUUM;
				work_order.vacuumFront.push_back(tmp);
			}
			else if(srv_.response.tool == "electric" && srv_.response.grasp == "angledLift")
			{
				tmp.requiredStrategy = ANGLEDLIFT;
				tmp.requiredTool = ELECTRIC;
				work_order.vacuumFront.push_back(tmp);
			}
			else
			{
				ROS_ERROR("ApcCortex::sortWorkOrder - Strategy not found for '%s'",srv_.request.item.c_str());
			}

			printf("Tool: %i (%s)",tmp.requiredTool,srv_.response.tool.c_str());
			printf("Grasp: %i (%s)",tmp.requiredStrategy,srv_.response.grasp.c_str());
			printf("---------");
		}
	}

	// Update number of items
	items_left.vacuumLift			= work_order.vacuumLift			.size();
	items_left.vacuumFront			= work_order.vacuumFront		.size();
	items_left.electricFront		= work_order.electricFront		.size();
	items_left.electricAngledLift	= work_order.electricAngledLift	.size();



	ROS_INFO("ApcCortex::sortWorkOrder - DONE");
	return true;
}

bool ApcCortex::updateGraspingStrategy(std::string item, int bin, std::string orientation)
{
	ROS_INFO("ApcCortex::updateGraspingStrategy - INIT");
	// Set arm
	stateVariables.requiredArm = determineArmFromBin(bin);
	stateVariables.requiredArmCartesian = getCartesianArm(stateVariables.requiredArm);
	stateVariables.requiredArmGripper = getGripperArm(stateVariables.requiredArm);

	// Get grasping strategy from JSON server
	apc_msgs::GetGraspStrategy srv_;

	srv_.request.item = item;
	srv_.request.bin = bin;
	srv_.request.orientation = orientation;

	if(srvClient_get_grasping_strategy.call(srv_))
	{
		// Set strategy
		if(srv_.response.grasp == "front")
			stateVariables.requiredStrategy = FRONT;
		else if(srv_.response.grasp == "lift")
			stateVariables.requiredStrategy = LIFT;
		else if(srv_.response.grasp == "angledLift")
			stateVariables.requiredStrategy = ANGLEDLIFT;
		else if(srv_.response.grasp == "vacuumfront")
			stateVariables.requiredStrategy = VACUUMFRONT;
		else
			ROS_ERROR("Received invalid grasping strategy.");

		// Set tool
		if(srv_.response.tool == "electric")
			stateVariables.requiredTool = ELECTRIC;
		else if(srv_.response.tool == "vacuum")
			stateVariables.requiredTool = VACUUM;
		else if(srv_.response.tool == "scoop")
			stateVariables.requiredTool = SCOOP;
		else
			ROS_ERROR("Received invalid grasping tool.");
	}
	else
	{
		ROS_ERROR("Could not call grasping strategy server.");
		return false;
	}

	printf("Tool: %i (%s)",stateVariables.requiredTool,srv_.response.tool.c_str());
	printf("Grasp: %i (%s)",stateVariables.requiredStrategy,srv_.response.grasp.c_str());

	ROS_INFO("ApcCortex::updateGraspingStrategy - DONE");
	return true;
}

bool ApcCortex::setShelfOrigin(geometry_msgs::Point p)
{

	apc_msgs::SetShelfOrigin srv_SetShelfOrigin;

	srv_SetShelfOrigin.request.point = p;

	if (srvClient_set_shelf_origin.call(srv_SetShelfOrigin))
	{
		return true;
	}
	else
	{
		ROS_ERROR("ApcCortex::setShelfOrigin - Failed to call service set_shelf_origin");
		return false;
	}

}

bool ApcCortex::getBinOrigin(int bin_nr, geometry_msgs::Point& p)
{

	apc_msgs::GetBinOrigin srv_;

	srv_.request.bin_number = bin_nr;
	srv_.request.frame = robot_frame;

	if (srvClient_get_bin_origin.call(srv_))
	{
		p = srv_.response.point;
		ROS_INFO_STREAM("ApcCortex::getBinOrigin - Bin "<<bin_nr<<": ("<<p.x<<","<<p.y<<","<<p.z<<")\n");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service get_bin_origin");
		return false;
	}
}

bool ApcCortex::getBinInfo(int bin_nr, apc_msgs::GetBinInfo& result)
{
	ROS_INFO("ApcCortex::getBinInfo - INIT");
	apc_msgs::GetBinInfo srv_;

	srv_.request.bin_number = bin_nr;
	srv_.request.frame = robot_frame;

	if (srvClient_get_bin_info.call(srv_))
	{
		result = srv_;
	}
	else
	{
		ROS_ERROR("Failed to call service get_bin_origin");
		return false;
	}
	ROS_INFO("ApcCortex::getBinInfo - DONE");
	return true;
}

int ApcCortex::getBinColumn(int bin_number)
{
	int col = bin_number%3;

	if(col==0)
		return 3;

	return col;
}


bool ApcCortex::moveToBin(int bin_number, ApcCortex::WhichArm a)
{
	ROS_INFO("ApcCortex::moveToBin - DONE");
	ArmsCartesian::WhichArm arm_cartesian = getCartesianArm(a);
	apc_msgs::GetBinInfo binInfo;
	geometry_msgs::Pose pose;

	if(!getBinInfo(bin_number, binInfo))
	{
		ROS_ERROR("ApcCortex::moveToBin -> bin not found");
		return false;
	}

	pose.position = binInfo.response.center;
	pose.position.x = -0.05;		// TODO don't hardcode
	pose.orientation.x = 0;
	pose.orientation.y = 0;
	pose.orientation.z = 0;
	pose.orientation.w = 1;

	ROS_INFO_STREAM("Moving to bin "<<bin_number<<": ("<<pose.position.x<<","<<pose.position.y<<","<<pose.position.z<<")\n");

	ROS_INFO("ApcCortex::moveToBin - DONE");
	return arms->moveToPose(arm_cartesian, pose);
}

bool ApcCortex::moveToBinPlane(int bin_number, ApcCortex::WhichArm a, double distance_from_robot)
{
	ROS_INFO("ApcCortex::moveToBinPlane - DONE");
	ArmsCartesian::WhichArm arm_cartesian = getCartesianArm(a);
	apc_msgs::GetBinInfo binInfo;
	geometry_msgs::Pose pose;

	if(!getBinInfo(bin_number, binInfo))
	{
		ROS_ERROR("ApcCortex::moveToBin -> bin not found");
		return false;
	}

	pose.position = binInfo.response.center;
	pose.position.x = distance_from_robot;
	pose.orientation.w = 1;

	ROS_INFO_STREAM("Moving to bin plane "<<bin_number<<": ("<<pose.position.x<<","<<pose.position.y<<","<<pose.position.z<<")\n");

	ROS_INFO("ApcCortex::moveToBinPlane - DONE");
	return arms->moveToPose(arm_cartesian, pose);
}


bool ApcCortex::lookAtOrderBin()
{
	head->sendGoalCart(	robot_frame,
						stateVariables.orderBinLocation.x,
						stateVariables.orderBinLocation.y,
						stateVariables.orderBinLocation.z,
						1.0);
	return true;
}


bool ApcCortex::lookAtBin(int bin_nr, ApcCortex::BinPoints p)
{
	// Get bin info
	//if(stateVariables.currentBinInfo.request.bin_number == bin_nr)
	apc_msgs::GetBinInfo bin;
	getBinInfo(stateVariables.currentBin, bin); 	// base could have moved so always update
	double t=1.0;

	geometry_msgs::Point point;
	switch(p)
	{
		case O:
			point.x = bin.response.o.x;
			point.y = bin.response.o.y;
			point.z = bin.response.o.z;
			break;
		case A:
			point.x = bin.response.a.x;
			point.y = bin.response.a.y;
			point.z = bin.response.a.z;
			break;
		case B:
			point.x = bin.response.b.x;
			point.y = bin.response.b.y;
			point.z = bin.response.b.z;
			break;
		case C:
			point.x = bin.response.c.x;
			point.y = bin.response.c.y;
			point.z = bin.response.c.z;
			break;
		defaultA:
			point.x = bin.response.center.x;
			point.y = bin.response.center.y;
			point.z = bin.response.center.z;
			break;
	}
	// Point Head
	head->sendGoalCart(	robot_frame,
						point.x,
						point.y,
						point.z,
						t);

	return true;
}

bool ApcCortex::runSM(ApcCortex::RunState start, ApcCortex::RunState stop)
{

	executingRun=true;

	currentRunState = start;

	while(executingRun)
	{
		switch(currentRunState)
		{
		//-------------------------------------------------------------------------------//
		case START:
			ROS_INFO("*SM* -> START");

			receivedJSON = false;

			torso->sendGoal(0.295);
			grippers->close(Gripper::BOTH);
			lookAtBin(5,ApcCortex::CENTER);
			stateVariables.currentToolRight = ELECTRIC;
			stateVariables.currentToolLeft = ELECTRIC;
			stateVariables.currentItemNumber = 0;

			//Load Gains
			arms->setGains("initial",ArmsCartesian::LEFT);
			arms->setGains("initial",ArmsCartesian::RIGHT);

			// Load Posture
			arms->loadPosture(ArmsCartesian::RIGHT,ArmsCartesian::elbowupr);
			arms->loadPosture(ArmsCartesian::LEFT,ArmsCartesian::elbowupl);

			//Set arm to home positions
			arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::homeLeft, stateVariables.currentGripperPoseLeft);
			arms->loadCartPose(ArmsCartesian::RIGHT, ArmsCartesian::homeRight, stateVariables.currentGripperPoseRight);

			nextRunState 	= WAIT_FOR_JSON;
			currentRunState = WAIT_FOR_MOTION;
			break;
		//-------------------------------------------------------------------------------//
		case WAIT_FOR_MOTION:														// FIXME? this is a blocking state
			ROS_INFO("*SM* -> WAIT_FOR_MOTION");

			motionsComplete = false;
			time_now = ros::Time::now();
			time_stop = ros::Time::now()+ros::Duration(5.0);
			while (time_now < time_stop && !motionsComplete)
			{
				if(grippers->motionComplete()){
					//ROS_INFO_STREAM("Grippers->motionComplete(): True");
				}else{
					ROS_INFO_STREAM("Grippers->motionComplete(): False");
				}

				if(torso->motionComplete()){
					//ROS_INFO_STREAM("torso->motionComplete(): True");
				}else{
					ROS_INFO_STREAM("torso->motionComplete(): False");
				}
				if(head->motionComplete()){
					//ROS_INFO_STREAM("head->motionComplete(): True");
				}else{
					ROS_INFO_STREAM("head->motionComplete(): False");
				}
				if(arms->motionComplete()){
					//ROS_INFO_STREAM("arms->motionComplete(): True");
				}else{
					ROS_INFO_STREAM("arms->motionComplete(): False");
				}
				motionsComplete =      grippers->motionComplete()
									&& torso->motionComplete()
									&& head->motionComplete()
									&& arms->motionComplete();
				ros::Duration(0.2).sleep();
				time_now = ros::Time::now();
			}

			if(!motionsComplete)
			{
				ROS_INFO("Time out - did not complete motions!");					// TODO add troubleshooting
				grippers->cancelMotion();
				torso->cancelMotion();
				head->cancelMotion();
			}
			else
			{
				ROS_INFO("runSM: Motions completed!");
			}
			currentRunState = nextRunState;
			break;
			//-------------------------------------------------------------------------------//
			case WAIT_FOR_JSON:
				ROS_INFO("*SM* -> WAIT_FOR_JSON");

				srv_msg_GetBool.response.variable = false;
				while (!srv_msg_GetBool.response.variable)
				{
					srvClient_get_status.call(srv_msg_GetBool);

					ros::Duration(0.2).sleep();
				}

				currentRunState = SORT_WORK_ORDER;
				break;
				//-------------------------------------------------------------------------------//
			case SORT_WORK_ORDER:
				ROS_INFO("*SM* -> SORT_WORK_ORDER");

				if( !srvClient_get_work_order.call(srv_msg_GetWorkOrder) )
				{
					ROS_ERROR("Failed to get workOrder");
					currentRunState=WAIT_FOR_JSON;
				}
				else
				{
					receivedJSON = true;
				}

				// Electric
				//	a) col 1,2 -> left
				//  b) col 3 -> right
				// Vacuum
				//  a) both arms
				//  b) single arms

				if(! sortWorkOrder(srv_msg_GetWorkOrder.response.work_order ) )
					ROS_ERROR("Could not sort work order!");

				orderIter = work_order.vacuumLift.begin();

				currentRunState = START_PICKING;
				break;
			//-------------------------------------------------------------------------------//
			//-------------------------------------------------------------------------------//
			case START_PICKING:
				ROS_INFO("*SM* -> START_PICKING");

				// Decide what object to pick
				if(items_left.vacuumLift>0)
				{
					orderIter++;
					items_left.vacuumLift--;
				}
				else if(items_left.electricFront>0)
				{
					orderIter++;
					items_left.electricFront--;
				}
				else if(items_left.vacuumFront>0)
				{
					orderIter++;
					items_left.vacuumFront--;
				}
				else if(items_left.electricAngledLift>0)
				{
					orderIter++;
					items_left.electricAngledLift--;
				}

				stateVariables.itemsLeftToPick			= items_left.vacuumLift;
				stateVariables.currentObject 			= (*orderIter).name;
				stateVariables.currentBin 				= (*orderIter).bin;
				stateVariables.requiredArm 				= (*orderIter).requiredArm;
				stateVariables.requiredArmCartesian 	= (*orderIter).requiredArmCartesian;
				stateVariables.requiredArmGripper 		= (*orderIter).requiredArmGripper;
				stateVariables.requiredStrategy			= (*orderIter).requiredStrategy;
				stateVariables.requiredTool 			= (*orderIter).requiredTool;

//				srv_msg_GetItemFromWorkOrder.request.item_num = stateVariables.currentItemNumber;
//				if(!srvClient_work_order_request.call(srv_msg_GetItemFromWorkOrder) )
//					ROS_ERROR("Failed to get item from work order %d", stateVariables.currentItemNumber);
//
//				stateVariables.itemsLeftToPick 		= srv_msg_GetItemFromWorkOrder.response.items_left;
//				stateVariables.currentObject 		= srv_msg_GetItemFromWorkOrder.response.item;
//				stateVariables.currentBin 			= srv_msg_GetItemFromWorkOrder.response.bin;
//				stateVariables.totalNumberOfItems 	= srv_msg_GetItemFromWorkOrder.response.item_number;
//
//				stateVariables.currentToolRight = ELECTRIC;
				std::cout<<"Items left to pick: "<<stateVariables.itemsLeftToPick<<"/"<<stateVariables.totalNumberOfItems<<"\n";
				std::cout<<"Object to pick: "<<stateVariables.currentObject<<"\n";
				std::cout<<"Bin number: "<<stateVariables.currentBin <<"\n";

				// Get bin info
				getBinInfo(stateVariables.currentBin, stateVariables.currentBinInfo);

				// Point Head
				head->sendGoalCart(	robot_frame,
									stateVariables.currentBinInfo.response.center.x,
									stateVariables.currentBinInfo.response.center.y,
									stateVariables.currentBinInfo.response.center.z,
									1.0);

				//nextRunState 	= GET_OBJECT_LOCATION; 	FIXME since we don't care about orienation, we don't have to detect the item right now
				//currentRunState = WAIT_FOR_MOTION;
				currentRunState 	= DETERMINE_STRATEGY;
				break;
			//-------------------------------------------------------------------------------//
			case GET_OBJECT_LOCATION:
				ROS_INFO("*SM* -> GET_OBJECT_LOCATION");

				// Start ORK
				srv_msg_SetBin.request.number = stateVariables.currentBin;
				if(! srvClient_startORK.call(srv_msg_SetBin) )
					ROS_ERROR("Could not start ORK");

				// Get Object pose
				srv_msg_GetObjectPose.request.obj_name = stateVariables.currentObject;
				srv_msg_GetObjectPose.response.found_obj = false;
				while(!srv_msg_GetObjectPose.response.found_obj)
				{
					if(! srvClient_get_object_pose.call(srv_msg_GetObjectPose) )
						ROS_ERROR("Could not stop ORK");

					ros::Duration(0.5).sleep();
				}

				// Stop ORK
				if(! srvClient_stopORK.call(srv_msg_Empty) )
					ROS_ERROR("Could not stop ORK");

				// TODO: convert pose to string with orientation name

				currentRunState = DETERMINE_STRATEGY;
				break;
			//-------------------------------------------------------------------------------//
			case DETERMINE_STRATEGY:
				ROS_INFO("*SM* -> DETERMINE_STRATEGY");

				// Set arm, strategy, and tool
				updateGraspingStrategy(	stateVariables.currentObject,
										stateVariables.currentBin,
										"all");								// TODO update

				// Check if we need to switch
				switchLeftTool = false;
				switchRightTool = false;

				if(stateVariables.requiredArm == LEFT)
				{
					stateVariables.requiredToolLeft = stateVariables.requiredTool;
					std::cout<<"Required tool for left: "<<stateVariables.requiredToolLeft<<"\n";
					std::cout<<"Current tool for left: "<<stateVariables.currentToolLeft<<"\n";
					if(stateVariables.requiredToolLeft != stateVariables.currentToolLeft)
					{
						switchLeftTool = true;
						currentRunState = MOVE_TO_TOOL;
					}
				}

				if(stateVariables.requiredArm == RIGHT)
				{
					stateVariables.requiredToolRight = stateVariables.requiredTool;
					std::cout<<"Required tool for left: "<<stateVariables.requiredToolRight<<"\n";
					std::cout<<"Current tool for left: "<<stateVariables.currentToolRight<<"\n";
					if(stateVariables.requiredToolRight != stateVariables.currentToolRight)
					{
						switchRightTool = true;
						currentRunState = MOVE_TO_TOOL;
					}
				}

				if(!switchLeftTool && !switchRightTool)
					currentRunState = MOVE_TO_CENTER;

				break;
			//-------------------------------------------------------------------------------//
			case MOVE_TO_TOOL:
				ROS_INFO("*SM* -> MOVE_TO_TOOL");

				base->move("tool");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = PICK_TOOL;
				break;
			//-------------------------------------------------------------------------------//
			case PICK_TOOL:
				ROS_INFO("*SM* -> PICK_TOOL");

				// Do arm movements
				if(switchLeftTool)
				{
					//Put down current tool
					//Pick up next tool
				}
				if(switchRightTool)
				{
					//Put down current tool
					//Pick up next tool
				}

				currentRunState = MOVE_TO_CENTER;
				break;
			//-------------------------------------------------------------------------------//
			case MOVE_TO_CENTER:
				ROS_INFO("*SM* -> MOVE_TO_CENTER");

				// Do arm movements
				if(switchLeftTool)
				{
					// Move arms to home position
				}
				if(switchRightTool)
				{
					// Move arms to home position
				}

				base->move("center");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = MOVE_ARM;
				break;
			//-------------------------------------------------------------------------------//
			case MOVE_ARM:
				ROS_INFO("*SM* -> MOVE_ARM");

				arms->getCurrentPose(	stateVariables.requiredArmCartesian,
										stateVariables.currentGripperPose);
				// Prepare arms for picking
				moveToBinPlane(stateVariables.currentBin, stateVariables.requiredArm, 0.4);
				//arms->waitForMotion(5.0);

				currentRunState = WAIT_FOR_MOTION;
				nextRunState = MOVE_TOWARDS_BIN;
				break;
			//-------------------------------------------------------------------------------//
			case MOVE_TOWARDS_BIN:
				ROS_INFO("*SM* -> MOVE_TOWARDS_BIN");

				base->move("col2");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = GET_OBJECT_LOCATION_2;
				break;
			//-------------------------------------------------------------------------------//
			case GET_OBJECT_LOCATION_2:
				ROS_INFO("*SM* -> GET_OBJECT_LOCATION");

				// Get bin info
				getBinInfo(stateVariables.currentBin, stateVariables.currentBinInfo);

				// Point Head
				head->sendGoalCart(	robot_frame,
									stateVariables.currentBinInfo.response.center.x,
									stateVariables.currentBinInfo.response.center.y,
									stateVariables.currentBinInfo.response.center.z,
									1.0);
/*
				// Start ORK
				srv_msg_SetBin.request.number = stateVariables.currentBin;
				if(! srvClient_startORK.call(srv_msg_SetBin) )
					ROS_ERROR("Could not start ORK");

				// Get Object pose
				srv_msg_GetObjectPose.request.obj_name = stateVariables.currentObject;
				srv_msg_GetObjectPose.response.found_obj = false;
				while(!srv_msg_GetObjectPose.response.found_obj)
				{
					if(! srvClient_get_object_pose.call(srv_msg_GetObjectPose) )
						ROS_ERROR("Could not stop ORK");

					ros::Duration(0.5).sleep();
				}

				// Stop ORK
				if(! srvClient_stopORK.call(srv_msg_Empty) )
					ROS_ERROR("Could not stop ORK");
*/
				// TODO: convert pose to string with orientation name

				///stateVariables.currentObjPose = srv_msg_GetObjectPose.response.pose; // TODO

				// Create a fake location for testing
				stateVariables.currentGripperPose.pose.position.x = stateVariables.currentBinInfo.response.center.x;
				stateVariables.currentGripperPose.pose.position.y = stateVariables.currentBinInfo.response.center.y;
				stateVariables.currentGripperPose.pose.position.z = stateVariables.currentBinInfo.response.center.z;
				stateVariables.currentGripperPose.pose.orientation.w = 1;
				std::cout << stateVariables.currentGripperPose;

				stateVariables.currentObjPose.position.x = stateVariables.currentBinInfo.response.center.x + 0.15;
				stateVariables.currentObjPose.position.y = stateVariables.currentBinInfo.response.center.y;
				stateVariables.currentObjPose.position.z = stateVariables.currentBinInfo.response.center.z;
				stateVariables.currentObjPose.orientation.w = 1;
				std::cout << stateVariables.currentObjPose;

				currentRunState = PICK_ITEM;
				break;
			//-------------------------------------------------------------------------------//
			case PICK_ITEM:
				ROS_INFO("*SM* -> PICK_ITEM");

				// Set gripper pose
				arms->getCurrentPose(	stateVariables.requiredArmCartesian,
										stateVariables.currentGripperPose);

				// Perform grasp
				grasp(	stateVariables.requiredStrategy,
						stateVariables.requiredArm,
						stateVariables.currentGripperPose,
						stateVariables.currentObjPose,
						stateVariables.currentBinInfo);

				currentRunState = MOVE_TO_CENTER_2;
				break;
			//-------------------------------------------------------------------------------//
			case MOVE_TO_CENTER_2:
				ROS_INFO("*SM* -> MOVE_TO_CENTER_2");

				base->move("center");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = MOVE_TO_ORDER_BIN;
				break;

			//-------------------------------------------------------------------------------//
			case MOVE_TO_ORDER_BIN:
				ROS_INFO("*SM* -> MOVE_TO_ORDER_BIN");

				base->move("bin");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = PLACE_ITEM;
				break;
			//-------------------------------------------------------------------------------//
			case PLACE_ITEM:
				ROS_INFO("*SM* -> PLACE_ITEM");

				arms->loadCartPose(stateVariables.requiredArmCartesian,ArmsCartesian::binHigh,stateVariables.currentGripperPose);
				arms->waitForMotion(max_arm_execution_time);

				arms->loadCartPose(stateVariables.requiredArmCartesian,ArmsCartesian::binMid,stateVariables.currentGripperPose);
				arms->waitForMotion(max_arm_execution_time);

				arms->loadCartPose(stateVariables.requiredArmCartesian,ArmsCartesian::binLow,stateVariables.currentGripperPose);
				arms->waitForMotion(max_arm_execution_time);


				if(stateVariables.currentToolLeft == ELECTRIC)
					grippers->position(0,-1,stateVariables.requiredArmGripper);

				if(stateVariables.currentToolRight == ELECTRIC)
					grippers->position(0,-1,stateVariables.requiredArmGripper);

				arms->loadCartPose(stateVariables.requiredArmCartesian,ArmsCartesian::binHigh,stateVariables.currentGripperPose);
				arms->waitForMotion(max_arm_execution_time);

				// Move arm to order bin

				currentRunState = MOVE_TO_HOME;
				break;
			//-------------------------------------------------------------------------------//
			case MOVE_TO_HOME:
				ROS_INFO("*SM* -> MOVE_TO_HOME");

				arms->loadCartPose(ArmsCartesian::LEFT,ArmsCartesian::homeLeft,stateVariables.currentGripperPoseLeft);
				arms->loadCartPose(ArmsCartesian::RIGHT,ArmsCartesian::homeRight,stateVariables.currentGripperPoseRight);
				arms->waitForMotion(max_arm_execution_time);

				base->move("center");
				//base->motionComplete();	 TODO implement, for now just wait
				ros::Duration(3.0).sleep();

				currentRunState = DONE_PICKING;
				break;
			//-------------------------------------------------------------------------------//
			case DONE_PICKING:
				ROS_INFO("*SM* -> DONE_PICKING");

				// some error handling
				if(items_left.vacuumLift == 0)
				{
					orderIter = work_order.electricFront.begin();
					ROS_INFO(" ## Done with vacuumLift!");
				}

				if(items_left.electricFront == 0)
				{
					orderIter = work_order.vacuumFront.begin();
					ROS_INFO(" ## Done with electricFront!");
				}

				if(items_left.vacuumFront == 0)
				{
					orderIter = work_order.vacuumFront.begin();
					ROS_INFO(" ## Done with electricFront!");

				}

				if(items_left.electricAngledLift == 0)
				{
					ROS_INFO(" ## Done with electricAngledLift!");
					executingRun = false;

				}

				if(!executingRun)
				{
					ROS_INFO("### DONE PICKING ###!");
					currentRunState = DONE;
				}
				else
				{
					currentRunState = SELECT_NEXT_ITEM;
				}
				break;
			//-------------------------------------------------------------------------------//
			//-------------------------------------------------------------------------------//
			case SELECT_NEXT_ITEM:
				ROS_INFO("*SM* -> SELECT_NEXT_ITEM");

				stateVariables.currentItemNumber++;
				currentRunState = START_PICKING;

				break;
			//-------------------------------------------------------------------------------//
			case DONE:
				ROS_INFO("runSM: DONE");
				return true;
				break;
			//-------------------------------------------------------------------------------//
			default:
				ROS_WARN("runSM: invalid state");
				break;
		}

		if(currentRunState == stop)
		{
			currentRunState = DONE;
		}

		ros::Duration(loopRateRunSM).sleep();
	}

	return true;
}

bool ApcCortex::checkIfMotionsComplete()
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


bool ApcCortex::testingMoveToBin()
{

	ROS_INFO("## Testing moving to bins ##");
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

//	  std::vector<double> joint_start;					// Joint position
//	  joint_start.push_back( 0.45712 );
//	  joint_start.push_back( 0.73362 );
//	  joint_start.push_back( 2.47815 );
//	  joint_start.push_back( 2.34047 );
//	  joint_start.push_back( -0.2323 );
//	  joint_start.push_back( -1.4691 );
//	  joint_start.push_back( 0.43756 );
//	  baxter_moveit.executeJointGoal(BaxterMoveit::RIGHT, joint_start);

	  while(counter<N)
	  {
		  int i = continuous_order[counter % length]; // Bin number

		  geometry_msgs::Point p;

			  bin_atempts[i-1]++;

			  ROS_INFO("Bin %d", i);
			  if( moveToBin(i, ApcCortex::LEFT) )
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
			  std::cout<<"------------------\n";
			  ros::Duration(2.0).sleep();

			  counter++;
	  }

	  ROS_INFO("## Success rate ##");
	  for(int i=0;i<12;i++)
	  {
		  std::cout<<"Bin "<<i+1<<": " << bin_successes[i] << "/" << bin_atempts[i] << "\n";
	  }

	  return true;
}

bool ApcCortex::testingCartesian()
{
	ROS_INFO("## Testing moving to bins ##");
	int bin_atempts[12]   = {0,0,0,0,0,0,0,0,0,0,0,0};
	int bin_successes[12] = {0,0,0,0,0,0,0,0,0,0,0,0};
	int continuous_order[] = {1,2,3,6,5,4,7,8,9,12,11,10,11,12,9,8,7,4,5,6,3,2}; 		//{5,6,9,8,7,4,5,6,3,2,1,4};
	int length = (sizeof(continuous_order)/sizeof(*continuous_order));
	int counter = 4;
	int N = counter+length * 1;

	geometry_msgs::Pose pose;

	while(counter<N)
	{
		int i = continuous_order[counter % length]; // Bin number


		bin_atempts[i-1]++;
		ROS_INFO("Bin %d", i);
		if( moveToBin(i, ApcCortex::RIGHT ) )
		{
			bin_successes[i-1] += 1;
		}
		std::cout<<"------------------\n";
		ros::Duration(2.0).sleep();

		counter++;
	}

	ROS_INFO("## Success rate ##");
	for(int i=0;i<12;i++)
	{
		std::cout<<"Bin "<<i+1<<": " << bin_successes[i] << "/" << bin_atempts[i] << "\n";
	}

	return true;
}

bool ApcCortex::testingCartPose()
{
	ROS_INFO("## Testing cartesian poses ##");

	geometry_msgs::PoseStamped pose;

//	double trans_p = 800;
//	double trans_d = 15;
//	double rot_p = 80;
//	double rot_d = 1.2;
//
//	std::vector<double> gains;
//	gains.resize(12);
//
//	gains[0]= trans_p;
//	gains[1]= trans_p;
//	gains[2]= trans_p;
//	gains[3]= rot_p;
//	gains[4]= rot_p;
//	gains[5]= rot_p;
//	gains[6]= trans_d;
//	gains[7]= trans_d;
//	gains[8]= trans_d;
//	gains[9] = rot_d;
//	gains[10]= rot_d;
//	gains[11]= rot_d;
//
//	ROS_INFO_STREAM("Changing Gains");
//	arms->setGains(gains,"left");

	pose.header.frame_id = robot_frame;
	//pose.header.stamp = ros::Time::now();

	ROS_INFO("Cart Pose homeLeft!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::homeLeft,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose homeRight!");
	arms->loadCartPose(ArmsCartesian::RIGHT, ArmsCartesian::homeRight,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose binHigh!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::binHigh,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose binMid!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::binMid,pose);
	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose binLow!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::binLow,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose toolVacuum!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::toolVacuum,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	ROS_INFO("Cart Pose toolScoop!");
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::toolScoop,pose);
	arms->waitForMotion(5.0);
	ROS_INFO(" -> done!");

	ros::Duration(2.0).sleep();

	return true;
}

bool ApcCortex::testingGripPose()
{
	ROS_INFO("## Testing gripping poses ##");

	geometry_msgs::PoseStamped pose;

	pose.header.frame_id = robot_frame;
	pose.header.stamp = ros::Time::now();

	ArmsCartesian::WhichArm arm;

	for(int i=0;i<2;i++)
	{
		if(i==0)
		{
			ROS_INFO("*** LEFT ARM ***");
			arm = ArmsCartesian::LEFT;
		}
		else
		{
			ROS_INFO("*** RIGHT ARM ***");
			arm = ArmsCartesian::RIGHT;
		}

		ROS_INFO("Grip Pose front!");
		arms->loadOrientation(arm, ArmsCartesian::front,pose);
		ros::Duration(2.0).sleep();

		ROS_INFO("Grip Pose side!");
		arms->loadOrientation(arm, ArmsCartesian::side,pose);
		ros::Duration(2.0).sleep();

		ROS_INFO("Grip Pose degree45!");
		arms->loadOrientation(arm, ArmsCartesian::degree45,pose);
		ros::Duration(2.0).sleep();

		ROS_INFO("Grip Pose degree90!");
		arms->loadOrientation(arm, ArmsCartesian::degree90,pose);
		ros::Duration(2.0).sleep();
	}

	return true;
}

bool ApcCortex::testingCartesianMoveInDirection()
{
	geometry_msgs::PoseStamped pose;

	pose.header.frame_id = robot_frame;
	pose.header.stamp = ros::Time::now();

	pose.pose.position.x = 0.4;
	pose.pose.position.z = 1.0;
	pose.pose.orientation.w = 1;

	double distance = 0.30;
	double dx = 0.01;
	double dt = 0.1;
	ArmsCartesian::Direction d1 = ArmsCartesian::X;
	ArmsCartesian::Direction d2 = ArmsCartesian::Y;
	ArmsCartesian::Direction d3 = ArmsCartesian::Z;
	ArmsCartesian::WhichArm arm;

	geometry_msgs::PoseStamped tmp;
	tmp.header.frame_id = "base_link";

	for(int i=0;i<2;i++)
	{
		if(i==0)
		{
			ROS_INFO("*** LEFT ARM ***");
			arm = ArmsCartesian::LEFT;
			pose.pose.position.y = 0.2;
		}
		else
		{
			ROS_INFO("*** RIGHT ARM ***");
			arm = ArmsCartesian::RIGHT;
			pose.pose.position.y = -0.2;
		}
		tmp.header.stamp=ros::Time::now();
		arms->moveToPose(arm,pose.pose);
		ros::Duration(2.0).sleep();

		ROS_INFO("Moving forward in X");
		arms->moveInDirection(arm, pose, d1, distance, dx, dt);
		ros::Duration(1.0).sleep();
		ROS_INFO("Moving backward in X");
		arms->moveInDirection(arm,pose, d1, -distance, dx, dt);
		ros::Duration(2.0).sleep();

		ROS_INFO("Moving forward in Y");
		arms->moveInDirection(arm,pose, d2, distance, dx, dt);
		ros::Duration(1.0).sleep();
		ROS_INFO("Moving backward in Y");
		arms->moveInDirection(arm,pose, d2, -distance, dx, dt);
		ROS_INFO("...");
		ros::Duration(2.0).sleep();

		ROS_INFO("Moving forward in Z");
		arms->moveInDirection(arm,pose, d3, distance, dx, dt);
		ros::Duration(1.0).sleep();
		ROS_INFO("Moving backward in Z");
		arms->moveInDirection(arm,pose, d3, -
				distance, dx, dt);
		ROS_INFO("...");
		ros::Duration(2.0).sleep();

	}

	return true;
}

void ApcCortex::testingCartesianGetCurrentPose()
{
	geometry_msgs::PoseStamped l;
	geometry_msgs::PoseStamped r;

	if( arms->getCurrentPose(ArmsCartesian::LEFT,l) )
		std::cout<<"---\n"<<l;
	else
		std::cout<<"Left failed";

	if( arms->getCurrentPose(ArmsCartesian::RIGHT,r) )
		std::cout<<"---\n"<<r;
	else
		std::cout<<"Right failed";
}


void ApcCortex::testingCartesianArmGains(){

	//What to test
	bool should_test_t = true;
	bool should_test_r = false;

	//Define Low gains
	double* gain_t = new double[2];
	double* gain_r = new double[2];
	//P gain for translation
	gain_t[0] = 50.0;
	//D gain for translation
	gain_t[1] = 15.0;
	//P gain for rotation
	gain_r[0] = 80.0;
	//D gain for rotation
	gain_r[1] = 1.2;

	//Step size
	double step_t_p = 25;
	double step_t_d = 5.0;
	double step_r_p = 50;
	double step_r_d = 50;

	//Loop: Different Gains + Arm Testing
	for(int i=-1;i<1;i++){

		std::vector<double> gains;
		gains.resize(12);

		//Set different gains for translation
		double p_gain_t = gain_t[0] + (step_t_p*i);
		double d_gain_t = gain_t[1] + (step_t_d*i);
		double p_gain_r = gain_r[0] + (step_r_p*i);
        double d_gain_r = gain_r[1] + (step_r_d*i);

        std::cout << "p_gain_t:" << p_gain_t << " d_gain_t:" << d_gain_t << " p_gain_r:" << p_gain_r << " d_gain_r:" << d_gain_r << std::endl;

		gains[0]= p_gain_t;
		gains[1]= p_gain_t;
		gains[2]= p_gain_t;
		gains[3]= p_gain_r;
		gains[4]= p_gain_r;
		gains[5]= p_gain_r;
		gains[6]= d_gain_t;
		gains[7]= d_gain_t;
		gains[8]= d_gain_t;
		gains[9]= d_gain_r;
		gains[10]= d_gain_r;
		gains[11]= d_gain_r;

		ROS_INFO_STREAM("Changing Gains");
		arms->setGains(gains,"left");
		/*if(!srvClient_set_arm_gain.call(srv_)){
			ROS_ERROR("client call failed. LEFT");
			return;
		}
		srv_.request.arm = "right";
		if(!srvClient_set_arm_gain.call(srv_)){
			ROS_ERROR("client call failed. RIGHT");
			return;
		}*/
		ros::Duration(1.0).sleep();

		//Test Motion
		testingCartesianMoveInDirection();
	}
}

bool ApcCortex::grasp(ApcCortex::GraspStrategy g, ApcCortex::WhichArm a,geometry_msgs::PoseStamped& gripperPose, geometry_msgs::Pose objPose, apc_msgs::GetBinInfo binInfo )
{

	Gripper::WhichArm arm_gripper = getGripperArm(a);
	ArmsCartesian::WhichArm arm_cartesian = getCartesianArm(a);

	// Assumes everything in odom_origin frame? (gripper and object pose?)

	// Gripper pose

	binInfo.request.bin_number;
	binInfo.response.center;
	binInfo.response.height;
	binInfo.response.width;

	double dx, dy, dz;

	double dt = 0.05;
	double x_step = 0.01;
	int N_steps;
	bool goalReached;

//	if( !arms->getCurrentPose(arm_cartesian, gripperPose) )
//		ROS_INFO("Failed to get pose.");

	switch(g)
		{
		//-------------------------------------------------------------------------------//
		case FRONT:

			dx = objPose.position.x - gripperPose.pose.position.x;
			dy = objPose.position.y - gripperPose.pose.position.y;
			dz = objPose.position.z - gripperPose.pose.position.z;

			//grippers->open(arm_gripper);
			grippers->position(100,-1,arm_gripper);

			// Align with object
			gripperPose.pose.position.y = objPose.position.y;	// TODO check if it is far enough from the shelf/other objects
			gripperPose.pose.position.z = objPose.position.z;
			arms->moveToPose(arm_cartesian,gripperPose.pose,true);
			ros::Duration(1.0).sleep();

			// Move forward
			ROS_INFO("Moving forward");
			arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, dx, x_step, dt);
			ros::Duration(2.0).sleep();

			// Close gripper
			//grippers->close(arm_gripper);	// TODO set effort
			grippers->position(0,-1,arm_gripper);

			// Lift
			ROS_INFO("Lifting");
			// TODO check object height
			arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::Z, 0.05, x_step, dt);
			ros::Duration(2.0).sleep();

			// Move backward
			ROS_INFO("Moving backward");
			arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, -dx, x_step, dt);
			ros::Duration(2.0).sleep();

			// Clear bin
//			arms->getCurrentPose(arm_cartesian,gripperPose);
//			gripperPose.pose.orientation.x -= 2*x_step;
//			arms->moveToPose(arm_cartesian,gripperPose.pose);

			break;
			//-------------------------------------------------------------------------------//
			case ANGLEDLIFT:

				/*
				 * 1. Tilt arm
				 * 2. Calculate dx, dy, dz
				 * 3. Move forward
				 * 4. Close gripper
				 * 5. Move backwards
				 */



				/*****Tilt Gripper*****/
				arms->loadOrientation(arm_cartesian, ArmsCartesian::degree25,gripperPose);
				ros::Duration(2.0).sleep();

				/*****Open gripper*****/
				grippers->position(100,-1,arm_gripper);

				/*****Calculate dx, dy and dz*****/
				dx = objPose.position.x - gripperPose.pose.position.x;
				dy = objPose.position.y - gripperPose.pose.position.y;
				dz = objPose.position.z - gripperPose.pose.position.z;

				/*****Move forward*****/
				ROS_INFO("Moving forward");
//				base->
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, dx, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Arm lower*****/
				ROS_INFO("Lifting");
				// TODO check object height
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::Z, -0.05, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Close gripper*****/
				grippers->position(0,-1,arm_gripper);

				/*****Arm raised*****/
				ROS_INFO("Lifting");
				// TODO check object height
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::Z, 0.05, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Move Backwards*****/
				ROS_INFO("Moving backward");
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, -dx, x_step, dt);
				ros::Duration(2.0).sleep();

				ROS_INFO("grasp: DONE");
				return true;
				break;
			//-------------------------------------------------------------------------------//
			case VACUUMFRONT:
				/*
				 * 1. Forward
				 * 2. Down
				 * 3. Wait?
				 * 4. Up
				 * 5. Backwards
				 */

				/*****Forward*****/
				ROS_INFO("Moving forward");
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, dx, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Arm lower*****/
				ROS_INFO("Lifting");
				// TODO check object height
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::Z, -0.05, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Wait?*****/
				//TODO Implement wait if needed

				/*****Arm raised*****/
				ROS_INFO("Lifting");
				// TODO check object height
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::Z, 0.05, x_step, dt);
				ros::Duration(2.0).sleep();

				/*****Move Backwards*****/
				ROS_INFO("Moving backward");
				arms->moveInDirection(arm_cartesian, gripperPose, ArmsCartesian::X, -dx, x_step, dt);
				ros::Duration(2.0).sleep();

				ROS_INFO("grasp: DONE");
				return true;
				break;
			//-------------------------------------------------------------------------------//
			case LIFT:
				ROS_INFO("grasp: DONE");
				return true;
				break;
			//-------------------------------------------------------------------------------//
			default:
				ROS_WARN("grasp: invalid state");
				break;
		}

	return true;
}

bool ApcCortex::getArmPose(ApcCortex::ArmConfigurations a, geometry_msgs::PoseStamped& p)
{

	p.header.frame_id = "base_link";

	switch(a)
	{
	case L_HOME_01:
		p.pose.position.x     = 0.1;
		p.pose.position.y     = 0.4;
		p.pose.position.z     = 1.15;
		p.pose.orientation.x  = 0;
		p.pose.orientation.y  = 0;
		p.pose.orientation.z  = 0;
		p.pose.orientation.w  = 1;
		break;
	case R_HOME_01:
		p.pose.position.x     = 0.1;
		p.pose.position.y     = -0.4;
		p.pose.position.z     = 1.15;
		p.pose.orientation.x  = 0;
		p.pose.orientation.y  = 0;
		p.pose.orientation.z  = 0;
		p.pose.orientation.w  = 1;
		break;
	default:
		ROS_ERROR("ApcCortex::getArmPose - Arm position does not exist");
		return false;
	}

	return true;
}

bool ApcCortex::testingGrasping()
{
	geometry_msgs::PoseStamped pose;
	ROS_INFO("Moving arms to  home position");
	//Set arm to home positions
	arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::homeLeft,pose);
	arms->loadCartPose(ArmsCartesian::RIGHT, ArmsCartesian::homeRight,pose);
	arms->waitForMotion(5.0);

	ApcCortex::GraspStrategy currentGraspingStrategy = FRONT;
	ApcCortex::WhichArm graspingArm = RIGHT;
	apc_msgs::GetBinInfo currentBinInfo;
	geometry_msgs::PoseStamped currentGripperPose;
	geometry_msgs::Pose currentObjPose;

	int bin = 5;
	double x = 0.30;

	for(int i=4;i<13;i++)
	{
		graspingArm = determineArmFromBin(i);

		moveToBinPlane(i, graspingArm, x);

		// Create a fake location for testing
		// Get bin info
		getBinInfo(i, currentBinInfo);
		currentBinInfo.response.center.x = x+0.05; // fake bin location

		currentGripperPose.pose.position.x = currentBinInfo.response.center.x;
		currentGripperPose.pose.position.y = currentBinInfo.response.center.y;
		currentGripperPose.pose.position.z = currentBinInfo.response.center.z;
		currentGripperPose.pose.orientation.w = 1;
		currentGripperPose.header.frame_id="base_link";

		currentObjPose.position.x = currentBinInfo.response.center.x +0.35;
		currentObjPose.position.y = currentBinInfo.response.center.y;
		currentObjPose.position.z = currentBinInfo.response.center.z;
		currentObjPose.orientation.w = 1;

		std::cout<<currentObjPose;

		grasp(currentGraspingStrategy, graspingArm, currentGripperPose, currentObjPose, currentBinInfo);

		//Set arm to home positions
		if(graspingArm==LEFT)
			arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::homeLeft,pose);
		else
			arms->loadCartPose(ArmsCartesian::RIGHT, ArmsCartesian::homeRight,pose);

		while( !arms->motionComplete() )
		{
			std::cout<<"Waiting for arm to move home...";
			ros::Duration(1.0).sleep();
		}

	}
	return true;
}

ApcCortex::WhichArm ApcCortex::determineArmFromBin(int bin_number)
{
	int result = bin_number%3;
	if(result != 0) return ApcCortex::LEFT;
	else return ApcCortex::RIGHT;
}

Gripper::WhichArm ApcCortex::getGripperArm(ApcCortex::WhichArm a)
{
	switch(a)
	{
	case LEFT:
		return Gripper::LEFT;
		break;
	case RIGHT:
		return Gripper::RIGHT;
		break;
	case BOTH:
		return Gripper::BOTH;
		break;
	default:
		ROS_WARN("grasp: invalid ApcCortex::WhichArm");
		break;
	}

	return Gripper::BOTH;
}
ArmsCartesian::WhichArm ApcCortex::getCartesianArm(ApcCortex::WhichArm a)
{
	switch(a)
	{
	case LEFT:
		return ArmsCartesian::LEFT;
		break;
	case RIGHT:
		return ArmsCartesian::RIGHT;
		break;
	case BOTH:
		return ArmsCartesian::BOTH;
		break;
	default:
		ROS_WARN("grasp: invalid ApcCortex::WhichArm");
		break;
	}

	return ArmsCartesian::BOTH;
}

bool ApcCortex::testingToolPickUp(){

	/*
	 * Testing Procedure:
	 * 1. Move base to center
	 * 2. Move base to tool
	 * 3. Position the arm to pick up tool
	 * 4. Actually pick up the tool
	 *
	 */

	ROS_INFO_STREAM("Moving base to center");
	base->move("center");
	head->sendGoal(0.0,0.0,0.0,0.0);
	//ROS_INFO_STREAM("Moving base to tool");
	//base->move("tool");

	Tool toolRequest = TEST; // Defined in the Tool enum

	ROS_INFO("Cart Pose toolVacuum!");

	std::string robot_frame = "base_link";


	head->sendGoal(1.57,0.3,0.2,0.2);

	//arms->loadCartPose(ArmsCartesian::LEFT, ArmsCartesian::toolVacuum);
	//arms->waitForMotion(5.0);
	//ROS_INFO(" -> done!");

	/*apc_msgs::ToolPos srv_;
	srv_.request.marker_num = toolRequest;
	if(!srvClient_toolPose.call(srv_))
	{
		ROS_ERROR("Failed to get tool position");
	}


	geometry_msgs::PoseStamped currentToolPose;

	currentToolPose.pose.position.x = srv_.response.marker_position.pose.position.x;
	currentToolPose.pose.position.y = srv_.response.marker_position.pose.position.y;
	currentToolPose.pose.position.z = srv_.response.marker_position.pose.position.z - 0.1;
	currentToolPose.pose.orientation.w = 1;
	currentToolPose.header.frame_id=srv_.response.marker_position.header.frame_id;

	arms->moveToPose(ArmsCartesian::LEFT,currentToolPose.pose);

	while(!arms->motionComplete()){

	}*/

	return true;
}

bool ApcCortex::testingGripperTilt()
{

	/**
	 * Need:
	 * 	1. Strategy		(DONE)
	 * 	2. Arm			(DONE)
	 * 	3. GripperPose
	 * 	4. Object Pose
	 * 	5. BinInfo		(DONE)
	 */
	int bin = 9;
	double x = 0.30;
	ApcCortex::GraspStrategy currentGraspingStrategy = ANGLEDLIFT;
	ApcCortex::WhichArm graspingArm = RIGHT;
	apc_msgs::GetBinInfo currentBinInfo;
	geometry_msgs::PoseStamped currentGripperPose;
	geometry_msgs::Pose currentObjPose;

	graspingArm = determineArmFromBin(bin);
	getBinInfo(bin, currentBinInfo);

	currentGripperPose.pose.position.x = currentBinInfo.response.center.x;
	currentGripperPose.pose.position.y = currentBinInfo.response.center.y;
	currentGripperPose.pose.position.z = currentBinInfo.response.center.z;
	currentGripperPose.pose.orientation.w = 1;
	currentGripperPose.header.frame_id="base_link";

	currentObjPose.position.x = currentBinInfo.response.center.x +0.35;
	currentObjPose.position.y = currentBinInfo.response.center.y;
	currentObjPose.position.z = currentBinInfo.response.center.z;
	currentObjPose.orientation.w = 1;

	base->move("center");
	while(!base->motionComplete())
	{

	}

	base->move("col2");
	base->move("center");
	while(!base->motionComplete())
	{

	}

	grasp(	currentGraspingStrategy,
			graspingArm,
			currentGripperPose,
			currentObjPose,
			currentBinInfo);

}


