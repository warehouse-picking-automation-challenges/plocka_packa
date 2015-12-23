cmake_minimum_required(VERSION 2.8.3)
project(apc_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  std_srvs
)

add_message_files(
  FILES
  BinContents.msg
  WorkOrder.msg
  JTCartesianControllerState.msg
)

add_service_files(DIRECTORY srv FILES
  GetObjectPose.srv
  SetShelfOrigin.srv
  GetBinOrigin.srv
  GetBinInfo.srv
  SetPose.srv
  SetBool.srv
  SetDouble.srv
  SetLimits.srv
  SetBaseDistance.srv
  SetBaseRotation.srv
  SetBaseLocation.srv
  SetBaseParameter.srv
  SetBin.srv
  GetGraspStrategy.srv
  Empty.srv
  GetBinContents.srv
  GetWorkOrder.srv
  GetBool.srv
  ToolPos.srv
  SetGain.srv
  SetImageCrop.srv
  ReturnJointStates.srv
  GetItemFromWorkOrder.srv
  GetRequestedItem.srv
	GetQRCodeForRequestedItem.srv
)

generate_messages(
   DEPENDENCIES
   geometry_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  std_srvs
)
