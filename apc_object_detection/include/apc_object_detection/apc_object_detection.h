/*
 * apc_object_detection.h
 *
 *  Created on: Mar 6, 2015
 *      Author: Sven Cremer
 */

#ifndef APC_OBJECT_DETECTION_H_
#define APC_OBJECT_DETECTION_H_

#include <ros/ros.h>

//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/image_encodings.h>
//#include <image_transport/image_transport.h>
//#include <cv_bridge/cv_bridge.h>
//#include <opencv2/imgproc/imgproc.hpp>
//#include <opencv2/highgui/highgui.hpp>
//#include <math.h>

#include <object_recognition_msgs/RecognizedObject.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>
#include <geometry_msgs/PoseWithCovariance.h>

#include "visualization_msgs/Marker.h"

#include <actionlib/client/simple_action_client.h>
#include <object_recognition_msgs/ObjectRecognitionAction.h>
#include <object_recognition_msgs/ObjectRecognitionGoal.h>
#include <tf/transform_listener.h>

#include <apc_msgs/GetObjectPose.h>
#include <apc_msgs/SetLimits.h>
#include <apc_msgs/SetBool.h>
#include <apc_msgs/GetBinInfo.h>
#include <apc_msgs/SetBin.h>
//#include <apc_msgs/SetImageCrop.h>

#include <dynamic_reconfigure/server.h>
#include <apc_object_detection/BinConfig.h>

struct Object {
  std::string key;
  std::string name;
} ;

struct RecObject {
  std::string key;									// TODO count number of data points received where the locations are the same
  std::string name;
  double confidence;
  geometry_msgs::PoseWithCovarianceStamped location;
} ;

class ApcObjectDectection
{
private:
	ros::NodeHandle nh;

	std::vector<Object> objects;

	ros::Time last_update;
	std::vector<RecObject> recognized_objects;
	double confidence_threshold;

	ros::Subscriber sub_recognized_obj_;
	void recognizedObjCB(const object_recognition_msgs::RecognizedObjectArray::ConstPtr& msg);

	ros::ServiceServer srv_get_obj_pose_;
	bool getObjectPoseCB(apc_msgs::GetObjectPose::Request &req, apc_msgs::GetObjectPose::Response &res);
	
	void callback(apc_pcl::BinConfig &config, uint32_t level);

//	ros::ServiceServer srv_image_crop_;
//	bool setImageCropLimitsCB(apc_msgs::SetImageCrop::Request &req, apc_msgs::SetImageCrop::Response &res);

	ros::ServiceClient client_setLimits_;
	ros::ServiceClient client_getBinInfo_;
	ros::ServiceClient client_enablePCL_;

	dynamic_reconfigure::Server<apc_pcl::BinConfig> dynamicReconfigureServer;
	dynamic_reconfigure::Server<apc_pcl::BinConfig>::CallbackType f;

	ros::Publisher pub_marker_;


	actionlib::SimpleActionClient<object_recognition_msgs::ObjectRecognitionAction>* ac;
	tf::TransformListener listener;

	std::string result;

	double x_min;
	double y_min;
	double z_min;
	double x_max;
	double y_max;
	double z_max;

	double min_dx;
	double min_dy;
	double min_dz;
	double max_dx;
	double max_dy;
	double max_dz;

//	// Cropping Kinect Images
//	image_transport::Subscriber sub_kinect_rgb_;
//	void kinectRGBImageCB(const sensor_msgs::ImageConstPtr& msg);
//
//	image_transport::Subscriber sub_kinect_depth_;
//	void kinectDepthImageCB(const sensor_msgs::ImageConstPtr& msg);
//
//	image_transport::Publisher pub_kinect_rgb_;
//	image_transport::Publisher pub_kinect_depth_;
//
//	int roi_x,roi_y, roi_width, roi_height;
//	bool cropped;
//	bool pubhlish_cropped_image;


public:

	ApcObjectDectection();
	~ApcObjectDectection();

	std::string getObjectName(std::string key);

	void print();
	void publishMarker();

	void getObject();

	bool filterPCL(int bin);
	void startObjectDetection(int bin);

};



#endif /* APC_OBJECT_DETECTION_H_ */
