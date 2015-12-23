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
 * shelf_detection.cpp
 *
 *  Created on: May 22, 2015
 *      Author: Isura Ranatunga
 */

#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>

#include <apc_msgs/SetLimits.h>
#include <apc_msgs/SetBool.h>
#include <apc_msgs/SetPose.h>

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

#include <boost/format.hpp>

// PCL specific includes
#include <pcl/common/time.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>

#include <pcl/console/parse.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

#include <pcl/search/pcl_search.h>

#include <pcl/tracking/tracking.h>
#include <pcl/tracking/particle_filter.h>
#include <pcl/tracking/kld_adaptive_particle_filter_omp.h>
#include <pcl/tracking/particle_filter_omp.h>
#include <pcl/tracking/coherence.h>
#include <pcl/tracking/distance_coherence.h>
#include <pcl/tracking/hsv_color_coherence.h>
#include <pcl/tracking/approx_nearest_pair_point_cloud_coherence.h>
#include <pcl/tracking/nearest_pair_point_cloud_coherence.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/registration/icp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/ransac.h>

#include <pcl/segmentation/sac_segmentation.h>

//#include <pcl/registration/sample_consensus_prerejective.h>

ros::Publisher shelfDetectionPub;
ros::Publisher filteredCloudPub;
ros::Publisher particleCloudPub;
ros::Publisher markerPub;

using namespace pcl::tracking;

// Types
typedef pcl::PointXYZRGBA RefPointType;
typedef ParticleXYZRPY ParticleT;
typedef pcl::PointCloud<pcl::PointXYZRGBA> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;
typedef ParticleFilterTracker<RefPointType, ParticleT> ParticleFilter;

// Robust pose estimation of rigid objects
typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;

// ICP
pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

CloudPtr cloud_in_;
CloudPtr cloud_pass_;
CloudPtr cloud_pass_downsampled_;
CloudPtr target_cloud;
CloudPtr result_cloud;
CloudPtr transed_ref_downsampled;

boost::shared_ptr<ParticleFilter> tracker_;
bool new_cloud_;
double downsampling_grid_size_ = 0.05;
int counter;

double x_min;
double x_max;
double y_min;
double y_max;
double z_min;
double z_max;

double x_shelf;
double y_shelf;
double z_shelf;
double r_shelf;
double p_shelf;
double w_shelf;

tf::StampedTransform transform_;
Eigen::Affine3d transformation_ = Eigen::Affine3d::Identity();
Eigen::Affine3f shelfToKinectTransformation_ = Eigen::Affine3f::Identity();
Eigen::Vector3f centroidInShelfTF = Eigen::Vector3f::Zero();
std::string base_frame_  = "/odom_combined";
std::string shelf_frame_  = "/shelf_origin";
std::string kinect_frame_ = "/head_mount_kinect_rgb_optical_frame";
std::string saveFileName_ = "shelf_99.pcd";

Eigen::Affine3f particleCenter;

bool transformClouds = false;
bool savePCD = false;

enum APC_Tracker { PARTICLE, NORMAL, ICP };
int tracker_type_ = PARTICLE;

bool zero_init_ = false;

void filterPassThrough( CloudPtr &cloud,
		                Cloud &result,
						std::string axis,
						double min,
						double max)
{
  pcl::PassThrough<RefPointType> pass;
  pass.setFilterFieldName (axis);
  pass.setFilterLimits (min, max);
  pass.setKeepOrganized (false);
  pass.setInputCloud (cloud);
  pass.filter (result);
}

//Filter along a specified dimension
void filterPassThrough (const CloudConstPtr &cloud, Cloud &result)
{
  CloudPtr input;
  input.reset (new Cloud(*cloud));

  filterPassThrough (input, result, "x", x_min, x_max);
  input.reset (new Cloud(result));
  filterPassThrough (input, result, "y", y_min, y_max);
  input.reset (new Cloud(result));
  filterPassThrough (input, result, "z", z_min, z_max);
}

void voxelGridFilter (const CloudConstPtr &cloud, Cloud &cloud_filtered, double leaf_size)
{
  // Create the filtering object
  pcl::VoxelGrid<RefPointType> sor;
  sor.setInputCloud (cloud);
  sor.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  sor.filter (cloud_filtered);
}

void gridSampleApprox (const CloudConstPtr &cloud, Cloud &result, double leaf_size)
{
  pcl::ApproximateVoxelGrid<RefPointType> grid;
  grid.setLeafSize (static_cast<float> (leaf_size), static_cast<float> (leaf_size), static_cast<float> (leaf_size));
  grid.setInputCloud (cloud);
  grid.filter (result);
}

void trackObjectParticleFilter()
{
	// Track the object
	tracker_->setInputCloud (cloud_pass_downsampled_);
	tracker_->compute ();
	new_cloud_ = true;

	ParticleXYZRPY result = tracker_->getResult ();
	shelfToKinectTransformation_ = tracker_->toEigenMatrix (result);

	//move close to camera a little for better visualization
	// FIXME do we need this?
	shelfToKinectTransformation_.translation () += Eigen::Vector3f (0.0f, 0.0f, -0.005f);
	result_cloud.reset (new Cloud ());
	pcl::transformPointCloud<RefPointType> (*(tracker_->getReferenceCloud ()), *result_cloud, shelfToKinectTransformation_);
}

void poseEstimationNormals()
{
	// Point clouds
	PointCloudT::Ptr object (new PointCloudT);
	PointCloudT::Ptr object_aligned (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	pcl::copyPointCloud(*transed_ref_downsampled, *object);
	pcl::copyPointCloud(*cloud_pass_downsampled_, *scene);

	const float leaf = 0.005f;

	// Estimate normals for scene
	//pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimationOMP<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (scene);
	nest.compute (*scene);

	// Estimate features
	//pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);

	// Perform alignment
	//pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusInitialAlignment<PointNT,PointNT,FeatureT> align;

	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);

//	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
//	align.setCorrespondenceRandomness (5); // Number of nearest features to use
//	align.setSimilarityThreshold (0.9f); // Polygonal edge length similarity threshold

	align.setMinSampleDistance (0.05f);
	align.setMaxCorrespondenceDistance (2.5f * leaf); // Inlier threshold
	align.setMaximumIterations (50000); // Number of RANSAC iterations

//	align.setInlierFraction (0.25f); // Required inlier fraction for accepting a pose hypothesis

	{
		//pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
		pcl::copyPointCloud(*object_aligned, *result_cloud);

//		result.fitness_score = (float) align.getFitnessScore (max_correspondence_distance_);
//		result.final_transformation = align.getFinalTransformation ();
	}

	if (align.hasConverged ())
	{
	// Print results
//	printf ("\n");
//	Eigen::Matrix4f transformation = align.getFinalTransformation ();
//	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
//	pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
//	pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
//	pcl::console::print_info ("\n");
//	pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
//	pcl::console::print_info ("\n");
//	pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());

	// Show alignment
//	pcl::visualization::PCLVisualizer visu("Alignment");
//	visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 0.0), "scene");
//	visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 0.0, 0.0, 255.0), "object_aligned");
//	visu.spin ();
	}
	else
	{
//	pcl::console::print_error ("Alignment failed!\n");
//	return (1);
	}
}

void icpAlign()
{
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>);


	pcl::copyPointCloud(*transed_ref_downsampled, *cloud_out);
	pcl::copyPointCloud(*cloud_pass_downsampled_, *cloud_in );

	icp.setInputCloud(cloud_in);
	icp.setInputTarget(cloud_out);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	pcl::copyPointCloud(Final, *result_cloud);

	ROS_INFO_STREAM( "Has converged:" << icp.hasConverged() << " Score: " << icp.getFitnessScore());
	ROS_INFO_STREAM(icp.getFinalTransformation());


}

bool drawParticles(std::string &frame_id)
{
	sensor_msgs::PointCloud2 particle_output;
	pcl::PointCloud<pcl::PointXYZ>::Ptr particle_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
	bool visualize_particles_ = true;

    ParticleFilter::PointCloudStatePtr particles = tracker_->getParticles ();
    if (particles)
    {
      if (visualize_particles_)
      {
        for (size_t i = 0; i < particles->points.size (); i++)
        {
          pcl::PointXYZ point;

          point.x = particles->points[i].x;
          point.y = particles->points[i].y;
          point.z = particles->points[i].z;
          particle_cloud->points.push_back (point);
        }
      }

      pcl::toROSMsg (*particle_cloud, particle_output);
      particle_output.header.frame_id  = frame_id;
      particleCloudPub.publish(particle_output);

      return true;
    }
    else
    {
      ROS_WARN ("no particles");
      return false;
    }
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input_msg)
{
	// Convert to PCL
	Cloud input, cloud;

	std::string cloud_frame_id = input_msg->header.frame_id;

	if( transformClouds )
	{
		// Transform the point cloud to a specific frame
		// Used this to generate a shelf cloud in the shelf origin frame
		pcl::fromROSMsg (*input_msg, input);
		pcl::transformPointCloud(input, cloud, transformation_);
		cloud_frame_id = shelf_frame_;
	}else
	{
		pcl::fromROSMsg (*input_msg, cloud);
	}

    cloud_in_.reset (new Cloud(cloud));
    cloud_pass_.reset (new Cloud);
	cloud_pass_downsampled_.reset (new Cloud);

	// Crop
	filterPassThrough (cloud_in_, *cloud_pass_);

	// Downsample
	voxelGridFilter (cloud_pass_, *cloud_pass_downsampled_, downsampling_grid_size_);

	switch( tracker_type_ )
	{
	    case PARTICLE :
	    	// Tracking uses cloud_pass_downsampled_ global and updates result_cloud global
	    	trackObjectParticleFilter();
	    	break;
	    case NORMAL :
			poseEstimationNormals();
			break;
	    case ICP :
	    	icpAlign();
			break;
	    default:
	    	ROS_ERROR_STREAM( tracker_type_ << " is not a valid tracker type!" );
	}

    // Convert back to ROS
    sensor_msgs::PointCloud2 filtered_output;
    sensor_msgs::PointCloud2 output;
    pcl::toROSMsg (*cloud_pass_downsampled_, filtered_output);
    pcl::toROSMsg (*result_cloud, output);
    output.header.frame_id = cloud_frame_id;
    filtered_output.header.frame_id = cloud_frame_id;
    shelfDetectionPub.publish(output);
    filteredCloudPub.publish(filtered_output);

    drawParticles(cloud_frame_id);

    if( savePCD )
    {
    	// Save shelf to PCD
    	pcl::io::savePCDFileASCII(saveFileName_, *cloud_pass_);
    }

}

void rosViz()
{
	visualization_msgs::Marker marker;
	// Set the frame ID and timestamp.  See the TF tutorials for information on these.
	marker.header.frame_id = kinect_frame_;
	marker.header.stamp = ros::Time(0);

	// Set the namespace and id for this marker.  This serves to create a unique ID
	// Any marker sent with the same namespace and id will overwrite the old one
	marker.ns = "basic_shapes";
	marker.id = 0;

	// Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
	marker.type = visualization_msgs::Marker::SPHERE;

	// Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
	marker.action = visualization_msgs::Marker::ADD;

	Eigen::Vector3f shelfTF = shelfToKinectTransformation_*(-centroidInShelfTF);

	// Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
	marker.pose.position.x = shelfTF.x(); // robotToShelfTransformation_.translation().x(); // particleCenter.inverse().translation().x();
	marker.pose.position.y = shelfTF.y(); // robotToShelfTransformation_.translation().y(); // particleCenter.inverse().translation().y();
	marker.pose.position.z = shelfTF.z(); // robotToShelfTransformation_.translation().z(); // particleCenter.inverse().translation().z();
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;

	// Set the scale of the marker -- 1x1x1 here means 1m on a side
	marker.scale.x = 0.05;
	marker.scale.y = 0.05;
	marker.scale.z = 0.05;

	// Set the color -- be sure to set alpha to something non-zero!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.color.a = 1.0;

	//marker.lifetime = ros::Duration();

	markerPub.publish(marker);

}

bool updateLimits(apc_msgs::SetLimits::Request &req, apc_msgs::SetLimits::Response &res)
{
	x_min = req.x_min;
	x_max = req.x_max;
	y_min = req.y_min;
	y_max = req.y_max;
	z_min = req.z_min;
	z_max = req.z_max;

	if(x_max < x_min)
	{
		ROS_WARN("Passthrough filter: x_max < x_min (switching values)");
		x_min = x_max;
		x_max = req.x_min;
	}
	if(y_max < y_min)
	{
		ROS_WARN("Passthrough filter: y_max < y_min (switching values)");
		y_min = y_max;
		y_max = req.y_min;
	}
	if(z_max < z_min)
	{
		ROS_WARN("Passthrough filter: z_max < z_min (switching values)");
		z_min = z_max;
		z_max = req.z_min;
	}

	ROS_INFO("Updated filter limits: x=[%f,%f],y=[%f,%f],z=[%f,%f]",x_min,x_max,y_min,y_max,z_min,z_max);

	return true;
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "apc_shelf_detector");
	ros::NodeHandle nh;

	//read pcd file
	target_cloud.reset(new Cloud());

	if (argc > 2)
	{
		if(pcl::io::loadPCDFile (argv[1], *target_cloud) == -1)
		{
			ROS_ERROR_STREAM( "pcd file not found!");
		}
	}else
	{
		std::string pcdFileName = "shelf_1.pcd";
		std::string para_pcdFileName = "/tracker/pcdFileName";
		if (!nh.getParam( para_pcdFileName, pcdFileName )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_pcdFileName.c_str()) ; return false; }

		std::string path = ros::package::getPath("apc_shelf");
		path += "/models/" + pcdFileName;
		ROS_INFO_STREAM("Loading " << path << " using rospack!");

		if(pcl::io::loadPCDFile (path, *target_cloud) == -1)
		{
			ROS_ERROR_STREAM( "pcd file not found!");
		}

	}

	int loop_rate = 10;

	// Read from parameter server
	std::string para_base_frame_             = "/tracker/base_frame";
	std::string para_shelf_frame_             = "/tracker/shelf_frame";
	std::string para_kinect_frame_            = "/tracker/kinect_frame";
    std::string para_transformClouds          = "/tracker/transformClouds";
	std::string para_savePCD                  = "/tracker/savePCD";
	std::string para_downsampling_grid_size   = "/tracker/downsampling_grid_size";

	std::string para_bin_size_x               = "/tracker/bin_size/x";
	std::string para_bin_size_y               = "/tracker/bin_size/y";
	std::string para_bin_size_z               = "/tracker/bin_size/z";
	std::string para_bin_size_roll            = "/tracker/bin_size/roll";
	std::string para_bin_size_pitch           = "/tracker/bin_size/pitch";
	std::string para_bin_size_yaw             = "/tracker/bin_size/yaw";

	std::string para_maximumParticleNum       = "/tracker/maximumParticleNum";
	std::string para_delta                    = "/tracker/delta";
	std::string para_epsilon                  = "/tracker/epsilon";
	std::string para_iterationNum             = "/tracker/iterationNum";
	std::string para_particleNum              = "/tracker/particleNum";
	std::string para_resampleLikelihoodThr    = "/tracker/resampleLikelihoodThr";
	std::string para_useNormal                = "/tracker/useNormal";

	std::string para_x_min                    = "/tracker/x_min";
	std::string para_x_max                    = "/tracker/x_max";
	std::string para_y_min                    = "/tracker/y_min";
	std::string para_y_max                    = "/tracker/y_max";
	std::string para_z_min                    = "/tracker/z_min";
	std::string para_z_max                    = "/tracker/z_max";

	std::string para_tracker_type             = "/tracker/type";
	std::string para_zero_init                = "/tracker/zero_init";
	std::string para_loop_rate                = "/tracker/loop_rate";

	std::string para_x_shelf                  = "/tracker/x_shelf";
	std::string para_y_shelf                  = "/tracker/x_shelf";
	std::string para_z_shelf                  = "/tracker/y_shelf";
	std::string para_r_shelf                  = "/tracker/r_shelf";
	std::string para_p_shelf                  = "/tracker/p_shelf";
	std::string para_w_shelf                  = "/tracker/w_shelf";

	x_shelf = -0.226453  ;
	y_shelf = -0.0462694 ;
	z_shelf =  1.3426    ;
	r_shelf =  0.0       ;
	p_shelf =  0.0       ;
	w_shelf =  0.0       ;

	if (!nh.getParam( para_x_shelf, x_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_shelf.c_str()) ; return false; }
	if (!nh.getParam( para_x_shelf, y_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_shelf.c_str()) ; return false; }
	if (!nh.getParam( para_y_shelf, z_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_z_shelf.c_str()) ; return false; }
	if (!nh.getParam( para_r_shelf, r_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_r_shelf.c_str()) ; return false; }
	if (!nh.getParam( para_p_shelf, p_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_p_shelf.c_str()) ; return false; }
	if (!nh.getParam( para_y_shelf, w_shelf )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_w_shelf.c_str()) ; return false; }

	if (!nh.getParam( para_tracker_type, tracker_type_ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_tracker_type.c_str()) ; return false; }
	if (!nh.getParam( para_zero_init   , zero_init_    )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_zero_init.   c_str()) ; return false; }
	if (!nh.getParam( para_loop_rate   , loop_rate     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_loop_rate.   c_str()) ; return false; }

	switch( tracker_type_ )
	{
	    case PARTICLE :
	    	ROS_INFO_STREAM( "Particle tracker!" );
	    	break;
	    case NORMAL :
	    	ROS_INFO_STREAM( "Normal tracker!" );
			break;
	    case ICP :
	    	ROS_INFO_STREAM( "ICP tracker!" );
			break;
	    default:
	    	ROS_INFO_STREAM( tracker_type_ << " is not a valid tracker type!" );
	}

	if (!nh.getParam( para_base_frame_              , base_frame_             )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_base_frame_           .c_str()) ; return false; }
	if (!nh.getParam( para_shelf_frame_             , shelf_frame_            )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_shelf_frame_          .c_str()) ; return false; }
	if (!nh.getParam( para_kinect_frame_            , kinect_frame_           )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_kinect_frame_         .c_str()) ; return false; }
	if (!nh.getParam( para_transformClouds          , transformClouds         )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_transformClouds       .c_str()) ; return false; }
    if (!nh.getParam( para_savePCD                  , savePCD                 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_savePCD               .c_str()) ; return false; }
	if (!nh.getParam( para_downsampling_grid_size   , downsampling_grid_size_ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_downsampling_grid_size.c_str()) ; return false; }

	double bin_size_x     = 0.1f;
	double bin_size_y     = 0.1f;
	double bin_size_z     = 0.1f;
	double bin_size_roll  = 0.1f;
	double bin_size_pitch = 0.1f;
	double bin_size_yaw   = 0.1f;

	if (!nh.getParam( para_bin_size_x               , bin_size_x     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_x    .c_str()) ; return false; }
	if (!nh.getParam( para_bin_size_y               , bin_size_y     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_y    .c_str()) ; return false; }
	if (!nh.getParam( para_bin_size_z               , bin_size_z     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_z    .c_str()) ; return false; }
	if (!nh.getParam( para_bin_size_roll            , bin_size_roll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_roll .c_str()) ; return false; }
    if (!nh.getParam( para_bin_size_pitch           , bin_size_pitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_pitch.c_str()) ; return false; }
	if (!nh.getParam( para_bin_size_yaw             , bin_size_yaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_bin_size_yaw  .c_str()) ; return false; }

	double maximumParticleNum    = 500   ;
	double delta                 = 0.99  ;
	double epsilon               = 0.2   ;
	double iterationNum          = 1     ;
	double particleNum           = 10    ;
	double resampleLikelihoodThr = 0.00  ;
	bool   useNormal             = false ;

	if (!nh.getParam( para_maximumParticleNum       , maximumParticleNum    )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_maximumParticleNum   .c_str()) ; return false; }
    if (!nh.getParam( para_delta                    , delta                 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_delta                .c_str()) ; return false; }
	if (!nh.getParam( para_epsilon                  , epsilon               )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_epsilon              .c_str()) ; return false; }
	if (!nh.getParam( para_iterationNum             , iterationNum          )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_iterationNum         .c_str()) ; return false; }
	if (!nh.getParam( para_particleNum              , particleNum           )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_particleNum          .c_str()) ; return false; }
    if (!nh.getParam( para_resampleLikelihoodThr    , resampleLikelihoodThr )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_resampleLikelihoodThr.c_str()) ; return false; }
	if (!nh.getParam( para_useNormal                , useNormal             )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useNormal            .c_str()) ; return false; }

	x_min = -5.0; // -0.2;
	x_max =  5.0; //  0.5;
	y_min = -5.0; // -5.0;
	y_max =  2.0; //  0.2;
	z_min =  0.0; // -0.2;
	z_max =  3.0; //  5.0;

    if (!nh.getParam( para_x_min , x_min )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_min.c_str()) ; return false; }
    if (!nh.getParam( para_x_max , x_max )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_x_max.c_str()) ; return false; }
    if (!nh.getParam( para_y_min , y_min )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_min.c_str()) ; return false; }
    if (!nh.getParam( para_y_max , y_max )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_y_max.c_str()) ; return false; }
    if (!nh.getParam( para_z_min , z_min )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_z_min.c_str()) ; return false; }
    if (!nh.getParam( para_z_max , z_max )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_z_max.c_str()) ; return false; }

	std::string para_saveFileName = "/tracker/saveFileName";
	if (!nh.getParam( para_saveFileName, saveFileName_ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_saveFileName.c_str()) ; return false; }

	double icpMaxCorrespondenceDistance              = 0.05 ;
	int    icpMaximumIterations                      = 50   ;
	double icpTransformationEpsilon                  = 1e-8 ;
	double icpEuclideanFitnessEpsilon                = 1    ;

	std::string para_icpMaxCorrespondenceDistance    = "/tracker/icpMaxCorrespondenceDistance";
	std::string para_icpMaximumIterations            = "/tracker/icpMaximumIterations";
	std::string para_icpTransformationEpsilon        = "/tracker/icpTransformationEpsilon";
	std::string para_icpEuclideanFitnessEpsilon      = "/tracker/icpEuclideanFitnessEpsilon";

	if (!nh.getParam( para_icpMaxCorrespondenceDistance , icpMaxCorrespondenceDistance )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_icpMaxCorrespondenceDistance.c_str()) ; return false; }
	if (!nh.getParam( para_icpMaximumIterations         , icpMaximumIterations         )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_icpMaximumIterations        .c_str()) ; return false; }
	if (!nh.getParam( para_icpTransformationEpsilon     , icpTransformationEpsilon     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_icpTransformationEpsilon    .c_str()) ; return false; }
	if (!nh.getParam( para_icpEuclideanFitnessEpsilon   , icpEuclideanFitnessEpsilon   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_icpEuclideanFitnessEpsilon  .c_str()) ; return false; }

	tf::TransformListener tf_listener_;

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("/head_mount_kinect/depth_registered/points", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	shelfDetectionPub = nh.advertise<sensor_msgs::PointCloud2> ("shelf_detector_output", 1);
	filteredCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("filtered_shelf_output", 1);
	particleCloudPub = nh.advertise<sensor_msgs::PointCloud2> ("paticle_shelf_output", 1);

	markerPub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);

	//Service
    ros::ServiceServer srv_setLimits = nh.advertiseService("apc/passthrough_filter/setLimits",updateLimits);
    ros::ServiceClient client_setOdomOrigin = nh.serviceClient<apc_msgs::SetPose>("apc/tf/set_odom_origin");

	// Tracker Init
	counter = 0;

	// Get transformation from the shelf origin to the Kinect frame
	try
	{
		tf_listener_.waitForTransform(shelf_frame_, kinect_frame_, ros::Time(0), ros::Duration(5.0));
		tf_listener_.lookupTransform(shelf_frame_, kinect_frame_, ros::Time(0), transform_);
		tf::transformTFToEigen (transform_, transformation_);
	}
	catch (tf::TransformException ex)
	{
		ROS_INFO("Waiting on TF cache to build: %s",ex.what());
	 }

	//Set parameters
	new_cloud_  = false;

	std::vector<double> default_step_covariance = std::vector<double> (6, 0.015 * 0.015);
	default_step_covariance[3] *= 40.0;
	default_step_covariance[4] *= 40.0;
	default_step_covariance[5] *= 40.0;

	std::vector<double> initial_noise_covariance = std::vector<double> (6, 0.00001);
	std::vector<double> default_initial_mean = std::vector<double> (6, 0.0);

	boost::shared_ptr<KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> > tracker
	(new KLDAdaptiveParticleFilterOMPTracker<RefPointType, ParticleT> (8));

	ParticleT bin_size;
	bin_size.x     = bin_size_x    ;
	bin_size.y     = bin_size_y    ;
	bin_size.z     = bin_size_z    ;
	bin_size.roll  = bin_size_roll ;
	bin_size.pitch = bin_size_pitch;
	bin_size.yaw   = bin_size_yaw  ;

	//Set all parameters for  KLDAdaptiveParticleFilterOMPTracker
	tracker->setMaximumParticleNum (maximumParticleNum);
	tracker->setDelta              (delta             );
	tracker->setEpsilon            (epsilon           );
	tracker->setBinSize            (bin_size          );

	//Set all parameters for  ParticleFilter
	tracker_ = tracker;
	tracker_->setTrans (Eigen::Affine3f::Identity());
	tracker_->setStepNoiseCovariance    (default_step_covariance );
	tracker_->setInitialNoiseCovariance (initial_noise_covariance);
	tracker_->setInitialNoiseMean       (default_initial_mean    );
	tracker_->setIterationNum           (iterationNum            );
	tracker_->setParticleNum            (particleNum             );
	tracker_->setResampleLikelihoodThr  (resampleLikelihoodThr   );
	tracker_->setUseNormal              (useNormal               );

	// Set ICP params
	// Set the max correspondence distance to 5cm (e.g., correspondences with higher distances will be ignored)
	icp.setMaxCorrespondenceDistance (icpMaxCorrespondenceDistance);
	// Set the maximum number of iterations (criterion 1)
	icp.setMaximumIterations (icpMaximumIterations);
	// Set the transformation epsilon (criterion 2)
	icp.setTransformationEpsilon (icpTransformationEpsilon);
	// Set the euclidean distance difference epsilon (criterion 3)
	icp.setEuclideanFitnessEpsilon (icpEuclideanFitnessEpsilon);

	//Setup coherence object for tracking
	ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr coherence = ApproxNearestPairPointCloudCoherence<RefPointType>::Ptr
	(new ApproxNearestPairPointCloudCoherence<RefPointType> ());

	boost::shared_ptr<DistanceCoherence<RefPointType> > distance_coherence
	= boost::shared_ptr<DistanceCoherence<RefPointType> > (new DistanceCoherence<RefPointType> ());
	coherence->addPointCoherence (distance_coherence);

	boost::shared_ptr<pcl::search::Octree<RefPointType> > search (new pcl::search::Octree<RefPointType> (0.01));
	coherence->setSearchMethod (search);
	coherence->setMaximumDistance (0.01);

	tracker_->setCloudCoherence (coherence);

	//prepare the model of tracker's target
	Eigen::Vector4f c;
	Eigen::Affine3f trans = Eigen::Affine3f::Identity(); //Eigen::Affine3f(transformation_);

	CloudPtr transed_ref (new Cloud);
	transed_ref_downsampled.reset(new Cloud);

	pcl::compute3DCentroid<RefPointType> (*target_cloud, c);
	trans.translation ().matrix () = Eigen::Vector3f (c[0], c[1], c[2]);
	pcl::transformPointCloud<RefPointType> (*target_cloud, *transed_ref, trans.inverse());
	voxelGridFilter (transed_ref, *transed_ref_downsampled, downsampling_grid_size_);

	//set reference model and trans
	tracker_->setReferenceCloud (transed_ref_downsampled);

	particleCenter = Eigen::Affine3f(transformation_);

	centroidInShelfTF = Eigen::Vector3f(c[0], c[1], c[2]);
//	Eigen::Vector3f centroidInKinectTF = particleCenter.inverse()*centroidInShelfTF;

	particleCenter.translation() -= centroidInShelfTF;

	Eigen::Affine3f defaultPose = Eigen::Affine3f::Identity();

	defaultPose.matrix() <<  -0.0534306,   -0.99814  ,   -0.0293459,   x_shelf   ,
                             -0.342264 ,    0.0459133,   -0.938482 ,   y_shelf   ,
	                          0.938084 ,   -0.0400996,   -0.34408  ,   z_shelf   ,
	                          0        ,    0        ,    0        ,   1         ;

//	Eigen::RotationBase
//	defaultPose.fromPositionOrientationScale()
//	defaultPose.rotation() << 	 -0.0533768   -0.99813 -0.0297734 ,
//			                     -0.340437   0.046219  -0.939131  ,
//			                      0.938751 -0.0399919  -0.342268  ;

// translation -0.226453 -0.0462694 1.3426

	if( zero_init_ )
		tracker_->setTrans (defaultPose);
	else
	{
		tracker_->setTrans (particleCenter.inverse());
	}

	ROS_INFO_STREAM( "Zero init: " << zero_init_ );

//	ROS_INFO_STREAM( tracker_->getTrans().matrix() );
	ROS_INFO_STREAM( "Translation: " << tracker_->getTrans().translation().transpose() );

	// Update rate TODO use flag instead
	ros::Rate r(loop_rate);

	geometry_msgs::Pose shelf_pose_in_base;

	shelf_pose_in_base.position.x = 1.2;
	shelf_pose_in_base.position.y = 0.9;
	shelf_pose_in_base.position.z = 0.0;

	shelf_pose_in_base.orientation.x = 0.0;
	shelf_pose_in_base.orientation.y = 0.0;
	shelf_pose_in_base.orientation.z = 0.2;
	shelf_pose_in_base.orientation.w = 1.0;

	// Main loop
	while (nh.ok())
	{
		try
		{
			tf_listener_.waitForTransform(shelf_frame_, kinect_frame_, ros::Time(0), ros::Duration(5.0));
			tf_listener_.lookupTransform(shelf_frame_, kinect_frame_, ros::Time(0), transform_);
			tf::transformTFToEigen (transform_, transformation_);
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Waiting on TF cache to build: %s",ex.what());
		 }

		geometry_msgs::PointStamped shelfPointInKinectFrame;
		shelfPointInKinectFrame.header.frame_id = kinect_frame_;
		shelfPointInKinectFrame.header.stamp = ros::Time(0);

		Eigen::Vector3f shelfTF = shelfToKinectTransformation_*(-centroidInShelfTF);

		shelfPointInKinectFrame.point.x = shelfTF.x();
		shelfPointInKinectFrame.point.y = shelfTF.y();
		shelfPointInKinectFrame.point.z = shelfTF.z();

		try
		{
			tf::StampedTransform transformKinectToOdomCombined;
			tf_listener_.waitForTransform(shelf_frame_, kinect_frame_, ros::Time(0), ros::Duration(5.0));
			tf_listener_.lookupTransform(base_frame_, kinect_frame_, ros::Time(0), transformKinectToOdomCombined);
			geometry_msgs::PointStamped base_point;
			tf_listener_.transformPoint(base_frame_, shelfPointInKinectFrame, base_point);

			Eigen::Affine3d kinectToOdomCombined;
			tf::transformTFToEigen(transformKinectToOdomCombined, kinectToOdomCombined);
			Eigen::Affine3d shelfTransformInOdomCombined = kinectToOdomCombined*Eigen::Affine3d(shelfToKinectTransformation_);

			Eigen::Quaterniond q(shelfTransformInOdomCombined.rotation());

			shelf_pose_in_base.position.x = base_point.point.x;
			shelf_pose_in_base.position.y = base_point.point.y;
			shelf_pose_in_base.position.z = 0.0;

			shelf_pose_in_base.orientation.x = q.x();
			shelf_pose_in_base.orientation.y = q.y();
			shelf_pose_in_base.orientation.z = q.z();
			shelf_pose_in_base.orientation.w = q.w();

			apc_msgs::SetPose srv;
			srv.request.pose = shelf_pose_in_base;

			client_setOdomOrigin.call(srv);
		}
		catch (tf::TransformException ex)
		{
			ROS_INFO("Waiting on TF cache to build: %s",ex.what());
		 }

		rosViz();

		ros::spinOnce();
		r.sleep();
	}

	return 0;
}




