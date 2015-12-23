/*
 * aligment.cpp
 *
 *  Created on: May 13, 2015
 *      Author: Modified tutorial from http://pointclouds.org/documentation/tutorials/template_alignment.php
 */


#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>

#include <apc_msgs/SetLimits.h>
#include <apc_msgs/SetBool.h>

// PCL specific includes
#include <pcl/ros/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_listener.h>


#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/point_types.h>
//#include <pcl/PCLPointCloud2.h>
//#include <pcl/conversions.h>
//#include <pcl_ros/transforms.h>

// PCL alignment
#include <limits>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

class FeatureCloud
{
  public:
    // A bit of shorthand
    typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
    typedef pcl::PointCloud<pcl::Normal> SurfaceNormals;
    typedef pcl::PointCloud<pcl::FPFHSignature33> LocalFeatures;
    typedef pcl::search::KdTree<pcl::PointXYZ> SearchMethod;

    FeatureCloud () :
      search_method_xyz_ (new SearchMethod),
      normal_radius_ (0.02f),
      feature_radius_ (0.02f)
    {}

    ~FeatureCloud () {}

    // Process the given cloud
    void
    setInputCloud (PointCloud::Ptr xyz)
    {
      xyz_ = xyz;
      processInput ();
    }

    // Load and process the cloud in the given PCD file
    void
    loadInputCloud (const std::string &pcd_file)
    {
      xyz_ = PointCloud::Ptr (new PointCloud);
      pcl::io::loadPCDFile (pcd_file, *xyz_);
      processInput ();
    }

    // Get a pointer to the cloud 3D points
    PointCloud::Ptr
    getPointCloud () const
    {
      return (xyz_);
    }

    // Get a pointer to the cloud of 3D surface normals
    SurfaceNormals::Ptr
    getSurfaceNormals () const
    {
      return (normals_);
    }

    // Get a pointer to the cloud of feature descriptors
    LocalFeatures::Ptr
    getLocalFeatures () const
    {
      return (features_);
    }

  protected:
    // Compute the surface normals and local features
    void
    processInput ()
    {
      computeSurfaceNormals ();
      computeLocalFeatures ();
    }

    // Compute the surface normals
    void
    computeSurfaceNormals ()
    {
      normals_ = SurfaceNormals::Ptr (new SurfaceNormals);

      pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> norm_est;
      norm_est.setInputCloud (xyz_);
      norm_est.setSearchMethod (search_method_xyz_);
      norm_est.setRadiusSearch (normal_radius_);
      norm_est.compute (*normals_);
    }

    // Compute the local feature descriptors
    void
    computeLocalFeatures ()
    {
      features_ = LocalFeatures::Ptr (new LocalFeatures);

      pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
      fpfh_est.setInputCloud (xyz_);
      fpfh_est.setInputNormals (normals_);
      fpfh_est.setSearchMethod (search_method_xyz_);
      fpfh_est.setRadiusSearch (feature_radius_);
      fpfh_est.compute (*features_);
    }

  private:
    // Point cloud data
    PointCloud::Ptr xyz_;
    SurfaceNormals::Ptr normals_;
    LocalFeatures::Ptr features_;
    SearchMethod::Ptr search_method_xyz_;

    // Parameters
    float normal_radius_;
    float feature_radius_;
};

class TemplateAlignment
{
  public:

    // A struct for storing alignment results
    struct Result
    {
      float fitness_score;
      Eigen::Matrix4f final_transformation;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    TemplateAlignment () :
      min_sample_distance_ (0.05f),
      max_correspondence_distance_ (0.01f*0.01f),
      nr_iterations_ (500) // 500
    {
      // Intialize the parameters in the Sample Consensus Intial Alignment (SAC-IA) algorithm
      sac_ia_.setMinSampleDistance (min_sample_distance_);
      sac_ia_.setMaxCorrespondenceDistance (max_correspondence_distance_);
      sac_ia_.setMaximumIterations (nr_iterations_);
    }

    ~TemplateAlignment () {}

    // Set the given cloud as the target to which the templates will be aligned
    void
    setTargetCloud (FeatureCloud &target_cloud)
    {
      target_ = target_cloud;
      sac_ia_.setInputTarget (target_cloud.getPointCloud ());
      sac_ia_.setTargetFeatures (target_cloud.getLocalFeatures ());
    }

    // Add the given cloud to the list of template clouds
    void
    addTemplateCloud (FeatureCloud &template_cloud)
    {
      templates_.push_back (template_cloud);
    }

    // Align the given template cloud to the target specified by setTargetCloud ()
    void
    align (FeatureCloud &template_cloud, TemplateAlignment::Result &result)
    {
      sac_ia_.setInputCloud (template_cloud.getPointCloud ());
      sac_ia_.setSourceFeatures (template_cloud.getLocalFeatures ());

      pcl::PointCloud<pcl::PointXYZ> registration_output;
      sac_ia_.align (registration_output);

      result.fitness_score = (float) sac_ia_.getFitnessScore (max_correspondence_distance_);
      result.final_transformation = sac_ia_.getFinalTransformation ();
    }

    // Align all of template clouds set by addTemplateCloud to the target specified by setTargetCloud ()
    void
    alignAll (std::vector<TemplateAlignment::Result, Eigen::aligned_allocator<Result> > &results)
    {
      results.resize (templates_.size ());
      for (size_t i = 0; i < templates_.size (); ++i)
      {
        align (templates_[i], results[i]);
      }
    }

    // Align all of template clouds to the target cloud to find the one with best alignment score
    int
    findBestAlignment (TemplateAlignment::Result &result)
    {
      // Align all of the templates to the target cloud
      std::vector<Result, Eigen::aligned_allocator<Result> > results;
      alignAll (results);

      // Find the template with the best (lowest) fitness score
      float lowest_score = std::numeric_limits<float>::infinity ();
      int best_template = 0;
      for (size_t i = 0; i < results.size (); ++i)
      {
        const Result &r = results[i];
        if (r.fitness_score < lowest_score)
        {
          lowest_score = r.fitness_score;
          best_template = (int) i;
        }
      }

      // Output the best alignment
      result = results[best_template];
      return (best_template);
    }

  private:
    // A list of template clouds and the target to which they will be aligned
    std::vector<FeatureCloud> templates_;
    FeatureCloud target_;

    // The Sample Consensus Initial Alignment (SAC-IA) registration routine and its parameters
    pcl::SampleConsensusInitialAlignment<pcl::PointXYZ, pcl::PointXYZ, pcl::FPFHSignature33> sac_ia_;
    float min_sample_distance_;
    float max_correspondence_distance_;
    int nr_iterations_;
};

std::vector<FeatureCloud> object_templates;

ros::Publisher pub;
tf::TransformListener *tf_listener;

bool publishPCL;

double x_min;
double x_max;
double y_min;
double y_max;
double z_min;
double z_max;

// rosservice call /apc/passthrough_filter/setLimits "{x_min: -0.5, x_max: 0.7, y_min: -0.5, y_max: 1.8, z_min: 0.7, z_max: 2.2}"
// rosservice call /apc/passthrough_filter/setLimits "{x_min: -10, x_max: 10, y_min: -10, y_max: 10, z_min: -10, z_max: 10}"
// rosservice call /apc/passthrough_filter/setLimits "{x_min: -0.15, x_max: 0.6, y_min: -0.96, y_max: 0.15, z_min: 0.68, z_max: 1.93}"

//double x_min = -10.0;
//double x_max = 10.0;
//double y_min = -10.0;
//double y_max = 10.0;
//double z_min = -10.0;
//double z_max = 10.0;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud)
{
	if(!publishPCL)
		return;

	/* Transform into fixed frame */
	sensor_msgs::PointCloud2 tf_cloud;
	if(!pcl_ros::transformPointCloud("odom_origin", *cloud, tf_cloud, *tf_listener))
	{
		ROS_WARN("Point cloud could not be transformed");
		return;
	}
	sensor_msgs::PointCloud2ConstPtr tf_cloud_ptr (new sensor_msgs::PointCloud2(tf_cloud));

	/* Filter cloud */
	sensor_msgs::PointCloud2 cloud_filtered;
	sensor_msgs::PointCloud2 cloud_filtered1;
	sensor_msgs::PointCloud2 cloud_filtered2;

	// Create the filtering objects
	pcl::PassThrough<sensor_msgs::PointCloud2> passX;
	pcl::PassThrough<sensor_msgs::PointCloud2> passY;
	pcl::PassThrough<sensor_msgs::PointCloud2> passZ;

	// Filter x
	passX.setInputCloud (tf_cloud_ptr);
	passX.setFilterFieldName ("x");
	passX.setFilterLimits(x_min, x_max);
	passX.filter (cloud_filtered);

	// Filter y
	sensor_msgs::PointCloud2ConstPtr cloud_filtered_x_ptr (new sensor_msgs::PointCloud2(cloud_filtered));
	passY.setInputCloud (cloud_filtered_x_ptr);
	passY.setFilterFieldName ("y");
	passY.setFilterLimits(y_min, y_max);
	passY.filter (cloud_filtered1);

	// Filter z
	sensor_msgs::PointCloud2ConstPtr cloud_filtered_xy_ptr (new sensor_msgs::PointCloud2(cloud_filtered1));
	passZ.setInputCloud (cloud_filtered_xy_ptr);
	passZ.setFilterFieldName ("z");
	passZ.setFilterLimits(z_min, z_max);
	passZ.filter (cloud_filtered2);

	//pass.setFilterLimitsNegative (true);

	// Publish the data
	pub.publish (cloud_filtered2);					// TODO only publish bin PCL; perfomr shelf detection inside node



	// Load the target cloud PCD file

	pcl::PointCloud<pcl::PointXYZ> pcl_temp;
	//pcl::fromROSMsg(cloud_filtered2, pcl_temp);
	pcl::fromROSMsg(tf_cloud, pcl_temp);
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>(pcl_temp));

//	pcl_conversions::
//	cloud_filtered2
//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_input (new pcl::PointCloud<pcl::PointXYZ>);

	  // Preprocess the cloud by...
	  // ...removing distant points
	  const float depth_limit = 10.0;
	  pcl::PassThrough<pcl::PointXYZ> pass;
	  pass.setInputCloud (cloud_input);
	  pass.setFilterFieldName ("z");
	  pass.setFilterLimits (0, depth_limit);
	  pass.filter (*cloud_input);

	  // ... and downsampling the point cloud
	  const float voxel_grid_size = 0.005f;
	  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
	  vox_grid.setInputCloud (cloud_input);
	  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
	  //vox_grid.filter (*cloud_input); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
	  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
	  vox_grid.filter (*tempCloud);
	  cloud_input = tempCloud;

	  // Assign to the target FeatureCloud
	  FeatureCloud target_cloud;
	  target_cloud.setInputCloud (cloud_input);

	  // Set the TemplateAlignment inputs
	  TemplateAlignment template_align;
	  for (size_t i = 0; i < object_templates.size (); ++i)
	  {
	    template_align.addTemplateCloud (object_templates[i]);
	  }
	  template_align.setTargetCloud (target_cloud);

	  ROS_INFO ("got here");

	  // Find the best template alignment
	  TemplateAlignment::Result best_alignment;
	  int best_index = template_align.findBestAlignment (best_alignment);
	  const FeatureCloud &best_template = object_templates[best_index];

	  // Print the alignment fitness score (values less than 0.00002 are good)
	  ROS_INFO ("Best fitness score: %f\n", best_alignment.fitness_score);

	  // Print the rotation matrix and translation vector
	  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
	  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

	  ROS_INFO ("\n");
	  ROS_INFO ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
	  ROS_INFO ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
	  ROS_INFO ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
	  ROS_INFO ("\n");
	  ROS_INFO ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

}

//void updateParam(ros::NodeHandle& nh)
//{
//	std::string path = "/apc/passthrough_filter/";
//	if(!(nh.getParam(std::string(path+"x/min"), x_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"x/max"), x_max))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"y/min"), y_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"y/max"), y_max))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"z/min"), z_min))){ROS_ERROR("Parameter Not found.");}
//	if(!(nh.getParam(std::string(path+"z/max"), z_max))){ROS_ERROR("Parameter Not found.");}
//}

bool enablePCL(apc_msgs::SetBool::Request &req, apc_msgs::SetBool::Response &res)
{
	publishPCL = req.variable;

	return true;
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

void loadPCDTemplates(std::string package_path)
{

	std::string path = package_path + "/data/object_templates.txt";

	// Load the object templates specified in the object_templates.txt file
	std::ifstream input_stream(path.c_str());
	object_templates.resize (0);
	std::string pcd_filename;
	while (input_stream.good ())
	{
		std::getline (input_stream, pcd_filename);
		if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
			continue;

		FeatureCloud template_cloud;
		template_cloud.loadInputCloud (package_path + "/data/" + pcd_filename);
		object_templates.push_back (template_cloud);
		ROS_INFO("Loading template");
	}

	input_stream.close ();

	ROS_INFO("Done");
}

int main (int argc, char** argv)
{
	// Initialize ROS
	ros::init (argc, argv, "apc_pcl");
	ros::NodeHandle nh;

//	updateParam(nh);
//	ROS_INFO("Loaded parameters");

	// Create a ROS subscriber for the input point cloud
	ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	// TODO pub = nh.advertise<sensor_msgs::PointCloud2> ("bin", 1);

	tf_listener    = new tf::TransformListener();

	//Service
	ros::ServiceServer srv_setLimits = nh.advertiseService("apc/passthrough_filter/setLimits",updateLimits);
	ros::ServiceServer srv_enablePCL = nh.advertiseService("apc/passthrough_filter/enablePCL",enablePCL);

	//ros::spin();

	// Update rate TODO use flag instead
	int rate = 10;
	ros::Rate r(rate);

	publishPCL = true;

	x_min = -0.15;
	x_max = 0.6;
	y_min = 0.15;
	y_max = -0.96;
	z_min = 0.68;
	z_max = 1.93;

	loadPCDTemplates("/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl"); //TODO ros::package::getPath("apc_pcl");

	// Main loop
	while (nh.ok())
	{
		//updateParam(nh);

		ros::spinOnce();		// check for incoming messages

		r.sleep();
	}

	return 0;
}
/*
// Align a collection of object templates to a sample point cloud
int
main (int argc, char **argv)
{

	// Initialize ROS
	ros::init (argc, argv, "apc_pcl");
	ros::NodeHandle nh;

	// Create a ROS publisher for the output point cloud
	pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

	int rate = 10;
	ros::Rate r(rate);

//  if (argc < 3)
//  {
//    printf ("No target PCD file given!\n");
//    return (-1);
//  }

//  std::string package_path = "/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl"; //ros::package::getPath("apc_pcl");
//  std::string path = package_path + "/data/object_templates.txt";
//
//  // Load the object templates specified in the object_templates.txt file
  std::vector<FeatureCloud> object_templates;
//  std::ifstream input_stream(path.c_str());
  object_templates.resize (0);
//  std::string pcd_filename;
//  while (input_stream.good ())
//  {
//    std::getline (input_stream, pcd_filename);
//    if (pcd_filename.empty () || pcd_filename.at (0) == '#') // Skip blank lines or comments
//      continue;
//
//    FeatureCloud template_cloud;
//    template_cloud.loadInputCloud (pcd_filename);
//    object_templates.push_back (template_cloud);
//  }
  FeatureCloud template_cloud;
  template_cloud.loadInputCloud ("/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl/data/bin_1_0.pcd");
  object_templates.push_back (template_cloud);
//  template_cloud.loadInputCloud ("/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl/data/bin_1_1.pcd");
//  object_templates.push_back (template_cloud);
//  template_cloud.loadInputCloud ("/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl/data/bin_1_2.pcd");
//  object_templates.push_back (template_cloud);


  //input_stream.close ();

  // Load the target cloud PCD file
  std::string target = "/home/sven/apcPR2_ws/src/apc-dev2/apc_pcl/data/bin_1_1.pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::io::loadPCDFile (target, *cloud);

  // Preprocess the cloud by...
  // ...removing distant points
  const float depth_limit = 3.0;
  pcl::PassThrough<pcl::PointXYZ> pass;
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, depth_limit);
  pass.filter (*cloud);

  // ... and downsampling the point cloud
  const float voxel_grid_size = 0.005f;	//32 secobnds
  //const float voxel_grid_size = 0.015f;		// 16 sec
  pcl::VoxelGrid<pcl::PointXYZ> vox_grid;
  vox_grid.setInputCloud (cloud);
  vox_grid.setLeafSize (voxel_grid_size, voxel_grid_size, voxel_grid_size);
  //vox_grid.filter (*cloud); // Please see this http://www.pcl-developers.org/Possible-problem-in-new-VoxelGrid-implementation-from-PCL-1-5-0-td5490361.html
  pcl::PointCloud<pcl::PointXYZ>::Ptr tempCloud (new pcl::PointCloud<pcl::PointXYZ>);
  vox_grid.filter (*tempCloud);
  cloud = tempCloud;

  // Assign to the target FeatureCloud
  FeatureCloud target_cloud;
  target_cloud.setInputCloud (cloud);

  // Set the TemplateAlignment inputs
  TemplateAlignment template_align;
  for (size_t i = 0; i < object_templates.size (); ++i)
  {
    template_align.addTemplateCloud (object_templates[i]);
  }
  template_align.setTargetCloud (target_cloud);


  // Find the best template alignment
  TemplateAlignment::Result best_alignment;
  int best_index = template_align.findBestAlignment (best_alignment);
  const FeatureCloud &best_template = object_templates[best_index];

  // Print the alignment fitness score (values less than 0.00002 are good)
  printf ("Best fitness score: %f\n", best_alignment.fitness_score);

  // Print the rotation matrix and translation vector
  Eigen::Matrix3f rotation = best_alignment.final_transformation.block<3,3>(0, 0);
  Eigen::Vector3f translation = best_alignment.final_transformation.block<3,1>(0, 3);

  printf ("\n");
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (0,0), rotation (0,1), rotation (0,2));
  printf ("R = | %6.3f %6.3f %6.3f | \n", rotation (1,0), rotation (1,1), rotation (1,2));
  printf ("    | %6.3f %6.3f %6.3f | \n", rotation (2,0), rotation (2,1), rotation (2,2));
  printf ("\n");
  printf ("t = < %0.3f, %0.3f, %0.3f >\n", translation (0), translation (1), translation (2));

  // Save the aligned template for visualization
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud (*best_template.getPointCloud (), transformed_cloud, best_alignment.final_transformation);
  pcl::io::savePCDFileBinary ("output.pcd", transformed_cloud);


  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg (transformed_cloud, output);
  output.header.frame_id = "odom_origin";

	// Main loop
	while (nh.ok())
	{
		pub.publish(output);

		r.sleep();
	}


  return (0);
}
*/

