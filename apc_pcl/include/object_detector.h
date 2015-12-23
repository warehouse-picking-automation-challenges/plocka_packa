


// Simulate input from depth/3D sensors

typedef pcl::PointCloud<pcl::PointXYZ>::Ptr CloudPtr;
pcl::apps::RenderViewsTesselatedSphere render_views;
render_views.setResolution (resolution_);
render_views.setTesselationLevel (1); //80 views
render_views.addModelFromPolyData (mapper); //vtk model
render_views.setGenOrganized(false);
render_views.generateViews ();
std::vector< CloudPtr > views;
std::vector < Eigen::Matrix4f > poses;
render_views.getViews (views);
render_views.getPoses (poses);


// Correspondence Grouping

// Geometric consistency
pcl::CorrespondencesPtr m_s_corrs; //fill it
std::vector<pcl::Correspondences> clusters;
pcl::GeometricConsistencyGrouping<PT, PT> gc_clusterer;
gc_clusterer.setGCSize (cg_size);
gc_clusterer.setGCThreshold (cg_thres);
gc_clusterer.setInputCloud (m_keypoints);
gc_clusterer.setSceneCloud (s_keypoints);
gc_clusterer.setModelSceneCorrespondences (m_s_corrs);
gc_clusterer.cluster (clusters);

// Hough
typedef pcl::ReferenceFrame RFType;
pcl::PointCloud<RFType>::Ptr model_rf; //fill with RFs
pcl::PointCloud<RFType>::Ptr scene_rf; //fill with RFs
pcl::CorrespondencesPtr m_s_corrs; //fill it
std::vector<pcl::Correspondences> clusters;
pcl::Hough3DGrouping<PT, PT, RFType, RFType> hc;
hc.setHoughBinSize (cg_size);
hc.setHoughThreshold (cg_thres);
hc.setUseInterpolation (true);
hc.setUseDistanceWeight (false);
hc.setInputCloud (m_keypoints);
hc.setInputRf (model_rf);
hc.setSceneCloud (s_keypoints);
hc.setSceneRf (scene_rf);
hc.setModelSceneCorrespondences (m_s_corrs);
hc.cluster (clusters);

// Hypothesis Verification
pcl::GreedyVerification<pcl::PointXYZ, pcl::PointXYZ> greedy_hv(lambda
greedy_hv.setResolution (0.005f);
greedy_hv.setInlierThreshold (0.005f);
greedy_hv.setSceneCloud (scene);
greedy_hv.addModels (aligned_hypotheses, true);
greedy_hv.verify ();
std::vector<bool> mask_hv;
greedy_hv.getMask (mask_hv);

pcl::PapazovHV<pcl::PointXYZ, pcl::PointXYZ> papazov;
papazov.setResolution (0.005f);
papazov.setInlierThreshold (0.005f);
papazov.setSupportThreshold (0.08f); //inliers
papazov.setPenaltyThreshold (0.05f); //outliers
papazov.setConflictThreshold (0.02f);
papazov.setSceneCloud (scene);
papazov.addModels (aligned_hypotheses, true);
papazov.verify ();
std::vector<bool> mask_hv;
papazov.getMask (mask_hv);

pcl::GlobalHypothesesVerification<pcl::PointXYZ, pcl::PointXYZ> go;
go.setResolution (0.005f);
go.setInlierThreshold (0.005f);
go.setRadiusClutter (0.04f);
go.setRegularizer (3.f); //outliers’ model weight
go.setClutterRegularizer (5.f); //clutter points weight
go.setDetectClutter (true);
go.setSceneCloud (scene);
go.addModels (aligned_hypotheses, true);
go.verify ();
std::vector<bool> mask_hv;
go.getMask (mask_hv);

