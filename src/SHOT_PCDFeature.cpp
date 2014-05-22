#include <aginika_pcl_ros/SHOT_PCDFeature.h>

SHOT_PCDFeature::SHOT_PCDFeature():radius_search_(0.03){
  feature_name_ = "SHOT";
};

void SHOT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){


  pcl::SHOTEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::SHOT352> descr_est;
  pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors (new pcl::PointCloud<pcl::SHOT352> ());
  descr_est.setRadiusSearch (radius_search_);

  descr_est.setInputCloud (input_normals);
  descr_est.setInputNormals (input_normals);
  descr_est.setSearchSurface (input_normals);
  descr_est.compute (*model_descriptors);

  int target_index = rand()%(model_descriptors->points.size());
  float* pf = model_descriptors->points[target_index].descriptor;
  std::vector<float> result = std::vector<float>(pf, pf+352);
  features_.push_back(result);
  ROS_INFO("SHOT_PCDFeature calculate");
};

