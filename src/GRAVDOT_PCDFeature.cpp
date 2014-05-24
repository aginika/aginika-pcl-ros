#include <aginika_pcl_ros/GRAVDOT_PCDFeature.h>

GRAVDOT_PCDFeature::GRAVDOT_PCDFeature(){
  feature_name_ = "GRAVDOT";
  resolution_ = 0.05;
  radius_search_ = 0.02;
  normal_nums_ = 50;
  normals_large_scale_ = 0.1;
  normals_small_scale_ = 0.02;
};

void GRAVDOT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
  // Compute normals using both small and large scales at each point

  std::vector< float > dot_array;
  for(int i = 0; i < normal_nums_; i ++){
    int target_id = std::rand() % input_normals->points.size() ;
    std::vector< int > k_indices;
    std::vector< float > k_sqr_distances;

    pcl::PointXYZRGBNormal point1 = input_normals->points[target_id];
    float dot_value =  point1.z;
    dot_array.push_back(dot_value);
  }
  std::sort(dot_array.begin(),dot_array.end(),std::greater<float>());

  features_.push_back(dot_array);

  ROS_INFO("GRAVDOT_PCDFeature calculate");
};

