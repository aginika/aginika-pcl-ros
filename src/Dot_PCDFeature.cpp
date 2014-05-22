#include <aginika_pcl_ros/Dot_PCDFeature.h>

DOT_PCDFeature::DOT_PCDFeature(){
  feature_name_ = "DOT";
  resolution_ = 0.05;
  radius_search_ = 0.02;
  normal_nums_ = 50;
};

void DOT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> search_octree (resolution_);
  search_octree.setInputCloud (input_normals);
  search_octree.addPointsFromInputCloud ();

  srand(time(NULL));
  int target_id = rand() % input_normals->points.size() ;
  std::vector< int > k_indices;
  std::vector< float > k_sqr_distances;

  std::vector< float > dot_array;
  for(int i = 0; i < normal_nums_; i ++){
    search_octree.nearestKSearch (target_id, 1, k_indices, k_sqr_distances);
    pcl::PointXYZRGBNormal point1 = input_normals->points[target_id];
    pcl::PointXYZRGBNormal point2 = input_normals->points[target_id];
    float dot_value = point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
    dot_array.push_back(dot_value);
  }
  std::sort(dot_array.begin(),dot_array.end(),std::greater<float>());

  features_.push_back(dot_array);

  ROS_INFO("DOT_PCDFeature calculate");
};

