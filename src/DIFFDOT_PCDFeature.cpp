#include <aginika_pcl_ros/DIFFDOT_PCDFeature.h>

DIFFDOT_PCDFeature::DIFFDOT_PCDFeature(){
  feature_name_ = "DIFFDOT";
  resolution_ = 0.05;
  radius_search_ = 0.02;
  normal_nums_ = 50;
  normals_large_scale_ = 0.1;
  normals_small_scale_ = 0.02;
};

void DIFFDOT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
  // Compute normals using both small and large scales at each point

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::NormalEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> ne;
  ne.setInputCloud (input_normals);
  ne.setSearchMethod (tree);

  // calculate normals with the large scale
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_large_scale (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

  ne.setRadiusSearch (normals_large_scale_);
  ne.compute (*normals_large_scale);

  // Create DoN operator
  pcl::DifferenceOfNormalsEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal> don;
  don.setInputCloud (input_normals);
  don.setNormalScaleLarge (normals_large_scale);
  don.setNormalScaleSmall (input_normals);

  if (!don.initCompute ())
    {
      std::cerr << "Error: Could not intialize DoN feature operator" << std::endl;
      exit (EXIT_FAILURE);
    }

  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr doncloud (new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // Compute DoN
  don.computeFeature (*doncloud);

  pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> search_octree (resolution_);
  search_octree.setInputCloud (doncloud);
  search_octree.addPointsFromInputCloud ();

  srand(time(NULL));

  std::vector< float > dot_array;
  for(int i = 0; i < normal_nums_; i ++){
    int target_id = rand() % doncloud->points.size() ;
    std::vector< int > k_indices;
    std::vector< float > k_sqr_distances;

    search_octree.nearestKSearch (target_id, 1, k_indices, k_sqr_distances);
    pcl::PointXYZRGBNormal point1 = doncloud->points[target_id];
    pcl::PointXYZRGBNormal point2 = doncloud->points[target_id];
    float dot_value = point1.x * point2.x + point1.y * point2.y + point1.z * point2.z;
    dot_array.push_back(dot_value);
  }
  std::sort(dot_array.begin(),dot_array.end(),std::greater<float>());

  features_.push_back(dot_array);

  ROS_INFO("DIFFDOT_PCDFeature calculate");
};

