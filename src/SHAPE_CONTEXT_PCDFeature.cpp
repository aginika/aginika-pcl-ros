#include <aginika_pcl_ros/SHAPE_CONTEXT_PCDFeature.h>

SHAPE_CONTEXT_PCDFeature::SHAPE_CONTEXT_PCDFeature():radius_search_(0.03){
  feature_name_ = "SHAPE_CONTEXT";
};

void SHAPE_CONTEXT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){

  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr kdtree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

  pcl::ShapeContext3DEstimation<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::ShapeContext1980> shape_context;
  shape_context.setInputCloud (input_normals);
  shape_context.setInputNormals (input_normals);
  shape_context.setSearchMethod (kdtree);

  pcl::PointCloud<pcl::ShapeContext1980>::Ptr shape_context_features (new pcl::PointCloud<pcl::ShapeContext1980>);
  shape_context.setRadiusSearch (0.05);
  shape_context.setPointDensityRadius (0.012);
  shape_context.setMinimalRadius (0.007);
  shape_context.compute (*shape_context_features);

  int target_index = rand()%(shape_context_features->points.size());
  float* pf = shape_context_features->points[target_index].descriptor;
  std::vector<float> result = std::vector<float>(pf, pf+1980);

  // std::vector<float> result;
  // for(int index = 0; index < 1980; index++){
  //   float sum_hist_points = 0;
  //   for(int kndex = 0; kndex < shape_context_features->points.size();kndex++)
  //     sum_hist_points+=shape_context_features->points[kndex].descriptor[index];
  //   result.push_back( sum_hist_points/shape_context_features->points.size() );
  // }

  features_.push_back(result);

  ROS_INFO("SHAPE_CONTEXT_PCDFeature calculate");
};

