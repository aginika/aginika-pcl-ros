#include <aginika_pcl_ros/DOT_PCDFeature.h>

DOT_PCDFeature::DOT_PCDFeature(){
  feature_name_ = "DOT";
};

void DOT_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
  ROS_INFO("DOT_PCDFeature calculate");
};

