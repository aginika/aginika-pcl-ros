#include <aginika_pcl_ros/AllPassFilter.h>

AllPassFilter::AllPassFilter(){
  filter_name_ = "all_pass";
}

void AllPassFilter::filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals){
  ROS_INFO("All Pass filter");
}
