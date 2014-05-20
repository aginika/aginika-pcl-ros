#include <aginika_pcl_ros/PCDFeatureProcedure.h>
#include <aginika_pcl_ros/FPFH_PCDFeature.h>
#include <aginika_pcl_ros/FPFH_Average_PCDFeature.h>
#include <aginika_pcl_ros/AllPassFilter.h>
#include <aginika_pcl_ros/RandomRegionFilter.h>
#include <ros/ros.h>
#include <sstream>
int main(int argc, char* argv[]){
  ros::init(argc, argv,"test_pcd_feature");

  if (argc < 3){
    ROS_ERROR("Too few aruguments");
    ROS_ERROR("%s method_filter method_feature", argv[0]);
    ROS_ERROR("filter  0:AllPass");
    ROS_ERROR("        1:RandomRegion");
    ROS_ERROR("feature 0:FPFH (random chose)");
    ROS_ERROR("         :FPFH_Average");
    exit(-1);
  }

  std::stringstream ss;
  int method_filter = 0, method_feature = 0;
  ss << argv[1];
  ss >> method_filter;
  std::stringstream ss2;
  ss2 << argv[2];
  ss2 >> method_feature;

  PCDPrevFilter* filter;
  PCDFeature* feature;

  switch(method_filter){
  case 0:
    {
      filter = new AllPassFilter();
    }
    break;
  case 1:
    {
      filter = new RandomRegionFilter();
    }
    break;
  default:
    {
      ROS_ERROR("Filter method %d is invalid", method_filter);
      exit(-1);
    }
  };

  switch(method_feature){
  case 0:
    {
      feature = new FPFH_PCDFeature();
    }
    break;
  case 1:
    {
      feature = new FPFH_Average_PCDFeature();
    }
    break;
  default:
    {
      ROS_ERROR("Feature method %d is invalid", method_feature);
      exit(-1);
    }
  };

  ros::NodeHandle n;
  PCDFeatureProcedure* pcd_fp = new PCDFeatureProcedure();
  // FPFH_PCDFeature* fpfh_pcd_feature = new FPFH_PCDFeature();
  // AllPassFilter* all_pass_filter = new AllPassFilter();
  // RandomRegionFilter* random_region_filter = new RandomRegionFilter();

  pcd_fp->setFilter(filter);
  pcd_fp->setFeature(feature);
  // pcd_fp->setFeature(fpfh_pcdfeature);
  // //  pcd_fp->setFilter(all_pass_filter);
  // pcd_fp->setFilter(random_region_filter);
  pcd_fp->registerPCDFile(std::string("/home/inagaki/.ros/1400580405.pcd"));

  ROS_INFO("Here");
  pcd_fp->calculate();
}
