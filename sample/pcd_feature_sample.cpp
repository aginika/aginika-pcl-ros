#include <aginika_pcl_ros/PCDFeatureProcedure.h>
#include <aginika_pcl_ros/FPFH_PCDFeature.h>
#include <aginika_pcl_ros/FPFH_Average_PCDFeature.h>
#include <aginika_pcl_ros/Dot_PCDFeature.h>
#include <aginika_pcl_ros/DIFFDOT_PCDFeature.h>
#include <aginika_pcl_ros/SHOT_PCDFeature.h>
#include <aginika_pcl_ros/SHAPE_CONTEXT_PCDFeature.h>
#include <aginika_pcl_ros/AllPassFilter.h>
#include <aginika_pcl_ros/RandomRegionFilter.h>
#include <aginika_pcl_ros/RandomSquareFilter.h>
#include <ros/ros.h>
#include <sstream>
#include <dirent.h>
#include <errno.h>
#include <vector>
#include <string>
#include <iostream>

using namespace std;

int main(int argc, char* argv[]){
  ros::init(argc, argv,"test_pcd_feature");

  if (argc < 4){
    ROS_ERROR("Too few aruguments");
    ROS_ERROR("%s method_filter method_feature target_directory", argv[0]);
    ROS_ERROR("filter  0:AllPass");
    ROS_ERROR("        1:RandomRegion");
    ROS_ERROR("        2:RandomSquare");
    ROS_ERROR("feature 0:FPFH (random chose)");
    ROS_ERROR("        1:FPFH_Average");
    ROS_ERROR("        2:Dot");
    ROS_ERROR("        3:SHOT");
    ROS_ERROR("        4:DIFFDOT");
    ROS_ERROR("        5:SHAPE_CONTEXT");
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
  case 2:
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
  case 2:
    {
      feature = new DOT_PCDFeature();
    }
    break;
  case 3:
    {
      feature = new SHOT_PCDFeature();
    }
    break;
  case 4:
    {
      feature = new DIFFDOT_PCDFeature();
    }
    break;
  case 5:
    {
      feature = new SHAPE_CONTEXT_PCDFeature();
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

  pcd_fp->setFilter(filter);
  pcd_fp->setFeature(feature);
  pcd_fp->setRepeatTime(30);


  int from_index;
  std::stringstream ss3;
  ss3 << argv[3];
  ss3 >> from_index;
  ROS_INFO("Start Index from %d", from_index);

  //Registe files from path
  int counter = 0;
  for (int i = 4; i < argc ; i++){
    DIR *dp;
    struct dirent *dirp;
    if((dp  = opendir(argv[i])) == NULL) {
      cout << "Error(" << errno << ") opening " << argv[i] <<  endl;
      return errno;
    }

    while ((dirp = readdir(dp)) != NULL) {
      std::string file_name = std::string(dirp->d_name);
      if(file_name.find(".pcd", 0) != string::npos && counter >= from_index){
        //        ROS_INFO("Target : %s registered", (std::string(argv[i]) + file_name).c_str());
        pcd_fp->registerPCDFile(std::string(argv[i]) + file_name);
      }
      counter++;
    }
    closedir(dp);
  }

  pcd_fp->calculate();
}
