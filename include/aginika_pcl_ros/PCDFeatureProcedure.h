#ifndef __PCDFEATURE_PRCEDURE_H__
#define __PCDFEATURE_PRCEDURE_H__
#include <ros/ros.h>
#include <string>
#include <vector>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>
#include <boost/foreach.hpp>
#include <sstream>
#include <pcl/io/pcd_io.h>
#include <fstream>

class PCDFeature{
public:
  PCDFeature(){
    feature_name_ = "base_feature";
  };

  virtual void calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
    ROS_INFO("Base PCDFeature calculate");
  };

  std::string get_name(){
    return feature_name_;
  };

  std::vector<std::vector<float> > get_result(){
    return features_;
  }

  std::string feature_name_;
  std::vector<std::vector<float> > features_;
};


class PCDPrevFilter{
public:
  PCDPrevFilter(){
    filter_name_ = "base_filter";
  };

  virtual void filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals){
    ROS_INFO("Base PCDPrevFilter calculate");
  };

  std::string get_name(){
    return filter_name_;
  };

  std::string filter_name_;
};


class PCDFeatureProcedure{
public:
  PCDFeatureProcedure();

  void registerPCDFile(std::string filename);
  void calculateFeatures();
  void calculate();
  void openPCDFile(std::string filename);
  void writeOutFeatures(std::string filename);
  void setFeature(PCDFeature* feature){target_feature_ = feature;};
  void setFilter(PCDPrevFilter* filter){target_filter_ = filter;};
private:
  PCDFeature* target_feature_;
  PCDPrevFilter* target_filter_;
  std::vector<std::string> filenames_;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals_;
};
#endif
