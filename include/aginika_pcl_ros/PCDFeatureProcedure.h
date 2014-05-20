#ifndef __PCDFEATURE_PRCEDURE_H__
#define __PCDFEATURE_PRCEDURE_H__
#include <ros/ros.h>
#include <string>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/common/centroid.h>

class PCDFeature{
public:
  PCDFeature(){
  };

  virtual void calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){
    ROS_INFO("Base PCDFeature calculate");
  };
};


class PCDFeatureProcedure{
public:
  PCDFeatureProcedure();

  void openPCDFiles(std::string filename);
  void calculateFeatures();
  void calculateFeature();
  void writeOutFeature();
  void setFeature(PCDFeature* feature);
private:
  PCDFeature* target_feature;
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr normals;
};
#endif
