#ifndef __FPFH_PCDFEATURE__
#define __FPFH_PCDFEATURE__

#include <aginika_pcl_ros/PCDFeatureProcedure.h>

class FPFH_PCDFeature : public PCDFeature{
public:
  FPFH_PCDFeature();

  virtual void calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals);

};

#endif
