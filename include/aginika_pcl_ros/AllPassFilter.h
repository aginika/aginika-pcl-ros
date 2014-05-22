#include <aginika_pcl_ros/PCDFeatureProcedure.h>

class AllPassFilter : public PCDPrevFilter{
public:
  AllPassFilter();
  virtual void filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& output_normals);
};
