#ifndef __DIFFDOT_PCDFEATURE__
#define __DIFFDOT_PCDFEATURE__

#include <aginika_pcl_ros/PCDFeatureProcedure.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <boost/format.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <algorithm>
#include <functional>
#include <pcl/octree/octree.h>
#include <pcl/features/don.h>

class DIFFDOT_PCDFeature : public PCDFeature{
public:
  DIFFDOT_PCDFeature();

  virtual void calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals);

  void setRadiusSearch(float radius_search){
    radius_search_ = radius_search;
  };
  void setResolution(float resolution){
    resolution_ = resolution;
  };

  void setNormalNums(int normal_nums){normal_nums_ = normal_nums;};
  float radius_search_;
  float resolution_;
  float normals_large_scale_;
  float normals_small_scale_;
  int normal_nums_;
};

#endif
