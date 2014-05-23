#ifndef __SHAPE_CONTEXT_PCDFEATURE__
#define __SHAPE_CONTEXT_PCDFEATURE__

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
#include <pcl/features/3dsc.h>
#include <pcl/features/impl/3dsc.hpp>


class SHAPE_CONTEXT_PCDFeature : public PCDFeature{
public:
  SHAPE_CONTEXT_PCDFeature();

  virtual void calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals);

  void setRadiusSearch(float radius_search){
    radius_search_ = radius_search;
  };

  float radius_search_;
};

#endif
