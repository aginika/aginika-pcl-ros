#include <aginika_pcl_ros/PCDFeatureProcedure.h>

#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>


class RandomSquareFilter : public PCDPrevFilter{
public:
  RandomSquareFilter();
  virtual void filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& output_normals);
  std::string pass_through( double pass_offset, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals_pass);


  void setPassOffset(float pass_offset){pass_offset_=pass_offset;};

  float pass_offset_;
};
