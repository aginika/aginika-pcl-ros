// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Yuto Inagaki
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/o2r other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

#ifndef AGINIKA_PCL_ROS_POINTCLOUD_MODEL_MANAGER_H_
#define AGINIKA_PCL_ROS_POINTCLOUD_MODEL_MANAGER_H_

// ros
#include <ros/ros.h>
#include <ros/names.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

// pcl
#include <pcl_ros/pcl_nodelet.h>
#include <pcl/point_types.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/pfh.h>
#include <pcl/features/pfhrgb.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/surface/grid_projection.h>
#include <pcl/surface/gp3.h>
#include <pcl/surface/marching_cubes_hoppe.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>

namespace aginika_pcl_ros
{
  template<typename FeatureType>
  class FeatureMatching
  {
  public:
    FeatureMatching();

    void segmentation (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented) const;
    void detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const;
    void extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, typename pcl::PointCloud<FeatureType>::Ptr features);
    void findCorrespondences (typename pcl::PointCloud<FeatureType>::Ptr source, typename pcl::PointCloud<FeatureType>::Ptr target, std::vector<int>& correspondences) const;

    void filterCorrespondences ();
    void determineInitialTransformation ();
    void prepareKeypointsAndDescriptors();
    void setInputCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud);
    void calculate();
    std::vector<std::string> getResult();
    void setSourcesClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds);

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> candidate_pointclouds_;

    typedef typename pcl::PointCloud<FeatureType>::Ptr FeatureCloudPtr;
    std::vector<FeatureCloudPtr> source_features_;
    FeatureCloudPtr target_features_;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> source_keypoints_;
    pcl::PointCloud<pcl::PointXYZI>::Ptr target_keypoints_;
    boost::shared_ptr<pcl::Keypoint<pcl::PointXYZRGB, pcl::PointXYZI> > keypoint_detector_;
    typename pcl::Feature<pcl::PointXYZRGB, FeatureType>::Ptr feature_extractor_;
    std::vector<std::vector<int> > source2target_;
    std::vector<std::vector<int> > target2source_;
    std::vector<pcl::CorrespondencesPtr> correspondences_;
    std::vector<Eigen::Matrix4f> initial_transformation_matrix_;
    typename std::vector<pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr> sources_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_;
    typename std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_segmented_;
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_segmented_;
    typename std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_transformed_;
    typename std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> source_registered_;
    typename pcl::PolygonMesh surface_;
  };

  class PointcloudModelManager: public pcl_ros::PCLNodelet
  {
  protected:
    ros::Subscriber sub_input_;
    tf::TransformBroadcaster br;
    std::string frame;
    FeatureMatching<pcl::FPFHSignature33>* fm_;
    virtual void cloudInput(const sensor_msgs::PointCloud2ConstPtr &input);
    virtual void prepareKeypointsAndDescriptors();
    void publishResults(std::vector<std::string> result);

  private:
    virtual void onInit();
  };
}

#endif
