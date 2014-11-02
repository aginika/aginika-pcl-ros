// -*- mode: C++ -*-
/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Yuto Inagaki
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

#ifndef AGINIKA_PCL_ROS_L1_SKELETONIZATION_H_
#define AGINIKA_PCL_ROS_L1_SKELETONIZATION_H_
#include <complex>

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
  class L1Skeletonization
  {
  public:
    typedef pcl::PointXYZ PointType;
    typedef typename pcl::PointCloud<PointType> Cloud;
    typedef typename Cloud::Ptr CloudPtr;
    typedef typename Cloud::ConstPtr CloudConstPtr;

    L1Skeletonization();
    ~L1Skeletonization(){};

    void randomSampling(int sample_rate);
    void setPointCloud(CloudPtr& cloud);
    void convertToEigen();
    void convertToPCL(CloudPtr& cloud, std::vector<Eigen::Vector3f> x_samples);
    void updateSamples();
    void publishProcess();
    void run();
    void inputCloud(sensor_msgs::PointCloud2 msgs);

    float theta(float r);
    float sigma(Eigen::EigenSolver<Eigen::Matrix3f> eig);
    Eigen::Matrix3f covariance(size_t x_i_index, std::vector<Eigen::Vector3f>& x_is);
    Eigen::EigenSolver<Eigen::Matrix3f> Eigens(Eigen::Matrix3f& cov_matrix);
    void calculateCovarianceAndEigens();
    std::vector<std::vector<float> > calculate_alphas();
    std::vector<std::vector<float> > calculate_betas();
    Eigen::Vector3f first_term(std::vector<float> alphas);
    Eigen::Vector3f second_term(Eigen::Vector3f x_i, std::vector<float> betas, float sigma);

    CloudPtr cloud_;
    CloudPtr sample_cloud_;
    std::vector<Eigen::Vector3f> origin_samples_;
    std::vector<Eigen::Vector3f> x_samples_;
    float h_;
    float myu_;
    ros::NodeHandle nh_;
    ros::Publisher pub_;
    ros::Subscriber sub_;
    std::string frame_id_;
    std::vector<Eigen::EigenSolver<Eigen::Matrix3f> > covariance_matrixes_;
  };
}

#endif
