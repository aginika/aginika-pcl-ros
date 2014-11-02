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


#include "aginika_pcl_ros/pointcloud_model_manager.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/common/centroid.h>

namespace aginika_pcl_ros
{
  template<typename FeatureType>
  FeatureMatching<FeatureType>::FeatureMatching(){
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::segmentation (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr source, typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented) const
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.02);

    seg.setInputCloud (source);
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud (source);
    extract.setIndices (inliers);
    extract.setNegative (true);

    extract.filter (*segmented);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*segmented, *segmented, indices);

    // euclidean clustering
    typename pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (segmented);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> clustering;
    clustering.setClusterTolerance (0.02); // 2cm
    clustering.setMinClusterSize (1000);
    clustering.setMaxClusterSize (250000);
    clustering.setSearchMethod (tree);
    clustering.setInputCloud(segmented);
    clustering.extract (cluster_indices);

    if (cluster_indices.size() > 0)//use largest cluster
      {
        if (cluster_indices.size() > 1)
          ROS_DEBUG(" Using largest one...");
        typename pcl::IndicesPtr indices (new std::vector<int>);
        *indices = cluster_indices[0].indices;
        extract.setInputCloud (segmented);
        extract.setIndices (indices);
        extract.setNegative (false);
        extract.filter (*segmented);
      }
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::detectKeypoints (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints) const
  {
    ROS_DEBUG("keypoint detection...");
    keypoint_detector_->setInputCloud(input);
    keypoint_detector_->compute(*keypoints);
    ROS_DEBUG("OK. keypoints found: %d" , keypoints->points.size());
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::extractDescriptors (typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input, typename pcl::PointCloud<pcl::PointXYZI>::Ptr keypoints, FeatureCloudPtr features)
  {
    typename pcl::PointCloud<pcl::PointXYZRGB>::Ptr kpts(new pcl::PointCloud<pcl::PointXYZRGB>);
    kpts->points.resize(keypoints->points.size());

    pcl::copyPointCloud(*keypoints, *kpts);

    typename pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType>::Ptr feature_from_normals
      = boost::dynamic_pointer_cast<pcl::FeatureFromNormals<pcl::PointXYZRGB, pcl::Normal, FeatureType> > (feature_extractor_);

    feature_extractor_->setSearchSurface(input);
    feature_extractor_->setInputCloud(kpts);

    if (feature_from_normals)
      {
        ROS_DEBUG("normal estimation...");
        typename pcl::PointCloud<pcl::Normal>::Ptr normals (new  pcl::PointCloud<pcl::Normal>);
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> normal_estimation;
        normal_estimation.setSearchMethod (pcl::search::Search<pcl::PointXYZRGB>::Ptr (new pcl::search::KdTree<pcl::PointXYZRGB>));
        normal_estimation.setRadiusSearch (0.01);
        normal_estimation.setInputCloud (input);
        normal_estimation.compute (*normals);
        feature_from_normals->setInputNormals(normals);
        ROS_DEBUG("OK");
      }

    ROS_DEBUG("descriptor extraction...");
    feature_extractor_->compute (*features);
    ROS_DEBUG("OK");
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::findCorrespondences (FeatureCloudPtr source, FeatureCloudPtr target, std::vector<int>& correspondences) const
  {
    ROS_DEBUG("correspondence assignment...");
    correspondences.resize (source->size());

    // Use a KdTree to search for the nearest matches in feature space
    pcl::KdTreeFLANN<FeatureType> descriptor_kdtree;
    descriptor_kdtree.setInputCloud (target);

    // Find the index of the best match for each keypoint, and store it in "correspondences_out"
    const int k = 1;
    std::vector<int> k_indices (k);
    std::vector<float> k_squared_distances (k);
    for (int i = 0; i < static_cast<int> (source->size ()); ++i)
      {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
      }
    ROS_DEBUG("OK");
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::filterCorrespondences ()
  {
    ROS_DEBUG("correspondence rejection...");
    for(unsigned index = 0; index < source2target_.size(); index++){
      std::vector<int> source2target = source2target_[index];
      std::vector<int> target2source = target2source_[index];
      pcl::PointCloud<pcl::PointXYZI>::Ptr source_keypoints = source_keypoints_[index];

      std::vector<std::pair<unsigned, unsigned> > correspondences;
      for (unsigned cIdx = 0; cIdx < source2target.size (); ++cIdx)
        if (target2source[source2target[cIdx]] == static_cast<int> (cIdx))
          correspondences.push_back(std::make_pair(cIdx, source2target[cIdx]));

      correspondences_[index]->resize (correspondences.size());
      for (unsigned cIdx = 0; cIdx < correspondences.size(); ++cIdx)
        {
          (*correspondences_[index])[cIdx].index_query = correspondences[cIdx].first;
          (*correspondences_[index])[cIdx].index_match = correspondences[cIdx].second;
        }

      //std::pair<unsigned, unsigned> a_correspondences;
      pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointXYZI> rejector;
      rejector.setInputSource (source_keypoints);
      rejector.setInputTarget (target_keypoints_);
      rejector.setInputCorrespondences(correspondences_[index]);
      rejector.getCorrespondences(*correspondences_[index]);
    }
    ROS_DEBUG("filterCorrespondences End");
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::determineInitialTransformation ()
  {
    ROS_DEBUG("initial alignment...");
    for (int index = 0; index < source_keypoints_.size(); index++){
      pcl::registration::TransformationEstimation<pcl::PointXYZI, pcl::PointXYZI>::Ptr transformation_estimation (new pcl::registration::TransformationEstimationSVD<pcl::PointXYZI, pcl::PointXYZI>);

      transformation_estimation->estimateRigidTransformation (*source_keypoints_[index], *target_keypoints_, *correspondences_[index], initial_transformation_matrix_[index]);

      pcl::transformPointCloud(*source_segmented_[index], *source_transformed_[index], initial_transformation_matrix_[index]);
    }
    ROS_DEBUG("initial alignment End");
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::prepareKeypointsAndDescriptors()
  {
    for(int i = 0; i < candidate_pointclouds_.size(); i++){
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr target_pointcloud = candidate_pointclouds_[i];
      sources_.push_back(target_pointcloud);
      source_segmented_.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));
      source_transformed_.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));
      source_registered_.push_back(pcl::PointCloud<pcl::PointXYZRGB>::Ptr (new pcl::PointCloud<pcl::PointXYZRGB>()));

      source_keypoints_.push_back(pcl::PointCloud<pcl::PointXYZI>::Ptr (new pcl::PointCloud<pcl::PointXYZI>()));
      source_features_.push_back(FeatureCloudPtr(new pcl::PointCloud<FeatureType>()));

      source2target_.resize(source2target_.size()+1);
      target2source_.resize(target2source_.size()+1);

      correspondences_.push_back(pcl::CorrespondencesPtr(new pcl::Correspondences()));
      initial_transformation_matrix_.resize(initial_transformation_matrix_.size());
      segmentation (sources_[i], source_segmented_[i]);
      detectKeypoints (source_segmented_[i], source_keypoints_[i]);
      extractDescriptors (source_segmented_[i], source_keypoints_[i], source_features_[i]);
    }
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::setInputCloud(typename pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr input_cloud)
  {
    pcl::copyPointCloud(*input_cloud, *target_);
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::calculate()
  {
    segmentation (target_, target_segmented_);
    detectKeypoints (target_segmented_, target_keypoints_);
    extractDescriptors (target_segmented_, target_keypoints_, target_features_);

    for(int i = 0; i < candidate_pointclouds_.size(); i++){
      findCorrespondences (source_features_[i], target_features_, source2target_[i]);
      findCorrespondences (target_features_, source_features_[i], target2source_[i]);
    }

    filterCorrespondences ();
    determineInitialTransformation ();
  }

  template<typename FeatureType>
  std::vector<std::string> FeatureMatching<FeatureType>::getResult()
  {
    //HERE
    std::vector<std::string> a;
    return a;
  }

  template<typename FeatureType>
  void FeatureMatching<FeatureType>::setSourcesClouds(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> clouds)
  {
    for(int i = 0; i < clouds.size(); i++)
      {
        candidate_pointclouds_.push_back(clouds[i]);
      }
  }
  void PointcloudModelManager::cloudInput(const sensor_msgs::PointCloud2ConstPtr &input)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*input, *cloud_xyz);
    fm_->setInputCloud(cloud_xyz);
    fm_->calculate();
    std::vector<std::string> results =  fm_->getResult();
    publishResults(results);
  }

  void PointcloudModelManager::publishResults(std::vector<std::string> result)
  {
  }

  void PointcloudModelManager::prepareKeypointsAndDescriptors(void)
  {
    fm_ = new FeatureMatching<pcl::FPFHSignature33>();
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> a;
    fm_->setSourcesClouds(a);
    fm_->prepareKeypointsAndDescriptors();
  }

  void PointcloudModelManager::onInit(void)
  {
    PCLNodelet::onInit();
    prepareKeypointsAndDescriptors();
    sub_input_ = pnh_->subscribe("input", 1, &PointcloudModelManager::cloudInput, this);
    if (!pnh_->getParam("frame", frame))
      {
        ROS_WARN("~frame is not specified, using %s", getName().c_str());
        frame = getName();
      }
  }
}

PLUGINLIB_EXPORT_CLASS (aginika_pcl_ros::PointcloudModelManager, nodelet::Nodelet);
