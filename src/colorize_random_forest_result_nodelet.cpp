/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, yuto_inagaki and JSK Lab
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


#include "aginika_pcl_ros/colorize_random_forest.h"
#include <pluginlib/class_list_macros.h>


#include <pcl/common/centroid.h>

namespace aginika_pcl_ros
{
  void ColorizeRandomForest::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &ColorizeRandomForest::extract, this);

    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("cloth_result", 1);

    srand(time(NULL));

    double tmp_radius, tmp_pass;
    tmp_radius = 0.03;
    tmp_pass = 0.03;
    if (!pnh_->getParam("mode", mode_))
      {
        ROS_WARN("~mode is not specified, set 1");
        mode_ = 1;
      }

    if (!pnh_->getParam("rs", tmp_radius))
      {
        ROS_WARN("~rs is not specified, set 1");
      }

    if (!pnh_->getParam("po", tmp_pass))
      {
        ROS_WARN("~po is not specified, set 1");
      }

    radius_search_ = tmp_radius;
    pass_offset_ = tmp_pass;

  }

  void ColorizeRandomForest::extract(const sensor_msgs::PointCloud2 pc)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);

    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZRGB> ());
    pcl::PCDWriter writer;
    seg.setOptimizeCoefficients (true);
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.02);

    int i=0, nr_points = (int) cloud->points.size ();
    while (cloud->points.size () > 0.3 * nr_points)
      {
        // Segment the largest planar component from the remaining cloud
        seg.setInputCloud (cloud);
        seg.segment (*inliers, *coefficients);
        if (inliers->indices.size () == 0)
          {
            std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
            break;
          }

        // Extract the planar inliers from the input cloud
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud (cloud);
        extract.setIndices (inliers);
        extract.setNegative (false);

        // Get the points associated with the planar surface
        extract.filter (*cloud_plane);
        std::cout << "PointCloud representing the planar component: " << cloud_plane->points.size () << " data points." << std::endl;

        // Remove the planar inliers, extract the rest
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_f (new pcl::PointCloud<pcl::PointXYZRGB> ());
        extract.setNegative (true);
        extract.filter (*cloud_f);
        *cloud = *cloud_f;
      }

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);



    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (std::string("z"));
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud);


    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance (0.025);
    ec.setMinClusterSize (300);
    ec.setMaxClusterSize (50000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (cluster_indices);

    int j = 0;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr all_cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);

    int cluster_num = 0;
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
      {
        ROS_INFO("Calculate time %d / %ld", cluster_num  , cluster_indices.size());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
          cloud_cluster->points.push_back (cloud->points[*pit]); //*
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cluster_num ++ ;

        //Normal Estimation
        // Create the normal estimation class, and pass the input dataset to it
        pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
        ne.setInputCloud (cloud_cluster);

        // Create an empty kdtree representation, and pass it to the normal estimation object.
        // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
        pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
        ne.setSearchMethod (tree);

        // Output datasets
        pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

        // Use all neighbors in a sphere of radius 3cm
        ne.setRadiusSearch (0.02);

        // Compute the features
        ne.compute (*cloud_normals);

        for (int cloud_index = 0; cloud_index <  cloud_normals->points.size(); cloud_index++){
          cloud_normals->points[cloud_index].x = cloud_cluster->points[cloud_index].x;
          cloud_normals->points[cloud_index].y = cloud_cluster->points[cloud_index].y;
          cloud_normals->points[cloud_index].z = cloud_cluster->points[cloud_index].z;
        }

        int result_counter=0;
        int call_counter = 0;
        for (int i = 0 ; i < 10; i++){
          ROS_INFO_COND(i%10 == 0,"Calculate time %d / 10", i);
          pcl::PointXYZRGBNormal max_pt,min_pt;
          pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);

          double lucky = 0;
          std::string axis("x");
          int rand_xyz = rand()%2;//%3;
          if (rand_xyz == 0)
            lucky = min_pt.x - pass_offset_ + (max_pt.x - min_pt.x - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
          else {//if (rand_xyz == 1){
            lucky = min_pt.y - pass_offset_ + (max_pt.y - min_pt.y - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
            axis = std::string("y");
          }

          pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_pass (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

          pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
          pass.setInputCloud (cloud_normals);
          pass.setFilterFieldName (axis);
          float small = std::min(lucky, lucky + pass_offset_);
          float large = std::max(lucky, lucky + pass_offset_);
          pass.setFilterLimits (small, large);
          pass.filter (*cloud_normals_pass);

          std::vector<int> tmp_indices;
          pcl::removeNaNFromPointCloud(*cloud_normals_pass, *cloud_normals_pass, tmp_indices);


          if(cloud_normals_pass->points.size() == 0){
            continue;
          }


          pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
          fpfh.setNumberOfThreads(8);
          fpfh.setInputCloud (cloud_normals_pass);
          fpfh.setInputNormals (cloud_normals_pass);
          pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

          fpfh.setSearchMethod (tree);
          pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
          fpfh.setRadiusSearch (radius_search_);
          fpfh.compute (*fpfhs);

          std::vector<float> result;
          bool success = true;
          for(int index = 0; index < 33; index++){
            float sum_hist_points = 0;
            for(int kndex = 0; kndex < fpfhs->points.size();kndex++)
              {
                if(pcl_isnan(fpfhs->points[kndex].histogram[index])){
                  success = false;
                }else{
                  sum_hist_points+=fpfhs->points[kndex].histogram[index];
                }
              }
            result.push_back( sum_hist_points/fpfhs->points.size() );
          }

          if(success){
            call_counter++;
            ros::ServiceClient client = pnh_->serviceClient<aginika_pcl_ros::PredictData>("/predict");
            aginika_pcl_ros::PredictData srv;
            srv.request.data = result;
            if(client.call(srv)){
              if (srv.response.index == 0)
                {
                  result_counter += 1;
                }
            }else{
            }
          }
        }

        float r = 255, g = 0, b = 0;
        if(result_counter > call_counter / 2 - 2){
          ROS_INFO("Cloth %d / %d", result_counter, call_counter);
        }else{
          r=0;g=0;b=255;
          ROS_INFO("Not Cloth %d / %d", result_counter, call_counter);
        }

        for (int cloud_index = 0; cloud_index <  cloud_cluster->points.size(); cloud_index++){
          cloud_cluster->points[cloud_index].r = r;
          cloud_cluster->points[cloud_index].g = g;
          cloud_cluster->points[cloud_index].b = b;
        }

        for (int i = 0; i < cloud_cluster->points.size(); i++)
          all_cloud_cluster->points.push_back (cloud_cluster->points[i]);
      }
    sensor_msgs::PointCloud2 _pointcloud2;
    pcl::toROSMsg(*all_cloud_cluster, _pointcloud2);
    _pointcloud2.header = pc.header;
    _pointcloud2.is_dense = false;
    pub_.publish(_pointcloud2);
  }

}

typedef aginika_pcl_ros::ColorizeRandomForest ColorizeRandomForest;
PLUGINLIB_DECLARE_CLASS (aginika_pcl_ros, ColorizeRandomForest, ColorizeRandomForest, nodelet::Nodelet);
