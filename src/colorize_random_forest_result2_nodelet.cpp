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


#include "aginika_pcl_ros/colorize_random_forest2.h"
#include <pluginlib/class_list_macros.h>


#include <pcl/common/centroid.h>

namespace aginika_pcl_ros
{
  void ColorizeMapRandomForest::onInit(void)
  {
    // not implemented yet
    PCLNodelet::onInit();
    sub_input_ = pnh_->subscribe("input", 1, &ColorizeMapRandomForest::extract, this);

    pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("cloth_result", 1);

    srand(time(NULL));

    double tmp_radius, tmp_pass, tmp_pass2;
    tmp_radius = 0.03;
    tmp_pass = 0.03;
    tmp_pass2 = 0.2;
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

    if (!pnh_->getParam("po2", tmp_pass2))
      {
        ROS_WARN("~po is not specified, set 1");
      }

    radius_search_ = tmp_radius;
    pass_offset_ = tmp_pass;
    pass_offset2_ = tmp_pass2;

  }

  void ColorizeMapRandomForest::extract(const sensor_msgs::PointCloud2 pc)
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

    std::vector<int> indices;
    pcl::fromROSMsg(pc, *cloud);

    cloud->is_dense = false;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);


    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud (cloud);



    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName (std::string("z"));
    pass.setFilterLimits (0.0, 1.5);
    pass.filter (*cloud);
    ROS_INFO("pass through");

    //Normal Estimation
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::PointXYZRGBNormal> ne;
    ne.setInputCloud (cloud);
    ROS_INFO("normal estimate");

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2 (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    ne.setSearchMethod (tree2);

    // Output datasets
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

    // Use all neighbors in a sphere of radius 3cm
    ne.setRadiusSearch (0.02);

    // Compute the features
    ne.compute (*cloud_normals);

    for (int cloud_index = 0; cloud_index <  cloud_normals->points.size(); cloud_index++){
      cloud_normals->points[cloud_index].x = cloud->points[cloud_index].x;
      cloud_normals->points[cloud_index].y = cloud->points[cloud_index].y;
      cloud_normals->points[cloud_index].z = cloud->points[cloud_index].z;
    }

    int result_counter=0;
    int call_counter = 0;


    for (int i = 0 ; i < 100; i++){
      ROS_INFO_COND(i%10 == 0,"Calculate time %d / 10", i);
      pcl::PointXYZRGBNormal max_pt,min_pt;
      pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);
      ROS_INFO("fpfh %d / 100", i);
      double lucky = 0;
      double lucky2 = 0;
      std::string axis("x");
      std::string other_axis("y");
      int rand_xyz = rand()%2;//%3;
      if (rand_xyz == 0){
        lucky = min_pt.x - pass_offset_ + (max_pt.x - min_pt.x - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
        lucky2 = min_pt.y + (max_pt.y - min_pt.y) * 1.0 * rand() / RAND_MAX;
      }
      else {
        lucky = min_pt.y - pass_offset_ + (max_pt.y - min_pt.y - pass_offset_*2) * 1.0 * rand() / RAND_MAX;
        lucky2 = min_pt.x + (max_pt.x - min_pt.x) * 1.0 * rand() / RAND_MAX;
        axis = std::string("y");
        other_axis = std::string("x");
      }

      pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud_normals_pass (new pcl::PointCloud<pcl::PointXYZRGBNormal>);

      pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
      pcl::IndicesPtr indices_x(new std::vector<int>());
      pass.setInputCloud (cloud_normals);
      pass.setFilterFieldName (axis);
      float small = std::min(lucky, lucky + pass_offset_);
      float large = std::max(lucky, lucky + pass_offset_);
      pass.setFilterLimits (small, large);
      pass.filter (*cloud_normals_pass);
      
      // ROS_INFO("num %ld", cloud_normals_pass->points.size());
      pass.setInputCloud (cloud_normals_pass);
      // pass.setIndices (indices_x);
      pass.setFilterFieldName (other_axis);
      float small2 = std::min(lucky2, lucky2 + pass_offset2_);
      float large2 = std::max(lucky2, lucky2 + pass_offset2_);
      pass.setFilterLimits (small2, large2);
      pass.filter (*cloud_normals_pass);

      ROS_INFO("min_x %f max_x %f  small %f large %f  min_y %f max_y %f small2 %f large2%f axis %s other %s", min_pt.x, max_pt.x, small, large ,  min_pt.y, max_pt.y, small2, large2, axis.c_str(), other_axis.c_str());

      std::vector<int> tmp_indices;
      pcl::removeNaNFromPointCloud(*cloud_normals_pass, *cloud_normals_pass, tmp_indices);


      if(cloud_normals_pass->points.size() == 0){
        continue;
      }
      
      ROS_INFO("fpfh start");
      pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
      fpfh.setNumberOfThreads(8);
      fpfh.setInputCloud (cloud_normals_pass);
      fpfh.setInputNormals (cloud_normals_pass);
      pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree4 (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
      
      fpfh.setSearchMethod (tree4);
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


      ROS_INFO("fpfh end");
      bool cloth = false;
      if(success){
        call_counter++;
        ros::ServiceClient client = pnh_->serviceClient<aginika_pcl_ros::PredictData>("/predict");
        aginika_pcl_ros::PredictData srv;
        srv.request.data = result;
        if(client.call(srv)){
          if (srv.response.index == 0)
            {
              cloth = true;
            }
        }else{
        }
      }else{
        continue;
      }
      
      
      Eigen::Vector4f c;
      pcl::compute3DCentroid (*cloud_normals_pass, c);
      
      pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBNormal> search_octree (0.02);
      search_octree.setInputCloud (cloud_normals);
      search_octree.addPointsFromInputCloud ();
      
      pcl::PointXYZRGBNormal point_a;
      point_a.x = c[0];point_a.y = c[1];point_a.z = c[2];
      std::vector<int> k_indices;
      std::vector<float> k_sqr_distances;
      search_octree.radiusSearch(point_a, 0.1, k_indices, k_sqr_distances);
      
      for(int index = 0; index < k_indices.size(); index++){
        int id = k_indices[index];
        if(cloth){
          cloud_normals->points[id].r = std::min(std::max(255.0 - k_sqr_distances[index] * 200, 0.0) + cloud_normals->points[id].r, 255.0);
          cloud_normals->points[id].g = 0;
          ROS_INFO("Set color r %d", cloud_normals->points[id].r);
        }else{

          cloud_normals->points[id].b = std::min(std::max(255.0 - k_sqr_distances[index] * 200, 0.0) + cloud_normals->points[id].b, 255.0);
          cloud_normals->points[id].g = 0;
        }
      }
      
    }
    sensor_msgs::PointCloud2 _pointcloud2;
    pcl::toROSMsg(*cloud_normals, _pointcloud2);
    _pointcloud2.header = pc.header;
    _pointcloud2.is_dense = false;
    pub_.publish(_pointcloud2);
  }
}


typedef aginika_pcl_ros::ColorizeMapRandomForest ColorizeMapRandomForest;
PLUGINLIB_DECLARE_CLASS (aginika_pcl_ros, ColorizeMapRandomForest, ColorizeMapRandomForest, nodelet::Nodelet);
