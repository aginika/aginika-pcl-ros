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


#include "aginika_pcl_ros/l1_skeletonization.h"
#include <pluginlib/class_list_macros.h>

#include <pcl/common/centroid.h>

namespace aginika_pcl_ros
{
  L1Skeletonization::L1Skeletonization():h_(0.5), myu_(0.35)
  {
    
  };

  void L1Skeletonization::randomSampling(int sample_rate)
  {
    if (sample_rate > 1.0 || sample_rate < 0.0)
      {
        ROS_ERROR("Sample Rate is over 1.0 or under 0.0 !!");
        exit(-1);
      }
    else{
      srand(time(0));
      size_t sample_num = sample_rate * cloud_->size();
      sample_cloud_.reset(new Cloud());
      CloudPtr candidate_cloud(new Cloud());
      pcl::copyPointCloud(*cloud_, *candidate_cloud);

      for(size_t i = 0; i < sample_num; i++){
        size_t random_index = rand() % candidate_cloud->size();
        PointType point = candidate_cloud->at(random_index);
        candidate_cloud->erase(candidate_cloud->begin()+random_index);
        sample_cloud_->push_back(point);
      };
    }
  };

  float L1Skeletonization::theta(float r)
  {
    return exp(-1 * pow(r,2) / pow((h_/2),2 ));
  };

  float L1Skeletonization::sigma(Eigen::EigenSolver<Eigen::Matrix3f> eig)
  {
    if (eig.eigenvalues()[0].imag() != 0 || eig.eigenvalues()[1].imag() != 0 || eig.eigenvalues()[2].imag() != 0)
      return 0;
    else
      return eig.eigenvalues()[2].real()/(eig.eigenvalues()[0].real() + eig.eigenvalues()[1].real() + eig.eigenvalues()[2].real());
  };

  Eigen::Matrix3f L1Skeletonization::covariance(size_t x_i_index, std::vector<Eigen::Vector3f>& x_is)
  {
    Eigen::Matrix3f sum;
    Eigen::Vector3f x_i = x_is[x_i_index];
    for(size_t i = 0 ; i < x_is.size(); i++)
      {
        if(x_i_index != i){
          Eigen::Vector3f x_i_dash = x_is[i];
          Eigen::Vector3f x_i_diff = x_i - x_i_dash;
          sum += (x_i_diff * x_i_diff.transpose()) * theta( x_i_diff.norm() );
        }
      }
    return sum;
  };

  Eigen::EigenSolver<Eigen::Matrix3f> L1Skeletonization::Eigens(Eigen::Matrix3f& cov_matrix)
  {
    Eigen::EigenSolver<Eigen::Matrix3f> es(cov_matrix);
    return es;
  };

  void L1Skeletonization::calculateCovarianceAndEigens()
  {
    for(size_t i = 0; i < x_samples_.size(); i++){
      Eigen::Matrix3f cov = covariance(i, x_samples_);
      Eigen::EigenSolver<Eigen::Matrix3f> eigen_matrix = Eigens(cov);
      covariance_matrixes_.push_back(eigen_matrix);
    };
  };

  std::vector<std::vector<float> > L1Skeletonization::calculate_alphas()
  {
    std::vector<std::vector<float> > alphas;
    alphas.resize(origin_samples_.size() * x_samples_.size()); //Is this ok?
    for(size_t i = 0; i < x_samples_.size() ; i++)
      {
        for(size_t j = 0; j < origin_samples_.size(); j++)
          {
            float diff_norm = (x_samples_[i] - origin_samples_[j]).norm();
            alphas[i][j] = theta(diff_norm)/diff_norm;
          };
      };
    return alphas;
  };

  std::vector<std::vector<float> > L1Skeletonization::calculate_betas()
  {
    std::vector<std::vector<float> > betas;
    betas.resize(x_samples_.size() * x_samples_.size()); //Is this ok?
    for(size_t i = 0; i < x_samples_.size() ; i++)
      {
        for(size_t j = 0; j < x_samples_.size(); j++)
          {
            if(i == j){
              betas[i][j] = 0;
            }else{
              float diff_norm = (x_samples_[i] - x_samples_[j]).norm();
              betas[i][j] = theta(diff_norm)/pow(diff_norm,2);
            }
          };
      };
    return betas;
  };

  Eigen::Vector3f L1Skeletonization::first_term(std::vector<float> alphas)
  {
    Eigen::Vector3f upper_val;
    float lower_val = 0;
    for( size_t j = 0; j < alphas.size(); j++){
      upper_val += alphas[j] * origin_samples_[j];
      lower_val += alphas[j];
    };
    return upper_val/lower_val;
  };

  Eigen::Vector3f L1Skeletonization::second_term(Eigen::Vector3f x_i, std::vector<float> betas, float sigma)
  {
    if(sigma == 0){
      return Eigen::Vector3f::Zero();
    }
    Eigen::Vector3f upper_val;
    float lower_val;
    for(size_t j = 0; j < betas.size(); j++){
      upper_val += (x_i - x_samples_[j]) * betas[j];
      lower_val += betas[j];
    }
    return myu_ * sigma * upper_val / lower_val;
  };

  void L1Skeletonization::updateSamples()
  {
    std::vector<Eigen::Vector3f> new_x_samples;
    ROS_INFO("Calcurate prepare");
    std::vector<std::vector<float> > alphas = calculate_alphas();
    std::vector<std::vector<float> > betas = calculate_betas();
   ROS_INFO("Calcurate prepare End");

   ROS_INFO("update start");
   for(int i = 0; i < x_samples_.size(); i++)
     {
       Eigen::Vector3f new_x_sample = first_term(alphas[i]) + second_term(x_samples_[i], betas[i], sigma(covariance_matrixes_[i]));
       new_x_samples.push_back(new_x_sample);
     }
   x_samples_ = new_x_samples;
   ROS_INFO("Update End");
  };

  void L1Skeletonization::setPointCloud(CloudPtr& cloud)
  {
    cloud_ = cloud;
  };

  void L1Skeletonization::convertToEigen()
  {
    for(size_t i = 0 ; i < cloud_->size(); i++){
      Eigen::Vector3f origin_sample(cloud_->points[i].x,
                                    cloud_->points[i].y,
                                    cloud_->points[i].z);
      origin_samples_.push_back(origin_sample);
    };

    for(size_t i = 0 ; i < sample_cloud_->size(); i++){
      Eigen::Vector3f x_sample(sample_cloud_->points[i].x,
                               sample_cloud_->points[i].y,
                               sample_cloud_->points[i].z);
      x_samples_.push_back(x_sample);
    };
  };

  void L1Skeletonization::run()
  {
    //Random sampling
    randomSampling(0.1);

    //convert to Eigen
    convertToEigen();

    //iterative process
    while(true)
      {
        calculateCovarianceAndEigens();
        updateSamples();
      };

    //downsampling

    //smoothing

    //re-centering

    //final curve skeleton
  };
}
