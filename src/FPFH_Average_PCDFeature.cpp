#include <aginika_pcl_ros/FPFH_Average_PCDFeature.h>

FPFH_Average_PCDFeature::FPFH_Average_PCDFeature():radius_search_(0.03){
  feature_name_ = "FPFH";
};

void FPFH_Average_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){

        pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
        fpfh.setNumberOfThreads(8);
        fpfh.setInputCloud (input_normals);
        fpfh.setInputNormals (input_normals);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);

        fpfh.setSearchMethod (tree);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh.setRadiusSearch (radius_search_);
        fpfh.compute (*fpfhs);

        std::vector<float> result;
        for(int index = 0; index < 33; index++){
          float sum_hist_points = 0;
          for(int kndex = 0; kndex < fpfhs->points.size();kndex++)
            sum_hist_points+=fpfhs->points[kndex].histogram[index];
          result.push_back( sum_hist_points/fpfhs->points.size() );
        }

        features_.push_back(result);

        ROS_INFO("FPFH_Average_PCDFeature calculate");
};

