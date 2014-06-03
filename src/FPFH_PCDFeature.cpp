#include <aginika_pcl_ros/FPFH_PCDFeature.h>
#include <time.h>

FPFH_PCDFeature::FPFH_PCDFeature():radius_search_(0.5){
  feature_name_ = "FPFH";
};

void FPFH_PCDFeature::calculate(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr input_normals){

  clock_t start, end;
  start = clock();

        pcl::FPFHEstimationOMP<pcl::PointXYZRGBNormal, pcl::PointXYZRGBNormal, pcl::FPFHSignature33> fpfh;
        fpfh.setNumberOfThreads(32);
        fpfh.setInputCloud (input_normals);
        fpfh.setInputNormals (input_normals);
        pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);


        fpfh.setSearchMethod (tree);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs (new pcl::PointCloud<pcl::FPFHSignature33> ());
        fpfh.setRadiusSearch (radius_search_);
        fpfh.compute (*fpfhs);

        int target_index = rand()%(fpfhs->points.size());
        float* pf = fpfhs->points[target_index].histogram;
        


        end = clock();
        ROS_INFO("time is  %f [s]", (float)(end - start)/CLOCKS_PER_SEC);
        std::vector<float> result = std::vector<float>(pf, pf+33);
        features_.push_back(result);

        ROS_INFO("FPFH_PCDFeature calculate");
};

