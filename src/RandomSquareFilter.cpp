#include <aginika_pcl_ros/RandomSquareFilter.h>

RandomSquareFilter::RandomSquareFilter(){
  filter_name_ = "random_square";
  pass_offset_ = 0.04;
  srand (time(NULL));
}

std::string RandomSquareFilter::pass_through( double pass_offset, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals_pass)
{
  pcl::PointXYZRGBNormal max_pt,min_pt;
  pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);

  double lucky = 0;

  pcl::IndicesPtr indices_x(new std::vector<int>());
  pcl::PassThrough<pcl::PointXYZRGBNormal> pass (true);
  pass.setInputCloud (cloud_normals);
  pass.setFilterFieldName ("x");
  lucky = min_pt.x - pass_offset + (max_pt.x - min_pt.x - pass_offset*2) * 1.0 * rand() / RAND_MAX;
  float small = std::min(lucky, lucky + pass_offset);
  float large = std::max(lucky, lucky + pass_offset);

  pass.setFilterLimits (small, large);
  pass.filter (*indices_x);

  pass.setIndices (indices_x);
  pass.setFilterFieldName ("y");
  lucky = min_pt.y - pass_offset + (max_pt.y - min_pt.y - pass_offset*2) * 1.0 * rand() / RAND_MAX;
  small = std::min(lucky, lucky + pass_offset);
  large = std::max(lucky, lucky + pass_offset);
  pass.setFilterLimits (small, large);
  pass.filter (*cloud_normals_pass);

  return std::string("");
}


void RandomSquareFilter::filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& output_normals){
  // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // while(new_normals->points.size() == 0)
  //   pass_through(pass_offset_, input_normals, new_normals);
  int counter = 0;
  while(output_normals->points.size() < 30){
    pass_through(pass_offset_, input_normals, output_normals);
    counter++;
    if (counter > 30){
      break;
    }
  }
  // input_normals.swap(new_normals);
  ROS_INFO("Random Square filter");
}

