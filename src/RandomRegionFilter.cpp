#include <aginika_pcl_ros/RandomRegionFilter.h>

RandomRegionFilter::RandomRegionFilter(){
  filter_name_ = "random_region";
  pass_offset_ = 0.02;
  srand (time(NULL));
}

std::string RandomRegionFilter::pass_through( double pass_offset, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr &cloud_normals_pass)
{
  pcl::PointXYZRGBNormal max_pt,min_pt;
  pcl::getMinMax3D(*cloud_normals, min_pt, max_pt);

  double lucky = 0;
  std::string axis("x");
  int rand_xyz = rand()%2;//%3;
  if (rand_xyz == 0)
    lucky = min_pt.x - pass_offset + (max_pt.x - min_pt.x - pass_offset*2) * 1.0 * rand() / RAND_MAX;
  else {//if (rand_xyz == 1){
    lucky = min_pt.y - pass_offset + (max_pt.y - min_pt.y - pass_offset*2) * 1.0 * rand() / RAND_MAX;
    axis = std::string("y");
  }

  // ROS_INFO("normals %ld, lucky %lf %f max_pt.x %f min_pt.x %f", cloud_normals->points.size(), lucky, lucky + pass_offset, max_pt.x, min_pt.x);
  pcl::PassThrough<pcl::PointXYZRGBNormal> pass;
  pass.setInputCloud (cloud_normals);
  pass.setFilterFieldName (axis);
  float small = std::min(lucky, lucky + pass_offset);
  float large = std::max(lucky, lucky + pass_offset);
  pass.setFilterLimits (small, large);
  pass.filter (*cloud_normals_pass);
  return axis;
}


void RandomRegionFilter::filter(pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& input_normals, pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr& output_normals){
  // pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr new_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>);
  // while(new_normals->points.size() == 0)
  //   pass_through(pass_offset_, input_normals, new_normals);
  int counter = 0;
  while(output_normals->points.size() == 0){
    pass_through(pass_offset_, input_normals, output_normals);
    counter++;
    if (counter > 30){
      break;
    }
  }
  // input_normals.swap(new_normals);
  ROS_INFO("Random Region filter");
}

