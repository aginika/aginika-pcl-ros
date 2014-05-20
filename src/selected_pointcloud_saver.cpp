#include <ros/ros.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/io.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/impl/filter.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <time.h>

void cloud_cb(const sensor_msgs::PointCloud2 pc)
{
  static int j = 0;
  ROS_ERROR("create pcd callback %d", j);
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PCLPointCloud2 pcl_pc;

  std::vector<int> indices;
  pcl_conversions::toPCL(pc, pcl_pc);
  pcl::fromPCLPointCloud2 (pcl_pc, *cloud);

  cloud->is_dense = false;
  pcl::removeNaNFromPointCloud(*cloud, *cloud, indices);

  pcl::PCDWriter writer;

  std::stringstream ss;
  ros::Time now = ros::Time::now();
  ss << now.sec << ".pcd";
  std::cout << "Saved " << ss.str() << std::endl;
  writer.write<pcl::PointXYZRGBNormal> (ss.str (), *cloud, false);
  j++;
}

int main(int argc, char* argv[]){
  ros::init(argc, argv, "selected_pointcloud_saver");
  ros::NodeHandle n;
  ros::Subscriber sub_ = n.subscribe("input", 1, &cloud_cb);

  ros::spin();
  return 0;
}
