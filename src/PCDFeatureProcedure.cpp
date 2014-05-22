#include <aginika_pcl_ros/PCDFeatureProcedure.h>

PCDFeatureProcedure::PCDFeatureProcedure(){
  normals_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  normals_for_feature_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pnh_ = new ros::NodeHandle("~");
  pub_ = pnh_->advertise<sensor_msgs::PointCloud2>("debug", 1);

  repeat_time_ = 30;
}

void PCDFeatureProcedure::registerPCDFile(std::string filename){
  filenames_.push_back(filename);
};

void PCDFeatureProcedure::openPCDFile(std::string filename){
  pcl::PCDReader reader;

  ROS_INFO("Read File");
  normals_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBNormal> (filename, *normals_)  == -1 ){
    ROS_ERROR("File couldn't open");
    exit (-1);
  }
}

void PCDFeatureProcedure::calculate(){

  int i = 0;
  BOOST_FOREACH (std::string filename, filenames_) {
    ROS_INFO("complete %d / %ld", i, filenames_.size());
    i++;
    //open pcd file.
    openPCDFile(filename);

    //calculate features
    calculateFeatures();

    //get features and write out to file
    ROS_INFO("Write out");
    writeOutFeatures(filename);
  }
};

int PCDFeatureProcedure::clustering(){
  pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
  std::vector<pcl::PointIndices> cluster_indices;

  pcl::EuclideanClusterExtraction<pcl::PointXYZRGBNormal> ec;
  ec.setClusterTolerance (0.02);
  ec.setMinClusterSize (50);
  ec.setMaxClusterSize (100000);
  ec.setSearchMethod (tree);

  ec.setInputCloud (normals_);
  ec.extract (cluster_indices);

  int id = 0;
  int largest_num = 0;
  //get the largest
  for (int i = 0; i < cluster_indices.size(); i++){
    if(largest_num < cluster_indices[i].indices.size()){
      id = i;
      largest_num = cluster_indices[i].indices.size();
    }
  }

  if(cluster_indices.size() == 0)
    return 0;

  //extract the target
  pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr tmp_normals(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices(cluster_indices[id]));
  pcl::ExtractIndices<pcl::PointXYZRGBNormal> extract;
  extract.setInputCloud (normals_);
  extract.setIndices (inliers);
  extract.filter (*tmp_normals);

  tmp_normals.swap(normals_);

  sensor_msgs::PointCloud2 debug_pc2;
  pcl::toROSMsg(*normals_, debug_pc2);
  debug_pc2.header.frame_id="base2";
  debug_pc2.header.stamp = ros::Time::now();

  pub_.publish(debug_pc2);
  return 1;
}

void PCDFeatureProcedure::calculateFeatures(){
  //Clustering and get Most Large.
  ROS_INFO("clustering before %ld ", normals_->points.size());
  int result = clustering();

  if ( result == 0 )
    return;
  //repeat as the time
  for (int i = 0; i < repeat_time_; i++){
    //filter the target normals
    ROS_INFO("filter before %ld ", normals_->points.size());
    normals_for_feature_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
    target_filter_->filter(normals_, normals_for_feature_);

    //calculate the features
    ROS_INFO("feature before %ld ", normals_for_feature_->points.size());
    target_feature_->calculate(normals_for_feature_);
  }
}

void PCDFeatureProcedure::writeOutFeatures(std::string filename){
  std::string feature_name = target_feature_->get_name();
  std::string filter_name  = target_filter_->get_name();
  ros::Time now = ros::Time::now();

  std::stringstream ss;
  ss << filter_name << "_" << feature_name << "_" << now.nsec << "_"<< filename.substr(filename.find_last_of('/') + 1) << ".txt";

  std::string new_filename;
  ss >> new_filename;

  ROS_INFO("new_file_name %s", new_filename.c_str());
  std::vector<std::vector<float> > results = target_feature_->get_result();

  std::ofstream output_file(new_filename.c_str());
  BOOST_FOREACH (std::vector<float> f_vector, results) {
    copy(f_vector.begin(), f_vector.end(), std::ostream_iterator<float>(output_file , ", "));
    output_file << std::endl;
  }
}
