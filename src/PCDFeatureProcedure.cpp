#include <aginika_pcl_ros/PCDFeatureProcedure.h>

PCDFeatureProcedure::PCDFeatureProcedure(){
  normals_.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
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
    ROS_INFO("Open File");
    openPCDFile(filename);

    //calculate features
    ROS_INFO("Calcurate Featues");
    calculateFeatures();
    ROS_INFO("PointCloud nums %ld", normals_->points.size());

    //get features and write out to file
    ROS_INFO("Write out");
    writeOutFeatures(filename);
  }
};

void PCDFeatureProcedure::calculateFeatures(){
  //filter the target normals
  target_filter_->filter(normals_);

  //calculate the features
  target_feature_->calculate(normals_);
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
