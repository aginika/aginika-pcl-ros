#include <aginika_pcl_ros/PCDFeatureProcedure.h>

PCDFeatureProcedure::PCDFeatureProcedure(){
  normals.reset(new pcl::PointCloud<pcl::PointXYZRGBNormal>());
}

void PCDFeatureProcedure::openPCDFiles(std::string filename){
}

void PCDFeatureProcedure::calculateFeatures(){
  //  target_feature.calculate(normals);
}

void PCDFeatureProcedure::calculateFeature(){
  target_feature->calculate(normals);
}

void PCDFeatureProcedure::writeOutFeature(){
}

void PCDFeatureProcedure::setFeature(PCDFeature* feature){
  target_feature = feature;
}
