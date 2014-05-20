#include <aginika_pcl_ros/PCDFeatureProcedure.h>
#include <aginika_pcl_ros/FPFH_PCDFeature.h>


int main(){
  PCDFeatureProcedure* pcd_fp = new PCDFeatureProcedure();
  FPFH_PCDFeature* fpfh_pcd_feature = new FPFH_PCDFeature();

  pcd_fp->setFeature(fpfh_pcd_feature);
  pcd_fp->calculateFeature();
}
