cmake_minimum_required(VERSION 2.8.3)
project(aginika_pcl_ros)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  pcl_conversions
  pcl_msgs
  pcl_ros
  roscpp
  rospy
  message_generation
  genmsg
  sensor_msgs
  tf
  aginika_ros_debug_tools
)

add_message_files(FILES IndicesArray.msg PointsArray.msg ClusterPointIndices.msg Int32Stamped.msg TrackerInfo.msg TrackerInfoArray.msg PointCloud2WithId.msg)
add_service_files(FILES SetPointCloud2.srv SetWorkingState.srv SetFrameId.srv SetFilterMode.srv ICPAlign.srv SetTrackerObjectId.srv ICPAlignReturn.srv SetMultiPointCloud2.srv SetWorkingStateAndTopic.srv SetMainObjectId.srv PointToContour.srv ClassifyCloth.srv  PredictData.srv )
generate_messages(DEPENDENCIES pcl_msgs sensor_msgs)

catkin_package(
  DEPENDS pcl
  CATKIN-DEPENDS pcl_ros message_runtime pcl_msgs jsk_rviz_plugins aginika_ros_debug_tools sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES aginika_pcl_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(aginika_pcl_ros
  SHARED
  src/PCDFeatureProcedure.cpp
  src/FPFH_PCDFeature.cpp
  src/FPFH_Average_PCDFeature.cpp
  src/SHOT_PCDFeature.cpp
  src/AllPassFilter.cpp
  src/RandomRegionFilter.cpp
  src/RandomSquareFilter.cpp
  src/Dot_PCDFeature.cpp
  src/DIFFDOT_PCDFeature.cpp
  src/GRAVDOT_PCDFeature.cpp
  src/SHAPE_CONTEXT_PCDFeature.cpp
)
target_link_libraries(aginika_pcl_ros ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(aginika_pcl_ros ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


add_executable(simple_mutable_model_tracking src/simple_mutable_model_tracking_not_nodelet.cpp)
target_link_libraries(simple_mutable_model_tracking ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(simple_mutable_model_tracking ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# add_executable(multi_tracking_manager src/multi_tracking_manager.cpp)
# target_link_libraries(multi_tracking_manager ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
# add_dependencies(multi_tracking_manager ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(multi_tracking_upgrade_manager src/multi_tracking_upgrade_manager.cpp)
target_link_libraries(multi_tracking_upgrade_manager ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(multi_tracking_upgrade_manager ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

# add_executable(icp_server src/icp_server.cpp)
# target_link_libraries(icp_server ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
# add_dependencies(icp_server ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(generalized_icp_server src/generalized_icp_server.cpp )
target_link_libraries(generalized_icp_server ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(generalized_icp_server ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(selected_pointcloud_saver src/selected_pointcloud_saver.cpp)
target_link_libraries(selected_pointcloud_saver ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(selected_pointcloud_saver ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


add_executable(pcd_feature_sample sample/pcd_feature_sample.cpp)
target_link_libraries(pcd_feature_sample ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES} aginika_pcl_ros)
add_dependencies(pcd_feature_sample ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS simple_mutable_model_tracking multi_tracking_upgrade_manager selected_pointcloud_saver generalized_icp_server pcd_feature_sample
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
