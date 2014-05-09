cmake_minimum_required(VERSION 2.8.3)
project(aginika_pcl_ros)

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
)

add_message_files(FILES IndicesArray.msg PointsArray.msg ClusterPointIndices.msg Int32Stamped.msg TrackerInfo.msg TrackerInfoArray.msg)
add_service_files(FILES SwitchTopic.srv  TransformScreenpoint.srv CheckCircle.srv EuclideanSegment.srv SetPointCloud2.srv SetWorkingState.srv SetFrameId.srv SetFilterMode.srv ICPAlign.srv SetTrackerObjectId.srv ICPAlignReturn.srv SetMultiPointCloud2.srv SetWorkingStateAndTopic.srv SetMainObjectId.srv PointToContour.srv)
generate_messages(DEPENDENCIES pcl_msgs sensor_msgs)

catkin_package(
  DEPENDS pcl
  CATKIN-DEPENDS pcl_ros message_runtime pcl_msgs jsk_rviz_plugins jsk_debug_tools std_msgs sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES jsk_pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


add_executable(multi_tracking_manager src/multi_tracking_manager.cpp)
target_link_libraries(multi_tracking_manager ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(multi_tracking_manager ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


add_executable(multi_tracking_upgrade_manager src/multi_tracking_upgrade_manager.cpp)
target_link_libraries(multi_tracking_upgrade_manager ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(multi_tracking_upgrade_manager ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(icp_server src/icp_server.cpp)
target_link_libraries(icp_server ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(icp_server ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)

add_executable(generalized_icp_server src/generalized_icp_server.cpp )
target_link_libraries(generalized_icp_server ${catkin_LIBRARIES} ${pcl_ros_LIBRARIES})
add_dependencies(generalized_icp_server ${PROJECT_NAME}_gencpp ${PROJECT_NAME}_gencfg)


install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(TARGETS jsk_pcl_ros pointcloud_screenpoint particle_filter_tracking octree_change_publisher simple_mutable_model_tracking simple_mutable_model_tracking_nonodelet selected_pointcloud_saver icp_server generalized_icp_server
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(FILES jsk_pcl_nodelets.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
