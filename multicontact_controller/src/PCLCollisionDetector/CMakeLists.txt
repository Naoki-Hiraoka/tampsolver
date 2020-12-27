find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  sensor_msgs
  eigen_conversions
  pcl_ros
  choreonoid_cpp
  multicontact_controller_msgs)

add_library(PCLCollisionDetector
  PCLCollisionDetector.cpp
  PCLCollisionDetectorROS.cpp
  PCLCollisionDetectorROSItem.cpp
  )

target_include_directories(PCLCollisionDetector PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(PCLCollisionDetector PUBLIC
  EndEffectorUtils
  CnoidBodyUtils
  ${catkin_LIBRARIES}
  )

add_executable(pcl_collision_detector PCLCollisionDetectorROSNode.cpp)
target_include_directories(pcl_collision_detector PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(pcl_collision_detector
  PCLCollisionDetector
  ${catkin_LIBRARIES}
  )
install(TARGETS pcl_collision_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
