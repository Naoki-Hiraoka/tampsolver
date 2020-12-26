find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_srvs
  sensor_msgs
  eigen_conversions
  choreonoid_cpp
  multicontact_controller_msgs)

add_library(SelfCollisionDetector
  SelfCollisionDetector.cpp
  SelfCollisionDetectorROS.cpp
  SelfCollisionDetectorROSItem.cpp
  )

target_include_directories(SelfCollisionDetector PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(SelfCollisionDetector PUBLIC
  EndEffectorUtils
  CnoidBodyUtils
  ${catkin_LIBRARIES}
  )

add_executable(self_collision_detector SelfCollisionDetectorROSNode.cpp)
target_include_directories(self_collision_detector PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(self_collision_detector
  SelfCollisionDetector
  ${catkin_LIBRARIES}
  )
install(TARGETS self_collision_detector
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
