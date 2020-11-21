find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  nav_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  eigen_conversions
  choreonoid_cpp
  hrpsys_ros_bridge
  multicontact_controller_msgs)

add_library(WalkController
  WalkController.cpp
  WalkControllerROS.cpp
  WalkControllerROSItem.cpp
  )

target_include_directories(WalkController PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(WalkController PUBLIC
  ${catkin_LIBRARIES}
  )

add_executable(walk_controller WalkControllerROSNode.cpp)
target_include_directories(walk_controller PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(walk_controller
  WalkController
  ${catkin_LIBRARIES}
  )
install(TARGETS walk_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
