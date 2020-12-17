find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  sensor_msgs
  eigen_conversions
  tf2_ros
  choreonoid_cpp
  multicontact_controller_msgs)

add_library(PWTController
  PWTController.cpp
  PWTControllerROS.cpp
  PWTControllerROSItem.cpp
  )

target_include_directories(PWTController PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(PWTController PUBLIC
  CnoidBodyUtils
  ${catkin_LIBRARIES}
  )

add_executable(pwt_controller PWTControllerROSNode.cpp)
target_include_directories(pwt_controller PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(pwt_controller
  PWTController
  ${catkin_LIBRARIES}
  )
install(TARGETS pwt_controller
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
