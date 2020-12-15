find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  tf2_ros
  urdf
  eigen_conversions
  tf_conversions
  interactive_markers
  multicontact_controller_msgs)

add_executable(end_effector_interactive_marker_server EndEffectorInteractiveMarkerServer.cpp)
target_include_directories(end_effector_interactive_marker_server PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(end_effector_interactive_marker_server
  ${catkin_LIBRARIES}
  )
install(TARGETS end_effector_interactive_marker_server
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
