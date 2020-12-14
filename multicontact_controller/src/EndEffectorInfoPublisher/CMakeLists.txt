find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  eigen_conversions
  multicontact_controller_msgs)

add_executable(end_effector_info_publisher EndEffectorInfoPublisher.cpp)
target_include_directories(end_effector_info_publisher PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(end_effector_info_publisher
  ${catkin_LIBRARIES}
  )
install(TARGETS end_effector_info_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
