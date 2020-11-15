find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  eigen_conversions
  multicontact_controller_msgs)

add_executable(end_effector_state_publisher EndEffectorStatePublisher.cpp)
target_include_directories(end_effector_state_publisher PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(end_effector_state_publisher
  ${catkin_LIBRARIES}
  )
install(TARGETS end_effector_state_publisher
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
