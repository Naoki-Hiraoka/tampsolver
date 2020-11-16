find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  iir_filter)

add_executable(joint_states_filter JointStatesFilter.cpp)
target_include_directories(joint_states_filter PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(joint_states_filter
  ${catkin_LIBRARIES}
  )
install(TARGETS joint_states_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
