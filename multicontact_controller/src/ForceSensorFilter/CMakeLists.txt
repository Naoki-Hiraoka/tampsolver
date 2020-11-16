find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  eigen_conversions
  iir_filter)

add_executable(force_sensor_filter ForceSensorFilter.cpp)
target_include_directories(force_sensor_filter PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(force_sensor_filter
  ${catkin_LIBRARIES}
  )
install(TARGETS force_sensor_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
