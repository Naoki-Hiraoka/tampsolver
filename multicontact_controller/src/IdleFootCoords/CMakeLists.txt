find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf2_ros
  std_srvs
  geometry_msgs
  eigen_conversions
  multicontact_controller_msgs)

add_executable(idle_footcoords IdleFootCoords.cpp)
target_include_directories(idle_footcoords PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(idle_footcoords
  ${catkin_LIBRARIES}
  )
install(TARGETS idle_footcoords
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
