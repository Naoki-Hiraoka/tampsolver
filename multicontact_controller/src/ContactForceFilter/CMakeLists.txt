find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  iir_filter
  geometry_msgs
  eigen_conversions
  multicontact_controller_msgs)

add_executable(contact_force_filter ContactForceFilter.cpp)
target_include_directories(contact_force_filter PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(contact_force_filter
  EndEffectorUtils
  ${catkin_LIBRARIES}
  )
install(TARGETS contact_force_filter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
