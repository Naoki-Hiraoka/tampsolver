find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  sensor_msgs
  eigen_conversions
  choreonoid_cpp
  multicontact_controller_msgs)

add_library(MultiContactFootCoords
  MultiContactFootCoords.cpp
  MultiContactFootCoordsROS.cpp
  MultiContactFootCoordsROSItem.cpp
  )

target_include_directories(MultiContactFootCoords PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(MultiContactFootCoords PUBLIC
  ${catkin_LIBRARIES}
  )

add_executable(multicontact_footcoords MultiContactFootCoordsROSNode.cpp)
target_include_directories(multicontact_footcoords PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(multicontact_footcoords
  MultiContactFootCoords
  ${catkin_LIBRARIES}
  )
install(TARGETS multicontact_footcoords
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
