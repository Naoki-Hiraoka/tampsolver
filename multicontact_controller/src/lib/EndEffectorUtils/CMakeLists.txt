find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  multicontact_controller_msgs
  eigen_conversions
  )

add_library(EndEffectorUtils
  EndEffectorUtils.cpp
  )

target_include_directories(EndEffectorUtils PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(EndEffectorUtils PUBLIC
  CnoidBodyUtils
  ${catkin_LIBRARIES}
  )
