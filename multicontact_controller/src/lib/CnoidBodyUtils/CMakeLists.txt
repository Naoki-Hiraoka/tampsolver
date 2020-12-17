find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  eigen_conversions
  choreonoid_cpp
  multicontact_controller_msgs
  )

add_library(CnoidBodyUtils
  CnoidBodyUtils.cpp
  Contact.cpp
  ContactPoint.cpp
  TorqueJacobianCalculator.cpp
  )

target_include_directories(CnoidBodyUtils PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(CnoidBodyUtils PUBLIC
  ${catkin_LIBRARIES}
  )
