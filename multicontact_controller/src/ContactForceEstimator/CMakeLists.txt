find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  std_srvs
  geometry_msgs
  sensor_msgs
  eigen_conversions
  choreonoid_cpp
  prioritized_qp
  multicontact_controller_msgs)

add_library(ContactForceEstimator
  ContactForceEstimator.cpp
  ContactForceEstimatorROS.cpp
  ContactForceEstimatorROSItem.cpp
  )

target_include_directories(ContactForceEstimator PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(ContactForceEstimator PUBLIC
  EndEffectorUtils
  CnoidBodyUtils
  ${catkin_LIBRARIES}
  )

add_executable(contact_force_estimator ContactForceEstimatorROSNode.cpp)
target_include_directories(contact_force_estimator PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(contact_force_estimator
  ContactForceEstimator
  ${catkin_LIBRARIES}
  )
install(TARGETS contact_force_estimator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
