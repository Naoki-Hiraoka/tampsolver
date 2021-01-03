find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  std_msgs
  nav_msgs
  sensor_msgs
  eigen_conversions
  tf2_ros
  choreonoid_cpp
  OsqpEigen
  prioritized_qp
  static_equilibuim_test
  multicontact_controller_msgs)

add_library(ContactBreakAbilityChecker
  ContactBreakAbilityChecker.cpp
  ContactBreakAbilityCheckerROS.cpp
  ContactBreakAbilityCheckerROSItem.cpp
  )

target_include_directories(ContactBreakAbilityChecker PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(ContactBreakAbilityChecker PUBLIC
  CnoidBodyUtils
  EndEffectorUtils
  PCLCollisionDetector
  ${catkin_LIBRARIES}
  )

add_executable(contact_breakability_checker ContactBreakAbilityCheckerROSNode.cpp)
target_include_directories(pwt_controller PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(contact_breakability_checker
  ContactBreakAbilityChecker
  ${catkin_LIBRARIES}
  )
install(TARGETS contact_breakability_checker
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
