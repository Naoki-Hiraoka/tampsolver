find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  hrpsys_ros_bridge
  multicontact_controller_msgs)

add_executable(motor_temperature_estimator
  MotorTemperatureEstimator.cpp
  MotorTemperatureEstimatorNode.cpp)
target_include_directories(motor_temperature_estimator PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(motor_temperature_estimator
  ${catkin_LIBRARIES}
  )
install(TARGETS motor_temperature_estimator
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
