find_package(catkin REQUIRED COMPONENTS
  roscpp
  hrpsys_ros_bridge
  multicontact_controller_msgs)

add_executable(motor_temperature_limiter
  MotorTemperatureLimiter.cpp
  )
target_include_directories(motor_temperature_limiter PUBLIC
  ${catkin_INCLUDE_DIRS}
  )
target_link_libraries(motor_temperature_limiter
  ${catkin_LIBRARIES}
  )
install(TARGETS motor_temperature_limiter
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
