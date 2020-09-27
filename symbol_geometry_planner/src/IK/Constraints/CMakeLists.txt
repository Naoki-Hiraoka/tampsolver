set(target IKConstraints)
set(sources
  IKConstraint.cpp
  PositionConstraint.cpp
  MinMaxJointConstraint.cpp
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} RobotConfig)
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
