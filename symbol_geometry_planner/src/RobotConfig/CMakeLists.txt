find_package(catkin REQUIRED COMPONENTS
  choreonoid
  )

find_package(yaml-cpp REQUIRED)
#find_package(Eigen3 3.2.92 REQUIRED)

set(target RobotConfig)
set(sources
  RobotConfig.cpp
  JointLimitTable.cpp
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} yaml-cpp)
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
