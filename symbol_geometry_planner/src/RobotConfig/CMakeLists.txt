find_package(catkin REQUIRED COMPONENTS
  roscpp roslib #RobotConfig.cpp
  choreonoid
  vclip #EditCollisionModel.cpp
  qhulleigen #EditCollisionModel.cpp
  )

find_package(yaml-cpp REQUIRED)
#find_package(Eigen3 3.2.92 REQUIRED)

set(target RobotConfig)
set(sources
  RobotConfig.cpp
  JointLimitTable.cpp
  EditCollisionModel.cpp
  EndEffector.cpp
  )
link_directories(${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS}) #temporary
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  yaml-cpp #RobotConfig.cpp
  qhull #EditCollisionModel.cpp
  )
# target_link_directories(${target} PUBLIC ${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS}) # require cmake3.13
