find_package(catkin REQUIRED COMPONENTS
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
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  CnoidBody CnoidUtil
  yaml-cpp #RobotConfig.cpp
  qhull #EditCollisionModel.cpp
  )
target_link_directories(${target} PUBLIC ${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS})
