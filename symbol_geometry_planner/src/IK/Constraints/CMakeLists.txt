find_package(catkin REQUIRED COMPONENTS
  vclip
  cddeigen #CddSCFRConstraint.cpp #CddSCFRConstraint2.cpp
  qhulleigen #CddSCFRConstraint.cpp #CddSCFRConstraint2.cpp
  )

#main
set(target IKConstraints)
set(sources
  IKConstraint.cpp
  PositionConstraint.cpp
  MinMaxJointConstraint.cpp
  CollisionConstraint.cpp
  AISTCollisionConstraint.cpp
  VclipLinkPair.cpp
  VClipCollisionConstraint.cpp
  SCFRConstraint.cpp
  CddSCFRConstraint.cpp
  CddSCFRConstraint2.cpp
  )

add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  CnoidBody CnoidUtil
  RobotConfig
  CnoidAISTCollisionDetector #ASITCollisionConstraint.cpp
  vclip #VclipLinkPair.cpp, VclipCollisionConstraint.cpp
  )
target_link_directories(${target} PUBLIC ${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS})
