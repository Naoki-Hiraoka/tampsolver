find_package(catkin REQUIRED COMPONENTS
  vclip
  cddeigen #CddSCFRConstraint.cpp #CddSCFRConstraint2.cpp
  qhulleigen #CddSCFRConstraint.cpp #CddSCFRConstraint2.cpp
  clpeigen #LPSCFRConstraint.cpp
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
  LPSCFRConstraint.cpp
  )

link_directories(${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS}) #temporary
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES}
  RobotConfig
  CnoidAISTCollisionDetector #ASITCollisionConstraint.cpp
  vclip #VclipLinkPair.cpp, VclipCollisionConstraint.cpp
  )
# target_link_directories(${target} PUBLIC ${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS}) #require cmake3.13
