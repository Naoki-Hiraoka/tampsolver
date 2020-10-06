find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib
  actionlib
  pddl_msgs
  roboptim-core
  )

add_subdirectory(RobotConfig)
add_subdirectory(IK)

set(target CnoidSymbolGeometryPlannerPlugin)
set(sources
  SymbolGeometryPlannerPlugin.cpp
  PlannerBaseItem/PlannerBaseItem.cpp

  IrtViewerSampleItem/IrtViewerSampleItem.cpp #roscpp roslib
  PddlSampleItem/PddlSampleItem.cpp #roscpp roslib actionlib pddl_msgs
  RoboptimSampleItem/RoboptimSampleItem.cpp #roboptim-core
  RoboptimSparseSampleItem/RoboptimSparseSampleItem.cpp #roboptim-core
  IKSampleItem/IKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  MultiContactIKSampleItem/MultiContactIKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} RobotConfig IKsolver)
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
