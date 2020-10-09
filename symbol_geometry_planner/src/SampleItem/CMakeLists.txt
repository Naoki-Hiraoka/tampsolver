find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib
  actionlib
  pddl_msgs
  roboptim-core
  )

set(target CnoidSymbolGeometryPlannerPlugin)
set(sources
  SymbolGeometryPlannerPlugin.cpp
  PlannerBaseItem.cpp

  IrtViewerSampleItem.cpp #roscpp roslib
  PddlSampleItem.cpp #roscpp roslib actionlib pddl_msgs
  RoboptimSampleItem.cpp #roboptim-core
  RoboptimSparseSampleItem.cpp #roboptim-core
  IKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  MultiContactIKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} RobotConfig IKsolver)
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
