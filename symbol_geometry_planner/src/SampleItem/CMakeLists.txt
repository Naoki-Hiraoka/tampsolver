find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  actionlib
  pddl_msgs
  choreonoid
  choreonoid_cpp
  )

find_package(PkgConfig)
pkg_check_modules(roboptim-core roboptim-core REQUIRED)

set(target CnoidSymbolGeometryPlannerPlugin)
set(sources
  SymbolGeometryPlannerPlugin.cpp
#  PlannerBaseItem.cpp

  IrtViewerSampleItem.cpp #roscpp roslib
  PddlSampleItem.cpp #roscpp roslib actionlib pddl_msgs
  RoboptimSampleItem.cpp #roboptim-core
  RoboptimSparseSampleItem.cpp #roboptim-core
  IKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  MultiContactIKSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  ReachSampleItem.cpp #roscpp roslib RobotConfig IKsolver
  )

link_directories(${roboptim-core_LIBRARY_DIRS}) #temporary
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${roboptim-core_INCLUDE_DIRS})
target_link_libraries(${target} ${catkin_LIBRARIES} ${roboptim-core_LIBRARIES} RobotConfig Utility IKsolver)
# target_link_directories(${target} PUBLIC ${roboptim-core_LIBRARY_DIRS}) # require cmake3.13
target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
