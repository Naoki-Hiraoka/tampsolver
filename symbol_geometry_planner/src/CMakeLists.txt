find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib
  actionlib
  pddl_msgs)

find_package(PkgConfig REQUIRED)
pkg_check_modules(roboptim-core roboptim-core REQUIRED)

set(target CnoidSymbolGeometryPlannerPlugin)
set(sources
  SymbolGeometryPlannerPlugin.cpp
  PlannerBaseItem/PlannerBaseItem.cpp

  IrtViewerSampleItem/IrtViewerSampleItem.cpp #roscpp roslib
  PddlSampleItem/PddlSampleItem.cpp #roscpp roslib actionlib pddl_msgs
  RoboptimSampleItem/RoboptimSampleItem.cpp #roboptim-core
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS} ${roboptim-core_INCLUDE_DIRS})
target_link_libraries(${target} ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} ${roboptim-core_LIBRARIES})
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS} ${roboptim-core_LIBRARIY_DIRS})
target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
