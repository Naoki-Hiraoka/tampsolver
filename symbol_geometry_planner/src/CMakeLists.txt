find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib)

set(target CnoidSymbolGeometryPlannerPlugin)
set(sources
  SymbolGeometryPlannerPlugin.cpp
  PlannerBaseItem/PlannerBaseItem.cpp

  IrtViewerSampleItem/IrtViewerSampleItem.cpp
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES})
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS})
target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
set_target_properties(${target} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
