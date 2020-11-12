find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib)

set(sources
  ViewerSamplePlugin.cpp
  ViewerSampleItem.cpp
  )

link_directories(${CHOREONOID_LIBRARY_DIRS}) #temporary
add_library(CnoidViewerSamplePlugin SHARED ${sources})
target_include_directories(CnoidViewerSamplePlugin PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(CnoidViewerSamplePlugin ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES})
# target_link_directories(CnoidViewerSamplePlugin PUBLIC ${CHOREONOID_LIBRARY_DIRS}) #require cmake3.13
target_compile_definitions(CnoidViewerSamplePlugin PUBLIC ${CHOREONOID_DEFINITIONS})

#install(TARGETS ViewerSamplePlugin LIBRARY DESTINATION ${CHOREONOID_PLUGIN_DIR})
set_target_properties(CnoidViewerSamplePlugin PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CHOREONOID_PLUGIN_DIR})
