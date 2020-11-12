# You have to compile choreonoid with -DBUILD_OPENHRP_PLUGIN=ON

find_package(catkin REQUIRED COMPONENTS
  choreonoid
  roscpp
  roslib)

find_package(Boost REQUIRED COMPONENTS system)

include(FindPkgConfig) 
pkg_check_modules(OMNIDYNAMIC omniDynamic4)

link_directories(${CHOREONOID_LIBRARY_DIRS} ${BOOST_LIBRARY_DIRS}) #temporary
add_executable(viewertutorial viewertutorial.cpp)
target_include_directories(viewertutorial PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
# target_link_directories(viewertutorial PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${BOOST_LIBRARY_DIRS}) #require cmake3.13
target_link_libraries(viewertutorial CnoidOpenHRP3.1 CnoidCorba ${CHOREONOID_LIBRARIES}  ${OMNIDYNAMIC_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS viewertutorial
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
