find_package(catkin REQUIRED COMPONENTS
  choreonoid
  )

set(target Utility)
set(sources
  Util.cpp
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  CnoidBody CnoidUtil
  RobotConfig
  )
target_link_directories(${target} PUBLIC ${catkin_LIBRARY_DIRS} ${CHOREONOID_LIBRARY_DIRS})
