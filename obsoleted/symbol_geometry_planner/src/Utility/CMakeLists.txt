find_package(catkin REQUIRED COMPONENTS
  choreonoid_cpp
  )

set(target Utility)
set(sources
  Util.cpp
  )

add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS})

target_link_libraries(${target} PUBLIC
  ${catkin_LIBRARIES}
  RobotConfig
  )

target_compile_definitions(${target} PUBLIC ${CHOREONOID_DEFINITIONS})
