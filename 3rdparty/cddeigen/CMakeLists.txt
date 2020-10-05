cmake_minimum_required(VERSION 3.0.2)
project(cddeigen)

find_package(catkin REQUIRED)
find_package(Eigen3 3.2.92 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cddeigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  )

add_library(cddeigen SHARED src/cddeigen.cpp)
target_compile_definitions(cddeigen PUBLIC -DGMPRATIONAL)
target_link_libraries(cddeigen cddgmp Eigen3::Eigen)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
