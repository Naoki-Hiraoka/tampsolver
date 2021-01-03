cmake_minimum_required(VERSION 3.0.2)
project(clpeigen)

# 高速化
set(CMAKE_BUILD_TYPE RelWithDebInfo) #options are: None Debug Release RelWithDebInfo MinSizeRel.

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3>=3.2.7)
pkg_check_modules(clp REQUIRED clp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES clpeigen
  DEPENDS EIGEN
)

add_library(clpeigen SHARED src/clpeigen.cpp)
target_link_libraries(clpeigen PUBLIC
  ${clp_LIBRARIES}
  ${EIGEN_LIBRARIES})
target_include_directories(clpeigen PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  )

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
