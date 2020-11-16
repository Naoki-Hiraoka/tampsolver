cmake_minimum_required(VERSION 3.0.2)
project(iir_filter)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
set(TORQUEFILTER_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/TorqueFilter)
set(iir_filter_sources
  src/Filter.cpp
  )
message(${TORQUEFILTER_SOURCE})
catkin_package(
  INCLUDE_DIRS include ${TORQUEFILTER_SOURCE}
  LIBRARIES iir_filter
)

add_library(iir_filter SHARED ${iir_filter_sources})
target_include_directories(iir_filter PUBLIC include ${TORQUEFILTER_SOURCE})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
