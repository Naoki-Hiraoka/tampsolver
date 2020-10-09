cmake_minimum_required(VERSION 3.0.2)
project(vclip)

#高速化
set(CMAKE_BUILD_TYPE Release) #options are: None Debug Release RelWithDebInfo MinSizeRel.
#set(CMAKE_BUILD_TYPE RelWithDebInfo) #options are: None Debug Release RelWithDebInfo MinSizeRel.

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(hrpsys hrpsys-base REQUIRED)
set(COLLISIONDETECTOR_SOURCE ${hrpsys_PREFIX}/share/hrpsys/src/rtc/CollisionDetector)
set(vclip_dir vclip_1.0/)
set(vclip_sources
  ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/src/vclip.C
  ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/src/PolyTree.C
  ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/src/mv.C
  )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/include
  LIBRARIES vclip
)

add_library(vclip SHARED ${vclip_sources})
target_include_directories(vclip PUBLIC ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/include)
target_compile_definitions(vclip PUBLIC -DQHULL)
target_link_libraries(vclip PUBLIC qhull)

#############
## Install ##
#############

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

## Mark cpp header files for installation
install(DIRECTORY ${COLLISIONDETECTOR_SOURCE}/${vclip_dir}/include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
