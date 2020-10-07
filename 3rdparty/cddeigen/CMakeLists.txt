cmake_minimum_required(VERSION 3.0.2)
project(cddeigen)

find_package(catkin REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN eigen3>=3.2.7)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cddeigen
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  )

add_library(cddeigen SHARED src/cddeigen.cpp)
target_compile_definitions(cddeigen PUBLIC -DGMPRATIONAL) #これをつけるとdd_set_dが使えなくなる
#target_compile_definitions(cddeigen PUBLIC -DGMPFLOAT) #これを付けないとddd_set_dが無いエラーになる
target_link_libraries(cddeigen
  #cdd
  cddgmp #cddとcddgmpのどちらかをリンクする．両方リンクしてはいけない．gmpの方が機能が多い．
  #mpfr
  #gmpxx # c++の場合はcddgmpに加えてgmpxxもリンクする必要?
  ${EIGEN_LIBRARIES})

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
  )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  )
