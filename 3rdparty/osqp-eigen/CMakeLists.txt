cmake_minimum_required(VERSION 3.0.2)
project(osqp-eigen)

find_package(catkin REQUIRED)

include(ExternalProject)

externalproject_add(osqp-eigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8
  )
