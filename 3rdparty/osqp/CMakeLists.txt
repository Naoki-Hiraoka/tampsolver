cmake_minimum_required(VERSION 3.0.2)
project(osqp)

find_package(catkin REQUIRED)

include(ExternalProject)

externalproject_add(osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG v0.6.0
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8
  )
