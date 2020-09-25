cmake_minimum_required(VERSION 3.0.2)
project(osqp)

find_package(catkin REQUIRED)

catkin_package()

include(ExternalProject)

externalproject_add(osqp
  GIT_REPOSITORY https://github.com/oxfordcontrol/osqp
  GIT_TAG v0.6.0
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  # add -j8
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8
  # install Config.cmake to share/osqp/cmake (not lib/cmake/osqp)
  INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR> && mkdir -p ${CATKIN_DEVEL_PREFIX}/share/osqp && rm -r ${CATKIN_DEVEL_PREFIX}/share/osqp/cmake && mv ${CATKIN_DEVEL_PREFIX}/lib/cmake/osqp ${CATKIN_DEVEL_PREFIX}/share/osqp/cmake
  )
