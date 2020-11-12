cmake_minimum_required(VERSION 3.0.2)
project(OsqpEigen)

find_package(catkin REQUIRED COMPONENTS
  osqp)

catkin_package()

include(ExternalProject)

externalproject_add(OsqpEigen
  GIT_REPOSITORY https://github.com/robotology/osqp-eigen
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  # add -j8
  # BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8 # require cmake3.12
  BUILD_COMMAND make -j8
  # install Config.cmake to share/OsqpEigen/cmake (not lib/cmake/OsqpEigen)
  # INSTALL_COMMAND ${CMAKE_COMMAND} --install <BINARY_DIR> && mkdir -p ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen && rm -r ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen/cmake && mv ${CATKIN_DEVEL_PREFIX}/lib/cmake/OsqpEigen ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen/cmake
  INSTALL_COMMAND make install && mkdir -p ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen && rm -r ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen/cmake && mv ${CATKIN_DEVEL_PREFIX}/lib/cmake/OsqpEigen ${CATKIN_DEVEL_PREFIX}/share/OsqpEigen/cmake
  )
