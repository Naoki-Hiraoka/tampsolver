cmake_minimum_required(VERSION 3.0.2)
project(roboptim-core)

find_package(catkin REQUIRED)

include(ExternalProject)

externalproject_add(roboptim-core
  GIT_REPOSITORY https://github.com/roboptim/roboptim-core
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8
  )

catkin_add_env_hooks(99.roboptim-core SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
