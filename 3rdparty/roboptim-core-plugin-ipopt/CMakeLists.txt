cmake_minimum_required(VERSION 3.0.2)
project(roboptim-core-plugin-ipopt)

find_package(catkin REQUIRED)

catkin_package()

include(ExternalProject)

externalproject_add(roboptim-core-plugin-ipopt
  GIT_REPOSITORY https://github.com/roboptim/roboptim-core-plugin-ipopt
  TIMEOUT 120
  CMAKE_ARGS -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=${CATKIN_DEVEL_PREFIX}
  BUILD_COMMAND ${CMAKE_COMMAND} --build <BINARY_DIR> -j8
  )

catkin_add_env_hooks(99.roboptim-core-plugin-ipopt SHELLS bash zsh
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/env-hooks)
