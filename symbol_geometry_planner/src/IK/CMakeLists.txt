find_package(catkin REQUIRED COMPONENTS
  choreonoid
  OsqpEigen
  )

#find_package(Eigen3 3.2.92 REQUIRED)

add_subdirectory(Constraints)

set(target IKsolver)
set(sources
  IKsolver.cpp
  OsqpEigenSolver.cpp
  )
add_library(${target} SHARED ${sources})
target_include_directories(${target} PUBLIC ${catkin_INCLUDE_DIRS} ${CHOREONOID_INCLUDE_DIRS})
target_link_libraries(${target} PUBLIC ${CHOREONOID_LIBRARIES} ${catkin_LIBRARIES} IKConstraints OsqpEigen)
target_link_directories(${target} PUBLIC ${CHOREONOID_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
