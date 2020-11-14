find_package(catkin REQUIRED COMPONENTS
  choreonoid_cpp
  prioritized_qp)

add_library(ContactForceEstimator
  ContactForceEstimator.cpp
  ContactForceEstimatorROS.cpp
  )

target_include_directories(ContactForceEstimator PUBLIC
  ${catkin_INCLUDE_DIRS}
  )

target_link_libraries(ContactForceEstimator PUBLIC
  ${catkin_LIBRARIES}
  )
