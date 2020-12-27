find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  eigen_conversions
  vclip
  qhulleigen
  choreonoid
  choreonoid_cpp
  multicontact_controller_msgs
  )

add_library(CnoidBodyUtils
  CnoidBodyUtils.cpp
  Contact.cpp
  ContactPoint.cpp
  TorqueJacobianCalculator.cpp
  JointLimitTable.cpp
  Interaction.cpp
  Collision.cpp
  )

target_include_directories(CnoidBodyUtils PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

set(CHOREONOID_LIBRARIES_FULL)
foreach(lib ${CHOREONOID_LIBRARIES})
  find_library(${lib}_full NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_LIBRARIES_FULL ${CHOREONOID_LIBRARIES_FULL} ${${lib}_full})
endforeach(lib)

target_link_libraries(CnoidBodyUtils PUBLIC
  ${catkin_LIBRARIES}
  ${CHOREONOID_LIBRARIES_FULL}
  )
