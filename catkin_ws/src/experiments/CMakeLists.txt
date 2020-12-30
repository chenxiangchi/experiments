cmake_minimum_required(VERSION 3.0.2)
project(experiments)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(ur_rtde REQUIRED)

add_library(${PROJECT_NAME}
src/AdmittanceControl.cpp)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur16e_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


add_executable(main src/admittance_control_main.cpp)
target_link_libraries(main PRIVATE ${PROJECT_NAME} ur_rtde::rtde ${catkin_LIBRARIES})


