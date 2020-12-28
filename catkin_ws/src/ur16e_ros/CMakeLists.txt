cmake_minimum_required(VERSION 3.0.2)
project(ur16e_ros)

 add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
)

find_package(ur_rtde REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ur16e_controller
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


 add_executable(receiver src/receiver.cpp)
 target_link_libraries(receiver PRIVATE ur_rtde::rtde ${catkin_LIBRARIES})

 add_executable(controller src/controller.cpp)
 target_link_libraries(controller PRIVATE ur_rtde::rtde ${catkin_LIBRARIES})
