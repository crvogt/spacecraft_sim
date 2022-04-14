cmake_minimum_required(VERSION 2.8.3)
project(sc_gazebo_plugins)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()

find_package(catkin REQUIRED COMPONENTS gazebo_dev roscpp message_generation xacro)
find_package(Eigen3 REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime gazebo_dev roscpp 
)

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

## Declare a C++ library
add_library(sc_gazebo_thrust_plugin
  src/sc_gazebo_thrust_plugin.cc
)
target_link_libraries(sc_gazebo_thrust_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS sc_gazebo_thrust_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

set(XACRO_INORDER)
if(DEFINED ENV{ROS_DISTRO})
  if($ENV{ROS_DISTRO} STREQUAL "kinetic")
    set(XACRO_INORDER INORDER)
  endif()
endif()

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
