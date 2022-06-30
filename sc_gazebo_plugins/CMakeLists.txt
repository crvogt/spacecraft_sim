cmake_minimum_required(VERSION 2.8.3)
project(sc_gazebo_plugins)

# Set policy for CMake 3.1+. Use OLD policy to let FindBoost.cmake, dependency
# of gazebo, use quoted variables in if()

find_package(catkin REQUIRED COMPONENTS 
  gazebo_dev 
  gazebo_plugins
  geometry_msgs
  roscpp 
  sensor_msgs
  std_msgs
  message_generation 
  sc_gazebo_plugins_msgs
  xacro)
find_package(Eigen3 REQUIRED)
find_package(Protobuf REQUIRED)

# Add Gazebo custom protobuf messages
set(PROTOBUF_IMPORT DIRS "")
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9.]+$")
    set(PROTOBUF_IMPORT_DIRS "${ITR}/gazebo/msgs/proto")
  endif()
endforeach()

set(GAZEBO_MSG_INCLUDE_DIRS)
foreach(ITR ${GAZEBO_INCLUDE_DIRS})
  if(ITR MATCHES ".*gazebo-[0-9]+$")
    set(GAZEBO_MSG_INCLUDE_DIRS "${ITR}/gazebo/msgs")
  endif()
endforeach()

file(GLOB msgs msgs/*.proto)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    sc_imu_gazebo_plugin
  CATKIN_DEPENDS 
    message_runtime 
    gazebo_dev 
    gazebo_plugins
    roscpp 
    sensor_msgs
    sc_gazebo_plugins_msgs
    geometry_msgs
    std_msgs
)

# Plugins require c++11
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

include_directories( include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GAZEBO_MSG_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
)
link_directories(
  ${catkin_LIBRARY_DIRS}
)

protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${msgs})
add_library(sc_sensor_gazebo_msgs SHARED ${PROTO_SRCS})
target_link_libraries(sc_sensor_gazebo_msgs ${PROTOBUF_LIBRARY} ${GAZEBO_LIBRARIES})

## Declare a C++ library
add_library(sc_led_visual_plugin
  src/sc_led_visual_plugin.cc
)
target_link_libraries(sc_led_visual_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS sc_led_visual_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

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

add_library(sc_plug_and_socket_plugin
  SHARED
  src/sc_plug_and_socket_plugin.cc
)
target_link_libraries(sc_plug_and_socket_plugin
  ${catkin_LIBRARIES}
  ${Eigen_LIBRARIES}
)
install(TARGETS sc_plug_and_socket_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(sc_imu_gazebo_plugin
  src/sc_imu_gazebo_plugin.cc
  src/ROSBasePlugin.cc
  src/ROSBaseModelPlugin.cc
)
target_link_libraries(sc_imu_gazebo_plugin
  ${catkin_LIBRARIES} 
  ${GAZEBO_LIBRARIES}
)
add_dependencies(sc_imu_gazebo_plugin 
  sc_sensor_gazebo_msgs
  ${catkin_EXPORTED_TARGETS}
)
install(TARGETS sc_imu_gazebo_plugin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_library(sc_pose_gt_plugin
  src/sc_pose_gt_plugin.cc
)
target_link_libraries(sc_pose_gt_plugin
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)
install(TARGETS sc_pose_gt_plugin
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
