cmake_minimum_required(VERSION 3.1.1)
project(sc_control)
find_package(catkin REQUIRED)

catkin_python_setup()

if(NOT "${CMAKE_VERSION}" VERSION_LESS "3.16")
    set(CMAKE_CXX_STANDARD 17)
    set(CMAKE_CXX_STANDARD_REQUIRED ON)
else()
    add_compile_options(-std=c++11)
endif()

find_package(catkin REQUIRED COMPONENTS
  )

find_package(gazebo REQUIRED)
find_package(roscpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(${roscpp_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")
set(SENSOR_ROS_PLUGINS_LIST "")

catkin_package(
  INCLUDE_DIRS
  LIBRARIES
  CATKIN_DEPENDS
 )

# for launch
install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

# for config
install(DIRECTORY config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN "*~" EXCLUDE)

# for Python scripts
catkin_install_python(PROGRAMS
  scripts/sc_manual_thrusterop.py
  scripts/position_control.py
  scripts/position_command.py
  scripts/position_manager.py
  scripts/bimanual_simple_demo.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
