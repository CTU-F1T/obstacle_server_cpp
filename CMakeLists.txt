set(ROS_VERSION $ENV{ROS_VERSION})

if (NOT DEFINED ROS_VERSION OR '${ROS_VERSION} STREQUAL '1)
    set (ROS_VERSION 1)
    cmake_minimum_required(VERSION 2.8.3)
else()
    cmake_minimum_required(VERSION 3.3)
endif()

project(obstacle_server_cpp)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++14)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

set(dep_pkgs
    obstacle_msgs
    std_msgs
    sensor_msgs
    nav_msgs
)

set(dep_ros1_pkgs
    tf
)

set(dep_ros2_pkgs
    tf2
    tf2_ros
    tf2_geometry_msgs
)


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
if (ROS_VERSION EQUAL 1)
add_definitions(-DROS1_BUILD)
find_package(catkin REQUIRED
    roscpp
    ${dep_pkgs}
    ${dep_ros1_pkgs}
)

catkin_package(
   CATKIN_DEPENDS
    roscpp
    ${dep_pkgs}
    ${dep_ros1_pkgs}
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/server.cpp)

add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
)
elseif (ROS_VERSION EQUAL 2)
add_definitions(-DROS2_BUILD)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

foreach (pkg ${dep_pkgs})
    find_package(${pkg} REQUIRED)
endforeach()

foreach (pkg ${dep_ros2_pkgs})
    find_package(${pkg} REQUIRED)
endforeach()

add_executable(${PROJECT_NAME}_node src/server.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  ${dep_pkgs}
  ${dep_ros2_pkgs}
)

install(
  TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
else()
    message(FATAL_ERROR "ROS_VERSION is not set.")
endif()

