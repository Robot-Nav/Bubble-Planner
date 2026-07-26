cmake_minimum_required(VERSION 3.0.2)
project(bubble_planner)

add_compile_options(-std=c++17 -O3 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  trajectory_msgs
  pcl_ros
  pcl_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common kdtree)

find_path(GCOPTER_INCLUDE_DIR
  NAMES gcopter/minco.hpp
  HINTS
    ${PROJECT_SOURCE_DIR}/third_party/GCOPTER/gcopter/include
    ${PROJECT_SOURCE_DIR}/../GCOPTER/gcopter/include
    $ENV{GCOPTER_ROOT}/gcopter/include
  PATHS /usr/local/include /usr/include
)

if(NOT GCOPTER_INCLUDE_DIR)
  message(FATAL_ERROR
    "GCOPTER headers were not found. Run scripts/setup_dependencies.sh, "
    "or set GCOPTER_ROOT to the cloned GCOPTER repository.")
endif()

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs nav_msgs std_msgs
                 visualization_msgs trajectory_msgs pcl_ros pcl_conversions
  DEPENDS Eigen3 PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GCOPTER_INCLUDE_DIR}
)
add_definitions(${PCL_DEFINITIONS})

add_library(bubble_planner_core
  src/obstacle_index.cpp
  src/astar3d.cpp
  src/corridor_generator.cpp
  src/rhc_manager.cpp
  src/minco_optimizer.cpp
  src/bubble_planner.cpp
)
target_link_libraries(bubble_planner_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(bubble_planner_node src/bubble_planner_node.cpp)
target_link_libraries(bubble_planner_node bubble_planner_core ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS bubble_planner_core bubble_planner_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/bubble_planner/
  DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}/bubble_planner)
install(PROGRAMS scripts/demo_world.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch config rviz docs
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
