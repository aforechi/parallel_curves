cmake_minimum_required(VERSION 3.3)

project(parallel_curves LANGUAGES CXX)

set( CMAKE_CXX_STANDARD 14 )

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package(NETPBM) #sudo apt install libnetpbm10-dev

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav2_common REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2_ros REQUIRED)

nav2_package() 

set(dependencies ament_cmake
  builtin_interfaces
  nav2_common
  nav2_core
  nav2_costmap_2d
  nav2_msgs
  nav2_util
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  tf2_ros
)

include_directories(include)

add_library(${PROJECT_NAME} SHARED
    src/parallel_curves.cpp
    src/parallel_curves_ros.cpp
    src/collision_detector.cpp
)

ament_target_dependencies( ${PROJECT_NAME} ${dependencies} )

pluginlib_export_plugin_description_file(nav2_core pc_plugin.xml)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION lib
   LIBRARY DESTINATION lib
   RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(shortest_path_test test/shortest_path.cpp)
  ament_add_gtest(parallel_curves_grid_test test/parallel_curves_grid.cpp src/parallel_curves.cpp
    ENV PARALLEL_CURVES_MAP_PATH=${CMAKE_CURRENT_SOURCE_DIR}/test/map/)
  ament_add_gtest(parallel_curves_costmap_test test/parallel_curves_costmap.cpp src/parallel_curves.cpp test/map/read_pgm.cpp
    ENV PARALLEL_CURVES_MAP_PATH=${CMAKE_CURRENT_SOURCE_DIR}/test/map/)
  ament_target_dependencies(parallel_curves_costmap_test ${dependencies})
  target_link_libraries(parallel_curves_costmap_test netpbm)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${dependencies})

ament_package()
