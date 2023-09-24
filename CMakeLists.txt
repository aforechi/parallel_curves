cmake_minimum_required(VERSION 3.0.2)
project(parallel_curves LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
#add_compile_options(-std=c++14)

set( CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake )

find_package(NETPBM) #sudo apt install libnetpbm10-dev

find_package(catkin REQUIRED COMPONENTS
  roscpp
  costmap_2d
  pluginlib
  nav_core
  base_local_planner
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS 
    roscpp
    roslib
    costmap_2d
    pluginlib
    nav_core
    base_local_planner
    geometry_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
    src/parallel_curves.cpp
)

add_library(${PROJECT_NAME}_ros 
    src/parallel_curves_ros.cpp
    src/collision_detector.cpp
)

add_dependencies(${PROJECT_NAME}_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_ros
   ${catkin_LIBRARIES} 
   ${PROJECT_NAME}
)

install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_ros
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(FILES pc_plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(shortest_path_test test/shortest_path.cpp)
  catkin_add_gtest(parallel_curves_grid_test test/parallel_curves_grid.cpp)
  catkin_add_gtest(parallel_curves_costmap_test test/parallel_curves_costmap.cpp test/map/read_pgm.cpp)
  target_link_libraries(parallel_curves_grid_test ${PROJECT_NAME})
  target_link_libraries(parallel_curves_costmap_test ${catkin_LIBRARIES} ${PROJECT_NAME} netpbm)
endif()
