cmake_minimum_required(VERSION 3.5)
project(gaode_api_global_planner_nodes)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# dependencies
find_package(ament_cmake_auto REQUIRED)
find_package(cpr REQUIRED)
find_package(gaode_api_route_msgs REQUIRED)
ament_auto_find_build_dependencies()

include_directories(3rd/GeographicLib/include)
include_directories(SYSTEM ${EIGEN3_INCLUDE_DIR})

# build
set(GB_PLANNER_NODE_LIB gaode_api_global_planner_node)
ament_auto_add_library(${GB_PLANNER_NODE_LIB} SHARED
  include/gaode_api_global_planner_nodes/gaode_api_global_planner_node.hpp
  include/gaode_api_global_planner_nodes/visibility_control.hpp
  src/gaode_api_global_planner_node.cpp
  3rd/GeographicLib/src/LocalCartesian.cpp
  3rd/GeographicLib/src/Geocentric.cpp
  3rd/GeographicLib/src/Math.cpp
)
target_link_libraries(${GB_PLANNER_NODE_LIB} cpr::cpr)
target_link_libraries(${GB_PLANNER_NODE_LIB} jsoncpp)
autoware_set_compile_options(${GB_PLANNER_NODE_LIB})

set(GB_PLANNER_NODE_EXE ${GB_PLANNER_NODE_LIB}_exe)
rclcpp_components_register_node(${GB_PLANNER_NODE_LIB}
  PLUGIN "autoware::planning::gaode_api_global_planner_nodes::GaodeApiGlobalPlannerNode"
  EXECUTABLE ${GB_PLANNER_NODE_EXE}
)

# Testing
if(BUILD_TESTING)
  ## Linters were previously not part of this package
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  add_ros_test(
    test/gaode_api_global_planner_node_launch.test.py
    TIMEOUT "30"
  )
endif()

ament_auto_package(
  INSTALL_TO_SHARE
  launch
)
