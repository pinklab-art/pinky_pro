cmake_minimum_required(VERSION 3.8)
project(pinky_gz_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  message(STATUS "This package is skipped on aarch64.")
  return()
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ros_gz REQUIRED)
find_package(gz_ros2_control REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)

include_directories(plugins)

add_library(gz-sim-lamp-control-system
  SHARED
  plugins/gz_lamp_control_plugin.cpp
)
target_link_libraries(gz-sim-lamp-control-system PRIVATE gz-sim8::core gz-plugin2::register)
ament_target_dependencies(gz-sim-lamp-control-system
)

install(
  DIRECTORY
    worlds
    models
    launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS
  gz-sim-lamp-control-system
  DESTINATION
  lib
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
