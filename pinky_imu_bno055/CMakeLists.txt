cmake_minimum_required(VERSION 3.8)
project(pinky_imu_bno055)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(realtime_tools REQUIRED)

if(CMAKE_SYSTEM_PROCESSOR STREQUAL "aarch64")
  add_executable(main_node src/main_node.cpp)
  ament_target_dependencies(main_node sensor_msgs rclcpp angles realtime_tools)
  target_link_libraries(main_node
    wiringPi
  )

  install(TARGETS
    main_node
    DESTINATION lib/${PROJECT_NAME}
  )
else()
  message(STATUS "${PROJECT_NAME} build only for ARM64. Skip building.")
endif()

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
