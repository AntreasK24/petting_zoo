cmake_minimum_required(VERSION 3.8)
project(petting_zoo_navigation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(std_msgs REQUIRED)


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

add_executable(walking_around src/walking_around.cpp)
add_executable(arm_track_controller src/arm_track_controller.cpp)
target_include_directories(walking_around PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_include_directories(arm_track_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
add_executable(human_detector src/human_detector.cpp)
target_include_directories(human_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

ament_target_dependencies(walking_around rclcpp geometry_msgs sensor_msgs std_msgs)
ament_target_dependencies(arm_track_controller rclcpp sensor_msgs vision_msgs trajectory_msgs std_msgs)
ament_target_dependencies(human_detector rclcpp sensor_msgs vision_msgs cv_bridge OpenCV)
install(TARGETS walking_around arm_track_controller human_detector DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY worlds launch DESTINATION share/${PROJECT_NAME})



ament_package()
