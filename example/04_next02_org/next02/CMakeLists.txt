cmake_minimum_required(VERSION 3.8)
project(next02)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

# gcat add+
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rviz_visual_tools REQUIRED)
# gcat add;

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

# gcat add+
add_executable(exe02 src/planning_scene_ros_api_tutorial.cpp)

ament_target_dependencies(exe02
  rclcpp
  moveit_ros_planning  
  rviz_visual_tools
)

install(TARGETS
  exe02
  DESTINATION lib/${PROJECT_NAME}
)
# gcat add;
ament_package()
