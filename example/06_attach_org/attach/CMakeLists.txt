cmake_minimum_required(VERSION 3.8)
project(attach)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)




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

# gcat add+
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(rviz_visual_tools REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(graph_msgs REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(moveit_visual_tools REQUIRED)

#find_package(geometry REQUIRED)
# gcat add;

include_directories(
	include
	/opt/ros/humble/include/graph_msgs
)

add_executable(exe02 src/move_group_interface_tutorial.cpp)

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/opt/ros/humble/lib")

ament_target_dependencies(exe02
  rclcpp
  rviz_visual_tools
  moveit_ros_planning    
  moveit_ros_move_group
  moveit_ros_planning_interface

  moveit_visual_tools
)

install(TARGETS
  exe02
  DESTINATION lib/${PROJECT_NAME}
)

install (DIRECTORY
	launch
	DESTINATION share/${PROJECT_NAME}/
)

# gcat add;

ament_package()
