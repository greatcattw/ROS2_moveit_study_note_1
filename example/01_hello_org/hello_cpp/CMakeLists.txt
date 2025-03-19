cmake_minimum_required(VERSION 3.8)
project(hello_cpp)



#cmake_minimum_required(VERSION 3.5)
#project(hello_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(hello_cpp src/hello_cpp.cpp)
ament_target_dependencies(hello_cpp rclcpp)

install(TARGETS
  hello_cpp
  DESTINATION lib/${PROJECT_NAME})

ament_package()



