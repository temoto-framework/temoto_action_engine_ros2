cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine_ros2)

add_compile_options(-std=c++17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(temoto_action_engine REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/UmrfGraphStart.msg"
  "msg/UmrfGraphStop.msg"
  "msg/UmrfGraphDiff.msg"
  "msg/UmrfGraphFeedback.msg"
)

ament_export_dependencies(rosidl_default_runtime)

set(headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/include
)

add_executable(action_engine_node src/action_engine_node.cpp)
target_include_directories(action_engine_node PUBLIC
  ${headers})

ament_target_dependencies(action_engine_node
  rclcpp
  std_msgs
  temoto_action_engine
)

target_link_libraries(action_engine_node
  temoto_action_engine
)

rosidl_target_interfaces(action_engine_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS action_engine_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
