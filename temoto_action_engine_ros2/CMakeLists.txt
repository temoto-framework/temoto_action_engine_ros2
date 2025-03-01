cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine_ros2)

add_compile_options(-std=gnu++20 -g)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(temoto_msgs REQUIRED)

add_subdirectory(temoto_action_engine)

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
  temoto_msgs
)

target_link_libraries(action_engine_node
  temoto_action_engine
)

ament_export_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/include)

install(TARGETS action_engine_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
