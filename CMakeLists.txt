cmake_minimum_required(VERSION 3.5)
project(temoto_action_engine_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

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
  "msg/UmrfGraphDiff.msg"
  "msg/BroadcastStartUmrfGraph.msg"
  "msg/BroadcastStopUmrfGraph.msg"
  "srv/StartUmrfGraph.srv"
  "srv/StopUmrfGraph.srv"
  "srv/GetUmrfGraphs.srv"
)

ament_export_dependencies(rosidl_default_runtime)

set(headers
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/include
  ${CMAKE_CURRENT_SOURCE_DIR}/temoto_action_engine/rapidjson/include
)

#
# Action Engine node
#
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

#
# UMRF graph publisher node
#
add_executable(umrf_graph_publisher src/umrf_graph_publisher.cpp)
target_include_directories(umrf_graph_publisher PUBLIC
  ${headers})

ament_target_dependencies(umrf_graph_publisher
  rclcpp
  std_msgs
  temoto_action_engine
)

target_link_libraries(umrf_graph_publisher
  temoto_action_engine
)

rosidl_target_interfaces(umrf_graph_publisher
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS umrf_graph_publisher
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
