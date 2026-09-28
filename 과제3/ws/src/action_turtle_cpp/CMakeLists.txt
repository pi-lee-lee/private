cmake_minimum_required(VERSION 3.8)
project(action_turtle_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(action_turtle REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(std_srvs REQUIRED)



add_library(
  action_server SHARED
  src/action_turtle_server.cpp)


add_library(
  action_bridge SHARED
  src/action_turtle_bridge.cpp)

target_link_libraries(action_bridge nlohmann_json::nlohmann_json)


target_include_directories(action_server PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

  target_include_directories(action_bridge PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)


ament_target_dependencies( action_server
  action_turtle
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs 
  turtlesim
  )
  ament_target_dependencies( action_bridge
  action_turtle
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs 
  turtlesim
  )

rclcpp_components_register_node(
  action_server 
  PLUGIN "action_turtle_cpp::ActionTurtleServer" 
  EXECUTABLE action_turtle_server) #노드등록 

rclcpp_components_register_node(
  action_bridge 
  PLUGIN "action_turtle_cpp::ActionTurtleBridge" 
  EXECUTABLE action_turtle_bridge) #노드등록 

# 브리지 2: 액션(이동·회전) + 서비스(reset) + 토픽 구독(pose)
add_library(
  action_bridge_2 SHARED
  src/action_turtle_bridge_2.cpp)

ament_target_dependencies(action_bridge_2
  action_turtle
  rclcpp
  rclcpp_action
  rclcpp_components
  turtlesim
  std_srvs
  )
target_link_libraries(action_bridge_2 nlohmann_json::nlohmann_json)

rclcpp_components_register_node(
  action_bridge_2
  PLUGIN "action_turtle_cpp::ActionTurtleBridge2"
  EXECUTABLE action_turtle_bridge_2)

install(TARGETS
  action_server
  action_bridge
  action_bridge_2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
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
