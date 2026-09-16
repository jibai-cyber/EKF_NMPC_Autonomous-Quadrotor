cmake_minimum_required(VERSION 3.8)
project(drone_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(drone_interfaces REQUIRED)
find_package(gz_msgs_vendor REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz_transport_vendor REQUIRED)
find_package(gz-msgs10 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(gz-sim8 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(rclcpp REQUIRED)

add_library(rotor_wrench_system SHARED
  src/rotor_wrench_system.cpp
)
target_include_directories(rotor_wrench_system PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rotor_wrench_system
  gz-msgs10::gz-msgs10
  gz-plugin2::gz-plugin2
  gz-sim8::gz-sim8
  gz-transport13::core
)

add_executable(actuator_bridge src/actuator_bridge.cpp)
ament_target_dependencies(actuator_bridge drone_interfaces rclcpp)
target_link_libraries(actuator_bridge gz-msgs10::gz-msgs10 gz-transport13::core)

add_executable(trajectory_visualizer src/trajectory_visualizer.cpp)
ament_target_dependencies(trajectory_visualizer drone_interfaces rclcpp)
target_link_libraries(trajectory_visualizer gz-msgs10::gz-msgs10 gz-transport13::core)

install(TARGETS rotor_wrench_system actuator_bridge trajectory_visualizer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
