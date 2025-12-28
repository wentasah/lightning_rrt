cmake_minimum_required(VERSION 3.8)
project(lightning_rrt)

if(NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(lightning_rrt_interfaces REQUIRED)

add_executable(lightning_rrt src/lightning_rrt.cpp)
add_executable(example_client src/example_client.cpp)

target_link_libraries(
  lightning_rrt PUBLIC rclcpp::rclcpp 
  ${std_srvs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${lightning_rrt_interfaces_TARGETS})

target_link_libraries(
  example_client PUBLIC rclcpp::rclcpp 
  ${std_srvs_TARGETS}
  ${std_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${lightning_rrt_interfaces_TARGETS})

install(TARGETS
  lightning_rrt
  example_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
