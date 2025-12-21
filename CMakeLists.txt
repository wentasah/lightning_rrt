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

add_executable(lightning_rrt src/lightning_rrt.cpp)

target_link_libraries(lightning_rrt PUBLIC rclcpp::rclcpp ${std_srvs_TARGETS})
target_link_libraries(lightning_rrt PUBLIC rclcpp::rclcpp ${std_msgs_TARGETS})
target_link_libraries(lightning_rrt PUBLIC rclcpp::rclcpp ${geometry_msgs_TARGETS})
target_link_libraries(lightning_rrt PUBLIC rclcpp::rclcpp ${nav_msgs_TARGETS})

install(TARGETS
  lightning_rrt
  DESTINATION lib/${PROJECT_NAME})

ament_package()
