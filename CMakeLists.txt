cmake_minimum_required(VERSION 3.8)
project(amr_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# 編譯 C++ 避障節點
add_executable(obstacle_avoidance src/obstacle_avoidance.cpp)
ament_target_dependencies(obstacle_avoidance rclcpp geometry_msgs sensor_msgs)

install(TARGETS
  obstacle_avoidance
  DESTINATION lib/${PROJECT_NAME})

# 安裝 Launch 檔和 URDF
install(DIRECTORY
  launch 
  urdf
  worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()