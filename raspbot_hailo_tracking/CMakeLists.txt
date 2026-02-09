cmake_minimum_required(VERSION 3.5)
project(raspbot_hailo_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

find_package(OpenCV REQUIRED)
find_package(HailoRT REQUIRED)

add_executable(hailo_detector src/hailo_detector_node.cpp)
ament_target_dependencies(hailo_detector rclcpp sensor_msgs geometry_msgs std_msgs)
target_include_directories(hailo_detector PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(hailo_detector ${OpenCV_LIBS} HailoRT::libhailort)

install(TARGETS hailo_detector
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/download_hailo_model
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
