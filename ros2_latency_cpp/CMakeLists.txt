cmake_minimum_required(VERSION 3.10)

project(ros2_latency_cpp)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros2_latency_interfaces REQUIRED)

add_library(${PROJECT_NAME}
  "src/${PROJECT_NAME}/byte_publisher.cpp"
  "src/${PROJECT_NAME}/subscriber.cpp"
)

target_include_directories(${PROJECT_NAME} PUBLIC "include")

ament_target_dependencies(${PROJECT_NAME} rclcpp ros2_latency_interfaces)

add_executable(byte_publisher "src/byte_publisher_main.cpp")
target_link_libraries(byte_publisher ${PROJECT_NAME})

add_executable(subscriber "src/subscriber_main.cpp")
target_link_libraries(subscriber ${PROJECT_NAME})

install(DIRECTORY "include" DESTINATION ".")
install(TARGETS ${PROJECT_NAME} DESTINATION "lib")
install(TARGETS byte_publisher subscriber DESTINATION "lib/${PROJECT_NAME}")

ament_export_include_directories("include")
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp ros2_latency_interfaces)
ament_package()
