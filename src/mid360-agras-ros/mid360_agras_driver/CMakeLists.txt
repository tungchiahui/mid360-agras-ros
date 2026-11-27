cmake_minimum_required(VERSION 3.8)
project(mid360_agras_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(MID360_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(MID360_COMMON_DIR "${MID360_PACKAGE_DIR}/../common" ABSOLUTE)
if(NOT EXISTS "${MID360_COMMON_DIR}/src/protocol.cpp")
  message(FATAL_ERROR "找不到公共协议源码: ${MID360_COMMON_DIR}")
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES std_msgs
)

add_executable(${PROJECT_NAME}_node
  src/driver_node.cpp
  ${MID360_COMMON_DIR}/src/protocol.cpp
  ${MID360_COMMON_DIR}/src/device.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${MID360_COMMON_DIR}/include
)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  rcl_interfaces
  diagnostic_msgs
  sensor_msgs
  std_msgs
)

if(COMMAND rosidl_get_typesupport_target)
  rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
  target_link_libraries(${PROJECT_NAME}_node "${cpp_typesupport_target}")
else()
  rosidl_target_interfaces(${PROJECT_NAME}_node ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wno-unused-parameter)
target_link_libraries(${PROJECT_NAME}_node pthread)

install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
