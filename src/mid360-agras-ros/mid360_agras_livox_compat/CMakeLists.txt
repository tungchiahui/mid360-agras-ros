cmake_minimum_required(VERSION 3.16)
project(mid360_agras_livox_compat)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(mid360_agras_driver REQUIRED)
find_package(livox_ros_driver2 REQUIRED)

# Link the generated messages without linking the Livox driver or its SDK.
if(TARGET livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp)
  set(livox_typesupport livox_ros_driver2::livox_interfaces2__rosidl_typesupport_cpp)
elseif(TARGET livox_ros_driver2::livox_ros_driver2__rosidl_typesupport_cpp)
  set(livox_typesupport livox_ros_driver2::livox_ros_driver2__rosidl_typesupport_cpp)
else()
  message(FATAL_ERROR "livox_ros_driver2 must export its ROS2 C++ message type support")
endif()

add_executable(custom_msg_bridge src/custom_msg_bridge.cpp)
target_compile_features(custom_msg_bridge PRIVATE cxx_std_17)
target_compile_options(custom_msg_bridge PRIVATE -Wall -Wextra)
ament_target_dependencies(custom_msg_bridge rclcpp)
target_link_libraries(custom_msg_bridge
  mid360_agras_driver::mid360_agras_driver__rosidl_typesupport_cpp
  ${livox_typesupport}
)
install(TARGETS custom_msg_bridge DESTINATION lib/${PROJECT_NAME})
ament_package()
