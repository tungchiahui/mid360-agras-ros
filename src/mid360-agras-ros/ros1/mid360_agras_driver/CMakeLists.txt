cmake_minimum_required(VERSION 3.0.2)
project(mid360_agras_driver)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

get_filename_component(MID360_PACKAGE_DIR "${CMAKE_CURRENT_SOURCE_DIR}" REALPATH)
get_filename_component(MID360_COMMON_DIR "${MID360_PACKAGE_DIR}/../../common" ABSOLUTE)
if(NOT EXISTS "${MID360_COMMON_DIR}/src/protocol.cpp")
  message(FATAL_ERROR "找不到公共协议源码: ${MID360_COMMON_DIR}")
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)

add_message_files(FILES
  CustomPoint.msg
  CustomMsg.msg
)

generate_messages(DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs std_msgs message_runtime
)

add_executable(${PROJECT_NAME}_node
  src/driver_node.cpp
  ${MID360_COMMON_DIR}/src/protocol.cpp
  ${MID360_COMMON_DIR}/src/device.cpp
)

add_dependencies(${PROJECT_NAME}_node ${PROJECT_NAME}_generate_messages_cpp)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${MID360_COMMON_DIR}/include
  ${catkin_INCLUDE_DIRS}
)

target_compile_options(${PROJECT_NAME}_node PRIVATE -Wall -Wextra -Wno-unused-parameter)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  pthread
)

install(TARGETS ${PROJECT_NAME}_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
