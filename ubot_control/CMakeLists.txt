cmake_minimum_required(VERSION 3.8)
project(ubot_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBSERIAL REQUIRED libserial)

add_library(ubot_control SHARED
  src/ubot_hardware_interface.cpp
)

target_include_directories(ubot_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${LIBSERIAL_INCLUDE_DIRS}
)

target_link_libraries(ubot_control
  ${LIBSERIAL_LIBRARIES}
)

ament_target_dependencies(ubot_control
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

pluginlib_export_plugin_description_file(
  hardware_interface
  ubot_control.xml
)

install(
  TARGETS ubot_control
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES ubot_control.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
