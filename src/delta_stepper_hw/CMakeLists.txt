cmake_minimum_required(VERSION 3.8)
project(delta_stepper_hw)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# -----------------------------
# Dependencies
# -----------------------------

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)


include_directories(include)

add_library(

 ${PROJECT_NAME}
 SHARED
 src/delta_stepper_hw.cpp
)


ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib

)


pluginlib_export_plugin_description_file(  
  hardware_interface
  delta_stepper_hw.xml
)


install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)


install(
  DIRECTORY include/
  DESTINATION include
)


install(
  FILES delta_stepper_hw.xml
  DESTINATION share/${PROJECT_NAME}
)


target_link_libraries(${PROJECT_NAME} serial)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pluginlib
)

ament_package()
