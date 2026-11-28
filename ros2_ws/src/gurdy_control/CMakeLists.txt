cmake_minimum_required(VERSION 3.8)
project(gurdy_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

set(DEPS controller_interface hardware_interface pluginlib rclcpp rclcpp_lifecycle std_msgs)
find_package(ament_cmake REQUIRED)
foreach(dep ${DEPS})
  find_package(${dep} REQUIRED)
endforeach()

add_library(gurdy_control SHARED src/joint_pd_controller.cpp src/coupled_gripper_controller.cpp)
ament_target_dependencies(gurdy_control ${DEPS})
pluginlib_export_plugin_description_file(controller_interface gurdy_control_plugins.xml)

install(TARGETS gurdy_control LIBRARY DESTINATION lib)

ament_export_dependencies(${DEPS})
ament_package()
