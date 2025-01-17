cmake_minimum_required(VERSION 3.10.2)
project(panther_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    hardware_interface
    ignition-gazebo6
    ignition-msgs8
    ignition-plugin1
    ignition-transport11
    ign_ros2_control
    pluginlib
    rclcpp
    realtime_tools
    std_msgs
    std_srvs)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

find_package(Qt5 REQUIRED COMPONENTS Core Quick QuickControls2)

include_directories(include ${Qt5Core_INCLUDE_DIRS} ${Qt5Qml_INCLUDE_DIRS}
                    ${Qt5Quick_INCLUDE_DIRS} ${Qt5QuickControls2_INCLUDE_DIRS})

add_library(panther_simulation_plugins SHARED src/gz_panther_system.cpp)
ament_target_dependencies(
  panther_simulation_plugins
  hardware_interface
  ign_ros2_control
  rclcpp
  rclcpp_lifecycle
  std_msgs
  std_srvs)
target_link_libraries(panther_simulation_plugins ignition-gazebo6)

set(CMAKE_AUTOMOC ON)
qt5_add_resources(resources_rcc src/gui/EStop.qrc)

add_library(EStop SHARED include/panther_gazebo/gui/e_stop.hpp
                         src/gui/e_stop.cpp ${resources_rcc})
ament_target_dependencies(EStop rclcpp std_srvs)
target_link_libraries(
  EStop
  ignition-gazebo6
  ignition-plugin1
  ${Qt5Core_LIBRARIES}
  ${Qt5Qml_LIBRARIES}
  ${Qt5Quick_LIBRARIES}
  ${Qt5QuickControls2_LIBRARIES})

add_library(LEDStrip SHARED src/led_strip.cpp)
ament_target_dependencies(LEDStrip realtime_tools)
target_link_libraries(LEDStrip ignition-gazebo6 ignition-msgs8 ignition-plugin1
                      ignition-transport11)

install(
  TARGETS panther_simulation_plugins
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  TARGETS EStop
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(
  TARGETS LEDStrip
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)

pluginlib_export_plugin_description_file(ign_ros2_control
                                         panther_simulation_plugins.xml)

ament_environment_hooks(
  "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")

ament_package()
