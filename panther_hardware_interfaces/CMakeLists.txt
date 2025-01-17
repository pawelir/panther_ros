cmake_minimum_required(VERSION 3.10.2)

# Handle superbuild first
option(USE_SUPERBUILD "Whether or not a superbuild should be invoked" ON)

if(USE_SUPERBUILD)
  project(SUPERBUILD NONE)
  include(cmake/SuperBuild.cmake)
  return()
else()
  project(panther_hardware_interfaces)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(PACKAGE_DEPENDENCIES
    ament_cmake
    controller_interface
    diagnostic_updater
    generate_parameter_library
    geometry_msgs
    hardware_interface
    imu_filter_madgwick
    panther_msgs
    panther_utils
    phidgets_api
    PkgConfig
    pluginlib
    rclcpp
    rclcpp_lifecycle
    realtime_tools
    std_srvs
    tf2_geometry_msgs
    tf2_ros)

foreach(PACKAGE IN ITEMS ${PACKAGE_DEPENDENCIES})
  find_package(${PACKAGE} REQUIRED)
endforeach()

include_directories(include)

generate_parameter_library(phidgets_spatial_parameters
                           config/phidgets_spatial_parameters.yaml)

set(ENV{PKG_CONFIG_PATH}
    "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")

pkg_check_modules(LIBLELY_COAPP REQUIRED IMPORTED_TARGET liblely-coapp)
pkg_check_modules(LIBGPIOD REQUIRED IMPORTED_TARGET libgpiodcxx)

add_library(
  ${PROJECT_NAME} SHARED
  src/panther_imu_sensor/panther_imu_sensor.cpp
  src/panther_system/gpio/gpio_controller.cpp
  src/panther_system/gpio/gpio_driver.cpp
  src/panther_system/motors_controller/canopen_controller.cpp
  src/panther_system/motors_controller/motors_controller.cpp
  src/panther_system/motors_controller/roboteq_data_converters.cpp
  src/panther_system/motors_controller/roboteq_driver.cpp
  src/panther_system/motors_controller/roboteq_error_filter.cpp
  src/panther_system/panther_system_e_stop.cpp
  src/panther_system/panther_system_ros_interface.cpp
  src/panther_system/panther_system.cpp
  src/utils.cpp)
ament_target_dependencies(${PROJECT_NAME} ${PACKAGE_DEPENDENCIES})
target_link_libraries(${PROJECT_NAME} PkgConfig::LIBLELY_COAPP
                      PkgConfig::LIBGPIOD phidgets_spatial_parameters)

target_compile_definitions(${PROJECT_NAME}
                           PRIVATE "PANTHER_HARDWARE_INTERFACES_BUILDING_DLL")

pluginlib_export_plugin_description_file(hardware_interface
                                         panther_hardware_interfaces.xml)

install(
  TARGETS ${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(panther_utils REQUIRED)

  install(DIRECTORY test/config DESTINATION share/${PROJECT_NAME}/test)

  ament_add_gtest(${PROJECT_NAME}_test_utils test/test_utils.cpp src/utils.cpp)

  ament_add_gtest(${PROJECT_NAME}_test_panther_imu_sensor
                  test/panther_imu_sensor/test_panther_imu_sensor.cpp)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_imu_sensor hardware_interface rclcpp
    panther_utils panther_msgs phidgets_api)
  target_link_libraries(${PROJECT_NAME}_test_panther_imu_sensor ${PROJECT_NAME}
                        phidgets_spatial_parameters)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_error_filter
    test/panther_system/motors_controller/test_roboteq_error_filter.cpp
    src/panther_system/motors_controller/roboteq_error_filter.cpp)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_data_converters
    test/panther_system/motors_controller/test_roboteq_data_converters.cpp
    src/panther_system/motors_controller/roboteq_data_converters.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_data_converters
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_data_converters
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_data_converters
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_canopen_controller
    test/panther_system/motors_controller/test_canopen_controller.cpp
    src/panther_system/motors_controller/canopen_controller.cpp
    src/panther_system/motors_controller/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_canopen_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_canopen_controller rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_canopen_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_roboteq_driver
    test/panther_system/motors_controller/test_roboteq_driver.cpp
    src/panther_system/motors_controller/canopen_controller.cpp
    src/panther_system/motors_controller/roboteq_driver.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_roboteq_driver
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_roboteq_driver rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_roboteq_driver
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(
    ${PROJECT_NAME}_test_motors_controller
    test/panther_system/motors_controller/test_motors_controller.cpp
    src/panther_system/motors_controller/canopen_controller.cpp
    src/panther_system/motors_controller/roboteq_driver.cpp
    src/panther_system/motors_controller/roboteq_data_converters.cpp
    src/panther_system/motors_controller/motors_controller.cpp
    src/utils.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_motors_controller
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(${PROJECT_NAME}_test_motors_controller rclcpp
                            panther_msgs panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_motors_controller
                        PkgConfig::LIBLELY_COAPP)

  ament_add_gtest(${PROJECT_NAME}_test_gpiod_driver
                  test/panther_system/gpio/test_gpio_driver.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_gpiod_driver panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_gpiod_driver ${PROJECT_NAME}
                        PkgConfig::LIBGPIOD)

  ament_add_gtest(
    ${PROJECT_NAME}_test_gpiod_controller
    test/panther_system/gpio/test_gpio_controller.cpp
    src/panther_system/gpio/gpio_controller.cpp
    src/panther_system/gpio/gpio_driver.cpp)
  ament_target_dependencies(${PROJECT_NAME}_test_gpiod_controller panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_gpiod_controller
                        PkgConfig::LIBGPIOD)

  ament_add_gtest(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    test/panther_system/test_panther_system_ros_interface.cpp
    src/panther_system/panther_system_ros_interface.cpp
    src/panther_system/motors_controller/roboteq_data_converters.cpp
    src/utils.cpp
    src/panther_system/gpio/gpio_controller.cpp
    src/panther_system/gpio/gpio_driver.cpp)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system_ros_interface
    diagnostic_updater
    rclcpp
    panther_msgs
    panther_utils
    realtime_tools
    std_srvs)
  target_link_libraries(${PROJECT_NAME}_test_panther_system_ros_interface
                        PkgConfig::LIBLELY_COAPP PkgConfig::LIBGPIOD)

  ament_add_gtest(${PROJECT_NAME}_test_panther_system
                  test/panther_system/test_panther_system.cpp)
  set_tests_properties(${PROJECT_NAME}_test_panther_system PROPERTIES TIMEOUT
                                                                      120)
  target_include_directories(
    ${PROJECT_NAME}_test_panther_system
    PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test>
           $<INSTALL_INTERFACE:include>)
  ament_target_dependencies(
    ${PROJECT_NAME}_test_panther_system hardware_interface rclcpp panther_msgs
    panther_utils)
  target_link_libraries(${PROJECT_NAME}_test_panther_system
                        PkgConfig::LIBLELY_COAPP)

endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(${PACKAGE_DEPENDENCIES})

ament_package()
