cmake_minimum_required(VERSION 3.16)
project(cm530_hardware LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CM530_SANITIZE "" CACHE STRING "Sanitizers for core and tests, e.g. address,undefined or thread")

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CM530_WARNINGS -Wall -Wextra -Wpedantic -Wconversion -Werror)
if(CM530_SANITIZE)
  add_compile_options(-fsanitize=${CM530_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${CM530_SANITIZE})
endif()

add_library(cm530_core STATIC
  src/calibration.cpp
  src/link.cpp
  src/posix_serial.cpp
  src/protocol.cpp
  src/scheduler.cpp)
target_compile_options(cm530_core PRIVATE ${CM530_WARNINGS})
target_include_directories(cm530_core PUBLIC include)
target_link_libraries(cm530_core PUBLIC yaml-cpp::yaml-cpp)

add_library(cm530_hardware SHARED src/system_interface.cpp)
target_compile_options(cm530_hardware PRIVATE ${CM530_WARNINGS})
target_link_libraries(cm530_hardware PRIVATE
  cm530_core
  hardware_interface::hardware_interface
  pluginlib::pluginlib
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle)
pluginlib_export_plugin_description_file(hardware_interface cm530_hardware_plugin.xml)

install(TARGETS cm530_hardware LIBRARY DESTINATION lib)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

# The mock compiles the unmodified v17 firmware, so tests exercise the real parser.
set(CM530_FIRMWARE "${CMAKE_CURRENT_SOURCE_DIR}/../../third_party/CM530_ROS_BRIDGE/17 ROS to CM530 ver. dual arm")
if(EXISTS "${CM530_FIRMWARE}/APP/src/bridge.c")
  set(CM530_FIRMWARE_SOURCES
    "${CM530_FIRMWARE}/APP/src/bridge.c"
    "${CM530_FIRMWARE}/APP/src/ax12.c"
    "${CM530_FIRMWARE}/APP/src/dynamixel.c")
  set_source_files_properties(${CM530_FIRMWARE_SOURCES} PROPERTIES COMPILE_OPTIONS "-std=gnu89;-w")
  add_library(cm530_firmware_sim STATIC test/firmware_sim.cpp ${CM530_FIRMWARE_SOURCES})
  target_include_directories(cm530_firmware_sim PUBLIC test "${CM530_FIRMWARE}/APP/inc")

  add_executable(cm530_mock_serial test/mock_serial_main.cpp)
  target_compile_options(cm530_mock_serial PRIVATE ${CM530_WARNINGS})
  target_link_libraries(cm530_mock_serial PRIVATE cm530_firmware_sim)
  install(TARGETS cm530_mock_serial RUNTIME DESTINATION lib/${PROJECT_NAME})
else()
  message(WARNING "CM-530 firmware sources not found; mock serial and firmware-loop tests are skipped")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(cm530_protocol_test test/test_protocol.cpp)
  target_link_libraries(cm530_protocol_test cm530_core)
  ament_add_gtest(cm530_calibration_test test/test_calibration.cpp
    ENV CM530_TEMPLATE=${CMAKE_CURRENT_SOURCE_DIR}/config/calibration.yaml)
  target_link_libraries(cm530_calibration_test cm530_core)
  if(TARGET cm530_firmware_sim)
    ament_add_gtest(cm530_link_test test/test_link.cpp)
    target_link_libraries(cm530_link_test cm530_core cm530_firmware_sim)
    ament_add_gtest(cm530_scheduler_test test/test_scheduler.cpp)
    target_link_libraries(cm530_scheduler_test cm530_core cm530_firmware_sim)
  endif()
endif()

ament_package()
