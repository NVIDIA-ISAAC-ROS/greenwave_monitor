# Copyright (c) 2025, NVIDIA CORPORATION & AFFILIATES. All rights reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.22.1)
project(greenwave_monitor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Add message_diagnostics.hpp as a header-only library
add_library(message_diagnostics INTERFACE)
target_include_directories(message_diagnostics INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(greenwave_monitor src/greenwave_monitor.cpp src/greenwave_monitor_main.cpp)
ament_target_dependencies(greenwave_monitor
  rclcpp
  std_msgs
  diagnostic_msgs
  greenwave_monitor_interfaces
)
target_link_libraries(greenwave_monitor message_diagnostics)

target_include_directories(greenwave_monitor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_compile_features(greenwave_monitor PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

install(TARGETS greenwave_monitor
  DESTINATION lib/${PROJECT_NAME})

add_executable(minimal_publisher_node
  src/minimal_publisher_node.cpp
  src/minimal_publisher_main.cpp)
ament_target_dependencies(minimal_publisher_node rclcpp std_msgs sensor_msgs diagnostic_msgs)
target_link_libraries(minimal_publisher_node message_diagnostics)
target_include_directories(minimal_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(minimal_publisher_node PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17
install(TARGETS minimal_publisher_node
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch examples
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(
  PROGRAMS
    scripts/ncurses_dashboard
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  # Add launch tests for greenwave_monitor
  ament_add_pytest_test(test_greenwave_monitor test/test_greenwave_monitor.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  # Add explicit linting tests
  ament_add_pytest_test(test_flake8 test/test_flake8.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  ament_add_pytest_test(test_pep257 test/test_pep257.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  ament_add_pytest_test(test_copyright test/test_copyright.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  # Add topic monitoring integration tests
  ament_add_pytest_test(test_topic_monitoring_integration test/test_topic_monitoring_integration.py
    TIMEOUT 120
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )

  # Add gtests
  ament_add_gtest(test_message_diagnostics test/test_message_diagnostics.cpp
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  ament_target_dependencies(test_message_diagnostics
    rclcpp
    std_msgs
    diagnostic_msgs
  )
  target_link_libraries(test_message_diagnostics message_diagnostics)
  target_include_directories(test_message_diagnostics PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(test_message_diagnostics PUBLIC c_std_99 cxx_std_17)

  ament_add_gtest(test_minimal_publisher
    test/test_minimal_publisher.cpp
    src/minimal_publisher_node.cpp
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  )
  ament_target_dependencies(test_minimal_publisher
    rclcpp
    sensor_msgs
    diagnostic_msgs
  )
  target_link_libraries(test_minimal_publisher message_diagnostics)
  target_include_directories(test_minimal_publisher PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  target_compile_features(test_minimal_publisher PUBLIC c_std_99 cxx_std_17)

endif()

ament_package()
