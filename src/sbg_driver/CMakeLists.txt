# Copyright 2026 sbg_driver maintainers
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

cmake_minimum_required(VERSION 3.22)
project(sbg_driver LANGUAGES CXX VERSION 0.1.0)

# ---- Dependencies -----------------------------------------------------------
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(generate_parameter_library REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(sbg_driver_core REQUIRED)
find_package(sbg_msgs REQUIRED)

# ---- generate_parameter_library --------------------------------------------
# Produces a static lib (sbg_driver_params) and a header (sbg_driver_params.hpp)
# with strongly-typed, validated, dynamically-reconfigurable parameter access.
generate_parameter_library(sbg_driver_params
  params/sbg_driver_params.yaml
)

# ---- Component library ------------------------------------------------------
add_library(sbg_driver_component SHARED
  src/conversions.cpp
  src/publishers.cpp
  src/driver_node.cpp
)
target_compile_features(sbg_driver_component PUBLIC cxx_std_23)

target_include_directories(sbg_driver_component
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# Modern target-based linking — `ament_target_dependencies` was removed
# in Lyrical. Message packages need BOTH the generator (C++ struct headers)
# and the typesupport runtime (publish/subscribe registration) targets.
target_link_libraries(sbg_driver_component
  PUBLIC
    rclcpp::rclcpp
    rclcpp_lifecycle::rclcpp_lifecycle
    rclcpp_components::component
    diagnostic_updater::diagnostic_updater
    tf2::tf2
    tf2_ros::tf2_ros
    sbg_driver_core::sbg_driver_core
    # message packages: generator (headers) + typesupport_cpp (runtime)
    lifecycle_msgs::lifecycle_msgs__rosidl_generator_cpp
    lifecycle_msgs::lifecycle_msgs__rosidl_typesupport_cpp
    sensor_msgs::sensor_msgs__rosidl_generator_cpp
    sensor_msgs::sensor_msgs__rosidl_typesupport_cpp
    geometry_msgs::geometry_msgs__rosidl_generator_cpp
    geometry_msgs::geometry_msgs__rosidl_typesupport_cpp
    nav_msgs::nav_msgs__rosidl_generator_cpp
    nav_msgs::nav_msgs__rosidl_typesupport_cpp
    std_msgs::std_msgs__rosidl_generator_cpp
    std_msgs::std_msgs__rosidl_typesupport_cpp
    std_srvs::std_srvs__rosidl_generator_cpp
    std_srvs::std_srvs__rosidl_typesupport_cpp
    builtin_interfaces::builtin_interfaces__rosidl_generator_cpp
    builtin_interfaces::builtin_interfaces__rosidl_typesupport_cpp
    diagnostic_msgs::diagnostic_msgs__rosidl_generator_cpp
    diagnostic_msgs::diagnostic_msgs__rosidl_typesupport_cpp
)
# generate_parameter_library's target is consumed only inside our .cpp
# files (it's a header-only INTERFACE library wrapping the generated header)
# and shouldn't propagate to downstream consumers, so PRIVATE.
target_link_libraries(sbg_driver_component PRIVATE sbg_driver_params)
# sbg_msgs is in this workspace — its export targets get added once its
# IDL pipeline runs. For phase 2 (no .msg files yet) we just reference
# the share-path artifacts via find_package; full linking happens in phase 4.

target_compile_options(sbg_driver_component PRIVATE
  -Wall -Wextra -Wpedantic
  -Wshadow -Wconversion -Wsign-conversion
  -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
  -Woverloaded-virtual -Wdouble-promotion -Wnull-dereference
  -Wmisleading-indentation -Wformat=2 -Werror=return-type
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(sbg_driver_component PRIVATE -Wduplicated-cond -Wlogical-op)
endif()

# Register as composable component.
rclcpp_components_register_node(sbg_driver_component
  PLUGIN     "sbg_driver::SbgDriverNode"
  EXECUTABLE sbg_driver_node_composed
)

# ---- Standalone executable --------------------------------------------------
add_executable(sbg_driver_node src/main.cpp)
target_link_libraries(sbg_driver_node PRIVATE sbg_driver_component)
target_compile_features(sbg_driver_node PRIVATE cxx_std_23)

# ---- Install ----------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)
install(TARGETS sbg_driver_component
  EXPORT  export_sbg_driver
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS sbg_driver_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch config params
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

ament_export_targets(export_sbg_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(
  rclcpp rclcpp_lifecycle rclcpp_components lifecycle_msgs
  sensor_msgs geometry_msgs nav_msgs std_msgs std_srvs
  diagnostic_msgs diagnostic_updater tf2 tf2_ros
  sbg_driver_core sbg_msgs generate_parameter_library
)

# ---- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_conversions test/conversions_test.cpp)
  target_link_libraries(test_conversions sbg_driver_component)

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/replay_test.launch.py
    TIMEOUT 30
  )
endif()

ament_package()