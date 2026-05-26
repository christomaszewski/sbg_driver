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
project(sbg_driver_core LANGUAGES C CXX VERSION 0.1.0)

find_package(ament_cmake REQUIRED)

# ---- Options ----------------------------------------------------------------
option(SBG_ENABLE_ASAN     "Enable AddressSanitizer + UndefinedBehaviorSanitizer" OFF)
option(SBG_ENABLE_TSAN     "Enable ThreadSanitizer" OFF)
option(SBG_ENABLE_COVERAGE "Enable code coverage instrumentation"               OFF)

# ---- sbgECom (vendored via FetchContent) ------------------------------------
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/FetchSbgECom.cmake)

# ---- Library target ---------------------------------------------------------
add_library(sbg_driver_core
  src/error.cpp
  src/transport.cpp
  src/log_view.cpp
  src/device.cpp
)
add_library(sbg::driver_core ALIAS sbg_driver_core)

target_compile_features(sbg_driver_core PUBLIC cxx_std_23)

# Static library, but PIC so it can be linked into downstream SHARED targets
# (the sbg_driver composable component is a .so).
set_target_properties(sbg_driver_core PROPERTIES POSITION_INDEPENDENT_CODE ON)

target_include_directories(sbg_driver_core
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# sbgECom is PRIVATE: its types appear in our public headers (log_view.hpp)
# through the detail/c_api.hpp facade, which propagates the include dirs
# transitively. We rely on INTERFACE include dirs from sbgECom's CMake target.
target_link_libraries(sbg_driver_core PUBLIC sbgECom)

# ---- Strict warnings (on OUR code only, not sbgECom) ------------------------
target_compile_options(sbg_driver_core PRIVATE
  -Wall -Wextra -Wpedantic
  -Wshadow -Wconversion -Wsign-conversion
  -Wnon-virtual-dtor -Wold-style-cast -Wcast-align
  -Woverloaded-virtual -Wdouble-promotion -Wnull-dereference
  -Wmisleading-indentation -Wformat=2 -Werror=return-type
)
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  target_compile_options(sbg_driver_core PRIVATE -Wduplicated-cond -Wlogical-op)
endif()

# ---- Sanitizers / coverage --------------------------------------------------
if(SBG_ENABLE_ASAN)
  target_compile_options(sbg_driver_core PRIVATE
    -fsanitize=address,undefined -fno-omit-frame-pointer)
  target_link_options(sbg_driver_core PUBLIC -fsanitize=address,undefined)
endif()
if(SBG_ENABLE_TSAN)
  target_compile_options(sbg_driver_core PRIVATE
    -fsanitize=thread -fno-omit-frame-pointer)
  target_link_options(sbg_driver_core PUBLIC -fsanitize=thread)
endif()
if(SBG_ENABLE_COVERAGE)
  target_compile_options(sbg_driver_core PRIVATE --coverage -O0 -g)
  target_link_options(sbg_driver_core PUBLIC --coverage)
endif()

# ---- Install ----------------------------------------------------------------
install(DIRECTORY include/ DESTINATION include)

# Install sbgECom's headers under include/sbgECom so its INSTALL_INTERFACE
# (set in FetchSbgECom.cmake) resolves. Two source roots: src/ and common/.
install(DIRECTORY
    ${sbgecom_SOURCE_DIR}/src/
    ${sbgecom_SOURCE_DIR}/common/
  DESTINATION include/sbgECom
  FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp")

# Install BOTH targets in the same export set so downstream find_package
# resolves sbgECom alongside sbg_driver_core.
install(TARGETS sbg_driver_core sbgECom
  EXPORT export_sbg_driver_core
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

ament_export_targets(export_sbg_driver_core HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

# ---- Tests ------------------------------------------------------------------
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_error_mapping test/error_mapping_test.cpp)
  target_link_libraries(test_error_mapping sbg_driver_core)

  ament_add_gtest(test_transport_config test/transport_config_test.cpp)
  target_link_libraries(test_transport_config sbg_driver_core)

  ament_add_gtest(test_log_view test/log_view_test.cpp)
  target_link_libraries(test_log_view sbg_driver_core)

  ament_add_gtest(test_device_smoke test/device_smoke_test.cpp)
  target_link_libraries(test_device_smoke sbg_driver_core)
endif()

ament_package()