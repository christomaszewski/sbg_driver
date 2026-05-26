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

# FetchSbgECom.cmake — bring in the SBG Systems sbgECom C SDK
#
# Two paths:
#   1. SBG_DRIVER_USE_SYSTEM_SBGECOM=ON: find an installed sbgECom via find_package
#      (for distro packagers shipping a system sbgECom-dev package).
#   2. Default: FetchContent the pinned tag from GitHub.
#
# Either way, the result is a CMake target `sbgECom` that exports its includes
# and is built with the SDK's own warning flags (NOT our strict project flags).

include_guard(GLOBAL)

option(SBG_DRIVER_USE_SYSTEM_SBGECOM
  "Use system-installed sbgECom (find_package) instead of FetchContent" OFF)

set(SBG_DRIVER_SBGECOM_GIT_TAG "5.6.2730-stable"
  CACHE STRING "sbgECom tag to fetch when not using system install")

if(SBG_DRIVER_USE_SYSTEM_SBGECOM)
  find_package(sbgECom CONFIG REQUIRED)
  message(STATUS "sbg_driver_core: using system-installed sbgECom")
  return()
endif()

include(FetchContent)
fetchcontent_declare(sbgECom
  GIT_REPOSITORY https://github.com/SBG-Systems/sbgECom.git
  GIT_TAG        ${SBG_DRIVER_SBGECOM_GIT_TAG}
  GIT_SHALLOW    TRUE
  EXCLUDE_FROM_ALL
)

# Disable the parts we don't need before fetchcontent_makeavailable.
set(BUILD_TOOLS    OFF CACHE BOOL "" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
set(BUILD_TESTS    OFF CACHE BOOL "" FORCE)

fetchcontent_makeavailable(sbgECom)

# sbgECom's CMake target should already be created by the subdirectory.
# Defensive: confirm it exists and silence its warnings on our build.
if(NOT TARGET sbgECom)
  message(FATAL_ERROR "sbgECom CMake target not found after FetchContent — "
                      "upstream layout may have changed, see "
                      "https://github.com/SBG-Systems/sbgECom")
endif()

# Suppress strict warnings on the vendored C SDK — we don't own it and our
# project-wide -Werror=return-type can trip on its code.
target_compile_options(sbgECom PRIVATE
  -w  # disable all warnings for sbgECom only
)

# Mark sbgECom's include directories as SYSTEM for consumers. This stops
# our strict -Wconversion / -Wsign-conversion from flagging the SDK's
# inline / templated header code when it's instantiated in our TUs.
# Pre-CMake 3.25 idiom — CMake 3.25+ supports `set_target_properties(... SYSTEM TRUE)`.
get_target_property(_sbgecom_iface_includes sbgECom INTERFACE_INCLUDE_DIRECTORIES)
if(_sbgecom_iface_includes)
  set_target_properties(sbgECom PROPERTIES
    INTERFACE_SYSTEM_INCLUDE_DIRECTORIES "${_sbgecom_iface_includes}")
endif()

# Add an install-tree interface include so the exported sbgECom target is
# resolvable from downstream packages after install. The headers themselves
# are installed via the install() call in sbg_driver_core/CMakeLists.txt.
target_include_directories(sbgECom INTERFACE
  $<INSTALL_INTERFACE:include/sbgECom>)

message(STATUS "sbg_driver_core: using sbgECom ${SBG_DRIVER_SBGECOM_GIT_TAG} via FetchContent")
message(STATUS "  source: ${sbgecom_SOURCE_DIR}")