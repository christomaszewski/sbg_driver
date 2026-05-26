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
project(sbg_msgs LANGUAGES C CXX)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

# Message files are added in phase 4. The package is intentionally a no-op
# until then so the workspace builds in phase 0.
set(SBG_MSG_FILES
  # msg/Status.msg
  # msg/EkfStatus.msg
  # msg/GpsRaw.msg
  # msg/MagCalib.msg
  # msg/ShipMotion.msg
  # msg/Event.msg
  # msg/AirDataStatus.msg
)

if(SBG_MSG_FILES)
  rosidl_generate_interfaces(${PROJECT_NAME}
    ${SBG_MSG_FILES}
    DEPENDENCIES std_msgs builtin_interfaces
  )
  ament_export_dependencies(rosidl_default_runtime)
endif()

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()