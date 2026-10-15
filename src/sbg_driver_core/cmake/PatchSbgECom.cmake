# Copyright 2026 sbg_driver maintainers
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
# http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# sbgECom 5.8.935-stable: empty commands such as GetInfo pass nullptr/0 to
# sbgStreamBufferWriteBuffer. Its assertion allows this, but memcpy does not.
# Keep the guard in the inline SDK function so both SDK and consumer code use
# the fix. Reconfiguration is idempotent; unexpected upstream changes fail
# loudly instead of silently losing the sanitizer fix.
set(_sbg_buffer_header "${sbgecom_SOURCE_DIR}/common/streamBuffer/sbgStreamBufferCommon.h")
file(READ "${_sbg_buffer_header}" _sbg_buffer_source)
set(_sbg_empty_copy "memcpy(pHandle->pCurrentPtr, pBuffer, numBytesToWrite);")
set(_sbg_guarded_copy "if (numBytesToWrite > 0)\n            {\n                ${_sbg_empty_copy}\n            }")
string(FIND "${_sbg_buffer_source}" "${_sbg_guarded_copy}" _sbg_already_patched)
if(_sbg_already_patched EQUAL -1)
  string(FIND "${_sbg_buffer_source}" "${_sbg_empty_copy}" _sbg_patch_location)
  if(_sbg_patch_location EQUAL -1)
    message(FATAL_ERROR "sbgECom stream buffer changed; review PatchSbgECom.cmake for this SDK version")
  endif()
  string(REPLACE "${_sbg_empty_copy}" "${_sbg_guarded_copy}"
    _sbg_buffer_source "${_sbg_buffer_source}")
  file(WRITE "${_sbg_buffer_header}" "${_sbg_buffer_source}")
  message(STATUS "sbg_driver_core: patched sbgECom empty-payload memcpy")
endif()
