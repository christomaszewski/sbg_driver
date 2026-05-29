// Copyright 2026 sbg_driver maintainers
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <cstdint>

#include "sbg/error.hpp"

namespace sbg
{

class Device;  // forward decl

// 2D vs 3D mag-cal sample collection. 2D requires the sensor stay roughly
// level while making at least one full 360° heading turn; 3D needs free
// motion in all three orientations but doesn't require a level surface.
enum class MagCalibMode : std::uint8_t
{
  TwoD = 1,
  ThreeD = 2,
};

enum class MagCalibQuality : std::uint8_t
{
  Optimal = 0,
  Good = 1,
  Poor = 2,
  Invalid = 3,
};

enum class MagCalibConfidence : std::uint8_t
{
  High = 0,
  Medium = 1,
  Low = 2,
};

// Summary returned by compute_mag_calibration(). The underlying offset[3] +
// matrix[9] sent to save_mag_calibration_results() is cached inside Device
// so callers don't need to manage opaque float arrays.
struct MagCalibResults
{
  MagCalibQuality quality = MagCalibQuality::Invalid;
  MagCalibConfidence confidence = MagCalibConfidence::Low;
  std::uint16_t advanced_status = 0;
  std::uint16_t num_points = 0;
  std::uint16_t max_num_points = 0;

  // 1σ heading error after the calibration is applied, in radians.
  float mean_accuracy_rad = 0.0F;
  float std_accuracy_rad = 0.0F;
  float max_accuracy_rad = 0.0F;

  // Field-norm error stats (arbitrary units; sensor-specific scale).
  float before_mean_error = 0.0F;
  float before_std_error = 0.0F;
  float before_max_error = 0.0F;
  float after_mean_error = 0.0F;
  float after_std_error = 0.0F;
  float after_max_error = 0.0F;
};

// Configurator — typed wrapper around the sbgECom command set.
//
// Threading: every Configurator method talks to the device via a synchronous
// request/response cycle on the SbgEComHandle. The polling I/O thread
// (Device::run / poll_once) shares that same handle and would steal the
// response frame if it ran concurrently. Configurator methods return
// `Error::DeviceBusy` if invoked while `Device::run()` is active; callers
// MUST stop the I/O thread first (e.g. by joining the std::jthread holding
// it) and may restart it once the command completes.
//
// Returned by value from `Device::configurator()` — internally just a
// pointer back to the Device, so cheap to construct on every call. Move-only
// to avoid accidental copies; users typically use it as a temporary:
//
//     auto cfg = device.configurator();
//     auto r = cfg.start_mag_calibration(sbg::MagCalibMode::ThreeD);
class Configurator
{
public:
  Configurator(Configurator &&) noexcept = default;
  Configurator(const Configurator &) = delete;
  Configurator & operator=(Configurator &&) noexcept = default;
  Configurator & operator=(const Configurator &) = delete;

  // ---- Mag-calibration workflow ------------------------------------------
  //
  //   1. start_mag_calibration(mode)               -- begin sample collection
  //   2. user moves sensor through required pattern (device buffers samples)
  //   3. compute_mag_calibration() → MagCalibResults; inspect quality
  //   4. save_mag_calibration_results()            -- upload offset/matrix
  //   5. save_settings()                            -- persist to NVRAM
  [[nodiscard]] Result<void> start_mag_calibration(MagCalibMode mode);
  [[nodiscard]] Result<MagCalibResults> compute_mag_calibration();
  [[nodiscard]] Result<void> save_mag_calibration_results();

  // ---- Persistence -------------------------------------------------------

  // Writes the device's in-RAM settings to non-volatile memory and reboots.
  // The driver should reconnect after this call (the reboot drops the link).
  [[nodiscard]] Result<void> save_settings();

private:
  friend class Device;
  // Holds a non-owning pointer to the parent Device. Device owns the
  // Configurator inside its pImpl, so this lifetime is naturally bounded.
  Device * device_;
  explicit Configurator(Device & device) noexcept : device_(&device) {}

  // Shared precondition for every command: device open AND I/O thread idle
  // (sbgECom is request/response — a concurrent poll would steal the reply).
  // Defined in device.cpp where Device::Impl is complete; command bodies
  // chain off it via .and_then().
  [[nodiscard]] Result<void> ready() const noexcept;
};

}  // namespace sbg
