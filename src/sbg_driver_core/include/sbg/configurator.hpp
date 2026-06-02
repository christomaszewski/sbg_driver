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

#include <array>
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

// ---- Device provisioning types (Phase 3h-2) -------------------------------
//
// Strongly-typed mirrors of the sbgECom configuration enums/structs, so the
// public core API never leaks the C SDK. Translated to the SDK types in
// device.cpp. Used by Configurator's set_* methods below.

// Vehicle motion dynamics model — tunes the EKF for the platform.
enum class MotionProfile : std::uint8_t
{
  GeneralPurpose = 1,
  Automotive = 2,
  Marine = 3,
  Airplane = 4,
  Helicopter = 5,
  Pedestrian = 6,
  UavRotaryWing = 7,
  HeavyMachinery = 8,
  Static = 9,
  Truck = 10,
  Railway = 11,
  OffRoadVehicle = 12,
  Underwater = 13,
};

// Physical port a module's data arrives on (or Internal / Disabled).
enum class PortAssignment : std::uint8_t
{
  PortA = 0,
  PortB = 1,
  PortC = 2,
  PortD = 3,
  PortE = 4,
  Internal = 5,
  Disabled = 0xFF,
};

// Which vehicle direction a sensor axis points along (coarse alignment).
enum class AxisDirection : std::uint8_t
{
  Forward = 0,
  Backward = 1,
  Left = 2,
  Right = 3,
  Up = 4,
  Down = 5,
};

enum class MagModel : std::uint8_t
{
  InternalNormal,  // SBG_ECOM_MAG_MODEL_INTERNAL_NORMAL
  ExternalEcom,    // SBG_ECOM_MAG_MODEL_ECOM_NORMAL
};

enum class GnssInstallationMode : std::uint8_t
{
  Single = 1,       // single-antenna; secondary lever arm unused
  DualAuto = 2,     // dual antenna, secondary lever arm auto-estimated
  DualRough = 3,    // dual antenna, approximate secondary lever arm
  DualPrecise = 4,  // dual antenna, precisely-known secondary lever arm
};

// IMU↔vehicle mounting: coarse axis directions + fine misalignment angles
// (rad) + lever arm (IMU→vehicle reference point, metres, IMU X/Y/Z).
struct ImuAlignment
{
  AxisDirection axis_x = AxisDirection::Forward;
  AxisDirection axis_y = AxisDirection::Right;
  float mis_roll = 0.0F;
  float mis_pitch = 0.0F;
  float mis_yaw = 0.0F;
  std::array<float, 3> lever_arm{0.0F, 0.0F, 0.0F};
};

// Aiding-input port assignment. Only these modules are exposed; sync-signal
// and odometer-pin assignments are preserved (set_aiding_assignment does a
// get-modify-set so it never clobbers fields the caller didn't specify).
struct AidingAssignment
{
  PortAssignment gps1_port = PortAssignment::Internal;
  PortAssignment rtcm_port = PortAssignment::Disabled;
  PortAssignment dvl_port = PortAssignment::Disabled;
  PortAssignment air_data_port = PortAssignment::Disabled;
};

// GNSS antenna lever arms (IMU→antenna, metres, IMU X/Y/Z) + dual-antenna mode.
struct GnssInstallation
{
  std::array<float, 3> lever_arm_primary{0.0F, 0.0F, 0.0F};
  std::array<float, 3> lever_arm_secondary{0.0F, 0.0F, 0.0F};
  GnssInstallationMode secondary_mode = GnssInstallationMode::Single;
};

// ---- Output-log rate configuration (Phase 3h-3) ---------------------------

// SBG output logs whose emission cadence can be configured. Each maps to an
// (SBG_ECOM_CLASS_LOG_ECOM_0, SBG_ECOM_LOG_*) pair in device.cpp; the C ids
// never appear in this header. GNSS entries are the primary (GPS1) channel.
enum class OutputLog : std::uint8_t
{
  ImuShort,
  ImuData,
  EkfEuler,
  EkfQuat,
  EkfNav,
  EkfVelBody,
  Mag,
  GnssPos,
  GnssVel,
  GnssHdt,
  GnssRaw,
  Utc,
  Status,
  AirData,
  ShipMotion,
  EventA,
  EventB,
  EventC,
  EventD,
  EventE,
};

// Output cadence. DivN divides the 200 Hz main loop (Div2=100 Hz, Div4=50,
// Div5=40, Div8=25, Div10=20, Div20=10, Div40=5, Div100=2, Div200=1 Hz).
// NewData emits whenever fresh data is available; Pps on the pulse-per-second
// edge; Disabled turns the log off.
enum class OutputRate : std::uint16_t
{
  Disabled,
  MainLoop,
  Div2,
  Div4,
  Div5,
  Div8,
  Div10,
  Div20,
  Div40,
  Div100,
  Div200,
  NewData,
  Pps,
};

// Device output port a log is emitted on. Main (PORT_A) is the link the driver
// connects to; PortC/PortE exist only on some device variants.
enum class OutputPort : std::uint8_t
{
  Main,
  PortC,
  PortE,
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

  // ---- Device provisioning (Phase 3h-2) ----------------------------------
  //
  // Apply install-time / solution-tuning settings. Each is a synchronous
  // command (same threading contract as the rest of Configurator). They write
  // the device's RAM settings; call save_settings() afterwards to persist to
  // NVRAM (which reboots the device). Typically driven from launch params via
  // SbgDriverNode's configure_device.* block.
  [[nodiscard]] Result<void> set_motion_profile(MotionProfile profile);
  [[nodiscard]] Result<void> set_imu_alignment(const ImuAlignment & alignment);
  [[nodiscard]] Result<void> set_aiding_assignment(const AidingAssignment & assignment);
  [[nodiscard]] Result<void> set_gnss_installation(const GnssInstallation & installation);
  [[nodiscard]] Result<void> set_magnetometer_model(MagModel model);

  // Set the emission cadence of a single output log on a device port (default
  // the main port). Use OutputRate::Disabled to turn a log off.
  [[nodiscard]] Result<void> set_output(
    OutputLog log, OutputRate rate, OutputPort port = OutputPort::Main);

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
