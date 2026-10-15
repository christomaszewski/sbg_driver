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
#include <chrono>
#include <cstdint>
#include <expected>  // NOLINT(build/include_order) -- C++20/23 header
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <string_view>
#include <utility>
#include <variant>
#include <vector>

// Fleet service-health reporting, producer side of the dashboard's
// docs/SERVICE_HEALTH.md contract (see doc/health.md for this driver's
// component inventory). This header is deliberately free of ROS and sbgECom
// types: HealthReporter turns observations into components, and
// health_publisher.hpp carries them onto /diagnostics. That split keeps the
// freshness/alert logic testable against an injected clock.

namespace sbg_driver
{

// DiagnosticStatus levels, by value.
enum class HealthLevel : std::uint8_t
{
  Ok = 0,
  Warn = 1,
  Error = 2,
  Stale = 3,
};

// A measurement the component should carry but could not obtain: JSON null,
// "null" in a ROS KeyValue. Never substituted by zero.
struct HealthNull
{
  friend bool operator==(HealthNull, HealthNull) = default;
};

// Flat scalar. Doubles are finite by construction (see HealthReporter).
using HealthValue = std::variant<std::string, double, std::int64_t, bool, HealthNull>;

// KeyValue.value rendering: strings verbatim, numbers in their shortest
// round-trip form, true/false, null.
[[nodiscard]] std::string to_diagnostic_string(const HealthValue & value);

struct HealthComponent
{
  std::string name;  // "<instance>: <component>"
  HealthLevel level = HealthLevel::Stale;
  std::string message;
  std::string hardware_id;  // device identifier; empty for software components
  std::vector<std::pair<std::string, HealthValue>> values;

  // nullptr when the component does not carry `key`.
  [[nodiscard]] const HealthValue * find(std::string_view key) const noexcept;
};

// Producer-side limits for one metric. Unset bounds do not apply; none are
// set by default (the contract forbids inventing universal limits).
struct HealthLimits
{
  std::optional<double> warn_above;
  std::optional<double> warn_below;
  std::optional<double> error_above;
  std::optional<double> error_below;
};

struct HealthSettings
{
  // Fleet identity, independent of the ROS namespace. An empty vehicle_id is
  // allowed (the consumer then lists the reporter under its ROS domain).
  std::string vehicle_id;
  std::string instance;
  std::string service = "sbg";

  // Reported on the service component; not interpreted.
  std::string transport;

  // Nominal heartbeat period.
  double publish_interval_s = 1.0;
  // How long a device observation (STATUS / IMU / EKF_NAV log) stays valid.
  double stale_after_s = 5.0;
  // Silence on the whole log stream, while active, that counts as a fault.
  double stream_timeout_s = 2.0;

  HealthLimits temp_imu_c;

  // The GNSS position grade the deployment expects; a lower one is a WARN on
  // the gnss component. The grade itself is always reported.
  enum class GnssFix : std::uint8_t
  {
    None,      // no expectation: any state is OK
    Fix,       // a computed position of any type
    RtkFloat,  // RTK float or fixed
    RtkFixed,  // RTK fixed (integer ambiguities)
  };
  GnssFix required_gnss_fix = GnssFix::Fix;

  // Device status flags (alert codes) that must not raise an alert, for
  // products that do not implement them.
  std::vector<std::string> ignored_alerts;
};

// Empty on success, else what is wrong with the settings.
[[nodiscard]] std::expected<void, std::string> validate(const HealthSettings & settings);

// One SBG_ECOM_LOG_STATUS general-status flag, reported as alert/<code>
// while its OK bit is cleared.
struct HealthAlertDef
{
  std::string_view code;
  std::uint16_t ok_mask;  // bit in SbgEComLogStatus::generalStatus, set = healthy
  HealthLevel level;
  std::string_view severity;
  std::string_view category;
  std::string_view vendor_flag;
  std::string_view message;
  std::string_view detail;
};

// The vendor assigns these flags no severity; the mapping is this driver's
// (doc/health.md): a lost supply, unloaded settings or out-of-limits
// temperature invalidate the outputs, the other two degrade them.
inline constexpr std::array<HealthAlertDef, 7> k_health_alerts{
  HealthAlertDef{
    .code = "MAIN_POWER",
    .ok_mask = (1U << 0),
    .level = HealthLevel::Error,
    .severity = "ERROR",
    .category = "power",
    .vendor_flag = "SBG_ECOM_GENERAL_MAIN_POWER_OK",
    .message = "main power supply fault",
    .detail = "The device reports that its main power supply is not OK.",
  },
  HealthAlertDef{
    .code = "IMU_POWER",
    .ok_mask = (1U << 1),
    .level = HealthLevel::Error,
    .severity = "ERROR",
    .category = "power",
    .vendor_flag = "SBG_ECOM_GENERAL_IMU_POWER_OK",
    .message = "IMU power supply fault",
    .detail = "The device reports that its IMU power supply is not OK.",
  },
  HealthAlertDef{
    .code = "GPS_POWER",
    .ok_mask = (1U << 2),
    .level = HealthLevel::Error,
    .severity = "ERROR",
    .category = "power",
    .vendor_flag = "SBG_ECOM_GENERAL_GPS_POWER_OK",
    .message = "GNSS power supply fault",
    .detail = "The device reports that its GNSS power supply is not OK.",
  },
  HealthAlertDef{
    .code = "SETTINGS",
    .ok_mask = (1U << 3),
    .level = HealthLevel::Error,
    .severity = "ERROR",
    .category = "settings",
    .vendor_flag = "SBG_ECOM_GENERAL_SETTINGS_OK",
    .message = "settings not loaded",
    .detail = "The device reports that its settings were not loaded correctly.",
  },
  HealthAlertDef{
    .code = "TEMPERATURE",
    .ok_mask = (1U << 4),
    .level = HealthLevel::Error,
    .severity = "ERROR",
    .category = "temperature",
    .vendor_flag = "SBG_ECOM_GENERAL_TEMPERATURE_OK",
    .message = "temperature outside specified limits",
    .detail = "The device reports that its temperature is outside its specified limits.",
  },
  HealthAlertDef{
    .code = "DATALOGGER",
    .ok_mask = (1U << 5),
    .level = HealthLevel::Warn,
    .severity = "WARNING",
    .category = "datalogger",
    .vendor_flag = "SBG_ECOM_GENERAL_DATALOGGER_OK",
    .message = "data logger fault",
    .detail = "The device reports that its data logger is not working correctly.",
  },
  HealthAlertDef{
    .code = "CPU",
    .ok_mask = (1U << 6),
    .level = HealthLevel::Warn,
    .severity = "WARNING",
    .category = "cpu",
    .vendor_flag = "SBG_ECOM_GENERAL_CPU_OK",
    .message = "insufficient CPU headroom",
    .detail = "The device reports that its CPU headroom is not correct.",
  },
};

// Collects observations and renders the instance's complete component
// inventory. Lives for the whole process: publisher_id is the process-start
// identity and the sequence never restarts under it.
//
// Threading: every method is safe from any thread (one mutex). The observe_*
// / note_* calls come from the device I/O thread, the lifecycle calls from
// the executor thread, snapshot() from the health publisher's thread.
class HealthReporter
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;
  using Clock = std::function<TimePoint()>;

  // Raw SBG_ECOM_LOG_STATUS fields.
  struct DeviceStatus
  {
    std::uint16_t general = 0;
    std::uint32_t com = 0;
    std::uint16_t com2 = 0;
    std::uint32_t aiding = 0;
    std::uint32_t uptime_s = 0;         // 0 = not available
    std::uint8_t cpu_usage_pct = 0xFF;  // 0xFF = not available
  };

  // From SBG_ECOM_CMD_INFO; all empty when the device could not be asked
  // (file replay, data-only interfaces).
  struct DeviceIdentity
  {
    std::string product_code;
    std::string serial_number;
    std::uint32_t firmware_rev = 0;
  };

  // Primary-receiver SBG_ECOM_LOG_GPS1_POS, vendor enums and sentinels as-is.
  struct GnssPosition
  {
    std::uint8_t solution_status = 0;            // SbgEComGnssPosStatus (0 = SOL_COMPUTED)
    std::uint8_t fix_type = 0;                   // SbgEComGnssPosType (6 RTK_FLOAT, 7 RTK_INT)
    std::uint8_t sv_used = 0xFF;                 // 0xFF = not available
    std::uint16_t differential_age_cs = 0xFFFF;  // 0.01 s; 0xFFFF = not available
    std::uint16_t base_station_id = 0xFFFF;      // 0xFFFF = not available
    double latitude_accuracy_m = 0.0;            // 1 sigma
    double longitude_accuracy_m = 0.0;
    double altitude_accuracy_m = 0.0;
  };

  // Primary-receiver SBG_ECOM_LOG_GPS1_HDT (dual-antenna true heading).
  struct GnssHeading
  {
    std::uint8_t solution_status = 0;  // SbgEComGnssHdtStatus (0 = SOL_COMPUTED)
    bool heading_valid = false;        // the SDK's own validity test
    double heading_deg = 0.0;
    double accuracy_deg = 0.0;  // 1 sigma
    bool baseline_valid = false;
    double baseline_m = 0.0;
    std::uint8_t sv_used = 0xFF;  // 0xFF = not available
  };

  // Counters owned by Publishers, sampled by the caller at snapshot time.
  struct StreamCounters
  {
    std::uint64_t composition_drops = 0;
    std::uint64_t imu_orientation_misses = 0;
    bool imu_short_in_use = false;
  };

  // `clock` defaults to steady_clock; `publisher_id` to a fresh UUID.
  explicit HealthReporter(Clock clock = {}, std::string publisher_id = {});

  // Validate and apply. On error the previous settings stay in force.
  [[nodiscard]] std::expected<void, std::string> configure(HealthSettings settings);

  // ---- Operational state (lifecycle callbacks) ----------------------------
  void begin_transition(std::string_view what);
  // Inactive. A non-empty `fault` (a failed activation) is an ERROR until the
  // next transition; a plain standby is healthy.
  void enter_standby(std::string fault = {});
  // Starts a new device session: observations must be seen again before they
  // count as current. Call before the I/O thread starts.
  void enter_active(DeviceIdentity identity);
  // `fault`: why the node fell back here (a lifecycle error), as for standby.
  void enter_unconfigured(std::string fault = {});
  void enter_shutdown();
  // The link was dropped on purpose while ACTIVE (settings save reboots the
  // unit); cleared by the next enter_active().
  void set_device_released();
  // A device command owns the link; the log stream is not expected meanwhile.
  void pause_stream(std::string_view reason);
  void resume_stream();

  // ---- Observations (device I/O thread) -----------------------------------
  void note_log();
  void note_callback_exception();
  void observe_status(const DeviceStatus & status);
  void observe_imu_status(std::uint16_t status);
  // Non-finite reads flag collection failure without replacing a valid sample.
  void observe_imu_temperature(double celsius);
  void observe_ekf(std::uint32_t ekf_nav_status);
  // Optional sources: the gnss / gnss heading components exist from the first
  // such log on, so a unit that never streams them does not look faulty.
  void observe_gnss(const GnssPosition & position);
  void observe_gnss_heading(const GnssHeading & heading);

  // The complete inventory, as of now. Each call is one publication: it
  // advances health.sequence. Cached observations keep their sample id and
  // only age.
  [[nodiscard]] std::vector<HealthComponent> snapshot(const StreamCounters & counters);
  [[nodiscard]] std::vector<HealthComponent> snapshot() { return snapshot(StreamCounters{}); }

  [[nodiscard]] const std::string & publisher_id() const noexcept { return publisher_id_; }
  [[nodiscard]] double publish_interval_s() const;

private:
  enum class State : std::uint8_t
  {
    Unconfigured,
    Standby,
    Transitioning,
    Active,
    Shutdown,
  };

  // One observation source (a device log kind).
  struct Cell
  {
    bool ever = false;     // observed during this reporter's lifetime
    bool session = false;  // observed since the last enter_active()
    TimePoint at{};
    std::uint64_t count = 0;
  };

  // A counter whose recent growth is the fault, not its lifetime total.
  struct Growth
  {
    std::uint64_t last = 0;
    std::optional<TimePoint> grew_at;
    void update(std::uint64_t value, TimePoint now);
    [[nodiscard]] bool recent(TimePoint now, double window_s) const;
  };

  struct Stream
  {
    bool demanded = false;  // active, link held, no device command running
    bool silent = false;    // demanded and quiet past stream_timeout_s
    double silence_s = 0.0;
  };

  struct Freshness
  {
    std::string_view availability;
    double age_s = 0.0;
    std::string sample_id;
    std::string error;
  };

  void change_state(State state);
  [[nodiscard]] Stream stream_at(TimePoint now) const;
  [[nodiscard]] Freshness freshness(
    const Cell & cell, std::string_view tag, std::string_view log_name, TimePoint now,
    const Stream & stream) const;
  [[nodiscard]] std::string hardware_id() const;
  [[nodiscard]] HealthComponent make(
    std::string_view component, HealthLevel level, std::string message,
    std::string hardware_id) const;
  // Applies STALE/last_level and appends the health.* metadata.
  void finish(HealthComponent & component, const Freshness & freshness) const;

  void add_service(std::vector<HealthComponent> & out, TimePoint now);
  void add_stream(
    std::vector<HealthComponent> & out, TimePoint now, const Stream & stream,
    const StreamCounters & counters);
  void add_device(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);
  void add_imu(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);
  void add_temperature(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);
  void add_ekf(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);
  void add_gnss(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);
  void add_gnss_heading(std::vector<HealthComponent> & out, TimePoint now, const Stream & stream);

  const Clock clock_;
  const std::string publisher_id_;

  mutable std::mutex mutex_;
  HealthSettings settings_{};
  std::uint64_t sequence_ = 0;

  State state_ = State::Unconfigured;
  std::uint64_t state_changes_ = 0;
  std::string transition_;
  std::string fault_;
  bool released_ = false;
  std::optional<std::string> stream_pause_;
  DeviceIdentity identity_{};

  TimePoint session_started_{};
  TimePoint stream_since_{};  // activation, or the end of the last device command
  Cell log_cell_{};           // any log at all
  std::uint64_t session_logs_ = 0;
  std::uint64_t callback_exceptions_ = 0;
  Growth exception_growth_{};
  Growth drop_growth_{};

  Cell status_cell_{};
  DeviceStatus status_{};
  std::array<bool, k_health_alerts.size()> alert_known_{};

  Cell imu_cell_{};
  std::optional<double> temperature_c_;  // last valid observation
  bool temperature_read_failed_ = false;
  Cell imu_status_cell_{};
  std::uint16_t imu_status_ = 0;

  Cell ekf_cell_{};
  std::uint32_t ekf_status_ = 0;

  Cell gnss_cell_{};
  GnssPosition gnss_{};

  Cell heading_cell_{};
  GnssHeading heading_{};
};

// Holds the stream paused for the duration of a device command.
class HealthStreamPause
{
public:
  HealthStreamPause(HealthReporter & reporter, std::string_view reason) : reporter_(reporter)
  {
    reporter_.pause_stream(reason);
  }
  ~HealthStreamPause() { reporter_.resume_stream(); }
  HealthStreamPause(const HealthStreamPause &) = delete;
  HealthStreamPause & operator=(const HealthStreamPause &) = delete;

private:
  HealthReporter & reporter_;
};

}  // namespace sbg_driver
