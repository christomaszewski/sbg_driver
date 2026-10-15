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

#include "sbg_driver/health.hpp"

#include <algorithm>
#include <cmath>
#include <format>  // NOLINT(build/include_order) -- C++20/23 header
#include <iterator>
#include <random>

#include <sbg/detail/c_api.hpp>

namespace sbg_driver
{

namespace
{

// health.availability values (SERVICE_HEALTH.md).
constexpr std::string_view k_current = "current";
constexpr std::string_view k_retrying = "retrying";
constexpr std::string_view k_stale = "stale";
constexpr std::string_view k_unavailable = "unavailable";
constexpr std::string_view k_paused = "paused";

// SbgEComLogEkfNav::status layout. health.hpp stays free of SDK types, so the
// layouts it and this file spell out are pinned to the SDK's below.
constexpr std::uint32_t k_ekf_mode_mask = 0x0FU;
constexpr std::uint32_t k_ekf_attitude_valid = 1U << 4;
constexpr std::uint32_t k_ekf_heading_valid = 1U << 5;
constexpr std::uint32_t k_ekf_velocity_valid = 1U << 6;
constexpr std::uint32_t k_ekf_position_valid = 1U << 7;
constexpr std::uint32_t k_ekf_gps1_vel_used = 1U << 10;
constexpr std::uint32_t k_ekf_gps1_pos_used = 1U << 11;
constexpr std::uint32_t k_ekf_gps1_hdt_used = 1U << 13;
constexpr std::uint32_t k_ekf_mag_ref_used = 1U << 9;
constexpr std::uint32_t k_ekf_align_valid = 1U << 27;

// SbgEComGnssPosType / SbgEComGnss{Pos,Hdt}Status, likewise checked there.
constexpr std::uint8_t k_gnss_sol_computed = 0;
constexpr std::uint8_t k_gnss_type_no_solution = 0;
constexpr std::uint8_t k_gnss_type_rtk_float = 6;
constexpr std::uint8_t k_gnss_type_rtk_int = 7;

static_assert(k_ekf_mode_mask == 0x0FU && SBG_ECOM_SOL_MODE_NAV_POSITION == 4);
static_assert(k_ekf_attitude_valid == SBG_ECOM_SOL_ATTITUDE_VALID);
static_assert(k_ekf_heading_valid == SBG_ECOM_SOL_HEADING_VALID);
static_assert(k_ekf_velocity_valid == SBG_ECOM_SOL_VELOCITY_VALID);
static_assert(k_ekf_position_valid == SBG_ECOM_SOL_POSITION_VALID);
static_assert(k_ekf_gps1_vel_used == SBG_ECOM_SOL_GPS1_VEL_USED);
static_assert(k_ekf_gps1_pos_used == SBG_ECOM_SOL_GPS1_POS_USED);
static_assert(k_ekf_gps1_hdt_used == SBG_ECOM_SOL_GPS1_HDT_USED);
static_assert(k_ekf_mag_ref_used == SBG_ECOM_SOL_MAG_REF_USED);
static_assert(k_ekf_align_valid == SBG_ECOM_SOL_ALIGN_VALID);
static_assert(k_gnss_sol_computed == SBG_ECOM_GNSS_POS_STATUS_SOL_COMPUTED);
static_assert(k_gnss_sol_computed == SBG_ECOM_GNSS_HDT_STATUS_SOL_COMPUTED);
static_assert(k_gnss_type_no_solution == SBG_ECOM_GNSS_POS_TYPE_NO_SOLUTION);
static_assert(k_gnss_type_rtk_float == SBG_ECOM_GNSS_POS_TYPE_RTK_FLOAT);
static_assert(k_gnss_type_rtk_int == SBG_ECOM_GNSS_POS_TYPE_RTK_INT);
static_assert(SBG_ECOM_GNSS_POS_TYPE_FIXED == 10 && SBG_ECOM_GNSS_POS_STATUS_HEIGHT_LIMIT == 3);
static_assert(k_health_alerts[0].ok_mask == SBG_ECOM_GENERAL_MAIN_POWER_OK);
static_assert(k_health_alerts[1].ok_mask == SBG_ECOM_GENERAL_IMU_POWER_OK);
static_assert(k_health_alerts[2].ok_mask == SBG_ECOM_GENERAL_GPS_POWER_OK);
static_assert(k_health_alerts[3].ok_mask == SBG_ECOM_GENERAL_SETTINGS_OK);
static_assert(k_health_alerts[4].ok_mask == SBG_ECOM_GENERAL_TEMPERATURE_OK);
static_assert(k_health_alerts[5].ok_mask == SBG_ECOM_GENERAL_DATALOGGER_OK);
static_assert(k_health_alerts[6].ok_mask == SBG_ECOM_GENERAL_CPU_OK);

std::string gnss_type_name(std::uint8_t type)
{
  constexpr std::array<std::string_view, 11> names{
    "NO_SOLUTION", "UNKNOWN", "SINGLE",    "PSRDIFF", "SBAS", "OMNISTAR",
    "RTK_FLOAT",   "RTK_INT", "PPP_FLOAT", "PPP_INT", "FIXED"};
  return type < names.size() ? std::string{names[type]} : std::format("TYPE_{}", type);
}

std::string gnss_status_name(std::uint8_t status)
{
  constexpr std::array<std::string_view, 4> names{
    "SOL_COMPUTED", "INSUFFICIENT_OBS", "INTERNAL_ERROR", "HEIGHT_LIMIT"};
  return status < names.size() ? std::string{names[status]} : std::format("STATUS_{}", status);
}

double seconds_between(HealthReporter::TimePoint from, HealthReporter::TimePoint to)
{
  return std::max(0.0, std::chrono::duration<double>(to - from).count());
}

// Millisecond/milli-unit resolution keeps the published strings short
// without hiding anything the sources resolve.
double round3(double value) { return std::round(value * 1000.0) / 1000.0; }

std::string make_uuid()
{
  std::random_device entropy;
  std::array<std::uint8_t, 16> bytes{};
  for (auto & byte : bytes) {
    byte = static_cast<std::uint8_t>(entropy());
  }
  bytes[6] = static_cast<std::uint8_t>((bytes[6] & 0x0FU) | 0x40U);  // version 4
  bytes[8] = static_cast<std::uint8_t>((bytes[8] & 0x3FU) | 0x80U);  // RFC 4122 variant
  std::string out;
  for (std::size_t i = 0; i < bytes.size(); ++i) {
    if (i == 4 || i == 6 || i == 8 || i == 10) {
      out += '-';
    }
    out += std::format("{:02x}", bytes[i]);
  }
  return out;
}

// Identity tokens become a Zenoh key chunk on the consumer side.
bool valid_segment(std::string_view token)
{
  return !token.empty() && std::ranges::none_of(token, [](char c) {
    return c == '/' || c == '*' || c == '?' || c == '#' || c == '$' ||
           static_cast<unsigned char>(c) <= 0x20U || c == 0x7F;
  });
}

std::expected<void, std::string> require_positive(std::string_view name, double value)
{
  if (!std::isfinite(value) || value <= 0.0) {
    return std::unexpected(std::format("health.{} must be a positive finite number", name));
  }
  return {};
}

const HealthAlertDef * find_alert(std::string_view code)
{
  const auto it = std::ranges::find(k_health_alerts, code, &HealthAlertDef::code);
  return it == k_health_alerts.end() ? nullptr : &*it;
}

std::expected<void, std::string> validate_limits(std::string_view metric, const HealthLimits & l)
{
  for (const auto & [name, bound] :
       {std::pair{"warn_above", l.warn_above}, std::pair{"warn_below", l.warn_below},
        std::pair{"error_above", l.error_above}, std::pair{"error_below", l.error_below}}) {
    if (bound && !std::isfinite(*bound)) {
      return std::unexpected(std::format("health.limits.{}.{} must be finite", metric, name));
    }
  }
  if (l.warn_above && l.error_above && *l.error_above < *l.warn_above) {
    return std::unexpected(
      std::format("health.limits.{}: error_above must not be below warn_above", metric));
  }
  if (l.warn_below && l.error_below && *l.error_below > *l.warn_below) {
    return std::unexpected(
      std::format("health.limits.{}: error_below must not be above warn_below", metric));
  }
  // The lowest upper bound must leave room above the highest lower bound, or
  // no reading could ever be healthy.
  const auto upper = l.warn_above ? l.warn_above : l.error_above;
  const auto lower = l.warn_below ? l.warn_below : l.error_below;
  if (upper && lower && !(*lower < *upper)) {
    return std::unexpected(
      std::format(
        "health.limits.{}: the *_below bounds must be less than the *_above bounds", metric));
  }
  return {};
}

// First crossed bound wins, errors before warnings (the worse verdict is the
// one the operator needs).
void apply_limits(
  const HealthLimits & limits, std::string_view metric, double value, HealthLevel & level,
  std::string & message)
{
  struct Check
  {
    const std::optional<double> & bound;
    bool above;
    HealthLevel severity;
    std::string_view name;
  };
  for (const auto & check :
       {Check{limits.error_above, true, HealthLevel::Error, "error_above"},
        Check{limits.error_below, false, HealthLevel::Error, "error_below"},
        Check{limits.warn_above, true, HealthLevel::Warn, "warn_above"},
        Check{limits.warn_below, false, HealthLevel::Warn, "warn_below"}}) {
    if (!check.bound || !(check.above ? value > *check.bound : value < *check.bound)) {
      continue;
    }
    level = std::max(level, check.severity);
    message = std::format(
      "{} {} is {} {} {}", metric, value, check.above ? "above" : "below", check.name,
      *check.bound);
    return;
  }
}

void touch(auto & cell, HealthReporter::TimePoint now)
{
  cell.ever = true;
  cell.session = true;
  cell.at = now;
  ++cell.count;
}

std::string hex16(std::uint32_t value) { return std::format("0x{:04x}", value); }
std::string hex32(std::uint32_t value) { return std::format("0x{:08x}", value); }

}  // namespace

std::string to_diagnostic_string(const HealthValue & value)
{
  struct Visitor
  {
    std::string operator()(const std::string & s) const { return s; }
    std::string operator()(double d) const
    {
      return std::isfinite(d) ? std::format("{}", d) : std::string{"null"};
    }
    std::string operator()(std::int64_t i) const { return std::to_string(i); }
    std::string operator()(bool b) const { return b ? "true" : "false"; }
    std::string operator()(HealthNull) const { return "null"; }
  };
  return std::visit(Visitor{}, value);
}

const HealthValue * HealthComponent::find(std::string_view key) const noexcept
{
  const auto it =
    std::ranges::find(values, key, [](const auto & kv) -> std::string_view { return kv.first; });
  return it == values.end() ? nullptr : &it->second;
}

std::expected<void, std::string> validate(const HealthSettings & settings)
{
  constexpr std::string_view segment_rule =
    "a single path segment (no '/', '*', '?', '#', '$', whitespace)";
  if (!valid_segment(settings.instance)) {
    return std::unexpected(std::format("health.instance must be non-empty and {}", segment_rule));
  }
  if (!settings.vehicle_id.empty() && !valid_segment(settings.vehicle_id)) {
    return std::unexpected(std::format("health.vehicle_id must be {}", segment_rule));
  }
  if (!valid_segment(settings.service)) {
    return std::unexpected(std::format("health service name must be {}", segment_rule));
  }
  for (const auto & [name, value] :
       {std::pair{"publish_interval_s", settings.publish_interval_s},
        std::pair{"stale_after_s", settings.stale_after_s},
        std::pair{"stream_timeout_s", settings.stream_timeout_s}}) {
    if (auto ok = require_positive(name, value); !ok) {
      return ok;
    }
  }
  // A validity interval shorter than two heartbeats makes every observation
  // expire on the consumer between publications.
  if (settings.stale_after_s < 2.0 * settings.publish_interval_s) {
    return std::unexpected(
      std::format(
        "health.stale_after_s ({}) must be at least twice health.publish_interval_s ({})",
        settings.stale_after_s, settings.publish_interval_s));
  }
  if (auto ok = validate_limits("temp_imu_c", settings.temp_imu_c); !ok) {
    return ok;
  }
  for (const auto & code : settings.ignored_alerts) {
    if (find_alert(code) == nullptr) {
      return std::unexpected(std::format("health.ignored_alerts: unknown alert code '{}'", code));
    }
  }
  return {};
}

void HealthReporter::Growth::update(std::uint64_t value, TimePoint now)
{
  // A smaller value is the owner resetting its counter, not growth.
  if (value > last) {
    grew_at = now;
  }
  last = value;
}

bool HealthReporter::Growth::recent(TimePoint now, double window_s) const
{
  return grew_at && seconds_between(*grew_at, now) <= window_s;
}

HealthReporter::HealthReporter(Clock clock, std::string publisher_id)
: clock_(clock ? std::move(clock) : Clock{[] { return std::chrono::steady_clock::now(); }}),
  publisher_id_(publisher_id.empty() ? make_uuid() : std::move(publisher_id))
{
}

std::expected<void, std::string> HealthReporter::configure(HealthSettings settings)
{
  if (auto ok = validate(settings); !ok) {
    return ok;
  }
  const std::lock_guard lock{mutex_};
  settings_ = std::move(settings);
  return {};
}

double HealthReporter::publish_interval_s() const
{
  const std::lock_guard lock{mutex_};
  return settings_.publish_interval_s;
}

void HealthReporter::change_state(State state)
{
  state_ = state;
  ++state_changes_;
  released_ = false;
  stream_pause_.reset();
}

void HealthReporter::begin_transition(std::string_view what)
{
  const std::lock_guard lock{mutex_};
  transition_ = what;
  fault_.clear();
  change_state(State::Transitioning);
}

void HealthReporter::enter_standby(std::string fault)
{
  const std::lock_guard lock{mutex_};
  fault_ = std::move(fault);
  change_state(State::Standby);
}

void HealthReporter::enter_active(DeviceIdentity identity)
{
  const std::lock_guard lock{mutex_};
  const auto now = clock_();
  identity_ = std::move(identity);
  fault_.clear();
  session_started_ = now;
  stream_since_ = now;
  session_logs_ = 0;
  for (Cell * cell :
       {&log_cell_, &status_cell_, &imu_cell_, &imu_status_cell_, &ekf_cell_, &gnss_cell_,
        &heading_cell_}) {
    cell->session = false;
  }
  change_state(State::Active);
}

void HealthReporter::enter_unconfigured(std::string fault)
{
  const std::lock_guard lock{mutex_};
  fault_ = std::move(fault);
  change_state(State::Unconfigured);
}

void HealthReporter::enter_shutdown()
{
  const std::lock_guard lock{mutex_};
  fault_.clear();
  change_state(State::Shutdown);
}

void HealthReporter::set_device_released()
{
  const std::lock_guard lock{mutex_};
  released_ = true;
  ++state_changes_;
}

void HealthReporter::pause_stream(std::string_view reason)
{
  const std::lock_guard lock{mutex_};
  stream_pause_ = std::string{reason};
}

void HealthReporter::resume_stream()
{
  const std::lock_guard lock{mutex_};
  stream_pause_.reset();
  // Silence is measured from here, not from the last pre-command log.
  stream_since_ = clock_();
}

void HealthReporter::note_log()
{
  const std::lock_guard lock{mutex_};
  touch(log_cell_, clock_());
  ++session_logs_;
}

void HealthReporter::note_callback_exception()
{
  const std::lock_guard lock{mutex_};
  ++callback_exceptions_;
}

void HealthReporter::observe_status(const DeviceStatus & status)
{
  const std::lock_guard lock{mutex_};
  touch(status_cell_, clock_());
  status_ = status;
  for (std::size_t i = 0; i < k_health_alerts.size(); ++i) {
    if ((status.general & k_health_alerts[i].ok_mask) == 0U) {
      alert_known_[i] = true;
    }
  }
}

void HealthReporter::observe_imu_status(std::uint16_t status)
{
  const std::lock_guard lock{mutex_};
  touch(imu_status_cell_, clock_());
  imu_status_ = status;
}

void HealthReporter::observe_imu_temperature(double celsius)
{
  const std::lock_guard lock{mutex_};
  temperature_read_failed_ = !std::isfinite(celsius);
  if (temperature_read_failed_) {
    return;
  }
  touch(imu_cell_, clock_());
  temperature_c_ = celsius;
}

void HealthReporter::observe_ekf(std::uint32_t ekf_nav_status)
{
  const std::lock_guard lock{mutex_};
  touch(ekf_cell_, clock_());
  ekf_status_ = ekf_nav_status;
}

void HealthReporter::observe_gnss(const GnssPosition & position)
{
  const std::lock_guard lock{mutex_};
  touch(gnss_cell_, clock_());
  gnss_ = position;
}

void HealthReporter::observe_gnss_heading(const GnssHeading & heading)
{
  const std::lock_guard lock{mutex_};
  touch(heading_cell_, clock_());
  heading_ = heading;
}

HealthReporter::Stream HealthReporter::stream_at(TimePoint now) const
{
  Stream stream;
  stream.demanded = state_ == State::Active && !released_ && !stream_pause_;
  if (!stream.demanded) {
    return stream;
  }
  const bool heard = log_cell_.session && log_cell_.at > stream_since_;
  stream.silence_s = seconds_between(heard ? log_cell_.at : stream_since_, now);
  stream.silent = stream.silence_s > settings_.stream_timeout_s;
  return stream;
}

HealthReporter::Freshness HealthReporter::freshness(
  const Cell & cell, std::string_view tag, std::string_view log_name, TimePoint now,
  const Stream & stream) const
{
  Freshness f;
  f.sample_id =
    std::format("{}:{}:{}", std::string_view{publisher_id_}.substr(0, 8), tag, cell.count);
  f.age_s = cell.ever ? seconds_between(cell.at, now) : 0.0;
  if (!stream.demanded) {
    // Standby, a transition, a released link or a device command: nothing is
    // being asked of the device, so nothing is missing. Last-known values and
    // their level stay as they are.
    f.availability = k_paused;
    return f;
  }
  if (!cell.session) {
    const double waited = seconds_between(session_started_, now);
    f.availability = cell.ever ? k_stale : k_unavailable;
    // Give the log one validity interval to show up before calling it a
    // collection failure.
    if (waited > settings_.stale_after_s) {
      f.error = std::format(
        "no {} received since activation ({:.1f} s) — is that log enabled on the device output?",
        log_name, waited);
    }
    return f;
  }
  if (f.age_s > settings_.stale_after_s) {
    f.availability = k_stale;
    f.error = std::format("no {} for {:.1f} s", log_name, f.age_s);
  } else if (stream.silent) {
    f.availability = k_retrying;
    f.error = std::format("device stream silent for {:.1f} s", stream.silence_s);
  } else {
    f.availability = k_current;
  }
  return f;
}

std::string HealthReporter::hardware_id() const
{
  if (identity_.product_code.empty() || identity_.serial_number.empty()) {
    return identity_.product_code + identity_.serial_number;
  }
  return identity_.product_code + " " + identity_.serial_number;
}

HealthComponent HealthReporter::make(
  std::string_view component, HealthLevel level, std::string message, std::string hardware_id) const
{
  HealthComponent c;
  c.name = std::format("{}: {}", settings_.instance, component);
  c.level = level;
  c.message = std::move(message);
  c.hardware_id = std::move(hardware_id);
  return c;
}

void HealthReporter::finish(HealthComponent & component, const Freshness & f) const
{
  auto & v = component.values;
  if (f.availability == k_stale && component.level != HealthLevel::Stale) {
    // The last-known verdict travels with the STALE level so a stale ERROR
    // still reads as an ERROR.
    v.emplace_back("health.last_level", static_cast<std::int64_t>(component.level));
    component.level = HealthLevel::Stale;
  }
  v.emplace_back("health.vehicle_id", settings_.vehicle_id);
  v.emplace_back("health.instance", settings_.instance);
  v.emplace_back("health.service", settings_.service);
  v.emplace_back("health.publisher_id", publisher_id_);
  v.emplace_back("health.sequence", static_cast<std::int64_t>(sequence_));
  v.emplace_back("health.publish_interval_s", settings_.publish_interval_s);
  v.emplace_back("health.sample_id", f.sample_id);
  v.emplace_back("health.sample_age_s", round3(f.age_s));
  v.emplace_back("health.stale_after_s", settings_.stale_after_s);
  v.emplace_back("health.availability", std::string{f.availability});
  if (!f.error.empty()) {
    v.emplace_back("health.error", f.error);
  }
}

void HealthReporter::add_service(std::vector<HealthComponent> & out, TimePoint now)
{
  exception_growth_.update(callback_exceptions_, now);

  HealthLevel level = HealthLevel::Ok;
  std::string_view state;
  switch (state_) {
    case State::Unconfigured:
      state = "unconfigured";
      break;
    case State::Standby:
      state = "standby";
      break;
    case State::Transitioning:
      state = "transitioning";
      break;
    case State::Active:
      state = "active";
      break;
    case State::Shutdown:
      state = "shutdown";
      break;
  }
  std::string message{state};
  if (state_ == State::Transitioning && !transition_.empty()) {
    message = transition_;
  }
  if (state_ == State::Active && released_) {
    level = HealthLevel::Warn;
    message =
      "device released after settings save (unit rebooting) — deactivate then activate to "
      "reconnect";
  }
  if (!fault_.empty()) {
    // Set only by a failed transition: the state is where the node fell back
    // to, the fault is why it is not where it was asked to be.
    level = HealthLevel::Error;
    message = fault_;
  }
  if (level == HealthLevel::Ok && exception_growth_.recent(now, settings_.stale_after_s)) {
    level = HealthLevel::Warn;
    message = "log handling is throwing exceptions — published outputs may be incomplete";
  }

  auto c = make("service", level, std::move(message), {});
  c.values.emplace_back("state", std::string{state});
  if (!settings_.transport.empty()) {
    c.values.emplace_back("transport", settings_.transport);
  }
  c.values.emplace_back("callback_exceptions", static_cast<std::int64_t>(callback_exceptions_));
  // The process observes its own state at every publication; the sample id
  // moves only when that state does.
  Freshness f;
  f.availability = k_current;
  f.sample_id =
    std::format("{}:service:{}", std::string_view{publisher_id_}.substr(0, 8), state_changes_);
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_stream(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream,
  const StreamCounters & counters)
{
  drop_growth_.update(counters.composition_drops, now);

  HealthLevel level = HealthLevel::Ok;
  std::string message;
  if (!stream.demanded) {
    if (stream_pause_) {
      message = std::format("paused for {}", *stream_pause_);
    } else if (released_) {
      message = "device link released";
    } else {
      message = "not streaming";
    }
  } else if (stream.silent) {
    level = HealthLevel::Error;
    message = log_cell_.session
              ? std::format("no logs for {:.1f} s", stream.silence_s)
              : std::format(
                  "no logs received since activation ({:.1f} s) — check the link and the "
                  "device output configuration",
                  stream.silence_s);
  } else if (!log_cell_.session) {
    // Nothing received yet is not evidence of health.
    level = HealthLevel::Warn;
    message = "waiting for the first log";
  } else if (drop_growth_.recent(now, settings_.stale_after_s)) {
    level = HealthLevel::Warn;
    message =
      "receiving, but /odom epochs are being dropped — EkfNav/EkfQuat/EkfVelBody rates are "
      "not on a common epoch (see configure_device.output.* or outputs.epoch_tolerance_us)";
  } else {
    message = "receiving";
  }

  auto c = make("stream", level, std::move(message), {});
  c.values.emplace_back("logs.received", static_cast<std::int64_t>(session_logs_));
  if (log_cell_.session) {
    c.values.emplace_back("frame_age_s", round3(seconds_between(log_cell_.at, now)));
  }
  c.values.emplace_back("stream.timeout_s", settings_.stream_timeout_s);
  c.values.emplace_back("composition_drops", static_cast<std::int64_t>(counters.composition_drops));
  c.values.emplace_back(
    "imu_orientation_misses", static_cast<std::int64_t>(counters.imu_orientation_misses));
  if (imu_cell_.session) {
    c.values.emplace_back(
      "imu_source", std::string{counters.imu_short_in_use ? "imu_short" : "imu_data"});
  }
  // Like the service, evaluated now; its sample id follows the log count.
  Freshness f;
  f.availability = stream.demanded ? k_current : k_paused;
  f.sample_id =
    std::format("{}:stream:{}", std::string_view{publisher_id_}.substr(0, 8), log_cell_.count);
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_device(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  constexpr std::string_view log_name = "SBG_ECOM_LOG_STATUS";
  const Freshness f = freshness(status_cell_, "status", log_name, now, stream);
  const std::string hw = hardware_id();
  const bool seen = status_cell_.ever;
  const std::string waiting =
    f.availability == k_paused ? "not observed yet" : std::format("waiting for {}", log_name);

  auto device =
    make("device", seen ? HealthLevel::Ok : HealthLevel::Stale, seen ? "OK" : waiting, hw);
  if (seen) {
    // 0 and 0xFF are the vendor's "not available" encodings, not readings.
    if (status_.uptime_s != 0U) {
      device.values.emplace_back("uptime_s", static_cast<std::int64_t>(status_.uptime_s));
    }
    if (status_.cpu_usage_pct != 0xFFU) {
      device.values.emplace_back("cpu.usage_pct", static_cast<std::int64_t>(status_.cpu_usage_pct));
    }
    device.values.emplace_back("device.general_status", hex16(status_.general));
    device.values.emplace_back("device.com_status", hex32(status_.com));
    device.values.emplace_back("device.com_status_2", hex16(status_.com2));
    device.values.emplace_back("device.aiding_status", hex32(status_.aiding));
  }
  if (!identity_.product_code.empty()) {
    device.values.emplace_back("device.product_code", identity_.product_code);
  }
  if (!identity_.serial_number.empty()) {
    device.values.emplace_back("device.serial_number", identity_.serial_number);
    device.values.emplace_back("device.firmware", hex32(identity_.firmware_rev));
  }
  // sbgECom reports supply health as OK flags (the *_POWER alerts) but no
  // voltage or current, so the electrical metrics are declared unsupported
  // rather than left to look missing.
  for (std::string_view metric : {"supply.voltage_v", "supply.current_a", "supply.power_w"}) {
    device.values.emplace_back(
      std::format("health.metric.{}.state", metric), std::string{"unsupported"});
  }
  finish(device, f);
  out.push_back(std::move(device));

  const auto ignored = [this](std::string_view code) {
    return std::ranges::find(settings_.ignored_alerts, code) != settings_.ignored_alerts.end();
  };
  std::int64_t active_count = 0;
  HealthLevel worst = seen ? HealthLevel::Ok : HealthLevel::Stale;
  std::vector<HealthComponent> alerts;
  for (std::size_t i = 0; i < k_health_alerts.size(); ++i) {
    const auto & def = k_health_alerts[i];
    if (!alert_known_[i] || ignored(def.code)) {
      continue;
    }
    // Reconciled against the last STATUS log only: without a new log a fault
    // stays as it was (and ages), it is never cleared by absence.
    const bool active = (status_.general & def.ok_mask) == 0U;
    if (active) {
      ++active_count;
      worst = std::max(worst, def.level);
    }
    auto alert = make(
      std::format("alert/{}", def.code), active ? def.level : HealthLevel::Ok,
      std::string{active ? def.message : "cleared"}, hw);
    alert.values.emplace_back("alert.code", std::string{def.code});
    alert.values.emplace_back("alert.category", std::string{def.category});
    alert.values.emplace_back("alert.active", active);
    alert.values.emplace_back("alert.severity", std::string{def.severity});
    alert.values.emplace_back("alert.flag", std::string{def.vendor_flag});
    alert.values.emplace_back(
      "alert.detail", std::format("{} ({} is cleared.)", def.detail, def.vendor_flag));
    finish(alert, f);
    alerts.push_back(std::move(alert));
  }

  auto summary = make(
    "sensor alerts", worst,
    seen ? std::format("{} active alert{}", active_count, active_count == 1 ? "" : "s") : waiting,
    hw);
  if (seen) {
    summary.values.emplace_back("alerts.active", active_count);
  }
  if (!settings_.ignored_alerts.empty()) {
    std::string list;
    for (const auto & code : settings_.ignored_alerts) {
      list += (list.empty() ? "" : ",") + code;
    }
    summary.values.emplace_back("alerts.ignored", std::move(list));
  }
  finish(summary, f);
  out.push_back(std::move(summary));
  std::ranges::move(alerts, std::back_inserter(out));
}

void HealthReporter::add_imu(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  const auto f = freshness(imu_status_cell_, "imu_status", "IMU status", now, stream);
  auto c = make(
    "imu", imu_status_cell_.ever ? HealthLevel::Ok : HealthLevel::Stale,
    imu_status_cell_.ever ? "OK" : "not observed yet", hardware_id());
  if (imu_status_cell_.ever) {
    c.values.emplace_back("imu.status", hex16(imu_status_));
    std::string failed;
    for (const auto & [mask, name] :
         {std::pair{SBG_ECOM_IMU_COM_OK, "SBG_ECOM_IMU_COM_OK"},
          std::pair{SBG_ECOM_IMU_STATUS_BIT, "SBG_ECOM_IMU_STATUS_BIT"},
          std::pair{SBG_ECOM_IMU_ACCEL_X_BIT, "SBG_ECOM_IMU_ACCEL_X_BIT"},
          std::pair{SBG_ECOM_IMU_ACCEL_Y_BIT, "SBG_ECOM_IMU_ACCEL_Y_BIT"},
          std::pair{SBG_ECOM_IMU_ACCEL_Z_BIT, "SBG_ECOM_IMU_ACCEL_Z_BIT"},
          std::pair{SBG_ECOM_IMU_GYRO_X_BIT, "SBG_ECOM_IMU_GYRO_X_BIT"},
          std::pair{SBG_ECOM_IMU_GYRO_Y_BIT, "SBG_ECOM_IMU_GYRO_Y_BIT"},
          std::pair{SBG_ECOM_IMU_GYRO_Z_BIT, "SBG_ECOM_IMU_GYRO_Z_BIT"},
          std::pair{SBG_ECOM_IMU_ACCELS_IN_RANGE, "SBG_ECOM_IMU_ACCELS_IN_RANGE"},
          std::pair{SBG_ECOM_IMU_GYROS_IN_RANGE, "SBG_ECOM_IMU_GYROS_IN_RANGE"}}) {
      if ((imu_status_ & mask) == 0U) {
        failed += (failed.empty() ? "" : ", ") + std::string{name};
      }
    }
    if (!failed.empty()) {
      c.level = HealthLevel::Error;
      c.message = "IMU communication, self-test or range failure: " + failed;
      c.values.emplace_back("imu.failed_checks", std::move(failed));
    }
  }
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_temperature(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  constexpr std::string_view metric = "temp.imu_c";
  Freshness f = freshness(imu_cell_, "imu", "IMU log (IMU_SHORT / IMU_DATA)", now, stream);
  const auto mark_unavailable = [&](HealthComponent & c) {
    c.values.emplace_back(std::string{metric}, HealthNull{});
    c.values.emplace_back(
      std::format("health.metric.{}.state", metric), std::string{k_unavailable});
  };

  HealthComponent c;
  if (temperature_read_failed_ && !temperature_c_) {
    c = make("temperature", HealthLevel::Warn, "measurement unavailable", hardware_id());
    mark_unavailable(c);
  } else if (!imu_cell_.ever) {
    c = make(
      "temperature", HealthLevel::Stale,
      f.availability == k_paused ? "not observed yet" : "waiting for an IMU log", hardware_id());
    // Declared missing only once it is overdue; in standby nothing is missing.
    if (!f.error.empty()) {
      mark_unavailable(c);
    }
  } else {
    const double value = round3(*temperature_c_);
    HealthLevel level = HealthLevel::Ok;
    std::string message = "OK";
    apply_limits(settings_.temp_imu_c, metric, value, level, message);
    c = make("temperature", level, std::move(message), hardware_id());
    c.values.emplace_back(std::string{metric}, value);
  }
  if (temperature_read_failed_) {
    f.error = "non-finite temperature reading";
    if (f.availability != k_paused) {
      f.availability = temperature_c_ ? k_stale : k_unavailable;
    }
    if (temperature_c_) {
      c.message += " — last known; " + f.error;
      c.values.emplace_back("health.metric.temp.imu_c.state", std::string{k_unavailable});
    }
  }
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_ekf(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  constexpr std::string_view log_name = "SBG_ECOM_LOG_EKF_NAV";
  const Freshness f = freshness(ekf_cell_, "ekf", log_name, now, stream);
  if (!ekf_cell_.ever) {
    auto c = make(
      "ekf", HealthLevel::Stale,
      f.availability == k_paused ? "not observed yet" : std::format("waiting for {}", log_name),
      hardware_id());
    finish(c, f);
    out.push_back(std::move(c));
    return;
  }

  // Solution mode: 0 uninitialized, 1 vertical gyro, 2 AHRS, 3 nav-velocity,
  // 4 nav-position (full INS).
  const std::uint32_t mode = ekf_status_ & k_ekf_mode_mask;
  HealthLevel level = HealthLevel::Ok;
  std::string name;
  std::string message;
  switch (mode) {
    case 0:
      level = HealthLevel::Error;
      name = "UNINITIALIZED";
      message = "EKF uninitialized";
      break;
    case 1:
      level = HealthLevel::Warn;
      name = "VERTICAL_GYRO";
      message = "vertical gyro only (no heading, no navigation)";
      break;
    case 2:
      level = HealthLevel::Warn;
      name = "AHRS";
      message = "AHRS only (no navigation)";
      break;
    case 3:
      name = message = "NAV_VELOCITY";
      break;
    case 4:
      name = message = "NAV_POSITION";
      break;
    default:
      name = message = std::format("MODE_{}", mode);
      break;
  }

  // A navigation mode cannot override invalid estimates. Keep errors ahead
  // of heading warnings so a missing heading never hides a position fault.
  if (mode != 0 && (ekf_status_ & k_ekf_attitude_valid) == 0) {
    level = HealthLevel::Error;
    message = "EKF attitude is invalid";
  } else if (mode == 4 && (ekf_status_ & k_ekf_position_valid) == 0) {
    level = HealthLevel::Error;
    message = "EKF position is invalid";
  } else if (mode >= 3 && mode <= 4 && (ekf_status_ & k_ekf_velocity_valid) == 0) {
    level = HealthLevel::Error;
    message = "EKF velocity is invalid";
  } else if (mode != 0 && (ekf_status_ & k_ekf_heading_valid) == 0) {
    level = HealthLevel::Warn;
    message = "EKF heading is invalid";
  } else if (mode > 4) {
    level = HealthLevel::Warn;
  }

  auto c = make("ekf", level, std::move(message), hardware_id());
  c.values.emplace_back("ekf.solution_mode", std::move(name));
  c.values.emplace_back("ekf.status", hex32(ekf_status_));
  for (const auto & [key, mask] :
       {std::pair{"ekf.attitude_valid", k_ekf_attitude_valid},
        std::pair{"ekf.heading_valid", k_ekf_heading_valid},
        std::pair{"ekf.velocity_valid", k_ekf_velocity_valid},
        std::pair{"ekf.position_valid", k_ekf_position_valid},
        std::pair{"ekf.gps1_vel_used", k_ekf_gps1_vel_used},
        std::pair{"ekf.gps1_pos_used", k_ekf_gps1_pos_used},
        std::pair{"ekf.gps1_hdt_used", k_ekf_gps1_hdt_used},
        std::pair{"ekf.mag_ref_used", k_ekf_mag_ref_used},
        std::pair{"ekf.align_valid", k_ekf_align_valid}}) {
    c.values.emplace_back(key, (ekf_status_ & mask) != 0U);
  }
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_gnss(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  if (!gnss_cell_.ever) {
    return;
  }
  const Freshness f = freshness(gnss_cell_, "gnss", "SBG_ECOM_LOG_GPS1_POS", now, stream);
  const bool computed =
    gnss_.solution_status == k_gnss_sol_computed && gnss_.fix_type != k_gnss_type_no_solution;
  const bool rtk_fixed = computed && gnss_.fix_type == k_gnss_type_rtk_int;
  const bool rtk_float = computed && gnss_.fix_type == k_gnss_type_rtk_float;

  std::string message;
  if (!computed) {
    message = std::format(
      "no position solution ({})", gnss_.solution_status == k_gnss_sol_computed
                                     ? gnss_type_name(gnss_.fix_type)
                                     : gnss_status_name(gnss_.solution_status));
  } else if (rtk_fixed) {
    message = "RTK fixed";
  } else if (rtk_float) {
    message = "RTK float";
  } else {
    message = std::format("{} fix", gnss_type_name(gnss_.fix_type));
  }

  using Required = HealthSettings::GnssFix;
  HealthLevel level = HealthLevel::Ok;
  std::string_view required = "none";
  bool met = true;
  switch (settings_.required_gnss_fix) {
    case Required::None:
      break;
    case Required::Fix:
      required = "fix";
      met = computed;
      break;
    case Required::RtkFloat:
      required = "rtk_float";
      met = rtk_float || rtk_fixed;
      break;
    case Required::RtkFixed:
      required = "rtk_fixed";
      met = rtk_fixed;
      break;
  }
  if (!met) {
    level = HealthLevel::Warn;
    if (computed) {
      message += std::format(" — below the required {}", required);
    }
  }

  auto c = make("gnss", level, std::move(message), hardware_id());
  c.values.emplace_back("gnss.fix_type", gnss_type_name(gnss_.fix_type));
  c.values.emplace_back("gnss.solution_status", gnss_status_name(gnss_.solution_status));
  const std::string rtk = rtk_fixed ? "fixed" : rtk_float ? "float" : "none";
  c.values.emplace_back("gnss.rtk", rtk);
  c.values.emplace_back("gnss.required_fix", std::string{required});
  // 0xFF / 0xFFFF are the vendor's "not available" encodings.
  if (gnss_.sv_used != 0xFFU) {
    c.values.emplace_back("gnss.sv_used", static_cast<std::int64_t>(gnss_.sv_used));
  }
  if (gnss_.differential_age_cs != 0xFFFFU) {
    c.values.emplace_back("gnss.differential_age_s", round3(gnss_.differential_age_cs * 0.01));
  }
  if (gnss_.base_station_id != 0xFFFFU) {
    c.values.emplace_back("gnss.base_station_id", static_cast<std::int64_t>(gnss_.base_station_id));
  }
  if (computed) {
    for (const auto & [key, value] :
         {std::pair{"gnss.latitude_accuracy_m", gnss_.latitude_accuracy_m},
          std::pair{"gnss.longitude_accuracy_m", gnss_.longitude_accuracy_m},
          std::pair{"gnss.altitude_accuracy_m", gnss_.altitude_accuracy_m}}) {
      if (std::isfinite(value)) {
        c.values.emplace_back(key, round3(value));
      }
    }
  }
  finish(c, f);
  out.push_back(std::move(c));
}

void HealthReporter::add_gnss_heading(
  std::vector<HealthComponent> & out, TimePoint now, const Stream & stream)
{
  if (!heading_cell_.ever) {
    return;
  }
  const Freshness f = freshness(heading_cell_, "hdt", "SBG_ECOM_LOG_GPS1_HDT", now, stream);
  const bool valid = heading_.solution_status == k_gnss_sol_computed && heading_.heading_valid &&
                     std::isfinite(heading_.heading_deg) && std::isfinite(heading_.accuracy_deg);

  auto c = make(
    "gnss heading", valid ? HealthLevel::Ok : HealthLevel::Warn,
    valid
      ? std::format("{:.1f}° ± {:.2f}°", heading_.heading_deg, heading_.accuracy_deg)
      : std::format(
          "no dual-antenna heading solution ({})", heading_.solution_status == k_gnss_sol_computed
                                                     ? std::string{"accuracy not valid"}
                                                     : gnss_status_name(heading_.solution_status)),
    hardware_id());
  c.values.emplace_back("gnss.heading_status", gnss_status_name(heading_.solution_status));
  c.values.emplace_back("gnss.heading_valid", valid);
  if (valid) {
    c.values.emplace_back("gnss.heading_deg", round3(heading_.heading_deg));
    c.values.emplace_back("gnss.heading_accuracy_deg", round3(heading_.accuracy_deg));
  }
  if (heading_.baseline_valid && std::isfinite(heading_.baseline_m)) {
    c.values.emplace_back("gnss.baseline_m", round3(heading_.baseline_m));
  }
  if (heading_.sv_used != 0xFFU) {
    c.values.emplace_back("gnss.heading_sv_used", static_cast<std::int64_t>(heading_.sv_used));
  }
  finish(c, f);
  out.push_back(std::move(c));
}

std::vector<HealthComponent> HealthReporter::snapshot(const StreamCounters & counters)
{
  const std::lock_guard lock{mutex_};
  const auto now = clock_();
  ++sequence_;
  const Stream stream = stream_at(now);

  std::vector<HealthComponent> out;
  out.reserve(8 + k_health_alerts.size());
  add_service(out, now);
  add_stream(out, now, stream, counters);
  add_device(out, now, stream);
  add_imu(out, now, stream);
  add_temperature(out, now, stream);
  add_ekf(out, now, stream);
  add_gnss(out, now, stream);
  add_gnss_heading(out, now, stream);
  return out;
}

}  // namespace sbg_driver
