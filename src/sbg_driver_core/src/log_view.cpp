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

#include "sbg/log_view.hpp"

namespace sbg
{

namespace
{

[[nodiscard]] LogView::Kind classify(std::uint8_t msg_class, std::uint16_t msg_id) noexcept
{
  // We only handle the standard binary log class for now. NMEA-out and
  // CAN classes are out of scope until later phases.
  if (msg_class != SBG_ECOM_CLASS_LOG_ECOM_0) {
    return LogView::Kind::Unknown;
  }

  using enum LogView::Kind;
  switch (msg_id) {
    case SBG_ECOM_LOG_IMU_DATA:
      return ImuData;
    case SBG_ECOM_LOG_IMU_SHORT:
      return ImuShort;
    case SBG_ECOM_LOG_EKF_EULER:
      return EkfEuler;
    case SBG_ECOM_LOG_EKF_QUAT:
      return EkfQuat;
    case SBG_ECOM_LOG_EKF_NAV:
      return EkfNav;
    case SBG_ECOM_LOG_EKF_VEL_BODY:
      return EkfVelBody;
    case SBG_ECOM_LOG_MAG:
      return Mag;
    case SBG_ECOM_LOG_GPS1_POS:
    case SBG_ECOM_LOG_GPS2_POS:
      return GnssPos;
    case SBG_ECOM_LOG_GPS1_VEL:
    case SBG_ECOM_LOG_GPS2_VEL:
      return GnssVel;
    case SBG_ECOM_LOG_GPS1_HDT:
    case SBG_ECOM_LOG_GPS2_HDT:
      return GnssHdt;
    case SBG_ECOM_LOG_AIR_DATA:
      return AirData;
    case SBG_ECOM_LOG_UTC_TIME:
      return Utc;
    case SBG_ECOM_LOG_STATUS:
      return Status;
    case SBG_ECOM_LOG_MAG_CALIB:
      return MagCalib;
    case SBG_ECOM_LOG_SHIP_MOTION:
    case SBG_ECOM_LOG_SHIP_MOTION_HP:
      return ShipMotion;
    case SBG_ECOM_LOG_EVENT_A:
    case SBG_ECOM_LOG_EVENT_B:
    case SBG_ECOM_LOG_EVENT_C:
    case SBG_ECOM_LOG_EVENT_D:
    case SBG_ECOM_LOG_EVENT_E:
      return Event;
    case SBG_ECOM_LOG_GPS1_RAW:
    case SBG_ECOM_LOG_GPS2_RAW:
      return GpsRawData;
    default:
      return Unknown;
  }
}

[[nodiscard]] std::uint32_t extract_time_stamp(
  LogView::Kind kind, const SbgEComLogUnion * u) noexcept
{
  if (u == nullptr) {
    return 0;
  }
  using enum LogView::Kind;
  switch (kind) {
    case ImuData:
      return u->imuData.timeStamp;
    case ImuShort:
      return u->imuShort.timeStamp;
    case EkfEuler:
      return u->ekfEulerData.timeStamp;
    case EkfQuat:
      return u->ekfQuatData.timeStamp;
    case EkfNav:
      return u->ekfNavData.timeStamp;
    case EkfVelBody:
      return u->ekfVelBody.timeStamp;
    case Mag:
      return u->magData.timeStamp;
    case GnssPos:
      return u->gpsPosData.timeOfWeek;
    case GnssVel:
      return u->gpsVelData.timeOfWeek;
    case GnssHdt:
      return u->gpsHdtData.timeOfWeek;
    case AirData:
      return u->airData.timeStamp;
    case Utc:
      return u->utcData.timeStamp;
    case Status:
      return u->statusData.timeStamp;
    case MagCalib:
      return u->magCalibData.timeStamp;
    case ShipMotion:
      return u->shipMotionData.timeStamp;
    case Event:
      return u->eventMarker.timeStamp;
    case GpsRawData:
      return 0;  // SbgEComLogRawData has no timestamp field
    case Unknown:
      return 0;
  }
  return 0;
}

}  // namespace

LogView::LogView(
  std::uint8_t msg_class, std::uint16_t msg_id, const SbgEComLogUnion * log_data) noexcept
: kind_(classify(msg_class, msg_id)),
  log_(log_data),
  time_stamp_us_(extract_time_stamp(kind_, log_data))
{
}

const SbgEComLogImuLegacy * LogView::as_imu_data() const noexcept
{
  return (kind_ == Kind::ImuData && log_ != nullptr) ? &log_->imuData : nullptr;
}
const SbgEComLogImuShort * LogView::as_imu_short() const noexcept
{
  return (kind_ == Kind::ImuShort && log_ != nullptr) ? &log_->imuShort : nullptr;
}
const SbgEComLogEkfEuler * LogView::as_ekf_euler() const noexcept
{
  return (kind_ == Kind::EkfEuler && log_ != nullptr) ? &log_->ekfEulerData : nullptr;
}
const SbgEComLogEkfQuat * LogView::as_ekf_quat() const noexcept
{
  return (kind_ == Kind::EkfQuat && log_ != nullptr) ? &log_->ekfQuatData : nullptr;
}
const SbgEComLogEkfNav * LogView::as_ekf_nav() const noexcept
{
  return (kind_ == Kind::EkfNav && log_ != nullptr) ? &log_->ekfNavData : nullptr;
}
const SbgEComLogEkfVelBody * LogView::as_ekf_vel_body() const noexcept
{
  return (kind_ == Kind::EkfVelBody && log_ != nullptr) ? &log_->ekfVelBody : nullptr;
}
const SbgEComLogMag * LogView::as_mag() const noexcept
{
  return (kind_ == Kind::Mag && log_ != nullptr) ? &log_->magData : nullptr;
}
const SbgEComLogGnssPos * LogView::as_gnss_pos() const noexcept
{
  return (kind_ == Kind::GnssPos && log_ != nullptr) ? &log_->gpsPosData : nullptr;
}
const SbgEComLogGnssVel * LogView::as_gnss_vel() const noexcept
{
  return (kind_ == Kind::GnssVel && log_ != nullptr) ? &log_->gpsVelData : nullptr;
}
const SbgEComLogGnssHdt * LogView::as_gnss_hdt() const noexcept
{
  return (kind_ == Kind::GnssHdt && log_ != nullptr) ? &log_->gpsHdtData : nullptr;
}
const SbgEComLogAirData * LogView::as_air_data() const noexcept
{
  return (kind_ == Kind::AirData && log_ != nullptr) ? &log_->airData : nullptr;
}
const SbgEComLogUtc * LogView::as_utc() const noexcept
{
  return (kind_ == Kind::Utc && log_ != nullptr) ? &log_->utcData : nullptr;
}
const SbgEComLogStatus * LogView::as_status() const noexcept
{
  return (kind_ == Kind::Status && log_ != nullptr) ? &log_->statusData : nullptr;
}
const SbgEComLogShipMotion * LogView::as_ship_motion() const noexcept
{
  return (kind_ == Kind::ShipMotion && log_ != nullptr) ? &log_->shipMotionData : nullptr;
}
const SbgEComLogEvent * LogView::as_event() const noexcept
{
  return (kind_ == Kind::Event && log_ != nullptr) ? &log_->eventMarker : nullptr;
}
const SbgEComLogMagCalib * LogView::as_mag_calib() const noexcept
{
  return (kind_ == Kind::MagCalib && log_ != nullptr) ? &log_->magCalibData : nullptr;
}
const SbgEComLogRawData * LogView::as_gps_raw() const noexcept
{
  return (kind_ == Kind::GpsRawData && log_ != nullptr) ? &log_->gpsRawData : nullptr;
}

}  // namespace sbg
