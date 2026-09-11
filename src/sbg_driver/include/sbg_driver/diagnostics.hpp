// Copyright 2026 sbg_driver maintainers
// SPDX-License-Identifier: Apache-2.0
#pragma once

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>

#include "sbg_driver/publishers.hpp"

namespace sbg_driver
{
inline std::int64_t steady_now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

void diagnose_device(
  const Publishers::DiagSnapshot & snapshot, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & status);
void diagnose_ekf(
  const Publishers::DiagSnapshot & snapshot, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & status);
void diagnose_temperature(
  const Publishers::DiagSnapshot & snapshot, std::int64_t now,
  diagnostic_updater::DiagnosticStatusWrapper & status);
}  // namespace sbg_driver
