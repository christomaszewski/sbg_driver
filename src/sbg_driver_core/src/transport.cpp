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

#include "sbg/transport.hpp"

#include <algorithm>
#include <array>
#include <cstdlib>
#include <memory>
#include <utility>

#include "sbg/detail/c_api.hpp"

namespace sbg
{

// SbgInterface is the C SDK's interface struct. We carry it heap-allocated
// inside a TransportImpl so the public Transport class can forward-declare
// it without leaking sbgECom into the header. The destructor releases the
// underlying SDK interface, so the owning unique_ptr in Transport needs no
// custom deleter and all of Transport's special members can be defaulted.
struct TransportImpl
{
  SbgInterface iface{};

  TransportImpl() = default;
  ~TransportImpl() { sbgInterfaceDestroy(&iface); }
  TransportImpl(const TransportImpl &) = delete;
  TransportImpl & operator=(const TransportImpl &) = delete;
  TransportImpl(TransportImpl &&) = delete;
  TransportImpl & operator=(TransportImpl &&) = delete;
};

namespace
{

constexpr std::array<std::uint32_t, 8> k_supported_bauds{
  9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600,
};

[[nodiscard]] constexpr bool baud_supported(std::uint32_t baud) noexcept
{
  return std::ranges::any_of(k_supported_bauds, [baud](std::uint32_t b) { return b == baud; });
}

using TransportImplPtr = std::unique_ptr<TransportImpl>;

Result<TransportImplPtr> open_serial(const transport::Serial & s) noexcept
{
  auto impl = std::make_unique<TransportImpl>();
  if (auto code = sbgInterfaceSerialCreate(&impl->iface, s.port.c_str(), s.baud);
      code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return impl;
}

Result<TransportImplPtr> open_udp(const transport::Udp & u) noexcept
{
  auto impl = std::make_unique<TransportImpl>();
  // sbgNetworkIpFromString returns the address directly (0 on failure / "0.0.0.0").
  // The SDK doesn't distinguish parse errors from a legitimate 0.0.0.0, so we
  // do a pre-check for the obvious empty-string case in validate().
  sbgIpAddress addr = sbgNetworkIpFromString(u.remote_ip.c_str());
  if (auto code = sbgInterfaceUdpCreate(&impl->iface, addr, u.out_port, u.in_port);
      code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return impl;
}

Result<TransportImplPtr> open_file(const transport::FileReplay & f) noexcept
{
  auto impl = std::make_unique<TransportImpl>();
  if (auto code = sbgInterfaceFileOpen(&impl->iface, f.path.c_str()); code != SBG_NO_ERROR) {
    return std::unexpected(detail::from_sbg(code));
  }
  return impl;
}

}  // namespace

// ---------------------------------------------------------------------------
// validate()
// ---------------------------------------------------------------------------

Result<void> validate(const TransportConfig & cfg) noexcept
{
  return std::visit(
    [](const auto & concrete) -> Result<void> {
      using T = std::decay_t<decltype(concrete)>;
      if constexpr (std::is_same_v<T, transport::Serial>) {
        if (concrete.port.empty()) {
          return std::unexpected(Error::InvalidParameter);
        }
        if (!baud_supported(concrete.baud)) {
          return std::unexpected(Error::InvalidParameter);
        }
        for (std::uint32_t fb : concrete.fallback_bauds) {
          if (!baud_supported(fb)) {
            return std::unexpected(Error::InvalidParameter);
          }
        }
        return {};
      } else if constexpr (std::is_same_v<T, transport::Udp>) {
        if (concrete.remote_ip.empty()) {
          return std::unexpected(Error::InvalidParameter);
        }
        if (concrete.in_port == 0 || concrete.out_port == 0) {
          return std::unexpected(Error::InvalidParameter);
        }
        return {};
      } else if constexpr (std::is_same_v<T, transport::FileReplay>) {
        if (concrete.path.empty()) {
          return std::unexpected(Error::InvalidParameter);
        }
        return {};
      } else {
        static_assert(sizeof(T) == 0, "unhandled transport variant");
      }
    },
    cfg);
}

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

Result<Transport> Transport::open(TransportConfig cfg)
{
  if (auto v = validate(cfg); !v) {
    return std::unexpected(v.error());
  }

  auto impl_result = std::visit(
    [](const auto & concrete) -> Result<TransportImplPtr> {
      using T = std::decay_t<decltype(concrete)>;
      if constexpr (std::is_same_v<T, transport::Serial>) {
        return open_serial(concrete);
      } else if constexpr (std::is_same_v<T, transport::Udp>) {
        return open_udp(concrete);
      } else if constexpr (std::is_same_v<T, transport::FileReplay>) {
        return open_file(concrete);
      }
    },
    cfg);

  if (!impl_result) {
    return std::unexpected(impl_result.error());
  }

  return Transport{std::move(*impl_result), std::move(cfg)};
}

Transport::Transport(std::unique_ptr<TransportImpl> impl, TransportConfig cfg) noexcept
: impl_(std::move(impl)), cfg_(std::move(cfg))
{
}

// Out-of-line because TransportImpl is incomplete in the header; unique_ptr's
// destructor (which runs TransportImpl::~TransportImpl -> sbgInterfaceDestroy)
// is instantiated here where the type is complete.
Transport::~Transport() = default;
Transport::Transport(Transport && other) noexcept = default;

Transport & Transport::operator=(Transport && other) noexcept
{
  if (this != &other) {
    impl_ = std::move(other.impl_);
    cfg_ = std::move(other.cfg_);
  }
  return *this;
}

void * Transport::native_handle() noexcept
{
  return impl_ != nullptr ? static_cast<void *>(&impl_->iface) : nullptr;
}

}  // namespace sbg
