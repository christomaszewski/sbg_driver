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

#include <fcntl.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdlib>
#include <limits>
#include <memory>
#include <mutex>
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
  int serial_fd = -1;
  std::uint32_t serial_baud = 0;
  std::timed_mutex write_mutex;

  SbgErrorCode write_serial(
    const void * data, std::size_t size, std::stop_token stop = {},
    std::chrono::milliseconds timeout = std::chrono::milliseconds{100})
  {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    std::unique_lock lock{write_mutex, std::defer_lock};
    while (!lock.try_lock()) {
      if (stop.stop_requested()) {
        return SBG_OPERATION_CANCELLED;
      }
      if (std::chrono::steady_clock::now() >= deadline) {
        return SBG_TIME_OUT;
      }
      lock.try_lock_for(std::chrono::milliseconds{1});
      if (lock.owns_lock()) {
        break;
      }
    }
    const auto * cursor = static_cast<const std::byte *>(data);
    while (size > 0) {
      if (stop.stop_requested()) {
        return SBG_OPERATION_CANCELLED;
      }
      const auto remaining = deadline - std::chrono::steady_clock::now();
      if (remaining <= std::chrono::steady_clock::duration::zero()) {
        return SBG_TIME_OUT;
      }
      // Nonblocking writes handle partial progress, interruptions and a full
      // kernel transmit queue without an unbounded EAGAIN retry loop.
      const auto written = ::write(serial_fd, cursor, size);
      if (written > 0) {
        const auto count = static_cast<std::size_t>(written);
        cursor += count;
        size -= count;
        continue;
      }
      if (written < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        return SBG_WRITE_ERROR;
      }
      struct pollfd descriptor{serial_fd, POLLOUT, 0};
      const auto remaining_ms = std::chrono::ceil<std::chrono::milliseconds>(remaining).count();
      const int wait_ms = static_cast<int>(std::min<std::int64_t>(remaining_ms, 10));
      const int ready = ::poll(&descriptor, 1, wait_ms);
      if (
        (ready < 0 && errno != EINTR) ||
        (ready > 0 && (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0)) {
        return SBG_WRITE_ERROR;
      }
    }
    return SBG_NO_ERROR;
  }

  TransportImpl() = default;
  ~TransportImpl()
  {
    if (serial_fd >= 0) {
      ::close(serial_fd);
    } else {
      sbgInterfaceDestroy(&iface);
    }
  }
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

// The SDK reports most create failures as the generic SBG_ERROR ("could not
// open the device/socket/file"); detail::from_sbg would surface that as
// ProtocolError, which misleads (nothing protocol-level happened yet). At the
// open boundary the right name for any failure is TransportFailure; richer
// codes (e.g. SBG_INVALID_ARGUMENT) still map through normally.
[[nodiscard]] Error open_error(SbgErrorCode code) noexcept
{
  return code == SBG_ERROR ? Error::TransportFailure : detail::from_sbg(code);
}

[[nodiscard]] speed_t serial_speed(std::uint32_t baud) noexcept
{
  switch (baud) {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
#ifdef B460800
    case 460800:
      return B460800;
#endif
#ifdef B921600
    case 921600:
      return B921600;
#endif
    default:
      return 0;
  }
}

SbgErrorCode set_serial_speed(SbgInterface * iface, std::uint32_t baud)
{
  auto & impl = *static_cast<TransportImpl *>(iface->handle);
  const auto speed = serial_speed(baud);
  if (speed == 0) {
    return SBG_INVALID_PARAMETER;
  }
  struct termios settings{};
  if (::tcgetattr(impl.serial_fd, &settings) != 0) {
    return SBG_ERROR;
  }
  ::cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
  settings.c_cflag &= static_cast<tcflag_t>(~(CSTOPB | PARENB | CRTSCTS));
  settings.c_cc[VMIN] = 0;
  settings.c_cc[VTIME] = 0;
  if (
    ::cfsetispeed(&settings, speed) != 0 || ::cfsetospeed(&settings, speed) != 0 ||
    ::tcsetattr(impl.serial_fd, TCSANOW, &settings) != 0) {
    return SBG_ERROR;
  }
  impl.serial_baud = baud;
  return SBG_NO_ERROR;
}

Result<TransportImplPtr> open_serial(const transport::Serial & s) noexcept
{
  auto impl = std::make_unique<TransportImpl>();
  // Own the POSIX serial implementation through the SDK's public interface
  // callbacks. The SDK serial writer retries EAGAIN forever; replacing it
  // here also bounds configuration command writes and works with system SDKs.
  impl->serial_fd = ::open(s.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
  if (impl->serial_fd < 0) {
    return std::unexpected(Error::TransportFailure);
  }
  impl->iface.handle = impl.get();
  impl->iface.type = SBG_IF_TYPE_SERIAL;
  sbgInterfaceNameSet(&impl->iface, s.port.c_str());
  impl->iface.pSetSpeedFunc = &set_serial_speed;
  if (auto code = set_serial_speed(&impl->iface, s.baud); code != SBG_NO_ERROR) {
    return std::unexpected(open_error(code));
  }
  // Discard stale bytes from a previous session without waiting for a
  // stalled transmit queue to drain during activation.
  if (::tcflush(impl->serial_fd, TCIOFLUSH) != 0) {
    return std::unexpected(Error::TransportFailure);
  }
  impl->iface.pWriteFunc = [](SbgInterface * iface, const void * data, std::size_t size) {
    return static_cast<TransportImpl *>(iface->handle)->write_serial(data, size);
  };
  impl->iface.pReadFunc =
    [](SbgInterface * iface, void * data, std::size_t * count, std::size_t size) {
    auto & transport = *static_cast<TransportImpl *>(iface->handle);
    const auto received = ::read(transport.serial_fd, data, size);
    *count = received > 0 ? static_cast<std::size_t>(received) : 0;
    return received >= 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR
           ? SBG_NO_ERROR
           : SBG_READ_ERROR;
  };
  impl->iface.pGetSpeedFunc = [](const SbgInterface * iface) {
    return static_cast<const TransportImpl *>(iface->handle)->serial_baud;
  };
  impl->iface.pDelayFunc = [](const SbgInterface * iface, std::size_t size) {
    const auto baud = static_cast<const TransportImpl *>(iface->handle)->serial_baud;
    const double us = static_cast<double>(size) * 10'000'000.0 / static_cast<double>(baud);
    return static_cast<std::uint32_t>(
      std::min(us, static_cast<double>(std::numeric_limits<std::uint32_t>::max())));
  };
  // SDK flush of output means drain, not discard. Bound that operation too.
  impl->iface.pFlushFunc = [](SbgInterface * iface, std::uint32_t flags) {
    const int fd = static_cast<const TransportImpl *>(iface->handle)->serial_fd;
    if ((flags & SBG_IF_FLUSH_INPUT) != 0 && ::tcflush(fd, TCIFLUSH) != 0) {
      return SBG_READ_ERROR;
    }
    if ((flags & SBG_IF_FLUSH_OUTPUT) != 0) {
      const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds{100};
      int pending = 0;
      do {
        if (::ioctl(fd, TIOCOUTQ, &pending) != 0) {
          return SBG_WRITE_ERROR;
        }
        if (pending == 0) {
          return SBG_NO_ERROR;
        }
        if (std::chrono::steady_clock::now() >= deadline) {
          return SBG_TIME_OUT;
        }
        // poll without descriptors is an interruptible, bounded sleep.
        ::poll(nullptr, 0, 1);
      } while (pending > 0);
    }
    return SBG_NO_ERROR;
  };
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
    return std::unexpected(open_error(code));
  }
  // The SDK creates the socket UNconnected (any host may inject frames into
  // our listen port). Enable connected mode so reads only accept datagrams
  // from remote_ip:out_port — the behavior transport.hpp documents.
  sbgInterfaceUdpSetConnectedMode(&impl->iface, true);
  return impl;
}

Result<TransportImplPtr> open_file(const transport::FileReplay & f) noexcept
{
  auto impl = std::make_unique<TransportImpl>();
  if (auto code = sbgInterfaceFileOpen(&impl->iface, f.path.c_str()); code != SBG_NO_ERROR) {
    return std::unexpected(open_error(code));
  }
  return impl;
}

}  // namespace

// ---------------------------------------------------------------------------
// validate()
// ---------------------------------------------------------------------------

Result<void> validate(const TransportConfig & cfg) noexcept
{
  return std::visit([](const auto & concrete) -> Result<void> {
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
  }, cfg);
}

// ---------------------------------------------------------------------------
// Transport
// ---------------------------------------------------------------------------

Result<Transport> Transport::open(TransportConfig cfg)
{
  if (auto v = validate(cfg); !v) {
    return std::unexpected(v.error());
  }

  auto impl_result = std::visit([](const auto & concrete) -> Result<TransportImplPtr> {
    using T = std::decay_t<decltype(concrete)>;
    if constexpr (std::is_same_v<T, transport::Serial>) {
      return open_serial(concrete);
    } else if constexpr (std::is_same_v<T, transport::Udp>) {
      return open_udp(concrete);
    } else if constexpr (std::is_same_v<T, transport::FileReplay>) {
      return open_file(concrete);
    }
  }, cfg);

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

Result<void> Transport::write(
  std::span<const std::byte> data, std::stop_token stop, std::chrono::milliseconds timeout)
{
  if (impl_ == nullptr || stop.stop_requested()) {
    return std::unexpected(Error::NotReady);
  }
  if (data.empty()) {
    return {};
  }
  const auto code = impl_->serial_fd >= 0
                    ? impl_->write_serial(data.data(), data.size(), stop, timeout)
                    : sbgInterfaceWrite(&impl_->iface, data.data(), data.size());
  if (code == SBG_OPERATION_CANCELLED) {
    return std::unexpected(Error::NotReady);
  }
  return detail::check(code);
}

}  // namespace sbg
