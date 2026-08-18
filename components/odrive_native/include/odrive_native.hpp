#pragma once

#include <functional>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#include "base_component.hpp"
#include "detail/odrive_native_core.hpp"

namespace espp {

/**
 * @brief ODrive legacy native (Fibre endpoint) binary protocol server.
 *
 * Implements the packet-based ODrive legacy endpoint protocol (fw <= 0.5.x) as
 * used over the USB vendor interface, where each USB bulk transfer carries
 * exactly one packet. The component is transport-agnostic and performs no I/O
 * itself: feed one inbound request packet to process_bytes() and transmit the
 * returned response packet (empty when no response is expected).
 *
 * Applications register typed properties from dotted paths (mirroring
 * espp::OdriveAscii). Endpoint ids are assigned sequentially starting at 1
 * (endpoint 0 is reserved for the JSON descriptor blob), and the compact JSON
 * descriptor plus its CRC are finalized lazily on first use. This lets a legacy
 * odrivetool / fibre-python client auto-discover the object tree and perform
 * typed get/set.
 *
 * The registration API and dispatch are provided by espp::detail::
 * OdriveNativeCore, a host-buildable wire core with no ESP dependencies; this
 * class adds the espp logging identity via BaseComponent.
 *
 * See PROTOCOL.md for the authoritative wire specification.
 *
 * \section odrive_native_ex1 Basic Example
 * \snippet odrive_native_example.cpp odrive_native_basic_example
 */
class OdriveNative : public BaseComponent, public detail::OdriveNativeCore {
public:
  /**
   * @brief Configuration for the OdriveNative server.
   */
  struct Config {
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; /**< Logger verbosity. */
  };

  /**
   * @brief Create an OdriveNative protocol server.
   * @param config Configuration parameters.
   */
  explicit OdriveNative(const Config &config)
      : BaseComponent("ODriveNative", config.log_level) {
    // The wire protocol has no error channel; route the core's dropped-request /
    // failed-write reports through the component logger so they are observable.
    set_error_callback([this](const std::string &msg) { logger_.warn("{}", msg); });
  }

  OdriveNative()
      : OdriveNative(Config{}) {}
};

} // namespace espp
