#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>
#include <span>

#include <sdkconfig.h>

#include <esp_mac.h>
#include <esp_random.h>
#include <nvs_flash.h>

#include "nearby_fp_client.h"
#include "nearby_fp_library.h"
#include "nearby_platform_audio.h"
#include "nearby_platform_battery.h"
#include "nearby_platform_ble.h"
#include "nearby_platform_bt.h"
#include "nearby_platform_os.h"
#include "nearby_platform_persistence.h"
#include "nearby_platform_se.h"
#include "nearby_platform_trace.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "high_resolution_timer.hpp"
#include "logger.hpp"
#include "task.hpp"

#if CONFIG_BT_NIMBLE_ENABLED
#include "NimBLEDevice.h"
#endif

namespace espp {
namespace gfps {

/// Function which GFPS will call to send notifications to the remote device
/// @param characteristic The characteristic to notify
/// @param data The data to send
/// @param length The length of the data
typedef std::function<bool(nearby_fp_Characteristic, const uint8_t *, size_t)> notify_callback_t;

/// Callback for assigning a passkey to the pairing peer
/// @param passkey The passkey value to be injected
typedef std::function<void(uint32_t)> set_passkey_callback_t;

/// Callback invoked when an Account Key is written by a remote device
/// @param peer_addr BLE address of the remote device
/// @param key Pointer to the 16-byte Account Key data
typedef std::function<void(uint64_t, const std::span<uint8_t, 16>&)> account_key_write_callback_t;

/// Callback triggered when the GFPS layer finishes constructing a Non-Discoverable Advertisement payload
/// @param adv_data Pointer to raw advertisement payload bytes
/// @param len Length of the advertisement data
typedef std::function<void(const std::span<const uint8_t>&)> nda_ready_callback_t;

/// Configuration for the Google Fast Pair Service
struct Config {
  notify_callback_t notify; ///< Callback to enable gfps to notify the remote device of changes

  /// Optional callback to handle passkey confirmation during pairing.
  /// If not provided, GFPS will default to using the first connected peer.
  /// The callback should call NimBLEDevice::injectConfirmPasskey with the correct peer if available.
  set_passkey_callback_t set_passkey_callback = nullptr;

  /// Optional callback for when an Account Key is written by the remote device.
  account_key_write_callback_t on_account_key_write_callback = nullptr;

  /// Optional callback triggered when the GFPS layer requests to initiate Non-Discoverable Advertising (NDA).
  nda_ready_callback_t on_non_discoverable_advertisement_ready_callback = nullptr;
};

/// Initialize the Google Fast Pair Service
/// @param config The configuration for the service
void init(const Config &config);

/// Deinitialize the Google Fast Pair Service
void deinit();

/// Get the GFPS model ID
uint32_t get_model_id();

/// Set the remote public key
/// @param public_key The public key
void set_remote_public_key(const std::vector<uint8_t> &public_key);

/// Set the remote public key
/// @param public_key The public key
/// @param length The length of the public key
void set_remote_public_key(const uint8_t *public_key, size_t length);

/// Get the remote public key
/// @return The remote public key
std::vector<uint8_t> get_remote_public_key();

/// Get the underlying GFPS BLE interface
const nearby_platform_BleInterface *get_ble_interface();

/// Get the underlying GFPS BT interface
const nearby_platform_BtInterface *get_bt_interface();

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
int ble_gap_event_handler(ble_gap_event *event, void *arg);
#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

static constexpr espp::Logger::Verbosity LOG_LEVEL =
#if CONFIG_GFPS_TRACE_LEVEL <= 2
    espp::Logger::Verbosity::DEBUG;
#elif CONFIG_GFPS_TRACE_LEVEL == 3
    espp::Logger::Verbosity::INFO;
#elif CONFIG_GFPS_TRACE_LEVEL == 4
    espp::Logger::Verbosity::WARN;
#elif CONFIG_GFPS_TRACE_LEVEL == 5
    espp::Logger::Verbosity::ERROR;
#else
    espp::Logger::Verbosity::NONE;
#endif

} // namespace gfps
} // namespace espp

#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
// for printing of BLE_GAP_EVENT_ using libfmt
template <> struct fmt::formatter<ble_gap_event> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const ble_gap_event &event, FormatContext &ctx) const {
    switch (event.type) {
    case BLE_GAP_EVENT_CONNECT:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_CONNECT");
    case BLE_GAP_EVENT_DISCONNECT:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_DISCONNECT");
    case BLE_GAP_EVENT_CONN_UPDATE:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_CONN_UPDATE");
    case BLE_GAP_EVENT_ENC_CHANGE:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_ENC_CHANGE");
    case BLE_GAP_EVENT_PASSKEY_ACTION:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_PASSKEY_ACTION");
    case BLE_GAP_EVENT_SUBSCRIBE:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_SUBSCRIBE");
    case BLE_GAP_EVENT_MTU:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_MTU");
    case BLE_GAP_EVENT_REPEAT_PAIRING:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_REPEAT_PAIRING");
    default:
      return fmt::format_to(ctx.out(), "BLE_GAP_EVENT_UNKNOWN ({})", (int)event.type);
    }
  }
};
#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

// for pritning of nearby_fp_Characteristic using libfmt
template <> struct fmt::formatter<nearby_fp_Characteristic> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }

  template <typename FormatContext>
  auto format(const nearby_fp_Characteristic &characteristic, FormatContext &ctx) const {
    switch (characteristic) {
    case kModelId:
      return fmt::format_to(ctx.out(), "kModelId");
    case kKeyBasedPairing:
      return fmt::format_to(ctx.out(), "kKeyBasedPairing");
    case kPasskey:
      return fmt::format_to(ctx.out(), "kPasskey");
    case kAccountKey:
      return fmt::format_to(ctx.out(), "kAccountKey");
    default:
      return fmt::format_to(ctx.out(), "nearby_fp_Characteristic_UNKNOWN ({})",
                            (int)characteristic);
    }
  }
};
