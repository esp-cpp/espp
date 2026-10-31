#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

/// @file switch2_pro_protocol.hpp
/// @brief Wire-protocol constants for the Nintendo Switch 2 Pro Controller BLE
///        interface (GATT UUIDs, command channel, pairing).
///
/// Protocol facts are from the community reverse-engineering effort
/// ndeadly/switch2_controller_research and the zhantss ESP32 emulator (MIT).
/// These are the values a real Switch 2 console expects; they describe an
/// interoperability interface, not Nintendo source.

namespace espp::switch2 {

// ---------------------------------------------------------------------------
// GATT UUIDs (128-bit, string form for NimBLEUUID)
// ---------------------------------------------------------------------------

/// Proprietary service 1 (purpose not fully understood).
inline constexpr const char *SERVICE1_UUID = "00c5af5d-1964-4e30-8f51-1956f96bd280";
inline constexpr const char *SERVICE1_CHR_281_UUID = "00c5af5d-1964-4e30-8f51-1956f96bd281";
inline constexpr const char *SERVICE1_CHR_282_UUID = "00c5af5d-1964-4e30-8f51-1956f96bd282";
inline constexpr const char *SERVICE1_CHR_283_UUID = "00c5af5d-1964-4e30-8f51-1956f96bd283";

/// Main HID-like service.
inline constexpr const char *SERVICE2_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd0";
/// Common input report (report id 0x05), all controller types. READ | NOTIFY.
inline constexpr const char *COMMON_INPUT_UUID = "ab7de9be-89fe-49ad-828f-118f09df7fd2";
/// Pro Controller 2 input report (report id 0x09). READ | NOTIFY.
inline constexpr const char *PRO2_INPUT_UUID = "7492866c-ec3e-4619-8258-32755ffcc0f8";
/// Vibration / HD rumble output. WRITE_NO_RSP.
inline constexpr const char *VIBRATION_UUID = "cc483f51-9258-427d-a939-630c31f72b05";
/// Command channel (basic). WRITE_NO_RSP.
inline constexpr const char *COMMAND_UUID = "649d4ac9-8eb7-4e6c-af44-1ea54fe5f005";
/// Vibration+command combined — the pairing handshake runs here. WRITE_NO_RSP.
inline constexpr const char *VIBRATION_COMMAND_UUID = "3dacbc7e-6955-40b5-8eaf-6f9809e8b379";
/// Firmware update (large writes). WRITE_NO_RSP (matches a real controller).
inline constexpr const char *FIRMWARE_UPDATE_UUID = "4147423d-fdae-4df7-a4f7-d23e5df59f8d";
/// Command response #1. NOTIFY.
inline constexpr const char *COMMAND_RESPONSE1_UUID = "c765a961-d9d8-4d36-a20a-5315b111836a";
/// Command response #2 — replies to writes on the vibration+command channel. NOTIFY.
inline constexpr const char *COMMAND_RESPONSE2_UUID = "506d9f7d-4278-4e95-a549-326ba77657e0";
/// Additional service-2 attributes a real Pro Controller 2 exposes; replicated so
/// the console's GATT discovery sees the same characteristic set (handles 0x0022,
/// 0x0026, 0x002a). Purpose unknown but their absence appears to make the console
/// reject the controller after discovery.
inline constexpr const char *UNKNOWN_INPUT1_UUID =
    "d3bd69d2-841c-4241-ab15-f86f406d2a80"; // 0x0022 NOTIFY
inline constexpr const char *UNKNOWN_INPUT2_UUID =
    "ab7de9be-89fe-49ad-828f-118f09df7fde"; // 0x0026 READ|NOTIFY
inline constexpr const char *UNKNOWN_OUTPUT_UUID =
    "ab7de9be-89fe-49ad-828f-118f09df7fdf"; // 0x002a WRITE_NR

/// Vendor descriptors a real controller attaches to its characteristics. The
/// "report rate" descriptor sits on the input-report characteristics; the other
/// on the command-response characteristics. Replicated for discovery parity.
inline constexpr const char *REPORT_RATE_DESC_UUID = "679d5510-5a24-4dee-9557-95df80486ecb";
inline constexpr const char *CMD_RESPONSE_DESC_UUID = "b746df8c-f358-495b-9cd2-e3bbeda4f979";

/// Headset-audio attributes exposed by a Pro Controller 2 that has been updated
/// from factory firmware (handles 0x002c/0x002e/0x0032). Their presence (and a
/// valid DSP version in the 0x10 firmware-info reply) is how the console tells a
/// fully-updated controller from factory firmware; without them the console
/// treats us as un-updated and diverges (probing firmware-info, rejecting).
inline constexpr const char *AUDIO_OUTPUT_UUID =
    "cc483f51-9258-427d-a939-630c31f72b06"; // 0x002c WRITE_NR
inline constexpr const char *AUDIO_INPUT_UUID =
    "7492866c-ec3e-4619-8258-32755ffcc0f9"; // 0x002e READ|NOTIFY
inline constexpr const char *AUDIO_COMMAND_UUID =
    "3dacbc7e-6955-40b5-8eaf-6f9809e8b380"; // 0x0032 WRITE_NR

// ---------------------------------------------------------------------------
// Advertising / identity
// ---------------------------------------------------------------------------

inline constexpr uint16_t NINTENDO_MANUFACTURER_ID = 0x0553;
inline constexpr uint16_t VENDOR_ID = 0x057E;       ///< Nintendo
inline constexpr uint16_t PRODUCT_ID_PRO2 = 0x2069; ///< Pro Controller 2

/// Manufacturer-specific advertising payload (AD type 0xFF) the console filters
/// on. This must byte-for-byte match a real Pro Controller 2 "standard"
/// advertisement (26 bytes, verified against the procon2 pairing capture) —
/// company id 0x0553, VID 0x057E, PID 0x2069, then fixed/flags/host-addr fields
/// and 7 trailing reserved zeros. With the 3-byte Flags AD this is exactly the
/// 31-byte legacy-advertisement limit, so the device name goes in the scan
/// response. Byte 0x0B is the wake indicator (0x00 discovery / 0x81 wake) and
/// bytes 0x0C..0x11 carry the bonded host BD_ADDR (byte-reversed); zero for
/// discovery.
inline constexpr std::array<uint8_t, 26> MANUFACTURER_DATA_DISCOVERY = {
    0x53, 0x05, 0x01, 0x00, 0x03, 0x7e, 0x05, 0x69, 0x20, 0x00, 0x01, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
inline constexpr size_t MANUFACTURER_WAKE_FLAG_OFFSET = 0x0b;
inline constexpr size_t MANUFACTURER_HOST_ADDR_OFFSET = 0x0c;
inline constexpr uint8_t WAKE_FLAG = 0x81;

// ---------------------------------------------------------------------------
// Command channel framing
// ---------------------------------------------------------------------------

/// 8-byte command header:
///   [0] command id      [1] direction   [2] transport  [3] subcommand
///   [4] (unknown)       [5] length/ACK  [6..7] 0x0000
inline constexpr size_t COMMAND_HEADER_SIZE = 8;
inline constexpr uint8_t DIR_HOST_TO_DEVICE = 0x91;
inline constexpr uint8_t DIR_DEVICE_TO_HOST = 0x01;
inline constexpr uint8_t TRANSPORT_USB = 0x00;
inline constexpr uint8_t TRANSPORT_BT = 0x01;
inline constexpr uint8_t ACK_MARKER = 0x78; ///< seen in header byte 5 of replies

/// The vibration+command channel (0x0016) carries a fixed-size vibration payload
/// BEFORE the command, so every command written to it is preceded by this many
/// 0x00 bytes (verified: all init-sequence writes on 0x0016 have a 33-byte
/// prefix). The command-only channel (0x0014) has no prefix.
inline constexpr size_t VIBRATION_COMMAND_PREFIX_SIZE = 33;
/// The command-response channel (0x001e) likewise prefixes every response with a
/// fixed 14-byte (zero) report header before the 8-byte response header.
inline constexpr size_t RESPONSE_PREFIX_SIZE = 14;
/// Header byte[4]/byte[5] for a Bluetooth response. The USB transport uses
/// 0x00/0xf8 for bare ACKs, but every Pro Controller 2 BLE response (ACK or with
/// data) uses 0x10/0x78.
inline constexpr uint8_t RSP_BYTE4_BT = 0x10;
inline constexpr uint8_t RSP_BYTE5_BT = 0x78;

enum class Command : uint8_t {
  NFC = 0x01,
  FLASH_READ = 0x02, ///< read calibration / device info
  INIT = 0x03,
  UNKNOWN_07 = 0x07, ///< init handshake; response is 1 zero data byte
  PLAYER_LEDS = 0x09,
  VIBRATION = 0x0a,
  BATTERY = 0x0b,
  FEATURE_SELECT = 0x0c, ///< enable motion / mouse / rumble / magnetometer; response 4 zero bytes
  FIRMWARE_UPDATE = 0x0d,
  FIRMWARE_INFO = 0x10,
  UNKNOWN_11 = 0x11, ///< init handshake (post-pairing); response is a device blob
  UNKNOWN_16 = 0x16, ///< init handshake; response is 24 zero data bytes
  PAIRING = 0x15,
  UNKNOWN_18 = 0x18, ///< late-init probe; 0x18/0x01 response is an 8-byte device blob
};

/// Subcommands of Command::PAIRING (0x15).
enum class PairingSub : uint8_t {
  EXCHANGE_ADDRESSES = 0x01,
  CONFIRM_LTK = 0x02, ///< console sends challenge A2, controller returns B2
  FINALISE = 0x03,
  EXCHANGE_KEYS = 0x04,     ///< console sends A1, controller returns fixed B1
  SEND_PAIRING_INFO = 0x07, ///< inject host addr + LTK directly
  STORE_PAIRING_INFO = 0x09,
};

/// Feature-select (0x0c) capability bits.
enum FeatureBits : uint8_t {
  FEATURE_BUTTONS = 0x01,
  FEATURE_STICKS = 0x02,
  FEATURE_IMU = 0x04,
  FEATURE_MOUSE = 0x10,
  FEATURE_RUMBLE = 0x20,
  FEATURE_MAGNETOMETER = 0x80,
};
/// Default feature mask the Pro Controller 2 reports.
inline constexpr uint8_t PRO2_FEATURE_MASK = 0x2f;

// ---------------------------------------------------------------------------
// Pairing crypto constants
// ---------------------------------------------------------------------------

/// Fixed controller-side "public key" B1 returned during key exchange. Because
/// this is a known constant and the LTK is A1 ⊕ B1, the link key is derivable.
inline constexpr std::array<uint8_t, 16> CONTROLLER_KEY_B1 = {
    0x5c, 0xf6, 0xee, 0x79, 0x2c, 0xdf, 0x05, 0xe1, 0xba, 0x2b, 0x63, 0x25, 0xc4, 0x1a, 0x5f, 0x10};

/// Golden test vector (host-verified) for the pairing crypto self-test.
namespace golden {
inline constexpr std::array<uint8_t, 16> A1 = {0x35, 0x03, 0xe9, 0x29, 0x82, 0x87, 0x71, 0x24,
                                               0xbe, 0xa8, 0x0c, 0x66, 0x46, 0x15, 0x83, 0x4b};
inline constexpr std::array<uint8_t, 16> A2 = {0x6f, 0xc6, 0xdf, 0x8a, 0xd8, 0xfe, 0xdf, 0x15,
                                               0xbb, 0x8c, 0x15, 0xe9, 0x1f, 0x32, 0x05, 0x44};
inline constexpr std::array<uint8_t, 16> LTK = {0x69, 0xf5, 0x07, 0x50, 0xae, 0x58, 0x74, 0xc5,
                                                0x04, 0x83, 0x6f, 0x43, 0x82, 0x0f, 0xdc, 0x5b};
inline constexpr std::array<uint8_t, 16> B2 = {0x13, 0x4c, 0x97, 0xf5, 0x11, 0xb9, 0xb6, 0xdd,
                                               0x4d, 0x86, 0xfd, 0x40, 0xf5, 0x36, 0xe9, 0xed};
} // namespace golden

} // namespace espp::switch2
