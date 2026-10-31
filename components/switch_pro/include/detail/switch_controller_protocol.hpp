#pragma once

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

namespace sp {
// Commands:
// { 0x01, "Manual Pairing"},
// { 0x02, "Get device info"},
// { 0x03, "Set input Mode"},
// { 0x04, "Trigger buttons elapsed time"},
// { 0x05, "Get page list state"},
// { 0x06, "Set HCI state"},
// { 0x07, "Reset pairing info"},
// { 0x08, "Set shipment low power state"},
// { 0x10, "SPI flash read"},
// { 0x11, "SPI flash Write"},
// { 0x12, "SPI sector erase"},
// { 0x20, "Reset NFC/IR MCU"},
// { 0x21, "Set NFC/IR MCU configuration"},
// { 0x22, "Set NFC/IR MCU state"},
// { 0x24, "Set unknown data (fw 3.86 and up)"},
// { 0x25, "Reset 0x24 unknown data (fw 3.86 and up)"},
// { 0x28, "Set unknown NFC/IR MCU data A"},
// { 0x29, "Get unknown NFC/IR MCU data A"},
// { 0x2A, "Set GPIO Pin Output value (2 @Port 2)"},
// { 0x2B, "Get x29 NFC/IR MCU data"},
// { 0x30, "Set player lights"},
// { 0x31, "Get player lights"},
// { 0x38, "Set HOME Light"},
// { 0x40, "Enable IMU (6-Axis sensor)"},
// { 0x41, "Set IMU sensitivity"},
// { 0x42, "Write to IMU registers"},
// { 0x43, "Read IMU registers"},
// { 0x48, "Enable vibration"},
// { 0x50, "Get regulated voltage"},
// { 0x51, "Set GPIO Pin Output value (7 & 15 @Port 1)"},
// { 0x52, "Get GPIO Pin Input/Output value"},

enum class Response {
  NO_DATA = -1,
  MALFORMED = -2,
  TOO_SHORT = -3,
  UNKNOWN_SUBCOMMAND = -4,
  ONLY_CONTROLLER_STATE = 0x00,
  BT_MANUAL_PAIRING = 0x01,
  REQUEST_DEVICE_INFO = 0x02,
  SET_MODE = 0x03,
  TRIGGER_BUTTONS_ELAPSED = 0x04,
  SET_SHIPMENT = 0x08,
  SPI_READ = 0x10,
  SET_NFC_IR_STATE = 0x22,
  SET_NFC_IR_CONFIG = 0x21,
  SET_UNKNOWN_DATA = 0x24,          // replies with 0x80 24 00 always
  RESET_UNKNOWN_DATA = 0x25,        // replies with 0x80 25 00 always
  SET_UNKNOWN_NFC_IR_DATA_A = 0x28, // replies with 0x80 28 always
  GET_UNKNOWN_NFC_IR_DATA_A = 0x29,
  SET_PLAYER = 0x30,
  GET_PLAYER = 0x31,
  SET_HOME_LIGHT = 0x38,
  TOGGLE_IMU = 0x40,
  ENABLE_VIBRATION = 0x48,
};

union TriggerTimes {
  struct {
    uint16_t l;
    uint16_t r;
    uint16_t zl;
    uint16_t zr;
    uint16_t sl;
    uint16_t sr;
    uint16_t home;
  } __attribute__((packed));
  uint16_t values[7];
  uint8_t bytes[14] = {0};
} __attribute__((packed));

struct Controller {
  uint8_t id;
  uint8_t connection_info;
};

static constexpr size_t REPORT_SIZE = 63;

static constexpr Controller JOYCON_LEFT = {0x01, 0x0E};
static constexpr Controller JOYCON_RIGHT = {0x02, 0x0E};
static constexpr Controller PRO_CONTROLLER = {0x03, 0x00};

// HID report IDs
static constexpr uint8_t HOST_INIT_REPORT = 0x80;
static constexpr uint8_t DEVICE_INIT_REPORT = 0x81;
static constexpr uint8_t HOST_OUTPUT_REPORT = 0x01;
static constexpr uint8_t HOST_RUMBLE_REPORT = 0x10;
static constexpr uint8_t DEVICE_RESPONSE_REPORT = 0x21;
static constexpr uint8_t DEVICE_INPUT_REPORT = 0x30;

// Commands for initialization sequence (0x80, 0x81 reports)
static constexpr uint8_t INIT_COMMAND_DEVICE_INFO = 0x01; // sent by device to kick off process
static constexpr uint8_t INIT_COMMAND_HANDSHAKE =
    0x02; // sent by host, device replies with same command back
static constexpr uint8_t INIT_COMMAND_SET_BAUD_RATE = 0x03;  // sent by host, not applicable to USB
static constexpr uint8_t INIT_COMMAND_ENABLE_USB_HID = 0x04; // sent by host, enable USB
static constexpr uint8_t INIT_COMMAND_ENABLE_BT_HID = 0x05;  // sent by host, switches back to BT

static constexpr size_t device_init_report_data_mac_addr_offset = 3;
static constexpr uint8_t device_init_report_data[REPORT_SIZE] = {
    INIT_COMMAND_DEVICE_INFO, 0, PRO_CONTROLLER.id,
    // mac address
    0xFD, 0x5E, 0xEC, 0xE9, 0xB6, 0x98};

static constexpr uint8_t device_info[] = {
    0x03, 0x48,        // Version info
    PRO_CONTROLLER.id, // Controller ID
    0x02,              // always 0x02
    // MAC address (reverse endianness compared to init sequence)
    0x98, 0xB6, 0xE9, 0xEC, 0x5E, 0xFD,
    0x03, // unknown. some use 0x01, some 0x03
    0x02, // If 01, colors in SPI are used. Otherwise default ones. some use 0x02, some use 0x01
    0x00, 0x00};

static constexpr uint8_t vibrator_bytes[] = {0xA0, 0xB0, 0xC0, 0x90};

typedef std::pair<uint8_t, Response> SubcommandResponse;
static constexpr const std::array<SubcommandResponse, 14> subcommands = {
    {{0x00, Response::ONLY_CONTROLLER_STATE},
     {0x01, Response::BT_MANUAL_PAIRING},
     {0x02, Response::REQUEST_DEVICE_INFO},
     {0x03, Response::SET_MODE},
     {0x04, Response::TRIGGER_BUTTONS_ELAPSED},
     {0x08, Response::SET_SHIPMENT},
     {0x10, Response::SPI_READ},
     {0x22, Response::SET_NFC_IR_STATE},
     {0x21, Response::SET_NFC_IR_CONFIG},
     {0x30, Response::SET_PLAYER},
     {0x31, Response::GET_PLAYER},
     {0x38, Response::SET_HOME_LIGHT},
     {0x40, Response::TOGGLE_IMU},
     {0x48, Response::ENABLE_VIBRATION}}};

// message represents:
// - response (of type Response enum)
// - subcommand (uint8_t array) which is the data starting at byte 10 and onwards
// - subcommand_id which is the first byte of the subcommand
struct Message {
  const uint8_t *payload{nullptr};
  const uint8_t *subcommand{nullptr};
  size_t subcommand_len{0}; ///< bytes available at `subcommand` (0 if too short)
  uint8_t subcommand_id{0};
  Response response{Response::NO_DATA};

  static constexpr size_t subcommand_offset = 10;

  Message(const uint8_t *payload, size_t size)
      : payload(payload) {
    if (payload == nullptr) {
      response = Response::NO_DATA;
      return;
    }
    // Need at least the subcommand id (byte 10). A well-formed Switch report is
    // full-size; guard here so a truncated / malformed OUTPUT report cannot make
    // the handlers read past the buffer (they only run for a matched subcommand,
    // and subcommand_len lets a handler check before indexing further).
    if (size <= subcommand_offset) {
      response = Response::TOO_SHORT;
      return;
    }

    subcommand = payload + subcommand_offset;
    subcommand_len = size - subcommand_offset;
    subcommand_id = subcommand[0];

    const auto it =
        std::find_if(subcommands.begin(), subcommands.end(),
                     [&](const SubcommandResponse &sub) { return sub.first == subcommand_id; });

    response = (it != subcommands.end()) ? it->second : Response::UNKNOWN_SUBCOMMAND;
  }
};
} // namespace sp
