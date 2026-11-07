#pragma once

// Microsoft OS 2.0 descriptor set builder.
//
// Dependency-free and host-testable (no ESP-IDF / TinyUSB headers). Windows
// reads this descriptor set through a vendor request advertised in the BOS
// descriptor and uses it to bind drivers to interfaces without an INF: a
// function subset with the compatible ID "WINUSB" (plus a DeviceInterfaceGUIDs
// registry property) gives an interface WinUSB, which is what WebUSB needs; one
// with "XUSB10" gives an interface Windows' own Xbox 360 controller driver
// (xusb22.sys matches the compatible ID USB\MS_COMP_XUSB10), which is how an
// X-Input interface inside a COMPOSITE device gets recognized - the retail
// controller is matched by VID/PID instead, but that match claims the whole
// device and would hide the other interfaces.
//
// Layout (Microsoft OS 2.0 Descriptors Specification):
//   set header (10)
//     configuration subset header (8)
//       function subset header (8)       one per Function
//         compatible ID feature (20)
//         [registry property (132)]      when the function wants the WinUSB GUID
//
// `espp::UsbDevice` builds the set from its enabled functions (see
// usb_device.cpp); `descriptor_set()` is the pure byte builder.

#include <array>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <span>
#include <string_view>
#include <vector>

namespace espp {
namespace ms_os_20 {

/// Descriptor types (wDescriptorType) of the MS OS 2.0 specification.
inline constexpr uint16_t kSetHeaderDescriptor = 0x00;
inline constexpr uint16_t kSubsetHeaderConfiguration = 0x01;
inline constexpr uint16_t kSubsetHeaderFunction = 0x02;
inline constexpr uint16_t kFeatureCompatibleId = 0x03;
inline constexpr uint16_t kFeatureRegProperty = 0x04;

/// dwWindowsVersion of the set header: Windows 8.1 (NTDDI_WINBLUE), the first
/// Windows that reads MS OS 2.0 descriptors.
inline constexpr uint32_t kWindowsVersion = 0x06030000;

/// Compatible IDs (8 bytes, NUL padded on the wire).
inline constexpr std::string_view kCompatibleIdWinUsb = "WINUSB";
inline constexpr std::string_view kCompatibleIdXusb10 =
    "XUSB10"; ///< Xbox 360 controller (xusb22.sys)

/// The DeviceInterfaceGUIDs registry property the WinUSB function carries, the
/// GUID TinyUSB's webusb_serial example uses (WebUSB in Chromium does not care
/// which GUID it is, only that WinUSB is bound).
inline constexpr std::string_view kDeviceInterfaceGuid = "{975F44D9-0D08-43FD-8B3E-127CA8AFFF9D}";

inline constexpr size_t kSetHeaderLen = 10;
inline constexpr size_t kConfigurationSubsetHeaderLen = 8;
inline constexpr size_t kFunctionSubsetHeaderLen = 8;
inline constexpr size_t kCompatibleIdFeatureLen = 20;
/// Registry property feature: header (8: length, type, data type, name length)
/// + name "DeviceInterfaceGUIDs\0" as UTF-16 (42) + data length (2) + the GUID
/// + "\0\0" as a REG_MULTI_SZ in UTF-16 (80).
inline constexpr size_t kRegPropertyFeatureLen = 8 + 42 + 2 + 80; // 132

/// One function (interface) of the descriptor set.
struct Function {
  uint8_t first_interface{0};        ///< bFirstInterface of the function subset
  std::string_view compatible_id{};  ///< "WINUSB", "XUSB10", ... (at most 8 bytes)
  bool device_interface_guid{false}; ///< add the DeviceInterfaceGUIDs registry property
};

/// Longest set the format can describe: every length field (the set's, the
/// configuration subset's, each function subset's) is 16 bits.
inline constexpr size_t kMaxSetLength = 0xFFFF;

/// Byte length descriptor_set() produces for these functions (not capped: compare
/// with kMaxSetLength to know whether descriptor_set() can build it).
inline constexpr size_t descriptor_set_length(std::span<const Function> functions) {
  size_t n = kSetHeaderLen + kConfigurationSubsetHeaderLen;
  for (const auto &f : functions) {
    n += kFunctionSubsetHeaderLen + kCompatibleIdFeatureLen;
    if (f.device_interface_guid)
      n += kRegPropertyFeatureLen;
  }
  return n;
}

/// Build the MS OS 2.0 descriptor set for the given functions (in interface
/// order). An empty list yields a set with no function subsets, which Windows
/// accepts but which binds nothing. Returns an empty vector if the set would
/// not fit the format's 16-bit length fields (kMaxSetLength; thousands of
/// functions, so a caller error rather than a real device).
inline std::vector<uint8_t> descriptor_set(std::span<const Function> functions) {
  std::vector<uint8_t> out;
  const size_t total = descriptor_set_length(functions);
  if (total > kMaxSetLength)
    return out;
  out.reserve(total);
  auto u16 = [&](uint16_t v) {
    out.push_back(static_cast<uint8_t>(v & 0xFF));
    out.push_back(static_cast<uint8_t>(v >> 8));
  };
  auto u32 = [&](uint32_t v) {
    u16(static_cast<uint16_t>(v & 0xFFFF));
    u16(static_cast<uint16_t>(v >> 16));
  };
  auto utf16 = [&](std::string_view s) {
    for (char c : s) {
      out.push_back(static_cast<uint8_t>(c));
      out.push_back(0x00);
    }
  };

  // set header
  u16(static_cast<uint16_t>(kSetHeaderLen));
  u16(kSetHeaderDescriptor);
  u32(kWindowsVersion);
  u16(static_cast<uint16_t>(total));
  // configuration subset header: configuration 0, reserved, subset length
  u16(static_cast<uint16_t>(kConfigurationSubsetHeaderLen));
  u16(kSubsetHeaderConfiguration);
  out.push_back(0x00);
  out.push_back(0x00);
  u16(static_cast<uint16_t>(total - kSetHeaderLen));

  for (const auto &f : functions) {
    size_t subset_len = kFunctionSubsetHeaderLen + kCompatibleIdFeatureLen;
    if (f.device_interface_guid)
      subset_len += kRegPropertyFeatureLen;
    // function subset header: first interface, reserved, subset length
    u16(static_cast<uint16_t>(kFunctionSubsetHeaderLen));
    u16(kSubsetHeaderFunction);
    out.push_back(f.first_interface);
    out.push_back(0x00);
    u16(static_cast<uint16_t>(subset_len));
    // compatible ID feature: 8-byte compatible ID + 8-byte sub-compatible ID,
    // both NUL padded
    u16(static_cast<uint16_t>(kCompatibleIdFeatureLen));
    u16(kFeatureCompatibleId);
    std::array<uint8_t, 16> ids{};
    const size_t n = f.compatible_id.size() < 8 ? f.compatible_id.size() : 8;
    std::memcpy(ids.data(), f.compatible_id.data(), n);
    out.insert(out.end(), ids.begin(), ids.end());
    if (f.device_interface_guid) {
      // registry property feature: REG_MULTI_SZ (7) "DeviceInterfaceGUIDs"
      u16(static_cast<uint16_t>(kRegPropertyFeatureLen));
      u16(kFeatureRegProperty);
      u16(0x0007); // wPropertyDataType: REG_MULTI_SZ
      u16(0x002A); // wPropertyNameLength: 21 UTF-16 code units
      utf16("DeviceInterfaceGUIDs");
      u16(0x0000); // NUL terminator of the name
      u16(0x0050); // wPropertyDataLength: 40 code units
      utf16(kDeviceInterfaceGuid);
      u16(0x0000); // string terminator
      u16(0x0000); // multi-string terminator
    }
  }
  assert(out.size() == total); // the length computation and the bytes agree
  return out;
}

} // namespace ms_os_20
} // namespace espp
