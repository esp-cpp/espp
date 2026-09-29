#pragma once

// A simulated CANopen (CiA 301) node with a CiA 402 (DS402) drive profile.
//
// Selected with CONFIG_CAN_BRIDGE_SIMULATED_NODE (off by default): the bridge
// then runs this node in firmware instead of driving the TWAI peripheral, so
// the hosted CAN console and DS402 panel web apps can be exercised end to end
// with no CAN transceiver, bus or drive attached. Everything the web apps talk
// to a real node with is answered here:
//
//   - NMT (start / stop / pre-operational / reset node / reset communication),
//     with the boot-up message and the producer heartbeat (0x1017)
//   - the default SDO server (0x600 + id / 0x580 + id): expedited and segmented
//     upload and download with toggle-bit checking, and CiA 301 abort codes
//   - an object dictionary: the CiA 301 communication / identity objects, the
//     stored EDS (0x1021, a DOMAIN generated from the dictionary itself, 0x1022
//     = 0) so a browser can read the device's own object list, the CiA 402
//     objects of a single-axis drive, and a manufacturer object (0x2000) that
//     injects a fault
//   - the CiA 402 power-drive-system state machine driven by the controlword
//     (0x6040), reported in the statusword (0x6041), with the fault reset edge
//     and a quick-stop that transits to Switch On Disabled once stopped
//   - profile position (with the new-set-point handshake), profile velocity,
//     profile torque and homing modes (0x6060 / 0x6061), with a simple
//     trapezoidal motion model that integrates position / velocity every tick
//   - TPDO1 (statusword + position actual, event timer 0x1800:5) while
//     Operational, RPDO1 (controlword + modes of operation), and an EMCY on
//     fault entry / reset
//
// The node is deliberately host-buildable (it depends only on the canopen
// component's wire core and the standard library) so it is unit-tested on the
// host, see test/simulated_node_host_test.cpp. The bridge wraps it in a
// SimulatedCanBus (simulated_can_bus.hpp) that stands in for espp::Twai.

#include <algorithm>
#include <bit>
#include <chrono>
#include <climits>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <map>
#include <optional>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include "detail/canopen_core.hpp"

namespace can_bridge {

class SimulatedDs402Node {
public:
  using CanFrame = espp::detail::CanFrame;
  using NmtState = espp::detail::canopen::NmtState;
  using State = espp::detail::ds402::State;
  using Frames = std::vector<CanFrame>;

  /// CiA 301 abort codes the node uses.
  enum Abort : uint32_t {
    kAbortToggle = 0x05030000,     ///< toggle bit not alternated
    kAbortUnknownCs = 0x05040001,  ///< client/server command specifier not valid
    kAbortWriteOnly = 0x06010001,  ///< attempt to read a write-only object
    kAbortReadOnly = 0x06010002,   ///< attempt to write a read-only object
    kAbortNoObject = 0x06020000,   ///< object does not exist
    kAbortLength = 0x06070010,     ///< data type does not match, length mismatch
    kAbortLengthHigh = 0x06070012, ///< length of service parameter too high
    kAbortLengthLow = 0x06070013,  ///< length of service parameter too low
    kAbortNoSub = 0x06090011,      ///< sub-index does not exist
    kAbortRange = 0x06090030,      ///< value range of parameter exceeded
    kAbortStore = 0x08000020,      ///< data cannot be transferred or stored
    kAbortWrongState = 0x08000022, ///< present device state prevents the access
  };

  /// CiA 306 data type codes (as used in an EDS and in 0x1021 consumers).
  enum DataType : uint8_t {
    kBool = 0x01,
    kI8 = 0x02,
    kI16 = 0x03,
    kI32 = 0x04,
    kU8 = 0x05,
    kU16 = 0x06,
    kU32 = 0x07,
    kVisibleString = 0x09,
    kDomain = 0x0F,
  };

  enum class Access : uint8_t { Const, ReadOnly, ReadWrite, WriteOnly };

  struct Config {
    uint8_t node_id{1};                                    ///< 1..127
    std::string device_name{"espp simulated DS402 drive"}; ///< 0x1008
    std::string hardware_version{"sim"};                   ///< 0x1009
    std::string software_version{"1.0.0"};                 ///< 0x100A
    uint32_t vendor_id{0x00001209};                        ///< 0x1018:1
    uint32_t product_code{0x0000C0DE};                     ///< 0x1018:2
    uint32_t revision{0x00010000};                         ///< 0x1018:3
    uint32_t serial{0x00000001};                           ///< 0x1018:4
    uint16_t heartbeat_ms{1000};                           ///< 0x1017 default
    uint16_t tpdo1_event_ms{100};                          ///< 0x1800:5 default
  };

  explicit SimulatedDs402Node(const Config &config)
      : config_(config) {
    build_dictionary();
    reset_application();
  }

  uint8_t node_id() const { return config_.node_id; }
  NmtState nmt_state() const { return nmt_; }
  State drive_state() const { return state_; }
  uint16_t statusword() const { return compose_statusword(); }
  int32_t position() const { return static_cast<int32_t>(position_); }
  int32_t velocity() const { return static_cast<int32_t>(velocity_); }
  int8_t mode_display() const { return mode_display_; }

  /// Application reset: dictionary defaults, drive to Switch On Disabled,
  /// communication reset (boot-up message + Pre-Operational).
  void reset_application() {
    restore_defaults();
    state_ = State::SwitchOnDisabled;
    mode_ = 0;
    mode_display_ = 0;
    position_ = velocity_ = 0.0;
    target_position_ = 0;
    torque_actual_ = 0;
    target_reached_ = true;
    setpoint_ack_ = false;
    halt_ = false;
    quick_stopping_ = false;
    homing_active_ = false;
    homing_attained_ = false;
    homing_ms_ = 0;
    prev_controlword_ = 0;
    reset_communication();
  }

  /// Communication reset (CiA 301 NMT): the communication-profile parameters
  /// (0x1000..0x1FFF: heartbeat time, PDO parameters, ...) return to their
  /// power-on defaults, SDO/PDO state is cleared, a boot-up message is queued
  /// and the node is in Pre-Operational. Application objects keep their values.
  void reset_communication() {
    restore_defaults(0x1000, 0x1FFF);
    sdo_ = {};
    nmt_ = NmtState::PreOperational;
    boot_up_pending_ = true;
    heartbeat_elapsed_ms_ = 0;
    tpdo_elapsed_ms_ = 0;
  }

  /// Feed one frame from the bus; frames the node sends in response are appended to `out`.
  void process(const CanFrame &in, Frames &out) {
    if (in.extended)
      return; // CANopen uses 11-bit identifiers only
    namespace co = espp::detail::canopen;
    const uint8_t id = config_.node_id;
    if (in.id == co::COB_NMT) {
      if (in.dlc >= 2 && !in.rtr && (in.data[1] == 0 || in.data[1] == id))
        handle_nmt(static_cast<co::NmtCommand>(in.data[0]), out);
      return;
    }
    if (nmt_ == NmtState::Stopped)
      return; // only NMT and the heartbeat work in Stopped
    if (in.id == co::COB_SDO_RX_BASE + id) {
      if (in.dlc == 8 && !in.rtr)
        handle_sdo(in, out);
      return;
    }
    if (nmt_ != NmtState::Operational)
      return; // PDOs only in Operational
    if (in.id == co::COB_RPDO1_BASE + id && !in.rtr && in.dlc >= 2) {
      // RPDO1 mapping (0x1600): controlword u16 [+ modes of operation i8]
      const uint16_t cw = static_cast<uint16_t>(co::get_le(&in.data[0], 2));
      if (in.dlc >= 3)
        set_mode(as_i8(in.data[2]));
      apply_controlword(cw, out);
    } else if (in.id == co::COB_TPDO1_BASE + id && in.rtr) {
      out.push_back(make_tpdo1()); // RTR-triggered TPDO1
    }
  }

  /// Advance time: heartbeat, boot-up, TPDO event timer and the motion model.
  void tick(std::chrono::milliseconds dt, Frames &out) {
    namespace co = espp::detail::canopen;
    const uint8_t id = config_.node_id;
    if (boot_up_pending_) {
      boot_up_pending_ = false;
      CanFrame f;
      f.id = co::COB_HEARTBEAT_BASE + id;
      f.dlc = 1;
      f.data[0] = static_cast<uint8_t>(NmtState::BootUp);
      out.push_back(f);
    }
    const uint32_t ms = static_cast<uint32_t>(dt.count());
    const uint16_t hb = heartbeat_ms();
    if (hb) {
      // one frame per elapsed period: a late tick (task scheduling) catches
      // up instead of dropping periods and drifting
      heartbeat_elapsed_ms_ += ms;
      for (; heartbeat_elapsed_ms_ >= hb; heartbeat_elapsed_ms_ -= hb) {
        CanFrame f;
        f.id = co::COB_HEARTBEAT_BASE + id;
        f.dlc = 1;
        f.data[0] = static_cast<uint8_t>(nmt_);
        out.push_back(f);
      }
    } else {
      heartbeat_elapsed_ms_ = 0;
    }
    step_motion(dt.count() / 1000.0, out);
    const uint16_t ev = read_u16(0x1800, 5);
    if (nmt_ == NmtState::Operational && ev) {
      tpdo_elapsed_ms_ += ms;
      for (; tpdo_elapsed_ms_ >= ev; tpdo_elapsed_ms_ -= ev)
        out.push_back(make_tpdo1());
    } else {
      tpdo_elapsed_ms_ = 0;
    }
  }

  /// Read an object as raw little-endian bytes (nullopt with the abort code on failure).
  std::optional<std::vector<uint8_t>> read_object(uint16_t index, uint8_t sub, uint32_t &abort) {
    sync_dynamic_objects();
    const Entry *e = find(index, sub, abort);
    if (!e)
      return std::nullopt;
    if (e->access == Access::WriteOnly) {
      abort = kAbortWriteOnly;
      return std::nullopt;
    }
    if (index == 0x1021 && sub == 0) {
      if (eds_cache_.empty())
        eds_cache_ = generate_eds();
      return std::vector<uint8_t>(eds_cache_.begin(), eds_cache_.end());
    }
    return e->value;
  }

  /// Write an object; returns false with the abort code on failure. Frames the
  /// write causes (an EMCY on a fault injection) are appended to `out`.
  bool write_object(uint16_t index, uint8_t sub, std::span<const uint8_t> data, uint32_t &abort,
                    Frames &out) {
    Entry *e = find(index, sub, abort);
    if (!e)
      return false;
    if (e->access == Access::Const || e->access == Access::ReadOnly) {
      abort = kAbortReadOnly;
      return false;
    }
    const size_t fixed = fixed_size(e->data_type);
    if (fixed && data.size() != fixed) {
      abort = data.size() > fixed ? kAbortLengthHigh : kAbortLengthLow;
      return false;
    }
    if (!fixed && data.size() > kMaxStringBytes) {
      abort = kAbortLengthHigh;
      return false;
    }
    // range / state checks and side effects, before the value is stored
    if (!on_write(index, sub, data, abort, out))
      return false;
    // 0x1010 / 0x1011 take a command signature ("save" / "load"), not a new
    // value: they keep reporting their capability (1)
    if (index != 0x1010 && index != 0x1011)
      e->value.assign(data.begin(), data.end());
    return true;
  }

  /// The EDS (CiA 306) text the node serves in 0x1021, generated from its dictionary.
  std::string generate_eds() const {
    std::string s;
    s.reserve(8192);
    s += "[FileInfo]\nFileName=espp_simulated_ds402.eds\nFileVersion=1\nFileRevision=0\n"
         "EDSVersion=4.0\nDescription=espp CAN bridge simulated CiA 402 drive\n"
         "CreatedBy=espp\n\n";
    s += "[DeviceInfo]\nVendorName=espp\n";
    s += "VendorNumber=" + hex32(config_.vendor_id) + "\n";
    s += "ProductName=" + config_.device_name + "\n";
    s += "ProductNumber=" + hex32(config_.product_code) + "\n";
    s += "RevisionNumber=" + hex32(config_.revision) + "\n";
    s += "BaudRate_125=1\nBaudRate_250=1\nBaudRate_500=1\nBaudRate_1000=1\n"
         "SimpleBootUpMaster=0\nSimpleBootUpSlave=1\nGranularity=8\n"
         "DynamicChannelsSupported=0\nGroupMessaging=0\nNrOfRXPDO=1\nNrOfTXPDO=1\n"
         "LSS_Supported=0\n\n";
    // the three object lists
    std::vector<uint16_t> mandatory, optional, manufacturer;
    for (const auto &[index, obj] : od_) {
      if (index == 0x1000 || index == 0x1001 || index == 0x1018)
        mandatory.push_back(index);
      else if (index >= 0x2000 && index <= 0x5FFF)
        manufacturer.push_back(index);
      else
        optional.push_back(index);
    }
    auto list = [&](const char *name, const std::vector<uint16_t> &v) {
      s += std::string("[") + name + "]\nSupportedObjects=" + std::to_string(v.size()) + "\n";
      for (size_t i = 0; i < v.size(); ++i)
        s += std::to_string(i + 1) + "=" + hex16(v[i]) + "\n";
      s += "\n";
    };
    list("MandatoryObjects", mandatory);
    list("OptionalObjects", optional);
    list("ManufacturerObjects", manufacturer);
    for (const auto &[index, obj] : od_) {
      char sec[8];
      std::snprintf(sec, sizeof(sec), "%04X", index);
      s += std::string("[") + sec + "]\nParameterName=" + obj.name + "\n";
      s += "ObjectType=0x" + std::to_string(obj.object_type) + "\n";
      if (obj.object_type == kVar) {
        const Entry &e = obj.subs.at(0);
        s += entry_fields(e);
      } else {
        s += "SubNumber=" + std::to_string(obj.subs.size()) + "\n\n";
        for (const auto &[sub, e] : obj.subs) {
          char sub_sec[16];
          std::snprintf(sub_sec, sizeof(sub_sec), "%04Xsub%X", index, sub);
          s += std::string("[") + sub_sec + "]\nParameterName=" + e.name + "\nObjectType=0x7\n";
          s += entry_fields(e);
        }
        continue;
      }
    }
    return s;
  }

private:
  static constexpr uint8_t kVar = 7, kArray = 8, kRecord = 9;
  static constexpr size_t kMaxStringBytes = 256;
  static constexpr uint32_t kSaveSignature = 0x65766173; // "save"
  static constexpr uint32_t kLoadSignature = 0x64616F6C; // "load"
  static constexpr uint16_t kSupportedModes = 0x002D;    // pp, pv, tq, hm (0x6502 bits 0,2,3,5)
  static constexpr uint32_t kHomingDurationMs = 500;
  static constexpr int8_t kModePp = 1, kModePv = 3, kModeTq = 4, kModeHm = 6;

  struct Entry {
    std::string name;
    uint8_t data_type{kU32};
    Access access{Access::ReadWrite};
    std::vector<uint8_t> value;
    std::vector<uint8_t> default_value;
    bool pdo_mappable{false};
  };
  struct Object {
    std::string name;
    uint8_t object_type{kVar};
    std::map<uint8_t, Entry> subs;
  };
  struct SdoTransfer {
    bool active{false};
    bool upload{false};
    uint16_t index{0};
    uint8_t sub{0};
    std::vector<uint8_t> buf;
    size_t offset{0};
    bool toggle{false};
  };

  // ---- object dictionary -------------------------------------------------
  static size_t fixed_size(uint8_t data_type) {
    switch (data_type) {
    case kBool:
    case kI8:
    case kU8:
      return 1;
    case kI16:
    case kU16:
      return 2;
    case kI32:
    case kU32:
      return 4;
    default:
      return 0; // string / domain: variable
    }
  }
  static std::vector<uint8_t> le(uint32_t v, size_t n) {
    std::vector<uint8_t> b(n);
    espp::detail::canopen::put_le(v, b.data(), n);
    return b;
  }
  static std::vector<uint8_t> str(const std::string &s) { return {s.begin(), s.end()}; }
  static std::string hex32(uint32_t v) {
    char b[16];
    std::snprintf(b, sizeof(b), "0x%08X", static_cast<unsigned>(v));
    return b;
  }
  static std::string hex16(uint16_t v) {
    char b[16];
    std::snprintf(b, sizeof(b), "0x%04X", static_cast<unsigned>(v));
    return b;
  }
  static const char *access_name(Access a) {
    switch (a) {
    case Access::Const:
      return "const";
    case Access::ReadOnly:
      return "ro";
    case Access::WriteOnly:
      return "wo";
    default:
      return "rw";
    }
  }
  static std::string entry_fields(const Entry &e) {
    std::string s;
    char b[32];
    std::snprintf(b, sizeof(b), "DataType=0x%04X\n", e.data_type);
    s += b;
    s += std::string("AccessType=") + access_name(e.access) + "\n";
    const size_t n = fixed_size(e.data_type);
    if (n && e.default_value.size() == n) {
      const uint32_t v = espp::detail::canopen::get_le(e.default_value.data(), n);
      std::snprintf(b, sizeof(b), "DefaultValue=0x%0*X\n", static_cast<int>(n * 2),
                    static_cast<unsigned>(v));
      s += b;
    } else if (e.data_type == kVisibleString) {
      s += "DefaultValue=" + std::string(e.default_value.begin(), e.default_value.end()) + "\n";
    }
    s += std::string("PDOMapping=") + (e.pdo_mappable ? "1" : "0") + "\n\n";
    return s;
  }

  Entry &var(uint16_t index, const char *name, uint8_t type, Access access,
             std::vector<uint8_t> value, bool pdo = false) {
    Object &o = od_[index];
    o.name = name;
    o.object_type = kVar;
    Entry &e = o.subs[0];
    e = Entry{name, type, access, value, value, pdo};
    return e;
  }
  Object &record(uint16_t index, const char *name, uint8_t object_type) {
    Object &o = od_[index];
    o.name = name;
    o.object_type = object_type;
    return o;
  }
  static void add_sub(Object &o, uint8_t sub, const char *name, uint8_t type, Access access,
                      std::vector<uint8_t> value, bool pdo = false) {
    o.subs[sub] = Entry{name, type, access, value, value, pdo};
  }
  static void count_sub(Object &o, uint8_t n) {
    add_sub(o, 0, "Number of entries", kU8, Access::ReadOnly, {n});
  }

  void build_dictionary() {
    const uint8_t id = config_.node_id;
    // --- CiA 301 communication profile ------------------------------------
    var(0x1000, "Device type", kU32, Access::ReadOnly, le(0x00020192, 4)); // 402, servo drive
    var(0x1001, "Error register", kU8, Access::ReadOnly, {0});
    var(0x1005, "COB-ID SYNC message", kU32, Access::ReadWrite, le(0x80, 4));
    var(0x1008, "Manufacturer device name", kVisibleString, Access::Const,
        str(config_.device_name));
    var(0x1009, "Manufacturer hardware version", kVisibleString, Access::Const,
        str(config_.hardware_version));
    var(0x100A, "Manufacturer software version", kVisibleString, Access::Const,
        str(config_.software_version));
    {
      Object &o = record(0x1010, "Store parameters", kArray);
      count_sub(o, 1);
      add_sub(o, 1, "Save all parameters", kU32, Access::ReadWrite, le(1, 4));
      Object &r = record(0x1011, "Restore default parameters", kArray);
      count_sub(r, 1);
      add_sub(r, 1, "Restore all default parameters", kU32, Access::ReadWrite, le(1, 4));
    }
    var(0x1017, "Producer heartbeat time", kU16, Access::ReadWrite, le(config_.heartbeat_ms, 2));
    {
      Object &o = record(0x1018, "Identity object", kRecord);
      count_sub(o, 4);
      add_sub(o, 1, "Vendor-ID", kU32, Access::ReadOnly, le(config_.vendor_id, 4));
      add_sub(o, 2, "Product code", kU32, Access::ReadOnly, le(config_.product_code, 4));
      add_sub(o, 3, "Revision number", kU32, Access::ReadOnly, le(config_.revision, 4));
      add_sub(o, 4, "Serial number", kU32, Access::ReadOnly, le(config_.serial, 4));
    }
    var(0x1021, "Store EDS", kDomain, Access::ReadOnly, {});
    var(0x1022, "Store format", kU16, Access::ReadOnly, le(0, 2)); // 0 = ISO 10646 EDS
    {
      Object &o = record(0x1200, "SDO server parameter 1", kRecord);
      count_sub(o, 2);
      add_sub(o, 1, "COB-ID client -> server (rx)", kU32, Access::ReadOnly, le(0x600u + id, 4));
      add_sub(o, 2, "COB-ID server -> client (tx)", kU32, Access::ReadOnly, le(0x580u + id, 4));
    }
    {
      Object &o = record(0x1400, "RPDO1 communication parameter", kRecord);
      count_sub(o, 2);
      // fixed: the simulation always listens on 0x200 + id with the mapping below
      add_sub(o, 1, "COB-ID used by RPDO", kU32, Access::ReadOnly, le(0x200u + id, 4));
      add_sub(o, 2, "Transmission type", kU8, Access::ReadOnly, {255});
      Object &m = record(0x1600, "RPDO1 mapping parameter", kRecord);
      count_sub(m, 2);
      add_sub(m, 1, "Mapping entry 1", kU32, Access::ReadOnly, le(0x60400010, 4));
      add_sub(m, 2, "Mapping entry 2", kU32, Access::ReadOnly, le(0x60600008, 4));
    }
    {
      Object &o = record(0x1800, "TPDO1 communication parameter", kRecord);
      count_sub(o, 5);
      // fixed: the simulation always sends on 0x180 + id, event-driven (type 255)
      // with no inhibit time; only the event timer (sub 5) is acted on
      add_sub(o, 1, "COB-ID used by TPDO", kU32, Access::ReadOnly, le(0x180u + id, 4));
      add_sub(o, 2, "Transmission type", kU8, Access::ReadOnly, {255});
      add_sub(o, 3, "Inhibit time", kU16, Access::ReadOnly, le(0, 2));
      add_sub(o, 4, "Reserved", kU8, Access::ReadOnly, {0});
      add_sub(o, 5, "Event timer", kU16, Access::ReadWrite, le(config_.tpdo1_event_ms, 2));
      Object &m = record(0x1A00, "TPDO1 mapping parameter", kRecord);
      count_sub(m, 2);
      add_sub(m, 1, "Mapping entry 1", kU32, Access::ReadOnly, le(0x60410010, 4));
      add_sub(m, 2, "Mapping entry 2", kU32, Access::ReadOnly, le(0x60640020, 4));
    }
    // --- manufacturer-specific ---------------------------------------------
    var(0x2000, "Simulate fault", kU8, Access::ReadWrite, {0});
    var(0x2001, "Simulation tick count", kU32, Access::ReadOnly, le(0, 4));
    // --- CiA 402 drive profile (single axis) -------------------------------
    var(0x603F, "Error code", kU16, Access::ReadOnly, le(0, 2), true);
    var(0x6040, "Controlword", kU16, Access::ReadWrite, le(0, 2), true);
    var(0x6041, "Statusword", kU16, Access::ReadOnly, le(0, 2), true);
    var(0x605A, "Quick stop option code", kI16, Access::ReadWrite, le(2, 2));
    var(0x6060, "Modes of operation", kI8, Access::ReadWrite, {0}, true);
    var(0x6061, "Modes of operation display", kI8, Access::ReadOnly, {0}, true);
    var(0x6062, "Position demand value", kI32, Access::ReadOnly, le(0, 4), true);
    var(0x6064, "Position actual value", kI32, Access::ReadOnly, le(0, 4), true);
    var(0x606C, "Velocity actual value", kI32, Access::ReadOnly, le(0, 4), true);
    var(0x6071, "Target torque", kI16, Access::ReadWrite, le(0, 2), true);
    var(0x6077, "Torque actual value", kI16, Access::ReadOnly, le(0, 2), true);
    var(0x607A, "Target position", kI32, Access::ReadWrite, le(0, 4), true);
    {
      Object &o = record(0x607D, "Software position limit", kArray);
      count_sub(o, 2);
      add_sub(o, 1, "Min position limit", kI32, Access::ReadWrite,
              le(static_cast<uint32_t>(-1000000), 4));
      add_sub(o, 2, "Max position limit", kI32, Access::ReadWrite, le(1000000, 4));
    }
    var(0x6081, "Profile velocity", kU32, Access::ReadWrite, le(1000, 4), true);
    var(0x6083, "Profile acceleration", kU32, Access::ReadWrite, le(5000, 4), true);
    var(0x6084, "Profile deceleration", kU32, Access::ReadWrite, le(5000, 4), true);
    var(0x6085, "Quick stop deceleration", kU32, Access::ReadWrite, le(20000, 4));
    var(0x6098, "Homing method", kI8, Access::ReadWrite, {35});
    var(0x60FF, "Target velocity", kI32, Access::ReadWrite, le(0, 4), true);
    var(0x6502, "Supported drive modes", kU32, Access::ReadOnly, le(kSupportedModes, 4));
  }

  void restore_defaults(uint16_t first = 0x0000, uint16_t last = 0xFFFF) {
    for (auto &[index, obj] : od_) {
      if (index < first || index > last)
        continue;
      for (auto &[sub, e] : obj.subs)
        e.value = e.default_value;
    }
    eds_cache_.clear();
  }

  /// The software position limits (0x607D), as doubles.
  std::pair<double, double> position_limits() const {
    return {static_cast<double>(read_i32(0x607D, 1)), static_cast<double>(read_i32(0x607D, 2))};
  }
  /// Keep the axis inside the software position limits (and the int32 range
  /// every position object is reported in): motion stops at a limit.
  void clamp_position() {
    auto [lo, hi] = position_limits();
    lo = std::max(lo, static_cast<double>(INT32_MIN));
    hi = std::min(hi, static_cast<double>(INT32_MAX));
    if (position_ <= lo) {
      position_ = lo;
      if (velocity_ < 0)
        velocity_ = 0.0;
    } else if (position_ >= hi) {
      position_ = hi;
      if (velocity_ > 0)
        velocity_ = 0.0;
    }
  }

  Entry *find(uint16_t index, uint8_t sub, uint32_t &abort) {
    auto it = od_.find(index);
    if (it == od_.end()) {
      abort = kAbortNoObject;
      return nullptr;
    }
    auto s = it->second.subs.find(sub);
    if (s == it->second.subs.end()) {
      abort = kAbortNoSub;
      return nullptr;
    }
    return &s->second;
  }
  uint32_t read_raw(uint16_t index, uint8_t sub, size_t n) const {
    auto it = od_.find(index);
    if (it == od_.end())
      return 0;
    auto s = it->second.subs.find(sub);
    if (s == it->second.subs.end() || s->second.value.size() < n)
      return 0;
    return espp::detail::canopen::get_le(s->second.value.data(), n);
  }
  uint16_t read_u16(uint16_t index, uint8_t sub) const {
    return static_cast<uint16_t>(read_raw(index, sub, 2));
  }
  // The dictionary holds raw two's-complement bytes; reinterpret them as the
  // signed type bit for bit (well-defined, unlike a narrowing static_cast of
  // an out-of-range unsigned value).
  static int8_t as_i8(uint32_t v) { return std::bit_cast<int8_t>(static_cast<uint8_t>(v)); }
  static int16_t as_i16(uint32_t v) { return std::bit_cast<int16_t>(static_cast<uint16_t>(v)); }
  static int32_t as_i32(uint32_t v) { return std::bit_cast<int32_t>(v); }
  int32_t read_i32(uint16_t index, uint8_t sub) const { return as_i32(read_raw(index, sub, 4)); }
  void store(uint16_t index, uint8_t sub, uint32_t v, size_t n) {
    auto it = od_.find(index);
    if (it == od_.end())
      return;
    auto s = it->second.subs.find(sub);
    if (s != it->second.subs.end())
      s->second.value = le(v, n);
  }
  uint16_t heartbeat_ms() const { return read_u16(0x1017, 0); }

  /// Mirror the live drive state into the dictionary before a read.
  void sync_dynamic_objects() {
    store(0x6041, 0, compose_statusword(), 2);
    store(0x6061, 0, static_cast<uint8_t>(mode_display_), 1);
    store(0x6062, 0, static_cast<uint32_t>(target_position_), 4);
    store(0x6064, 0, static_cast<uint32_t>(static_cast<int32_t>(position_)), 4);
    store(0x606C, 0, static_cast<uint32_t>(static_cast<int32_t>(velocity_)), 4);
    store(0x6077, 0, static_cast<uint16_t>(torque_actual_), 2);
    store(0x1001, 0, state_ == State::Fault ? 0x01 : 0x00, 1);
    store(0x603F, 0, state_ == State::Fault ? 0xFF00 : 0x0000, 2);
    store(0x2001, 0, tick_count_, 4);
  }

  /// Validation and side effects of a write; the value is stored by the caller on success.
  bool on_write(uint16_t index, uint8_t sub, std::span<const uint8_t> data, uint32_t &abort,
                Frames &out) {
    namespace co = espp::detail::canopen;
    const uint32_t v = co::get_le(data.data(), std::min<size_t>(data.size(), 4));
    if ((index == 0x1010 || index == 0x1011) && sub != 1)
      return true; // only sub 1 (all parameters) carries the signature
    switch (index) {
    case 0x1010:
      if (v != kSaveSignature) {
        abort = kAbortStore;
        return false;
      }
      return true; // nothing persistent in a simulation: accept the signature
    case 0x1011:
      if (v != kLoadSignature) {
        abort = kAbortStore;
        return false;
      }
      restore_defaults();
      return true;
    case 0x1017:
      heartbeat_elapsed_ms_ = 0;
      return true;
    case 0x2000:
      if (v && state_ != State::Fault)
        enter_fault(out);
      return true;
    case 0x6040:
      apply_controlword(static_cast<uint16_t>(v), out);
      return true;
    case 0x6060:
      if (!set_mode(as_i8(v))) {
        abort = kAbortRange;
        return false;
      }
      return true;
    case 0x605A:
      if (as_i16(v) < 0 || as_i16(v) > 8) {
        abort = kAbortRange;
        return false;
      }
      return true;
    case 0x607D: {
      // min must stay below max; the axis is pulled inside the new limits
      const int32_t nv = as_i32(v);
      const int32_t other = read_i32(0x607D, sub == 1 ? 2 : 1);
      if ((sub == 1 && nv >= other) || (sub == 2 && nv <= other)) {
        abort = kAbortRange;
        return false;
      }
      pending_limit_clamp_ = true;
      return true;
    }
    default:
      return true;
    }
  }

  // ---- NMT ----------------------------------------------------------------
  void handle_nmt(espp::detail::canopen::NmtCommand cmd, Frames & /*out*/) {
    using espp::detail::canopen::NmtCommand;
    switch (cmd) {
    case NmtCommand::Start:
      nmt_ = NmtState::Operational;
      break;
    case NmtCommand::Stop:
      nmt_ = NmtState::Stopped;
      break;
    case NmtCommand::PreOperational:
      nmt_ = NmtState::PreOperational;
      break;
    case NmtCommand::ResetNode:
      reset_application();
      break;
    case NmtCommand::ResetCommunication:
      reset_communication();
      break;
    default:
      break;
    }
  }

  // ---- SDO server ---------------------------------------------------------
  CanFrame sdo_frame(uint8_t cs, uint16_t index, uint8_t sub) const {
    CanFrame f;
    f.id = espp::detail::canopen::COB_SDO_TX_BASE + config_.node_id;
    f.dlc = 8;
    f.data[0] = cs;
    espp::detail::canopen::put_le(index, &f.data[1], 2);
    f.data[3] = sub;
    return f;
  }
  void sdo_abort(uint16_t index, uint8_t sub, uint32_t code, Frames &out) {
    sdo_ = {};
    CanFrame f = sdo_frame(0x80, index, sub);
    espp::detail::canopen::put_le(code, &f.data[4], 4);
    out.push_back(f);
  }

  void handle_sdo(const CanFrame &in, Frames &out) {
    namespace co = espp::detail::canopen;
    const uint8_t cs = in.data[0];
    const uint8_t ccs = cs >> 5;
    // bytes 1..3 are the multiplexer only in initiate (ccs 1 / 2) and abort
    // frames; in segment frames they are payload, so an abort raised there
    // names the active transfer (or 0/0 when there is none)
    const bool has_mux = ccs == 1 || ccs == 2 || ccs == 4;
    const uint16_t index = has_mux ? static_cast<uint16_t>(co::get_le(&in.data[1], 2))
                                   : (sdo_.active ? sdo_.index : 0);
    const uint8_t sub = has_mux ? in.data[3] : (sdo_.active ? sdo_.sub : 0);
    uint32_t abort = 0;
    switch (ccs) {
    case 1: { // initiate download (write)
      const bool expedited = (cs & 0x02) != 0;
      const bool sized = (cs & 0x01) != 0;
      sdo_ = {};
      if (expedited) {
        const size_t n = sized ? 4 - ((cs >> 2) & 0x03) : 4;
        if (!write_object(index, sub, std::span<const uint8_t>(&in.data[4], n), abort, out)) {
          sdo_abort(index, sub, abort, out);
          return;
        }
        out.push_back(sdo_frame(0x60, index, sub));
        return;
      }
      // segmented download: validate the object now, collect the segments
      {
        const Entry *e = find(index, sub, abort);
        if (!e) {
          sdo_abort(index, sub, abort, out);
          return;
        }
        if (e->access == Access::Const || e->access == Access::ReadOnly) {
          sdo_abort(index, sub, kAbortReadOnly, out);
          return;
        }
        if (sized) {
          const uint32_t total = co::get_le(&in.data[4], 4);
          const size_t fixed = fixed_size(e->data_type);
          if ((fixed && total != fixed) || (!fixed && total > kMaxStringBytes)) {
            sdo_abort(index, sub,
                      total > (fixed ? fixed : kMaxStringBytes) ? kAbortLengthHigh
                                                                : kAbortLengthLow,
                      out);
            return;
          }
        }
      }
      sdo_.active = true;
      sdo_.upload = false;
      sdo_.index = index;
      sdo_.sub = sub;
      out.push_back(sdo_frame(0x60, index, sub));
      return;
    }
    case 0: { // download segment
      if (!sdo_.active || sdo_.upload) {
        sdo_abort(index, sub, kAbortUnknownCs, out);
        return;
      }
      const bool toggle = (cs & 0x10) != 0;
      if (toggle != sdo_.toggle) {
        sdo_abort(sdo_.index, sdo_.sub, kAbortToggle, out);
        return;
      }
      const size_t n = 7 - ((cs >> 1) & 0x07);
      sdo_.buf.insert(sdo_.buf.end(), &in.data[1], &in.data[1] + n);
      if (sdo_.buf.size() > kMaxStringBytes) {
        sdo_abort(sdo_.index, sdo_.sub, kAbortLengthHigh, out);
        return;
      }
      CanFrame f;
      f.id = co::COB_SDO_TX_BASE + config_.node_id;
      f.dlc = 8;
      f.data[0] = static_cast<uint8_t>(0x20 | (toggle ? 0x10 : 0x00));
      sdo_.toggle = !sdo_.toggle;
      if (cs & 0x01) { // last segment: commit
        const uint16_t idx = sdo_.index;
        const uint8_t s = sdo_.sub;
        std::vector<uint8_t> data = std::move(sdo_.buf);
        sdo_ = {};
        if (!write_object(idx, s, data, abort, out)) {
          sdo_abort(idx, s, abort, out);
          return;
        }
      }
      out.push_back(f);
      return;
    }
    case 2: { // initiate upload (read)
      sdo_ = {};
      auto value = read_object(index, sub, abort);
      if (!value) {
        sdo_abort(index, sub, abort, out);
        return;
      }
      if (value->size() <= 4) {
        const size_t n = value->size();
        CanFrame f =
            sdo_frame(static_cast<uint8_t>(0x40 | ((4 - n) << 2) | 0x02 | 0x01), index, sub);
        std::copy_n(value->begin(), n, f.data.begin() + 4);
        out.push_back(f);
        return;
      }
      sdo_.active = true;
      sdo_.upload = true;
      sdo_.index = index;
      sdo_.sub = sub;
      sdo_.buf = std::move(*value);
      CanFrame f = sdo_frame(0x41, index, sub); // segmented, size indicated
      co::put_le(static_cast<uint32_t>(sdo_.buf.size()), &f.data[4], 4);
      out.push_back(f);
      return;
    }
    case 3: { // upload segment
      if (!sdo_.active || !sdo_.upload) {
        sdo_abort(index, sub, kAbortUnknownCs, out);
        return;
      }
      const bool toggle = (cs & 0x10) != 0;
      if (toggle != sdo_.toggle) {
        sdo_abort(sdo_.index, sdo_.sub, kAbortToggle, out);
        return;
      }
      const size_t remaining = sdo_.buf.size() - sdo_.offset;
      const size_t n = std::min<size_t>(7, remaining);
      const bool last = remaining <= 7;
      CanFrame f;
      f.id = co::COB_SDO_TX_BASE + config_.node_id;
      f.dlc = 8;
      f.data[0] = static_cast<uint8_t>((toggle ? 0x10 : 0x00) | ((7 - n) << 1) | (last ? 1 : 0));
      std::copy_n(sdo_.buf.begin() + static_cast<std::ptrdiff_t>(sdo_.offset), n,
                  f.data.begin() + 1);
      sdo_.offset += n;
      sdo_.toggle = !sdo_.toggle;
      if (last)
        sdo_ = {};
      out.push_back(f);
      return;
    }
    case 4: // abort from the client
      sdo_ = {};
      return;
    default:
      sdo_abort(index, sub, kAbortUnknownCs, out);
      return;
    }
  }

  // ---- CiA 402 state machine --------------------------------------------
  uint16_t compose_statusword() const {
    uint16_t sw = 0;
    switch (state_) {
    case State::NotReadyToSwitchOn:
      sw = 0x0000;
      break;
    case State::SwitchOnDisabled:
      sw = 0x0040;
      break;
    case State::ReadyToSwitchOn:
      sw = 0x0021;
      break;
    case State::SwitchedOn:
      sw = 0x0023;
      break;
    case State::OperationEnabled:
      sw = 0x0027;
      break;
    case State::QuickStopActive:
      sw = 0x0007;
      break;
    case State::FaultReactionActive:
      sw = 0x000F;
      break;
    case State::Fault:
      sw = 0x0008;
      break;
    default:
      break;
    }
    // bit 4 (voltage enabled): high voltage is applied once the drive left
    // Switch On Disabled (the simulated supply is switched with the enable
    // voltage command)
    if (state_ != State::NotReadyToSwitchOn && state_ != State::SwitchOnDisabled)
      sw |= 0x0010;
    sw |= 0x0200; // remote
    if (target_reached_)
      sw |= 0x0400;
    switch (mode_display_) {
    case kModePp:
      if (setpoint_ack_)
        sw |= 0x1000;
      break;
    case kModePv:
      if (std::abs(velocity_) < 0.5)
        sw |= 0x1000; // speed (zero)
      break;
    case kModeHm:
      if (homing_attained_)
        sw |= 0x1000;
      break;
    default:
      break;
    }
    return sw;
  }

  bool set_mode(int8_t mode) {
    if (mode != 0 && !(mode > 0 && mode < 32 && ((kSupportedModes >> (mode - 1)) & 1)))
      return false;
    mode_ = mode;
    mode_display_ = mode;
    setpoint_ack_ = false;
    homing_attained_ = false;
    return true;
  }

  void apply_controlword(uint16_t cw, Frames &out) {
    const bool switch_on = cw & 0x01, enable_voltage = cw & 0x02, quick_stop = cw & 0x04,
               enable_op = cw & 0x08, fault_reset_edge = (cw & 0x80) && !(prev_controlword_ & 0x80);
    const bool new_setpoint_edge = (cw & 0x10) && !(prev_controlword_ & 0x10);
    prev_controlword_ = cw;
    if (state_ == State::Fault) {
      if (fault_reset_edge) {
        state_ = State::SwitchOnDisabled;
        emit_emcy(0x0000, out); // error reset / no error
      }
      return;
    }
    if (state_ == State::NotReadyToSwitchOn || state_ == State::FaultReactionActive)
      return;
    if (!enable_voltage) {
      state_ = State::SwitchOnDisabled; // disable voltage (transitions 7, 9, 10, 12)
    } else if (!quick_stop) {
      if (state_ == State::OperationEnabled) {
        state_ = State::QuickStopActive; // transition 11; -> SOD once stopped (0x605A = 2)
        quick_stopping_ = true;
      } else if (state_ != State::QuickStopActive) {
        state_ = State::SwitchOnDisabled; // transitions 7, 10
      }
    } else if (!switch_on) {
      if (state_ != State::QuickStopActive)
        state_ = State::ReadyToSwitchOn; // shutdown (transitions 2, 6, 8)
    } else if (!enable_op) {
      if (state_ == State::ReadyToSwitchOn || state_ == State::OperationEnabled)
        state_ = State::SwitchedOn; // switch on (3) / disable operation (5)
    } else {
      if (state_ == State::ReadyToSwitchOn || state_ == State::SwitchedOn ||
          state_ == State::QuickStopActive)
        state_ = State::OperationEnabled; // enable operation (3+4, 4, 16)
    }
    // mode-specific controlword bits (only meaningful in Operation Enabled)
    if (state_ == State::OperationEnabled) {
      if (mode_display_ == kModePp) {
        if (new_setpoint_edge) {
          const int32_t target = read_i32(0x607A, 0);
          // relative: sum in 64 bits, then clamp into the software position
          // limits (which are themselves inside the int32 range)
          int64_t t = (cw & 0x40) ? static_cast<int64_t>(static_cast<int32_t>(position_)) + target
                                  : static_cast<int64_t>(target);
          const auto [lo, hi] = position_limits();
          t = std::clamp<int64_t>(t, static_cast<int64_t>(lo), static_cast<int64_t>(hi));
          target_position_ = static_cast<int32_t>(t);
          target_reached_ = false;
          setpoint_ack_ = true;
        }
        if (!(cw & 0x10))
          setpoint_ack_ = false; // released once the master drops bit 4
        // bit 8 (halt) stops the move
        halt_ = (cw & 0x100) != 0;
      } else if (mode_display_ == kModeHm) {
        if (new_setpoint_edge && !homing_attained_) {
          homing_ms_ = 0;
          homing_active_ = true;
          target_reached_ = false;
        }
      } else if (mode_display_ == kModePv) {
        halt_ = (cw & 0x100) != 0;
      }
    }
  }

  void enter_fault(Frames &out) {
    state_ = State::Fault;
    velocity_ = 0.0;
    torque_actual_ = 0;
    quick_stopping_ = false;
    emit_emcy(0xFF00, out); // device-specific error
  }

  void emit_emcy(uint16_t error_code, Frames &out) const {
    CanFrame f;
    f.id = espp::detail::canopen::COB_EMCY_BASE + config_.node_id;
    f.dlc = 8;
    espp::detail::canopen::put_le(error_code, &f.data[0], 2);
    f.data[2] = error_code ? 0x01 : 0x00; // error register
    out.push_back(f);
  }

  CanFrame make_tpdo1() const {
    CanFrame f;
    f.id = espp::detail::canopen::COB_TPDO1_BASE + config_.node_id;
    f.dlc = 6;
    espp::detail::canopen::put_le(compose_statusword(), &f.data[0], 2);
    espp::detail::canopen::put_le(static_cast<uint32_t>(static_cast<int32_t>(position_)),
                                  &f.data[2], 4);
    return f;
  }

  // ---- motion model -------------------------------------------------------
  static double toward(double v, double target, double rate, double dt) {
    const double step = rate * dt;
    if (v < target)
      return std::min(v + step, target);
    if (v > target)
      return std::max(v - step, target);
    return v;
  }

  void step_motion(double dt, Frames & /*out*/) {
    ++tick_count_;
    if (pending_limit_clamp_) {
      pending_limit_clamp_ = false;
      clamp_position();
    }
    const double vmax = static_cast<double>(read_raw(0x6081, 0, 4));
    const double accel = std::max(1.0, static_cast<double>(read_raw(0x6083, 0, 4)));
    const double decel = std::max(1.0, static_cast<double>(read_raw(0x6084, 0, 4)));
    const double qs_decel = std::max(1.0, static_cast<double>(read_raw(0x6085, 0, 4)));
    if (state_ == State::QuickStopActive) {
      velocity_ = toward(velocity_, 0.0, qs_decel, dt);
      position_ += velocity_ * dt;
      clamp_position();
      if (velocity_ == 0.0 && quick_stopping_) {
        quick_stopping_ = false;
        if (as_i16(read_u16(0x605A, 0)) <= 4)
          state_ = State::SwitchOnDisabled; // option codes 0..4: transit after the stop
      }
      return;
    }
    if (state_ != State::OperationEnabled) {
      velocity_ = 0.0; // power stage off: the (ideal) axis holds
      torque_actual_ = 0;
      return;
    }
    switch (mode_display_) {
    case kModePv: {
      const double target = halt_ ? 0.0 : static_cast<double>(read_i32(0x60FF, 0));
      const double rate = std::abs(target) > std::abs(velocity_) ? accel : decel;
      velocity_ = toward(velocity_, target, rate, dt);
      position_ += velocity_ * dt;
      clamp_position();
      target_reached_ = velocity_ == target;
      break;
    }
    case kModePp: {
      const double remaining = static_cast<double>(target_position_) - position_;
      if (halt_ || std::abs(remaining) < 0.5) {
        velocity_ = toward(velocity_, 0.0, decel, dt);
        if (velocity_ == 0.0 && !halt_) {
          position_ = static_cast<double>(target_position_);
          target_reached_ = true;
        }
      } else {
        const double dir = remaining > 0 ? 1.0 : -1.0;
        // decelerate when the stopping distance reaches the remaining distance
        const double stop_dist = (velocity_ * velocity_) / (2.0 * decel);
        double desired = dir * vmax;
        if (std::abs(remaining) <= stop_dist)
          desired = 0.0;
        velocity_ =
            toward(velocity_, desired, std::abs(desired) > std::abs(velocity_) ? accel : decel, dt);
        // never overshoot the target within one tick
        const double step = velocity_ * dt;
        if (std::abs(step) >= std::abs(remaining)) {
          position_ = static_cast<double>(target_position_);
          velocity_ = 0.0;
          target_reached_ = true;
        } else {
          position_ += step;
          clamp_position();
        }
      }
      break;
    }
    case kModeTq:
      torque_actual_ = as_i16(read_u16(0x6071, 0));
      target_reached_ = true;
      break;
    case kModeHm:
      if (homing_active_) {
        homing_ms_ += static_cast<uint32_t>(dt * 1000.0);
        if (homing_ms_ >= kHomingDurationMs) {
          homing_active_ = false;
          homing_attained_ = true;
          target_reached_ = true;
          position_ = 0.0;
          velocity_ = 0.0;
        }
      }
      break;
    default:
      velocity_ = toward(velocity_, 0.0, decel, dt);
      position_ += velocity_ * dt;
      clamp_position();
      break;
    }
  }

  Config config_;
  std::map<uint16_t, Object> od_;
  std::string eds_cache_;
  SdoTransfer sdo_;
  NmtState nmt_{NmtState::PreOperational};
  bool boot_up_pending_{true};
  uint32_t heartbeat_elapsed_ms_{0};
  uint32_t tpdo_elapsed_ms_{0};
  uint32_t tick_count_{0};
  // drive
  State state_{State::SwitchOnDisabled};
  int8_t mode_{0};
  int8_t mode_display_{0};
  uint16_t prev_controlword_{0};
  double position_{0.0};
  double velocity_{0.0};
  int32_t target_position_{0};
  int16_t torque_actual_{0};
  bool target_reached_{true};
  bool setpoint_ack_{false};
  bool halt_{false};
  bool quick_stopping_{false};
  bool homing_active_{false};
  bool homing_attained_{false};
  uint32_t homing_ms_{0};
  bool pending_limit_clamp_{false}; // 0x607D changed: re-clamp on the next tick
};

} // namespace can_bridge
