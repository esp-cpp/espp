// Host-buildable tests for the bridge example's simulated CiA 402 node
// (main/simulated_ds402_node.hpp). Build & run with:
//   c++ -std=c++20 -I../../include -I../main simulated_node_host_test.cpp -o test && ./test
//
// The node is driven with the canopen component's client-side frame builders
// / parsers, so a passing run means espp::CanopenClient / Ds402Drive (and the
// browser DS402 panel, which speaks the same wire protocol) would see a
// conforming CiA 301 / 402 device.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "simulated_ds402_node.hpp"

namespace co = espp::detail::canopen;
namespace ds = espp::detail::ds402;
using can_bridge::SimulatedDs402Node;
using espp::detail::CanFrame;
using Frames = SimulatedDs402Node::Frames;
using namespace std::chrono_literals;

static int g_failures = 0;
#define CHECK(cond)                                                                                \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("  FAIL: %s (line %d)\n", #cond, __LINE__);                                      \
      ++g_failures;                                                                                \
    }                                                                                              \
  } while (0)

static constexpr uint8_t kNode = 7;

// One SDO request -> the single response frame on 0x580 + node.
static co::SdoResponse sdo(SimulatedDs402Node &node, const CanFrame &request) {
  Frames out;
  node.process(request, out);
  co::SdoResponse r;
  for (const auto &f : out)
    if (f.id == co::COB_SDO_TX_BASE + kNode)
      return co::parse_sdo_response(f);
  return r; // Unknown: no reply
}

static std::optional<uint32_t> read_u(SimulatedDs402Node &node, uint16_t index, uint8_t sub,
                                      size_t n) {
  auto r = sdo(node, co::make_sdo_upload_request(kNode, index, sub));
  if (r.type != co::SdoResponse::Type::ExpeditedUpload || r.len != n)
    return std::nullopt;
  return co::get_le(r.data.data(), n);
}

static bool write_u(SimulatedDs402Node &node, uint16_t index, uint8_t sub, uint32_t v, size_t n,
                    uint32_t *abort = nullptr) {
  uint8_t b[4];
  co::put_le(v, b, n);
  auto r =
      sdo(node, co::make_sdo_expedited_download(kNode, index, sub, std::span<const uint8_t>(b, n)));
  if (abort)
    *abort = r.type == co::SdoResponse::Type::Abort ? r.abort_code : 0;
  return r.type == co::SdoResponse::Type::DownloadOk;
}

// A full segmented upload (initiate + segments, with the toggle bit).
static std::optional<std::vector<uint8_t>> read_segmented(SimulatedDs402Node &node, uint16_t index,
                                                          uint8_t sub) {
  auto r = sdo(node, co::make_sdo_upload_request(kNode, index, sub));
  if (r.type == co::SdoResponse::Type::ExpeditedUpload)
    return std::vector<uint8_t>(r.data.begin(), r.data.begin() + r.len);
  if (r.type != co::SdoResponse::Type::SegmentedUploadInit)
    return std::nullopt;
  const uint32_t total = r.size_indicated ? r.total_size : 0;
  std::vector<uint8_t> out;
  bool toggle = false;
  for (int i = 0; i < 100000; ++i) {
    auto s = sdo(node, co::make_sdo_upload_segment_request(kNode, toggle));
    if (s.type != co::SdoResponse::Type::UploadSegment || s.toggle != toggle)
      return std::nullopt;
    out.insert(out.end(), s.data.begin(), s.data.begin() + s.len);
    toggle = !toggle;
    if (s.last)
      break;
  }
  if (total && out.size() != total)
    return std::nullopt;
  return out;
}

static void tick(SimulatedDs402Node &node, std::chrono::milliseconds total, Frames *sink = nullptr,
                 std::chrono::milliseconds step = 10ms) {
  Frames out;
  for (auto t = 0ms; t < total; t += step) {
    out.clear();
    node.tick(step, out);
    if (sink)
      sink->insert(sink->end(), out.begin(), out.end());
  }
}

static ds::State state(SimulatedDs402Node &node) {
  auto sw = read_u(node, ds::OBJ_STATUSWORD, 0, 2);
  return sw ? ds::decode_state(static_cast<uint16_t>(*sw)) : ds::State::Unknown;
}

static SimulatedDs402Node make_node() {
  SimulatedDs402Node::Config cfg;
  cfg.node_id = kNode;
  return SimulatedDs402Node(cfg);
}

static void test_boot_and_nmt() {
  std::printf("test_boot_and_nmt\n");
  auto node = make_node();
  Frames out;
  node.tick(1ms, out);
  // the boot-up message comes first, once
  CHECK(!out.empty() && out[0].id == co::COB_HEARTBEAT_BASE + kNode && out[0].dlc == 1 &&
        out[0].data[0] == 0x00);
  uint8_t nid = 0;
  CHECK(co::parse_heartbeat(out[0], nid) == co::NmtState::BootUp && nid == kNode);
  CHECK(node.nmt_state() == co::NmtState::PreOperational);
  // heartbeat every 0x1017 ms (1000 by default)
  out.clear();
  tick(node, 2500ms, &out);
  int hb = 0;
  for (const auto &f : out)
    if (f.id == co::COB_HEARTBEAT_BASE + kNode) {
      ++hb;
      CHECK(f.data[0] == static_cast<uint8_t>(co::NmtState::PreOperational));
    }
  CHECK(hb == 2);
  // NMT start -> Operational (the heartbeat reports it), stop -> no SDO
  out.clear();
  node.process(co::make_nmt(co::NmtCommand::Start, kNode), out);
  CHECK(node.nmt_state() == co::NmtState::Operational);
  node.process(co::make_nmt(co::NmtCommand::Stop, 0), out); // broadcast
  CHECK(node.nmt_state() == co::NmtState::Stopped);
  CHECK(sdo(node, co::make_sdo_upload_request(kNode, 0x1000, 0)).type ==
        co::SdoResponse::Type::Unknown);
  // an NMT for another node is ignored
  node.process(co::make_nmt(co::NmtCommand::Start, kNode + 1), out);
  CHECK(node.nmt_state() == co::NmtState::Stopped);
  // reset communication: boot-up again, pre-operational, SDO back
  node.process(co::make_nmt(co::NmtCommand::ResetCommunication, kNode), out);
  out.clear();
  node.tick(1ms, out);
  CHECK(!out.empty() && out[0].data[0] == 0x00);
  CHECK(node.nmt_state() == co::NmtState::PreOperational);
  CHECK(read_u(node, 0x1000, 0, 4) == 0x00020192u);
  // a heartbeat time change takes effect
  CHECK(write_u(node, 0x1017, 0, 200, 2));
  out.clear();
  tick(node, 1050ms, &out);
  hb = 0;
  for (const auto &f : out)
    if (f.id == co::COB_HEARTBEAT_BASE + kNode)
      ++hb;
  CHECK(hb == 5);
}

static void test_identity_and_strings() {
  std::printf("test_identity_and_strings\n");
  auto node = make_node();
  CHECK(read_u(node, ds::OBJ_IDENTITY, 0, 1) == 4u);
  CHECK(read_u(node, ds::OBJ_IDENTITY, 1, 4) == 0x1209u);
  CHECK(read_u(node, ds::OBJ_IDENTITY, 2, 4) == 0xC0DEu);
  CHECK(read_u(node, 0x1022, 0, 2) == 0u);
  CHECK(read_u(node, 0x6502, 0, 4) == 0x2Du);
  CHECK(read_u(node, 0x1200, 1, 4) == 0x600u + kNode);
  // the device name is a segmented upload
  auto name = read_segmented(node, ds::OBJ_DEVICE_NAME, 0);
  CHECK(name && std::string(name->begin(), name->end()) == "espp simulated DS402 drive");
  // an expedited-size string still comes back expedited
  auto hw = read_segmented(node, 0x1009, 0);
  CHECK(hw && std::string(hw->begin(), hw->end()) == "sim");
}

static void test_sdo_aborts() {
  std::printf("test_sdo_aborts\n");
  auto node = make_node();
  uint32_t abort = 0;
  auto r = sdo(node, co::make_sdo_upload_request(kNode, 0x1234, 0));
  CHECK(r.type == co::SdoResponse::Type::Abort && r.abort_code == 0x06020000 && r.index == 0x1234);
  r = sdo(node, co::make_sdo_upload_request(kNode, ds::OBJ_IDENTITY, 9));
  CHECK(r.type == co::SdoResponse::Type::Abort && r.abort_code == 0x06090011);
  CHECK(!write_u(node, ds::OBJ_STATUSWORD, 0, 1, 2, &abort) && abort == 0x06010002);
  CHECK(!write_u(node, ds::OBJ_CONTROLWORD, 0, 1, 4, &abort) && abort == 0x06070012);
  CHECK(!write_u(node, ds::OBJ_CONTROLWORD, 0, 1, 1, &abort) && abort == 0x06070013);
  CHECK(!write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 2, 1, &abort) && abort == 0x06090030);
  CHECK(!write_u(node, 0x1010, 1, 0x12345678, 4, &abort) && abort == 0x08000020);
  CHECK(write_u(node, 0x1010, 1, 0x65766173, 4)); // "save"
  // unknown command specifier
  CanFrame bad;
  bad.id = co::COB_SDO_RX_BASE + kNode;
  bad.dlc = 8;
  bad.data[0] = 0xE0;
  r = sdo(node, bad);
  CHECK(r.type == co::SdoResponse::Type::Abort && r.abort_code == 0x05040001);
  // a segment request with no transfer in progress
  r = sdo(node, co::make_sdo_upload_segment_request(kNode, false));
  CHECK(r.type == co::SdoResponse::Type::Abort);
  // toggle bit not alternated on a segmented upload
  r = sdo(node, co::make_sdo_upload_request(kNode, ds::OBJ_DEVICE_NAME, 0));
  CHECK(r.type == co::SdoResponse::Type::SegmentedUploadInit && r.total_size == 26);
  r = sdo(node, co::make_sdo_upload_segment_request(kNode, false));
  CHECK(r.type == co::SdoResponse::Type::UploadSegment && !r.last);
  r = sdo(node, co::make_sdo_upload_segment_request(kNode, false)); // should be true
  CHECK(r.type == co::SdoResponse::Type::Abort && r.abort_code == 0x05030000);
  // RTR / extended / short frames on the SDO id are ignored, not decoded
  CanFrame rtr = co::make_sdo_upload_request(kNode, 0x1000, 0);
  rtr.rtr = true;
  CHECK(sdo(node, rtr).type == co::SdoResponse::Type::Unknown);
  CanFrame ext = co::make_sdo_upload_request(kNode, 0x1000, 0);
  ext.extended = true;
  CHECK(sdo(node, ext).type == co::SdoResponse::Type::Unknown);
}

static void test_segmented_download() {
  std::printf("test_segmented_download\n");
  auto node = make_node();
  // 0x1017 accepts only its fixed size through a segmented download too
  CanFrame init;
  init.id = co::COB_SDO_RX_BASE + kNode;
  init.dlc = 8;
  init.data[0] = 0x21; // ccs=1, e=0, s=1
  co::put_le(0x1017, &init.data[1], 2);
  init.data[3] = 0;
  co::put_le(2, &init.data[4], 4);
  auto r = sdo(node, init);
  CHECK(r.type == co::SdoResponse::Type::DownloadOk);
  CanFrame seg;
  seg.id = co::COB_SDO_RX_BASE + kNode;
  seg.dlc = 8;
  seg.data[0] = static_cast<uint8_t>((0 << 4) | ((7 - 2) << 1) | 1); // t=0, n=5, c=1
  co::put_le(300, &seg.data[1], 2);
  Frames out;
  node.process(seg, out);
  CHECK(!out.empty() && (out[0].data[0] & 0xE0) == 0x20 && (out[0].data[0] & 0x10) == 0);
  CHECK(read_u(node, 0x1017, 0, 2) == 300u);
  // a wrong declared size is refused at initiate
  co::put_le(4, &init.data[4], 4);
  r = sdo(node, init);
  CHECK(r.type == co::SdoResponse::Type::Abort && r.abort_code == 0x06070012);
}

static void test_eds_domain() {
  std::printf("test_eds_domain\n");
  auto node = make_node();
  auto eds = read_segmented(node, 0x1021, 0);
  CHECK(eds && eds->size() > 2000);
  if (!eds)
    return;
  const std::string text(eds->begin(), eds->end());
  CHECK(text == node.generate_eds());
  CHECK(text.find("[FileInfo]") != std::string::npos);
  CHECK(text.find("[DeviceInfo]\nVendorName=espp\n") != std::string::npos);
  CHECK(text.find("[MandatoryObjects]\nSupportedObjects=3\n1=0x1000\n2=0x1001\n3=0x1018\n") !=
        std::string::npos);
  CHECK(text.find("[1018]\nParameterName=Identity object\nObjectType=0x9\nSubNumber=5\n") !=
        std::string::npos);
  CHECK(text.find("[1018sub1]\nParameterName=Vendor-ID\nObjectType=0x7\nDataType=0x0007\n"
                  "AccessType=ro\nDefaultValue=0x00001209\nPDOMapping=0\n") != std::string::npos);
  CHECK(text.find("[6041]\nParameterName=Statusword\nObjectType=0x7\nDataType=0x0006\n"
                  "AccessType=ro\nDefaultValue=0x0000\nPDOMapping=1\n") != std::string::npos);
  CHECK(text.find("[1021]\nParameterName=Store EDS\nObjectType=0x7\nDataType=0x000F\n") !=
        std::string::npos);
  CHECK(text.find("[ManufacturerObjects]\nSupportedObjects=2\n1=0x2000\n2=0x2001\n") !=
        std::string::npos);
  // every object listed has a section
  size_t pos = 0;
  int listed = 0, sections = 0;
  while ((pos = text.find("=0x", pos)) != std::string::npos) {
    if (pos >= 2 && std::isdigit(static_cast<unsigned char>(text[pos - 1])) &&
        (text[pos - 2] == '\n' || std::isdigit(static_cast<unsigned char>(text[pos - 2])))) {
      const std::string sec = "[" + text.substr(pos + 3, 4) + "]";
      ++listed;
      if (text.find(sec) != std::string::npos)
        ++sections;
    }
    pos += 3;
  }
  CHECK(listed > 30 && listed == sections);
}

static void test_state_machine() {
  std::printf("test_state_machine\n");
  auto node = make_node();
  CHECK(state(node) == ds::State::SwitchOnDisabled);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2));
  CHECK(state(node) == ds::State::ReadyToSwitchOn);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SWITCH_ON, 2));
  CHECK(state(node) == ds::State::SwitchedOn);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION, 2));
  CHECK(state(node) == ds::State::OperationEnabled);
  // disable operation -> switched on; shutdown -> ready; disable voltage -> SOD
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SWITCH_ON, 2));
  CHECK(state(node) == ds::State::SwitchedOn);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2));
  CHECK(state(node) == ds::State::ReadyToSwitchOn);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_DISABLE_VOLTAGE, 2));
  CHECK(state(node) == ds::State::SwitchOnDisabled);
  // switch on + enable operation in one write (transitions 3 + 4)
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2));
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION, 2));
  CHECK(state(node) == ds::State::OperationEnabled);
  // quick stop: active until stopped, then Switch On Disabled (0x605A = 2)
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 3, 1)); // pv
  CHECK(write_u(node, ds::OBJ_TARGET_VELOCITY, 0, 1000, 4));
  tick(node, 500ms);
  CHECK(node.velocity() > 900);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_QUICK_STOP, 2));
  CHECK(state(node) == ds::State::QuickStopActive);
  tick(node, 2000ms);
  CHECK(node.velocity() == 0);
  CHECK(state(node) == ds::State::SwitchOnDisabled);
  // fault injection + EMCY + fault reset (rising edge of bit 7)
  Frames out;
  uint8_t one = 1;
  node.process(co::make_sdo_expedited_download(kNode, 0x2000, 0, std::span<const uint8_t>(&one, 1)),
               out);
  bool emcy = false;
  for (const auto &f : out)
    if (f.id == co::COB_EMCY_BASE + kNode && f.dlc == 8 && co::get_le(&f.data[0], 2) == 0xFF00)
      emcy = true;
  CHECK(emcy);
  CHECK(state(node) == ds::State::Fault);
  CHECK(read_u(node, ds::OBJ_ERROR_REGISTER, 0, 1) == 1u);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2)); // ignored in Fault
  CHECK(state(node) == ds::State::Fault);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, 0x0000, 2));
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_FAULT_RESET, 2));
  CHECK(state(node) == ds::State::SwitchOnDisabled);
  CHECK(read_u(node, ds::OBJ_ERROR_REGISTER, 0, 1) == 0u);
}

static void enable(SimulatedDs402Node &node) {
  write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2);
  write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION, 2);
}

static void test_profile_velocity_and_position() {
  std::printf("test_profile_velocity_and_position\n");
  auto node = make_node();
  // mode display follows the mode, only supported modes are accepted
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 3, 1));
  CHECK(read_u(node, ds::OBJ_MODES_OF_OPERATION_DISPLAY, 0, 1) == 3u);
  enable(node);
  // profile velocity: ramps at the profile acceleration, position integrates
  CHECK(write_u(node, ds::OBJ_PROFILE_ACCELERATION, 0, 2000, 4));
  CHECK(write_u(node, ds::OBJ_TARGET_VELOCITY, 0, 1000, 4));
  tick(node, 250ms);
  auto v = read_u(node, ds::OBJ_VELOCITY_ACTUAL, 0, 4);
  CHECK(v && static_cast<int32_t>(*v) > 400 && static_cast<int32_t>(*v) < 600);
  tick(node, 1000ms);
  CHECK(read_u(node, ds::OBJ_VELOCITY_ACTUAL, 0, 4) == 1000u);
  auto p = read_u(node, ds::OBJ_POSITION_ACTUAL, 0, 4);
  CHECK(p && static_cast<int32_t>(*p) > 900 && static_cast<int32_t>(*p) < 1100);
  CHECK(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & ds::SW_BIT_TARGET_REACHED);
  // reverse
  CHECK(write_u(node, ds::OBJ_TARGET_VELOCITY, 0, static_cast<uint32_t>(-500), 4));
  tick(node, 2000ms);
  CHECK(static_cast<int32_t>(read_u(node, ds::OBJ_VELOCITY_ACTUAL, 0, 4).value_or(0)) == -500);
  CHECK(write_u(node, ds::OBJ_TARGET_VELOCITY, 0, 0, 4));
  tick(node, 1000ms);
  CHECK(node.velocity() == 0);
  // profile position: new-set-point handshake, target reached
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 1, 1));
  const int32_t start = node.position();
  CHECK(write_u(node, ds::OBJ_TARGET_POSITION, 0, static_cast<uint32_t>(start + 3000), 4));
  CHECK(write_u(node, ds::OBJ_PROFILE_VELOCITY, 0, 2000, 4));
  CHECK(write_u(
      node, ds::OBJ_CONTROLWORD, 0,
      ds::CW_ENABLE_OPERATION | ds::CW_BIT_NEW_SETPOINT | ds::CW_BIT_CHANGE_SET_IMMEDIATELY, 2));
  auto sw = read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0);
  CHECK((sw & ds::SW_BIT_SETPOINT_ACKNOWLEDGE) && !(sw & ds::SW_BIT_TARGET_REACHED));
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION, 2));
  CHECK(!(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & ds::SW_BIT_SETPOINT_ACKNOWLEDGE));
  tick(node, 500ms);
  CHECK(!(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & ds::SW_BIT_TARGET_REACHED));
  tick(node, 5000ms);
  CHECK(node.position() == start + 3000);
  CHECK(node.velocity() == 0);
  CHECK(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & ds::SW_BIT_TARGET_REACHED);
  // relative move
  CHECK(write_u(node, ds::OBJ_TARGET_POSITION, 0, static_cast<uint32_t>(-1000), 4));
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0,
                ds::CW_ENABLE_OPERATION | ds::CW_BIT_NEW_SETPOINT | ds::CW_BIT_RELATIVE, 2));
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION, 2));
  tick(node, 5000ms);
  CHECK(node.position() == start + 2000);
  // torque mode mirrors the target; homing attains after a while
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 4, 1));
  CHECK(write_u(node, 0x6071, 0, 250, 2));
  tick(node, 20ms);
  CHECK(read_u(node, 0x6077, 0, 2) == 250u);
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 6, 1));
  CHECK(
      write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_ENABLE_OPERATION | ds::CW_BIT_NEW_SETPOINT, 2));
  tick(node, 100ms);
  CHECK(!(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & 0x1000));
  tick(node, 600ms);
  CHECK(read_u(node, ds::OBJ_STATUSWORD, 0, 2).value_or(0) & 0x1000);
  CHECK(node.position() == 0);
  // power stage off: the axis stops
  CHECK(write_u(node, ds::OBJ_MODES_OF_OPERATION, 0, 3, 1));
  CHECK(write_u(node, ds::OBJ_TARGET_VELOCITY, 0, 1000, 4));
  tick(node, 1000ms);
  CHECK(node.velocity() == 1000);
  CHECK(write_u(node, ds::OBJ_CONTROLWORD, 0, ds::CW_SHUTDOWN, 2));
  tick(node, 20ms);
  CHECK(node.velocity() == 0);
}

static void test_pdos() {
  std::printf("test_pdos\n");
  auto node = make_node();
  Frames out;
  // no TPDO in pre-operational; TPDO1 every 100 ms when operational
  tick(node, 500ms, &out);
  int tpdo = 0;
  for (const auto &f : out)
    if (f.id == co::COB_TPDO1_BASE + kNode)
      ++tpdo;
  CHECK(tpdo == 0);
  node.process(co::make_nmt(co::NmtCommand::Start, kNode), out);
  out.clear();
  tick(node, 1000ms, &out);
  tpdo = 0;
  for (const auto &f : out)
    if (f.id == co::COB_TPDO1_BASE + kNode) {
      ++tpdo;
      CHECK(f.dlc == 6 && ds::decode_state(static_cast<uint16_t>(co::get_le(&f.data[0], 2))) ==
                              ds::State::SwitchOnDisabled);
    }
  CHECK(tpdo == 10);
  // RPDO1: controlword + mode; the state machine follows it
  uint8_t rpdo[3];
  co::put_le(ds::CW_SHUTDOWN, rpdo, 2);
  rpdo[2] = 3;
  out.clear();
  node.process(co::make_pdo(co::COB_RPDO1_BASE + kNode, rpdo), out);
  CHECK(state(node) == ds::State::ReadyToSwitchOn);
  CHECK(read_u(node, ds::OBJ_MODES_OF_OPERATION_DISPLAY, 0, 1) == 3u);
  co::put_le(ds::CW_ENABLE_OPERATION, rpdo, 2);
  node.process(co::make_pdo(co::COB_RPDO1_BASE + kNode, rpdo), out);
  CHECK(state(node) == ds::State::OperationEnabled);
  // an RTR on the TPDO1 id triggers one
  CanFrame rtr;
  rtr.id = co::COB_TPDO1_BASE + kNode;
  rtr.rtr = true;
  rtr.dlc = 6;
  out.clear();
  node.process(rtr, out);
  CHECK(out.size() == 1 && out[0].id == co::COB_TPDO1_BASE + kNode && !out[0].rtr);
  // the event timer can be turned off
  CHECK(write_u(node, 0x1800, 5, 0, 2));
  out.clear();
  tick(node, 500ms, &out);
  tpdo = 0;
  for (const auto &f : out)
    if (f.id == co::COB_TPDO1_BASE + kNode)
      ++tpdo;
  CHECK(tpdo == 0);
  // reset node restores the defaults (heartbeat / event timer) and the drive state
  CHECK(write_u(node, 0x1017, 0, 50, 2));
  node.process(co::make_nmt(co::NmtCommand::ResetNode, kNode), out);
  CHECK(read_u(node, 0x1017, 0, 2) == 1000u);
  CHECK(read_u(node, 0x1800, 5, 2) == 100u);
  CHECK(state(node) == ds::State::SwitchOnDisabled);
  CHECK(node.nmt_state() == co::NmtState::PreOperational);
}

int main() {
  test_boot_and_nmt();
  test_identity_and_strings();
  test_sdo_aborts();
  test_segmented_download();
  test_eds_domain();
  test_state_machine();
  test_profile_velocity_and_position();
  test_pdos();
  if (g_failures) {
    std::printf("%d FAILURE(S)\n", g_failures);
    return 1;
  }
  std::printf("ALL TESTS PASSED\n");
  return 0;
}
