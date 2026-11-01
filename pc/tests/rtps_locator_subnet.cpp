// Unicast locator selection for a peer on several networks.
//
// A FastDDS or CycloneDDS host with several interfaces (Ethernet, Wi-Fi, a VPN,
// docker0) announces a unicast locator for each, in its own order. Only the
// locator on our subnet is reachable. These checks parse SEDP and SPDP
// parameter lists and confirm the engine keeps that one:
//
//  1. SEDP (TopicData): of several PID_UNICAST_LOCATORs, the first one on our
//     subnet is kept, wherever it is in the list.
//  2. SEDP: with none on our subnet, or no local address yet, the first
//     locator is kept.
//  3. SPDP (ParticipantProxyData): when the locator list is already full of
//     off-subnet locators (Config::SPDP_MAX_NUM_LOCATORS, 1 on ESP32), a later
//     on-subnet locator replaces one of them, for both the metatraffic and the
//     default unicast lists.
//
// Exits 0 on success, 1 on the first failed check.

#include <cstdio>
#include <cstring>
#include <span>
#include <vector>

#include "rtps/discovery/ParticipantProxyData.hpp"
#include "rtps/discovery/TopicData.hpp"
#include "rtps/entities/Participant.hpp"

namespace {

using rtps::SMElement::ParameterId;

constexpr rtps::Ip4AddressBytes kLocalIp{192, 168, 4, 1};
constexpr rtps::Ip4AddressBytes kOnSubnet{192, 168, 4, 2};
constexpr rtps::Ip4AddressBytes kWifi{10, 5, 15, 154};
constexpr rtps::Ip4AddressBytes kVpn{100, 76, 151, 42};

#define CHECK(cond, msg)                                                                           \
  do {                                                                                             \
    if (!(cond)) {                                                                                 \
      std::printf("FAIL: %s (line %d)\n", msg, __LINE__);                                          \
      return false;                                                                                \
    }                                                                                              \
  } while (0)

void push_u16(std::vector<uint8_t> &p, uint16_t v) {
  p.push_back(static_cast<uint8_t>(v & 0xFF));
  p.push_back(static_cast<uint8_t>(v >> 8));
}

// One locator parameter: pid(2) len(2) kind(4) port(4) address(16), little-endian.
void push_locator(std::vector<uint8_t> &p, ParameterId pid, const rtps::Ip4AddressBytes &ip) {
  const rtps::FullLengthLocator loc =
      rtps::FullLengthLocator::createUDPv4Locator(ip[0], ip[1], ip[2], ip[3], 7411);
  push_u16(p, pid);
  push_u16(p, sizeof(loc));
  const auto *bytes = reinterpret_cast<const uint8_t *>(&loc);
  p.insert(p.end(), bytes, bytes + sizeof(loc));
}

std::vector<uint8_t> with_sentinel(std::vector<uint8_t> p) {
  push_u16(p, ParameterId::PID_SENTINEL);
  push_u16(p, 0);
  return p;
}

rtps::Ip4AddressBytes sedp_unicast(const std::vector<rtps::Ip4AddressBytes> &announced,
                                   const rtps::Ip4AddressBytes &localIp) {
  std::vector<uint8_t> p;
  for (const auto &ip : announced) {
    push_locator(p, ParameterId::PID_UNICAST_LOCATOR, ip);
  }
  const std::vector<uint8_t> bytes = with_sentinel(p);
  rtps::TopicData td;
  if (!td.readFromBuffer(std::span<const uint8_t>(bytes.data(), bytes.size()), localIp)) {
    return {};
  }
  return td.unicastLocator.getIp4AddressBytes();
}

bool check_sedp() {
  // 1. The order FastDDS used on a host with Ethernet, Wi-Fi and Tailscale.
  CHECK(sedp_unicast({kOnSubnet, kWifi, kVpn}, kLocalIp) == kOnSubnet, "on-subnet first is kept");
  CHECK(sedp_unicast({kWifi, kVpn, kOnSubnet}, kLocalIp) == kOnSubnet, "on-subnet last is kept");
  CHECK(sedp_unicast({kWifi, kOnSubnet, kVpn}, kLocalIp) == kOnSubnet, "on-subnet middle is kept");
  // 2. Nothing on our subnet, or our address isn't known yet: the first locator.
  CHECK(sedp_unicast({kWifi, kVpn}, kLocalIp) == kWifi, "no on-subnet locator keeps the first");
  CHECK(sedp_unicast({kWifi, kOnSubnet}, {}) == kWifi, "no local address keeps the first");
  return true;
}

bool check_spdp() {
  // 3. Fill each list with off-subnet locators, then announce the on-subnet one.
  std::vector<uint8_t> p;
  for (int i = 0; i < rtps::Config::SPDP_MAX_NUM_LOCATORS; ++i) {
    push_locator(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, kWifi);
    push_locator(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, kWifi);
  }
  push_locator(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, kOnSubnet);
  push_locator(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, kOnSubnet);
  const std::vector<uint8_t> bytes = with_sentinel(p);

  rtps::Participant part;
  part.m_localIpAddress = kLocalIp;
  rtps::ParticipantProxyData proxy;
  rtps::CdrReader reader(rtps::asBytes(bytes.data(), bytes.size()));
  CHECK(proxy.readFromBuffer(reader, &part), "SPDP parse");

  const auto has_on_subnet = [](const auto &list) {
    for (const auto &loc : list) {
      if (loc.isValid() && loc.getIp4AddressBytes() == kOnSubnet) {
        return true;
      }
    }
    return false;
  };
  CHECK(has_on_subnet(proxy.m_metatrafficUnicastLocatorList),
        "on-subnet metatraffic locator replaces an off-subnet one");
  CHECK(has_on_subnet(proxy.m_defaultUnicastLocatorList),
        "on-subnet default locator replaces an off-subnet one");
  return true;
}

} // namespace

int main() {
  if (!check_sedp() || !check_spdp()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
