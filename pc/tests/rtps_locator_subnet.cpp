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
//  4. The network is the local address's prefix length, not always a /24:
//     isSameSubnetAddress() with explicit prefixes, prefixLengthFromNetmask(),
//     and the SEDP / SPDP selection on a /17 (the issue #865 host's Wi-Fi was
//     10.5.15.154/17).
//
// Exits 0 on success, 1 on the first failed check.

#include <algorithm>
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
// A /17: 10.5.0.0 - 10.5.127.255. kWifi is on it although its third octet
// differs from ours; kOther17 (10.5.128.0/17) is not.
constexpr rtps::Ip4AddressBytes kLocal17{10, 5, 0, 10};
constexpr rtps::Ip4AddressBytes kOther17{10, 5, 200, 7};

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
                                   const rtps::Ip4AddressBytes &localIp,
                                   uint8_t prefixLength = rtps::DEFAULT_SUBNET_PREFIX_LENGTH) {
  std::vector<uint8_t> p;
  for (const auto &ip : announced) {
    push_locator(p, ParameterId::PID_UNICAST_LOCATOR, ip);
  }
  const std::vector<uint8_t> bytes = with_sentinel(p);
  rtps::TopicData td;
  if (!td.readFromBuffer(std::span<const uint8_t>(bytes.data(), bytes.size()), localIp,
                         prefixLength)) {
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
  // 4. On a /17 the Wi-Fi address is on our network although its third octet
  // differs; with the /24 default it is not, and the first locator is kept.
  CHECK(sedp_unicast({kOnSubnet, kWifi}, kLocal17, 17) == kWifi, "/17: on-network locator is kept");
  CHECK(sedp_unicast({kOnSubnet, kWifi}, kLocal17) == kOnSubnet, "/24 default keeps the first");
  CHECK(sedp_unicast({kOther17, kWifi}, kLocal17, 17) == kWifi,
        "/17: the other half of the /16 is not our network");
  return true;
}

// A prefix length above 32 behaves as /32.
bool isSameSubnetPrefixClamp() {
  return rtps::isSameSubnetAddress(kLocal17, kLocal17, 40) &&
         !rtps::isSameSubnetAddress(kWifi, kLocal17, 40);
}

bool check_prefix() {
  using rtps::isSameSubnetAddress;
  using rtps::prefixLengthFromNetmask;
  CHECK(isSameSubnetAddress(kOnSubnet, kLocalIp), "default is a /24");
  CHECK(!isSameSubnetAddress(kWifi, kLocalIp), "different /24");
  CHECK(isSameSubnetAddress(kWifi, kLocal17, 17), "same /17 across the third octet");
  CHECK(!isSameSubnetAddress(kOther17, kLocal17, 17), "other /17");
  CHECK(isSameSubnetAddress(kOther17, kLocal17, 16), "same /16");
  CHECK(!isSameSubnetAddress(kWifi, kLocal17, 32), "/32 is the address itself");
  CHECK(isSameSubnetAddress(kLocal17, kLocal17, 32), "/32 matches itself");
  CHECK(isSameSubnetAddress(kVpn, kLocalIp, 0), "/0 is every address");
  CHECK(isSameSubnetPrefixClamp(), "prefix > 32 behaves as /32");
  CHECK(prefixLengthFromNetmask({255, 255, 255, 0}) == 24, "255.255.255.0 -> 24");
  CHECK(prefixLengthFromNetmask({255, 255, 128, 0}) == 17, "255.255.128.0 -> 17");
  CHECK(prefixLengthFromNetmask({255, 255, 255, 255}) == 32, "255.255.255.255 -> 32");
  CHECK(prefixLengthFromNetmask({0, 0, 0, 0}) == 0, "0.0.0.0 -> 0");
  CHECK(prefixLengthFromNetmask({255, 0, 255, 0}) == rtps::DEFAULT_SUBNET_PREFIX_LENGTH,
        "a non-contiguous mask is not a netmask: default");
  return true;
}

// 3. Fill each list with `offNetwork` locators, then announce `onNetwork`; the
// participant's network is `localIp`/`prefixLength`. Returns whether both
// unicast lists end up holding `onNetwork`.
bool spdp_replaces(const rtps::Ip4AddressBytes &offNetwork, const rtps::Ip4AddressBytes &onNetwork,
                   const rtps::Ip4AddressBytes &localIp, uint8_t prefixLength) {
  std::vector<uint8_t> p;
  for (int i = 0; i < rtps::Config::SPDP_MAX_NUM_LOCATORS; ++i) {
    push_locator(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, offNetwork);
    push_locator(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, offNetwork);
  }
  push_locator(p, ParameterId::PID_METATRAFFIC_UNICAST_LOCATOR, onNetwork);
  push_locator(p, ParameterId::PID_DEFAULT_UNICAST_LOCATOR, onNetwork);
  const std::vector<uint8_t> bytes = with_sentinel(p);

  rtps::Participant part;
  part.m_localIpAddress = localIp;
  part.m_localPrefixLength = prefixLength;
  rtps::ParticipantProxyData proxy;
  rtps::CdrReader reader(rtps::asBytes(bytes.data(), bytes.size()));
  if (!proxy.readFromBuffer(reader, &part)) {
    std::printf("FAIL: SPDP parse\n");
    return false;
  }
  const auto holds = [&onNetwork](const auto &list) {
    return std::any_of(list.begin(), list.end(), [&onNetwork](const auto &loc) {
      return loc.isValid() && loc.getIp4AddressBytes() == onNetwork;
    });
  };
  return holds(proxy.m_metatrafficUnicastLocatorList) && holds(proxy.m_defaultUnicastLocatorList);
}

bool check_spdp() {
  CHECK(spdp_replaces(kWifi, kOnSubnet, kLocalIp, rtps::DEFAULT_SUBNET_PREFIX_LENGTH),
        "on-subnet locator replaces an off-subnet one (metatraffic + default lists)");
  // 4. On a /17 the Wi-Fi address is on our network and replaces the other half's.
  CHECK(spdp_replaces(kOther17, kWifi, kLocal17, 17), "/17: on-network locator replaces");
  // With the /24 default it is off-network too, so the first announced stays.
  CHECK(!spdp_replaces(kOther17, kWifi, kLocal17, rtps::DEFAULT_SUBNET_PREFIX_LENGTH),
        "/24 default: a different third octet is not our network");
  return true;
}

} // namespace

int main() {
  if (!check_prefix() || !check_sedp() || !check_spdp()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
