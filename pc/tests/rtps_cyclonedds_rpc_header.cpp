// rmw_cyclonedds_cpp request/reply header codec (host unit test): the 16-byte
// {guid, seq} header that Cyclone's RMW splices between the CDR encapsulation
// and the message body of every service request and reply, which espp's
// services must parse (requests from a Cyclone client), echo (replies to one),
// write (requests to a Cyclone server) and strip (replies from one). The live
// round trips run in the docker interop matrix (run_interop.sh, CycloneDDS
// pass); this test pins the byte layout so a codec regression fails without
// ROS 2.
//
// Reference bytes: an AddTwoInts request {a=7, b=35} as rmw_cyclonedds_cpp
// (humble/jazzy) serializes it on a little-endian host - encapsulation
// 00 01 00 00, guid, seq, then the two int64s exactly where a FastDDS/espp
// request would put them 16 bytes earlier.
//
// Exits 0 iff every check passes.

#include <cstdint>
#include <cstdio>
#include <span>
#include <vector>

#include "rtps/entities/Reader.hpp"
#include "rtps/rpc/cyclonedds_request_header.hpp"

namespace {
int failures = 0;
void check(bool ok, const char *what) {
  if (!ok) {
    std::printf("FAIL: %s\n", what);
    ++failures;
  }
}

void put_u64(std::vector<uint8_t> &v, uint64_t x, bool le) {
  for (int i = 0; i < 8; ++i) {
    const int shift = le ? 8 * i : 8 * (7 - i);
    v.push_back(static_cast<uint8_t>((x >> shift) & 0xFF));
  }
}

// A request body as any ROS 2 RMW serializes AddTwoInts {a, b}: encapsulation +
// two int64 (little-endian when `le`).
std::vector<uint8_t> bare_request(int64_t a, int64_t b, bool le) {
  std::vector<uint8_t> v = le ? std::vector<uint8_t>{0x00, 0x01, 0x00, 0x00}
                              : std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00};
  put_u64(v, static_cast<uint64_t>(a), le);
  put_u64(v, static_cast<uint64_t>(b), le);
  return v;
}

// The same request as rmw_cyclonedds_cpp puts it on the wire.
std::vector<uint8_t> cyclone_request(uint64_t guid, int64_t seq, int64_t a, int64_t b, bool le) {
  std::vector<uint8_t> v = le ? std::vector<uint8_t>{0x00, 0x01, 0x00, 0x00}
                              : std::vector<uint8_t>{0x00, 0x00, 0x00, 0x00};
  put_u64(v, guid, le);
  put_u64(v, static_cast<uint64_t>(seq), le);
  put_u64(v, static_cast<uint64_t>(a), le);
  put_u64(v, static_cast<uint64_t>(b), le);
  return v;
}
} // namespace

int main() {
  using namespace rtps::rpc;

  // Vendor detection: only Eclipse Cyclone DDS (01.10) speaks the header.
  check(isCycloneDds(rtps::VENDOR_ECLIPSE_CYCLONEDDS), "01.10 is Cyclone DDS");
  check(!isCycloneDds(rtps::VENDOR_EPROSIMA_FASTDDS), "01.0F (Fast DDS) is not");
  check(!isCycloneDds(rtps::VENDOR_UNKNOWN), "unknown vendor is not");
  check(!isCycloneDds(rtps::Config::VENDOR_ID), "espp's own vendor id is not");

  // A reader change made without a vendor (plain pub/sub, the tests' fakes)
  // reports VENDOR_UNKNOWN, so the service layer never mistakes it for Cyclone.
  {
    rtps::Guid_t guid{};
    const uint8_t byte = 0;
    rtps::ReaderCacheChange change{rtps::ChangeKind_t::ALIVE, guid, rtps::SequenceNumber_t{0, 1},
                                   &byte, 1};
    check(change.sourceVendor == rtps::VENDOR_UNKNOWN, "default sourceVendor is unknown");
    check(!change.hasRelatedSampleIdentity, "default change has no inline identity");
  }

  // Parse + strip a little-endian Cyclone request down to the bare request.
  const uint64_t guid = 0x0123456789abcdefULL;
  const int64_t seq = 42;
  {
    const auto wire = cyclone_request(guid, seq, 7, 35, /*le=*/true);
    check(wire.size() == 4 + 16 + 16, "cyclone request is encap + header + body");
    CycloneDdsRequestHeader h{};
    check(parseCycloneDdsRequestHeader(wire, h), "parse LE header");
    check(h.guid == guid && h.seq == seq, "LE header values");
    check(stripCycloneDdsRequestHeader(wire) == bare_request(7, 35, true),
          "strip LE yields the bare request");
    // Byte-exact reference for the header region (little-endian).
    const uint8_t expected[16] = {0xef, 0xcd, 0xab, 0x89, 0x67, 0x45, 0x23, 0x01,
                                  0x2a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    bool same = true;
    for (int i = 0; i < 16; ++i) {
      same = same && wire[4 + i] == expected[i];
    }
    check(same, "LE header bytes match the reference layout");
  }

  // Prepend echoes the header into a reply in the reply's own endianness, and
  // the round trip is byte-identical to what Cyclone itself would send.
  {
    const CycloneDdsRequestHeader h{guid, seq};
    const auto reply = prependCycloneDdsRequestHeader(h, bare_request(7, 35, true));
    check(reply == cyclone_request(guid, seq, 7, 35, true), "prepend LE round trip");
    CycloneDdsRequestHeader back{};
    check(parseCycloneDdsRequestHeader(reply, back) && back == h, "parse(prepend(h)) == h");
  }

  // Big-endian encapsulation (CDR_BE, 0x0000): the header follows it.
  {
    const auto wire = cyclone_request(guid, seq, 7, 35, /*le=*/false);
    CycloneDdsRequestHeader h{};
    check(parseCycloneDdsRequestHeader(wire, h), "parse BE header");
    check(h.guid == guid && h.seq == seq, "BE header values");
    check(stripCycloneDdsRequestHeader(wire) == bare_request(7, 35, false),
          "strip BE yields the bare request");
    check(prependCycloneDdsRequestHeader(h, bare_request(7, 35, false)) == wire,
          "prepend BE round trip");
  }

  // Negative seq values survive (int64).
  {
    const CycloneDdsRequestHeader h{1, -5};
    const auto reply = prependCycloneDdsRequestHeader(h, bare_request(0, 0, true));
    CycloneDdsRequestHeader back{};
    check(parseCycloneDdsRequestHeader(reply, back) && back.seq == -5, "negative seq round trip");
    // Two's complement on the wire, exactly as Cyclone writes its int64.
    check(reply[4 + 8] == 0xFB && reply[4 + 15] == 0xFF, "negative seq is two's complement bits");
  }
  // Only a CDR encapsulation ({0x00, kind 0..3, options}) is a payload the
  // codec will read a header from; anything else is refused, never misparsed.
  {
    const uint64_t guid = 0x1122334455667788ULL;
    const int64_t seq = 42;
    auto bad_kind = cyclone_request(guid, seq, 7, 35, true);
    bad_kind[1] = 0x05;
    auto bad_first = cyclone_request(guid, seq, 7, 35, true);
    bad_first[0] = 0xFF;
    CycloneDdsRequestHeader h{};
    check(!parseCycloneDdsRequestHeader(bad_kind, h), "unknown encapsulation kind rejected");
    check(!parseCycloneDdsRequestHeader(bad_first, h),
          "non-zero first encapsulation byte rejected");
    check(prependCycloneDdsRequestHeader(h, bad_kind).empty(),
          "prepend refuses an unknown encapsulation");
    auto options = cyclone_request(guid, seq, 7, 35, true);
    options[2] = 0x00;
    options[3] = 0x02; // options bytes are the encoder's business
    check(parseCycloneDdsRequestHeader(options, h) && h.seq == seq,
          "options bytes do not affect parsing");
  }

  // Too short to hold a header: rejected, never read past the end.
  {
    const std::vector<uint8_t> short_payload = {0x00, 0x01, 0x00, 0x00, 1, 2, 3};
    CycloneDdsRequestHeader h{};
    check(!parseCycloneDdsRequestHeader(short_payload, h), "short payload rejected");
    check(!parseCycloneDdsRequestHeader(std::span<const uint8_t>{}, h), "empty payload rejected");
    const std::vector<uint8_t> no_encap = {0x00, 0x01};
    check(prependCycloneDdsRequestHeader(h, no_encap).empty(),
          "prepend without an encapsulation header yields nothing");
    // An empty body (encapsulation only) is a legal request/reply, e.g. Trigger.
    const std::vector<uint8_t> encap_only = {0x00, 0x01, 0x00, 0x00};
    check(prependCycloneDdsRequestHeader(h, encap_only).size() == 20, "encap-only body + header");
    check(stripCycloneDdsRequestHeader(prependCycloneDdsRequestHeader(h, encap_only)) == encap_only,
          "encap-only round trip");
  }

  // Client ids: deterministic per GUID, different for different GUIDs.
  {
    rtps::Guid_t a{};
    a.prefix.id = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    a.entityId.entityKey = {0, 0, 1};
    a.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
    rtps::Guid_t b = a;
    b.entityId.entityKey = {0, 0, 2};
    check(cycloneDdsClientId(a) == cycloneDdsClientId(a), "client id is deterministic");
    check(cycloneDdsClientId(a) != cycloneDdsClientId(b), "client id differs per entity");
    check(cycloneDdsClientId(a) != 0, "client id is non-zero");
  }

  if (failures != 0) {
    std::printf("cyclonedds_rpc_header: %d failure(s)\n", failures);
    return 1;
  }
  std::printf("cyclonedds_rpc_header: PASS\n");
  return 0;
}
