// A reliable reader behind by more than 32 samples asks for all of them:
// WriterProxy::getMissing() builds the ACKNACK's SequenceNumberSet, which is
// SNS_MAX_NUM_BITS wide in 32-bit buckets. It used to write bucket 0 only, with
// a shift of (31 - bit) that goes negative past bit 31, so a wider gap was
// never fully requested and the reader crawled back 32 samples per heartbeat
// (seen by the QEMU interop matrix under injected faults). This pins the
// bitmap for gaps of every width up to the set's capacity.

#include <cstdio>

#include "rtps/entities/WriterProxy.hpp"

namespace {
bool check_gap(uint32_t expected, uint32_t last, uint32_t want_bits) {
  rtps::WriterProxy proxy{rtps::Guid_t{}, rtps::LocatorIPv4{}, true};
  proxy.expectedSN = rtps::SequenceNumber_t{0, expected};
  const rtps::SequenceNumberSet set =
      proxy.getMissing(rtps::SequenceNumber_t{0, 1}, rtps::SequenceNumber_t{0, last});
  if (set.base.low != expected || set.numBits != want_bits) {
    std::printf("FAIL: gap %u..%u: base %u numBits %u, expected base %u numBits %u\n", expected,
                last, set.base.low, set.numBits, expected, want_bits);
    return false;
  }
  for (uint32_t bit = 0; bit < SNS_MAX_NUM_BITS; ++bit) {
    const bool want = bit < want_bits;
    if (set.isSet(bit) != want) {
      std::printf("FAIL: gap %u..%u: bit %u is %s, expected %s\n", expected, last, bit,
                  set.isSet(bit) ? "set" : "clear", want ? "set" : "clear");
      return false;
    }
  }
  return true;
}
} // namespace

int main() {
  bool ok = true;
  ok = check_gap(1, 1, 1) && ok;                                    // one missing
  ok = check_gap(5, 36, 32) && ok;                                  // exactly one bucket
  ok = check_gap(5, 37, 33) && ok;                                  // one past the first bucket
  ok = check_gap(10, 109, 100) && ok;                               // four buckets
  ok = check_gap(1, SNS_MAX_NUM_BITS, SNS_MAX_NUM_BITS) && ok;      // the whole set
  ok = check_gap(1, SNS_MAX_NUM_BITS + 50, SNS_MAX_NUM_BITS) && ok; // capped at the set
  // nothing missing: lastAvail below the expected SN
  {
    rtps::WriterProxy proxy{rtps::Guid_t{}, rtps::LocatorIPv4{}, true};
    proxy.expectedSN = rtps::SequenceNumber_t{0, 40};
    const auto set = proxy.getMissing(rtps::SequenceNumber_t{0, 1}, rtps::SequenceNumber_t{0, 39});
    if (set.base.low != 40 || set.numBits != 0) {
      std::printf("FAIL: nothing missing: base %u numBits %u\n", set.base.low, set.numBits);
      ok = false;
    }
  }
  if (!ok) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
