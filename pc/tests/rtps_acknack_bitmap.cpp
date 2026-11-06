// A reliable reader behind by more than 32 samples asks for the next batch of
// them correctly: WriterProxy::getMissing() builds the ACKNACK's
// SequenceNumberSet, which is SNS_MAX_NUM_BITS wide in 32-bit buckets. It used
// to write bucket 0 only, with a shift of (31 - bit) that goes negative past bit
// 31 (undefined behaviour). The request is also capped at kMaxNackSns per
// ACKNACK on purpose: asking for the whole set on every heartbeat raised a
// resend storm that starved a reader joining a flood (rtps_banded_churn). This
// pins the bitmap for gaps narrower than, at, and beyond the cap.

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
  ok = check_gap(1, 1, 1) && ok;     // one missing
  ok = check_gap(5, 20, 16) && ok;   // half a bucket
  ok = check_gap(5, 36, 32) && ok;   // exactly the cap
  ok = check_gap(5, 37, 32) && ok;   // one past the cap: capped
  ok = check_gap(10, 109, 32) && ok; // a wide gap: the first 32
  ok = check_gap(1, SNS_MAX_NUM_BITS, rtps::WriterProxy::kMaxNackSns) && ok;      // the set's width
  ok = check_gap(1, SNS_MAX_NUM_BITS + 50, rtps::WriterProxy::kMaxNackSns) && ok; // beyond it
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
