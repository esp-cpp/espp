// A stray DATA_FRAG from a writer the reader never matched cannot disturb a
// reassembly in progress.
//
// A reader has one fragment-reassembly slot, and any fragment of a sample
// other than the one in progress evicts it (best-effort, by design). The
// slot was filled by whatever DATA_FRAG named the reader, with no look at the
// writer: whole samples (newChange) are only taken from matched writers, but
// fragments were not, so one stray fragment - a writer the reader never
// matched, sending a sample of MAX_SAMPLE_SIZE - evicted a legitimate
// 200 KB sample mid-way and allocated the stray's buffer for nothing. Now a
// user reader ignores fragments from writers it has no proxy for
// (Reader::newFragment checks isProxy before the slot). Two MATCHED
// fragmenting writers on one reader still share the slot and evict each
// other; that is documented, not tested here.
//
// Drives Reader::newFragment directly on a StatelessReader with one matched
// writer; no network:
//  1. A 3-fragment sample from the matched writer, a fragment from an unknown
//     writer interleaved after the first, then the rest: the sample is
//     delivered byte-exact; the stray never is.
//  2. A whole 3-fragment sample from the unknown writer alone: nothing is
//     delivered.
//  3. After the writer is matched, the same sample is delivered (so it was
//     the proxy check, not the bytes).
//  4. Unmatching a writer releases the sample it had half reassembled (it can
//     never complete), and the slot then serves another matched writer.
//
// Exits 0 on success, 1 on the first failed check. Trivially passes when the
// engine is built without fragmentation.

#include <cstdio>
#include <cstring>
#include <vector>

#include "rtps/entities/StatelessReader.hpp"

#ifdef RTPS_ENABLE_FRAGMENTATION
namespace {
struct Delivered {
  std::vector<std::vector<uint8_t>> samples;
};

void on_change(void *arg, const rtps::ReaderCacheChange &change) {
  std::vector<uint8_t> bytes(change.getDataSize());
  if (!bytes.empty()) {
    change.copyInto(bytes.data(), change.getDataSize());
  }
  static_cast<Delivered *>(arg)->samples.push_back(std::move(bytes));
}

rtps::Guid_t writer_guid(uint8_t id) {
  rtps::Guid_t guid{};
  guid.prefix.id[0] = id;
  guid.entityId.entityKey[0] = id;
  guid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_WRITER_WITHOUT_KEY;
  return guid;
}

constexpr uint16_t kFragmentSize = 4;
constexpr uint32_t kSampleSize = 12;

// The sample's bytes: fragment n (1-based) carries bytes [4(n-1), 4n).
std::vector<uint8_t> sample_bytes(uint8_t seed) {
  std::vector<uint8_t> v(kSampleSize);
  for (uint32_t i = 0; i < kSampleSize; ++i) {
    v[i] = static_cast<uint8_t>(seed + i);
  }
  return v;
}

void fragment(rtps::StatelessReader &reader, const rtps::Guid_t &writer, uint32_t sn,
              uint32_t fragment_num, const std::vector<uint8_t> &sample) {
  reader.newFragment(writer, rtps::SequenceNumber_t{0, sn}, fragment_num, 1, kFragmentSize,
                     kSampleSize, sample.data() + (fragment_num - 1) * kFragmentSize, kFragmentSize,
                     rtps::VENDOR_UNKNOWN);
}

bool run_checks() {
  rtps::StatelessReader reader;
  rtps::TopicData attributes{};
  std::strncpy(attributes.topicName, "frag_slot", sizeof(attributes.topicName) - 1);
  std::strncpy(attributes.typeName, "test", sizeof(attributes.typeName) - 1);
  attributes.endpointGuid.entityId.entityKind = rtps::EntityKind_t::USER_DEFINED_READER_WITHOUT_KEY;
  if (!reader.init(attributes)) {
    std::printf("FAIL: reader init\n");
    return false;
  }
  Delivered delivered;
  if (reader.registerCallback(&on_change, &delivered) == 0) {
    std::printf("FAIL: registerCallback\n");
    return false;
  }
  const rtps::Guid_t matched = writer_guid(1);
  const rtps::Guid_t unknown = writer_guid(2);
  if (!reader.addNewMatchedWriter(rtps::WriterProxy{matched, rtps::LocatorIPv4{}, false})) {
    std::printf("FAIL: addNewMatchedWriter\n");
    return false;
  }
  const auto good = sample_bytes(0x10);
  const auto stray = sample_bytes(0x80);

  // 1. A stray fragment in the middle of a legitimate reassembly.
  fragment(reader, matched, 1, 1, good);
  fragment(reader, unknown, 1, 1, stray); // would evict the slot if taken
  fragment(reader, matched, 1, 2, good);
  fragment(reader, matched, 1, 3, good);
  if (delivered.samples.size() != 1 || delivered.samples[0] != good) {
    std::printf("FAIL: the matched writer's sample was not delivered intact (%zu delivered)\n",
                delivered.samples.size());
    return false;
  }

  // 2. A whole sample from the unknown writer.
  delivered.samples.clear();
  fragment(reader, unknown, 2, 1, stray);
  fragment(reader, unknown, 2, 2, stray);
  fragment(reader, unknown, 2, 3, stray);
  // fragment() fills `delivered` through the reader's callback, which cppcheck's
  // value flow does not see; the same below.
  // cppcheck-suppress knownConditionTrueFalse
  if (!delivered.samples.empty()) {
    std::printf("FAIL: a sample from an unmatched writer was delivered\n");
    return false;
  }

  // 3. Matched now: the same bytes go through.
  if (!reader.addNewMatchedWriter(rtps::WriterProxy{unknown, rtps::LocatorIPv4{}, false})) {
    std::printf("FAIL: addNewMatchedWriter (late)\n");
    return false;
  }
  fragment(reader, unknown, 3, 1, stray);
  fragment(reader, unknown, 3, 2, stray);
  fragment(reader, unknown, 3, 3, stray);
  const std::size_t late_delivered = delivered.samples.size();
  // cppcheck-suppress knownConditionTrueFalse
  if (late_delivered != 1 || delivered.samples.front() != stray) {
    std::printf("FAIL: the late-matched writer's sample was not delivered (%zu delivered)\n",
                late_delivered);
    return false;
  }

  // 4. A sample half reassembled when its writer is unmatched can never
  //    complete: the slot is released at once (the buffer is MAX_SAMPLE_SIZE
  //    bound, and the departing peer's last sample would otherwise hold it
  //    until the next fragmented sample from anyone).
  delivered.samples.clear();
  fragment(reader, matched, 4, 1, good);
  fragment(reader, matched, 4, 2, good);
  if (!reader.hasSampleUnderReassembly()) {
    std::printf("FAIL: a partial sample is not under reassembly\n");
    return false;
  }
  if (!reader.removeProxy(matched) || reader.hasSampleUnderReassembly()) {
    std::printf("FAIL: unmatching the writer did not release its partial sample\n");
    return false;
  }
  fragment(reader, matched, 4, 3, good); // ignored: unmatched again
  if (!delivered.samples.empty() || reader.hasSampleUnderReassembly()) {
    std::printf("FAIL: a fragment from the unmatched writer was taken after the release\n");
    return false;
  }
  // The slot works again for a matched writer.
  fragment(reader, unknown, 4, 1, stray);
  fragment(reader, unknown, 4, 2, stray);
  fragment(reader, unknown, 4, 3, stray);
  if (delivered.samples.size() != 1 || delivered.samples[0] != stray) {
    std::printf("FAIL: the slot did not work after the release (%zu delivered)\n",
                delivered.samples.size());
    return false;
  }
  return true;
}
} // namespace

int main() {
  if (!run_checks()) {
    return 1;
  }
  std::printf("PASS\n");
  return 0;
}
#else
int main() {
  std::printf("PASS (fragmentation not compiled in)\n");
  return 0;
}
#endif
