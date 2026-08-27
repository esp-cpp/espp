#include <algorithm>
#include <chrono>
#include <mutex>
#include <rtps/entities/Reader.hpp>
#include <rtps/entities/StatefulReader.hpp>
#include <rtps/entities/StatelessReader.hpp>
#include <rtps/utils/Log.hpp>
#include <rtps/utils/printutils.hpp>
#include <thread>

using namespace rtps;

Reader::Reader()
    : espp::BaseComponent("RtpsReader", espp::Logger::Verbosity::WARN) {
  m_callbacks.fill({nullptr, nullptr, 0});
}

void Reader::executeCallbacks(const ReaderCacheChange &cacheChange) {
  // Snapshot the registrations under m_callback_mutex, then invoke UNLOCKED:
  // holding the mutex across the callbacks put it on every user/discovery
  // callback stack (the SPDP/SEDP handlers take the participant/SEDP mutexes,
  // and user handlers may create endpoints, i.e. register callbacks on other
  // readers), creating lock-order inversions against registerCallback().
  // Invoking a just-removed callback is prevented at the LIFECYCLE level, not
  // here: every engine invocation path runs inside a generation-guarded
  // dispatch (see the *IfCurrent wrappers), and Reader::reset() retires the
  // generation and drains in-flight dispatches before a slot's callbacks are
  // cleared or its owner torn down.
  decltype(m_callbacks) snapshot;
  {
    std::lock_guard<std::recursive_mutex> lock(m_callback_mutex);
    snapshot = m_callbacks;
  }
  for (unsigned int i = 0; i < snapshot.size(); i++) {
    if (snapshot[i].function != nullptr) {
      snapshot[i].function(snapshot[i].arg, cacheChange);
    }
  }
}

bool Reader::initMutex() { return true; }

#ifdef RTPS_ENABLE_FRAGMENTATION
void Reader::newFragment(const Guid_t &writerGuid, const SequenceNumber_t &sn,
                         uint32_t fragmentStartingNum, uint16_t fragmentsInSubmessage,
                         uint16_t fragmentSize, uint32_t sampleSize, const uint8_t *fragData,
                         DataSize_t fragDataLen) {
  if (fragmentSize == 0 || sampleSize == 0 || fragmentStartingNum == 0) {
    return;
  }
  if (sampleSize > Config::MAX_SAMPLE_SIZE) {
    logger_.warn("Dropping fragmented sample: sampleSize {} exceeds MAX_SAMPLE_SIZE {}",
                 static_cast<unsigned>(sampleSize), static_cast<unsigned>(Config::MAX_SAMPLE_SIZE));
    return;
  }

  std::vector<uint8_t> completedBuffer; // moved out on completion; owns the bytes
  bool haveCompleted = false;

  {
    std::lock_guard<std::mutex> lock(m_reassembly_mutex);

    // Start a fresh reassembly if this is a different sample (best-effort
    // eviction of any older incomplete one) or nothing is in flight.
    const bool sameSample =
        m_reassembly.active && m_reassembly.writerGuid == writerGuid && m_reassembly.sn == sn;
    if (!sameSample) {
      m_reassembly.active = true;
      m_reassembly.writerGuid = writerGuid;
      m_reassembly.sn = sn;
      m_reassembly.sampleSize = sampleSize;
      m_reassembly.fragmentSize = fragmentSize;
      m_reassembly.totalFragments = (sampleSize + fragmentSize - 1) / fragmentSize;
      m_reassembly.receivedFragments = 0;
      m_reassembly.buffer.assign(sampleSize, 0);
      m_reassembly.received.assign(m_reassembly.totalFragments, false);
    } else if (m_reassembly.sampleSize != sampleSize || m_reassembly.fragmentSize != fragmentSize) {
      // Inconsistent metadata within one SN: drop and reset.
      m_reassembly.active = false;
      return;
    }

    // Copy the fragment payload at its byte offset. fragmentsInSubmessage packed
    // fragments are contiguous, so a single memcpy covers them all.
    const uint64_t offset =
        static_cast<uint64_t>(fragmentStartingNum - 1) * m_reassembly.fragmentSize;
    if (offset >= sampleSize) {
      return; // out-of-range fragment; ignore
    }
    // A submessage may pack several contiguous fragments (fragmentsInSubmessage).
    // Require the serialized data to actually contain all advertised fragments
    // before copying or marking them received - otherwise a short DATA_FRAG could
    // mark missing ranges complete and deliver zero-filled bytes. The expected
    // length is fragmentSize per fragment, except the sample's final fragment
    // ends at sampleSize; fragDataLen may be LARGER (trailing 4-byte alignment
    // padding, which FastDDS adds) but must not be smaller.
    if (fragmentsInSubmessage == 0) {
      return; // must advertise at least one fragment
    }
    const uint64_t expected =
        std::min<uint64_t>(static_cast<uint64_t>(fragmentsInSubmessage) * m_reassembly.fragmentSize,
                           sampleSize - offset);
    if (fragDataLen < expected) {
      return; // short / malformed submessage; do not mark ranges complete
    }
    if (expected > 0 && fragData != nullptr) {
      std::memcpy(m_reassembly.buffer.data() + offset, fragData, expected);
    }
    for (uint16_t k = 0; k < fragmentsInSubmessage; ++k) {
      const uint32_t idx = fragmentStartingNum - 1 + k;
      if (idx < m_reassembly.totalFragments && !m_reassembly.received[idx]) {
        m_reassembly.received[idx] = true;
        ++m_reassembly.receivedFragments;
      }
    }

    if (m_reassembly.receivedFragments >= m_reassembly.totalFragments) {
      // Sample complete: move the assembled buffer out and deliver it AFTER
      // releasing the reassembly lock (the callback runs synchronously and must
      // not be invoked under this lock).
      completedBuffer = std::move(m_reassembly.buffer);
      haveCompleted = true;
      m_reassembly.active = false;
    }
  }

  if (haveCompleted) {
    Guid_t guid = writerGuid;
    ReaderCacheChange completed{ChangeKind_t::ALIVE, guid, sn, completedBuffer.data(),
                                static_cast<DataSize_t>(completedBuffer.size())};
    newChange(completed);
  }
}
#endif

void Reader::reset() {
  // Retire this pooled slot's generation FIRST: any receive dispatch that
  // captured the old generation at lookup but has not yet passed its guarded
  // check will now no-op (see the *IfCurrent wrappers). Then wait - WITHOUT
  // holding the reader mutexes, which an in-flight dispatch needs to finish -
  // for dispatches that passed their check before the bump: they run against
  // the still-intact endpoint and must complete before its state is torn down
  // (and before the slot can be reused for another endpoint).
  ++m_generation_;
  while (m_active_dispatches_.load() != 0) {
    std::this_thread::sleep_for(std::chrono::microseconds(100));
  }

  std::lock_guard<std::recursive_mutex> lock1(m_proxies_mutex);
  std::lock_guard<std::recursive_mutex> lock2(m_callback_mutex);

  m_proxies.clear();
  for (unsigned int i = 0; i < m_callbacks.size(); i++) {
    m_callbacks[i].function = nullptr;
    m_callbacks[i].arg = nullptr;
  }

  m_callback_count = 0;
  m_is_initialized_ = false;
}

namespace {
// Counts a guarded receive dispatch in/out of the reader (RAII so an early
// return cannot leak the count and wedge reset()'s drain wait).
struct DispatchGuard {
  explicit DispatchGuard(std::atomic<int> &count)
      : count_(count) {
    count_.fetch_add(1);
  }
  ~DispatchGuard() { count_.fetch_sub(1); }
  std::atomic<int> &count_;
};
} // namespace

void Reader::newChangeIfCurrent(uint32_t generation, const ReaderCacheChange &cacheChange) {
  DispatchGuard guard(m_active_dispatches_);
  if (generation != m_generation_.load()) {
    return; // slot deleted (and possibly reused) since the lookup
  }
  newChange(cacheChange);
}

bool Reader::onNewHeartbeatIfCurrent(uint32_t generation, const SubmessageHeartbeat &msg,
                                     const GuidPrefix_t &remotePrefix) {
  DispatchGuard guard(m_active_dispatches_);
  if (generation != m_generation_.load()) {
    return false;
  }
  return onNewHeartbeat(msg, remotePrefix);
}

bool Reader::onNewGapIfCurrent(uint32_t generation, const SubmessageGap &msg,
                               const GuidPrefix_t &remotePrefix) {
  DispatchGuard guard(m_active_dispatches_);
  if (generation != m_generation_.load()) {
    return false;
  }
  return onNewGapMessage(msg, remotePrefix);
}

#ifdef RTPS_ENABLE_FRAGMENTATION
void Reader::newFragmentIfCurrent(uint32_t generation, const Guid_t &writerGuid,
                                  const SequenceNumber_t &sn, uint32_t fragmentStartingNum,
                                  uint16_t fragmentsInSubmessage, uint16_t fragmentSize,
                                  uint32_t sampleSize, const uint8_t *fragData,
                                  DataSize_t fragDataLen) {
  DispatchGuard guard(m_active_dispatches_);
  if (generation != m_generation_.load()) {
    return;
  }
  newFragment(writerGuid, sn, fragmentStartingNum, fragmentsInSubmessage, fragmentSize, sampleSize,
              fragData, fragDataLen);
}
#endif

bool Reader::isProxy(const Guid_t &guid) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  for (const auto &proxy : m_proxies) {
    if (proxy.remoteWriterGuid.operator==(guid)) {
      return true;
    }
  }
  return false;
}

WriterProxy *Reader::getProxy(Guid_t guid) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  auto isElementToFind = [&](const WriterProxy &proxy) { return proxy.remoteWriterGuid == guid; };
  auto thunk = [](void *arg, const WriterProxy &value) {
    return (*static_cast<decltype(isElementToFind) *>(arg))(value);
  };
  return m_proxies.find(thunk, &isElementToFind);
}

Reader::callbackIdentifier_t Reader::registerCallback(Reader::callbackFunction_t cb, void *arg) {
  std::lock_guard<std::recursive_mutex> lock(m_callback_mutex);
  if (m_callback_count == m_callbacks.size() || cb == nullptr) {
    return false;
  }

  for (unsigned int i = 0; i < m_callbacks.size(); i++) {
    if (m_callbacks[i].function == nullptr) {
      m_callbacks[i].function = cb;
      m_callbacks[i].arg = arg;
      m_callbacks[i].identifier = m_callback_identifier++;
      m_callback_count++;
      return m_callbacks[i].identifier;
    }
  }

  return 0;
}

uint32_t Reader::getProxiesCount() { return m_proxies.getNumElements(); }

bool Reader::removeCallback(Reader::callbackIdentifier_t identifier) {
  std::lock_guard<std::recursive_mutex> lock(m_callback_mutex);
  for (unsigned int i = 0; i < m_callbacks.size(); i++) {
    if (m_callbacks[i].identifier == identifier) {
      m_callbacks[i].function = nullptr;
      m_callbacks[i].arg = nullptr;
      m_callback_count--;
      return true;
    }
  }

  return false;
}

uint8_t Reader::getNumCallbacks() { return m_callback_count; }

void Reader::removeAllProxiesOfParticipant(const GuidPrefix_t &guidPrefix) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  auto isElementToRemove = [&](const WriterProxy &proxy) {
    return proxy.remoteWriterGuid.prefix == guidPrefix;
  };
  auto thunk = [](void *arg, const WriterProxy &value) {
    return (*static_cast<decltype(isElementToRemove) *>(arg))(value);
  };

  m_proxies.remove(thunk, &isElementToRemove);
}

bool Reader::removeProxy(const Guid_t &guid) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  auto isElementToRemove = [&](const WriterProxy &proxy) { return proxy.remoteWriterGuid == guid; };
  auto thunk = [](void *arg, const WriterProxy &value) {
    return (*static_cast<decltype(isElementToRemove) *>(arg))(value);
  };

  return m_proxies.remove(thunk, &isElementToRemove);
}

bool Reader::addNewMatchedWriter(const WriterProxy &newProxy) {
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
#if (SFR_VERBOSE || SLR_VERBOSE) && RTPS_GLOBAL_VERBOSE
  SFR_LOG("New writer added with id: ");
  printGuid(newProxy.remoteWriterGuid);
#endif
  return m_proxies.add(newProxy);
}

void rtps::Reader::setSEDPSequenceNumber(const SequenceNumber_t &sn) {
  m_sedp_sequence_number = sn;
}
const rtps::SequenceNumber_t &rtps::Reader::getSEDPSequenceNumber() {
  return m_sedp_sequence_number;
}

int rtps::Reader::dumpAllProxies(dumpProxyCallback target, void *arg) {
  if (target == nullptr) {
    return 0;
  }
  std::lock_guard<std::recursive_mutex> lock(m_proxies_mutex);
  int dump_count = 0;
  for (auto it = m_proxies.begin(); it != m_proxies.end(); ++it, ++dump_count) {
    target(this, *it, arg);
  }
  return dump_count;
}

bool rtps::Reader::sendPreemptiveAckNack(const WriterProxy &writer) { return true; }
