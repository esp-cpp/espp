/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#ifndef RTPS_CONFIG_HOST_LARGE_H
#define RTPS_CONFIG_HOST_LARGE_H

#include "rtps/common/types.hpp"

namespace rtps {

#define IS_LITTLE_ENDIAN 1

// NOTE: this header sets capacity LIMITS only. The storage POLICY (static
// std::array vs heap-backed growable std::deque) is orthogonal and controlled
// centrally by RTPS_STORAGE_DYNAMIC, selected in config.hpp: dynamic by default
// on host/PC builds, explicit opt-in on ESP (Kconfig, default static). Selecting
// this profile therefore does NOT by itself switch storage to dynamic - which
// matters on ESP, where a relaxed limits profile must not silently enable heap
// storage. See storages/StorageArray.hpp. Capacity only - never touches wire bytes.

namespace Config {
// ---------------------------------------------------------------------------
// Per-limit overrides: every capacity cap below can be raised (or lowered)
// individually WITHOUT switching profiles by defining RTPS_CFG_<NAME> as a
// compile definition before this header is included - e.g.
// -DRTPS_CFG_NUM_STATELESS_WRITERS=16. On ESP-IDF the Kconfig options under
// "RTPS -> Custom capacity overrides" wire these up (0 = keep the profile
// default); on host builds pass RTPS_LIMIT_OVERRIDES to espp.cmake. The
// overrides MUST be applied when compiling the rtps sources themselves (the
// pools are sized in the library), which both mechanisms guarantee; defining
// them for only a consumer translation unit would silently disagree with the
// library. Capacity-only: no bytes on the wire change.
// ---------------------------------------------------------------------------

const VendorId_t VENDOR_ID = {13, 37};
const std::array<uint8_t, 4> IP_ADDRESS = {192, 168, 4, 1}; // Needs to be set in lwipcfg.h too.
// GUID_RANDOM: derive each participant prefix from OS entropy (see
// Domain::generateGuidPrefix). A fixed prefix here makes every desktop
// participant share an identity, which breaks discovery between them.
const GuidPrefix_t BASE_GUID_PREFIX = GUID_RANDOM;

// ---------------------------------------------------------------------------
// "host_large" limits profile (opt-in via RTPS_LIMITS_PROFILE=host_large).
//
// GENEROUS, fully-static capacity caps for large DDS graphs (big ROS 2 systems)
// on a compute host with plenty of RAM. Same deterministic, compile-time-static
// allocation model as the other profiles - just sized an order of magnitude
// larger than "host" (config_desktop.hpp). These are pure capacity caps and do
// NOT affect any bytes on the wire.
// ---------------------------------------------------------------------------
const uint8_t DOMAIN_ID = 0; // 230 possible with UDP

// Reassembly / large-sample cap (bytes). See config_desktop.hpp. Capacity only.
#ifndef RTPS_MAX_SAMPLE_SIZE
#define RTPS_MAX_SAMPLE_SIZE (8u * 1024u * 1024u) // 8 MB
#endif
const DataSize_t MAX_SAMPLE_SIZE = RTPS_MAX_SAMPLE_SIZE;

#ifndef RTPS_CFG_MAX_NUM_PARTICIPANTS
#define RTPS_CFG_MAX_NUM_PARTICIPANTS 32
#endif
const uint8_t MAX_NUM_PARTICIPANTS = RTPS_CFG_MAX_NUM_PARTICIPANTS;
#ifndef RTPS_CFG_NUM_STATELESS_WRITERS
#define RTPS_CFG_NUM_STATELESS_WRITERS 64
#endif
const uint8_t NUM_STATELESS_WRITERS = RTPS_CFG_NUM_STATELESS_WRITERS;
#ifndef RTPS_CFG_NUM_STATELESS_READERS
#define RTPS_CFG_NUM_STATELESS_READERS 64
#endif
const uint8_t NUM_STATELESS_READERS = RTPS_CFG_NUM_STATELESS_READERS;
#ifndef RTPS_CFG_NUM_STATEFUL_READERS
#define RTPS_CFG_NUM_STATEFUL_READERS 128
#endif
const uint8_t NUM_STATEFUL_READERS = RTPS_CFG_NUM_STATEFUL_READERS;
#ifndef RTPS_CFG_NUM_STATEFUL_WRITERS
#define RTPS_CFG_NUM_STATEFUL_WRITERS 128
#endif
const uint8_t NUM_STATEFUL_WRITERS = RTPS_CFG_NUM_STATEFUL_WRITERS;
#ifndef RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT
#define RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT 64
#endif
const uint8_t NUM_WRITERS_PER_PARTICIPANT = RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT;
#ifndef RTPS_CFG_NUM_READERS_PER_PARTICIPANT
#define RTPS_CFG_NUM_READERS_PER_PARTICIPANT 64
#endif
const uint8_t NUM_READERS_PER_PARTICIPANT = RTPS_CFG_NUM_READERS_PER_PARTICIPANT;
#ifndef RTPS_CFG_NUM_WRITER_PROXIES_PER_READER
#define RTPS_CFG_NUM_WRITER_PROXIES_PER_READER 16
#endif
const uint8_t NUM_WRITER_PROXIES_PER_READER = RTPS_CFG_NUM_WRITER_PROXIES_PER_READER;
#ifndef RTPS_CFG_NUM_READER_PROXIES_PER_WRITER
#define RTPS_CFG_NUM_READER_PROXIES_PER_WRITER 16
#endif
const uint8_t NUM_READER_PROXIES_PER_WRITER = RTPS_CFG_NUM_READER_PROXIES_PER_WRITER;

// uint16_t (not uint8_t): these bound SEDP MemoryPool<> sizes and the values
// here (1024 / 512) far exceed the 255 uint8_t range.
// MemoryPool<TYPE, uint32_t SIZE> widens the value, so uint16_t is safe.
#ifndef RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS
#define RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS 1024
#endif
const uint16_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS;
#ifndef RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS
#define RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS 512
#endif
const uint16_t MAX_NUM_UNMATCHED_REMOTE_READERS = RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS;

#ifndef RTPS_CFG_MAX_NUM_READER_CALLBACKS
#define RTPS_CFG_MAX_NUM_READER_CALLBACKS 16
#endif
const uint8_t MAX_NUM_READER_CALLBACKS = RTPS_CFG_MAX_NUM_READER_CALLBACKS;

#ifndef RTPS_CFG_HISTORY_SIZE_STATELESS
// 32 (was 2): see config_desktop.hpp - the generous profile should not share
// the embedded profile's minimal best-effort history.
#define RTPS_CFG_HISTORY_SIZE_STATELESS 32
#endif
const uint8_t HISTORY_SIZE_STATELESS = RTPS_CFG_HISTORY_SIZE_STATELESS;
#ifndef RTPS_CFG_HISTORY_SIZE_STATEFUL
#define RTPS_CFG_HISTORY_SIZE_STATEFUL 32
#endif
const uint8_t HISTORY_SIZE_STATEFUL = RTPS_CFG_HISTORY_SIZE_STATEFUL;

#ifndef RTPS_CFG_MAX_TYPENAME_LENGTH
#define RTPS_CFG_MAX_TYPENAME_LENGTH 64
#endif
const uint8_t MAX_TYPENAME_LENGTH = RTPS_CFG_MAX_TYPENAME_LENGTH;
#ifndef RTPS_CFG_MAX_TOPICNAME_LENGTH
#define RTPS_CFG_MAX_TOPICNAME_LENGTH 64
#endif
const uint8_t MAX_TOPICNAME_LENGTH = RTPS_CFG_MAX_TOPICNAME_LENGTH;

const int HEARTBEAT_STACKSIZE = 1200;          // byte
const int THREAD_POOL_WRITER_STACKSIZE = 1100; // byte
const int THREAD_POOL_READER_STACKSIZE = 1600; // byte
const uint16_t SPDP_WRITER_STACKSIZE = 550;    // byte

const uint16_t SF_WRITER_HB_PERIOD_MS = 2000;
const uint16_t SPDP_RESEND_PERIOD_MS = 1000;
const uint8_t SPDP_CYCLECOUNT_HEARTBEAT = 2; // skip x SPDP rounds before checking liveliness
const uint8_t SPDP_WRITER_PRIO = 3;
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = 128;
const uint8_t SPDP_MAX_NUM_LOCATORS = 16;
const Duration_t SPDP_DEFAULT_REMOTE_LEASE_DURATION = {
    100, 0}; // Default lease duration for remote participants, usually
             // overwritten by remote info
const Duration_t SPDP_MAX_REMOTE_LEASE_DURATION = {
    180, 0}; // Absolute maximum lease duration, ignoring remote participant info

const int MAX_NUM_UDP_CONNECTIONS = 32;

const int THREAD_POOL_NUM_WRITERS = 2;
const int THREAD_POOL_NUM_READERS = 2;
const int THREAD_POOL_WRITER_PRIO = 3;
const int THREAD_POOL_READER_PRIO = 3;
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH_USERTRAFFIC = 64;
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH_METATRAFFIC = 64;

constexpr int OVERALL_HEAP_SIZE = THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +
                                  THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +
                                  MAX_NUM_PARTICIPANTS * SPDP_WRITER_STACKSIZE +
                                  NUM_STATEFUL_WRITERS * HEARTBEAT_STACKSIZE;
} // namespace Config
} // namespace rtps

#endif // RTPS_CONFIG_HOST_LARGE_H
