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

#ifndef RTPS_CONFIG_ESP32_H
#define RTPS_CONFIG_ESP32_H

#include "rtps/common/types.hpp"

namespace rtps {

#define IS_LITTLE_ENDIAN 1
#define OS_IS_FREERTOS

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
const std::array<uint8_t, 4> IP_ADDRESS = {192, 168, 4,
                                           1}; // Fallback: must match DHCPS server netif IP.
const GuidPrefix_t BASE_GUID_PREFIX{1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 13};

// DDS domain id (the ROS_DOMAIN_ID equivalent; 0..232 with UDP). The port base
// (7400 + 250 * id) and GUID prefix byte 9 derive from it, so every participant
// on a domain must use the same value. On ESP-IDF: menuconfig RTPS -> Domain id.
#ifndef RTPS_CFG_DOMAIN_ID
#define RTPS_CFG_DOMAIN_ID 0
#endif
const uint8_t DOMAIN_ID = RTPS_CFG_DOMAIN_ID;

// Reassembly / large-sample cap (bytes) when RTPS_ENABLE_FRAGMENTATION is opted
// in on this MCU (Kconfig, default off). Bounded (256 KB) - a fragmented sample
// larger than this is refused/dropped. Unused when fragmentation is off.
#ifndef RTPS_MAX_SAMPLE_SIZE
#define RTPS_MAX_SAMPLE_SIZE (256u * 1024u) // 256 KB
#endif
const DataSize_t MAX_SAMPLE_SIZE = RTPS_MAX_SAMPLE_SIZE;

#ifndef RTPS_CFG_NUM_STATELESS_WRITERS
#define RTPS_CFG_NUM_STATELESS_WRITERS 5
#endif
const uint8_t NUM_STATELESS_WRITERS = RTPS_CFG_NUM_STATELESS_WRITERS;
#ifndef RTPS_CFG_NUM_STATELESS_READERS
#define RTPS_CFG_NUM_STATELESS_READERS 5
#endif
const uint8_t NUM_STATELESS_READERS = RTPS_CFG_NUM_STATELESS_READERS;
#ifndef RTPS_CFG_NUM_STATEFUL_READERS
#define RTPS_CFG_NUM_STATEFUL_READERS 5
#endif
const uint8_t NUM_STATEFUL_READERS = RTPS_CFG_NUM_STATEFUL_READERS;
#ifndef RTPS_CFG_NUM_STATEFUL_WRITERS
#define RTPS_CFG_NUM_STATEFUL_WRITERS 5
#endif
const uint8_t NUM_STATEFUL_WRITERS = RTPS_CFG_NUM_STATEFUL_WRITERS;
#ifndef RTPS_CFG_MAX_NUM_PARTICIPANTS
#define RTPS_CFG_MAX_NUM_PARTICIPANTS 1
#endif
const uint8_t MAX_NUM_PARTICIPANTS = RTPS_CFG_MAX_NUM_PARTICIPANTS;
#ifndef RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT
#define RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT 10
#endif
const uint8_t NUM_WRITERS_PER_PARTICIPANT = RTPS_CFG_NUM_WRITERS_PER_PARTICIPANT;
#ifndef RTPS_CFG_NUM_READERS_PER_PARTICIPANT
#define RTPS_CFG_NUM_READERS_PER_PARTICIPANT 10
#endif
const uint8_t NUM_READERS_PER_PARTICIPANT = RTPS_CFG_NUM_READERS_PER_PARTICIPANT;
#ifndef RTPS_CFG_NUM_WRITER_PROXIES_PER_READER
#define RTPS_CFG_NUM_WRITER_PROXIES_PER_READER 6
#endif
const uint8_t NUM_WRITER_PROXIES_PER_READER = RTPS_CFG_NUM_WRITER_PROXIES_PER_READER;
#ifndef RTPS_CFG_NUM_READER_PROXIES_PER_WRITER
#define RTPS_CFG_NUM_READER_PROXIES_PER_WRITER 6
#endif
const uint8_t NUM_READER_PROXIES_PER_WRITER = RTPS_CFG_NUM_READER_PROXIES_PER_WRITER;

#ifndef RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS
#define RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS 50
#endif
const uint8_t MAX_NUM_UNMATCHED_REMOTE_WRITERS = RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_WRITERS;
#ifndef RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS
#define RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS 50
#endif
const uint8_t MAX_NUM_UNMATCHED_REMOTE_READERS = RTPS_CFG_MAX_NUM_UNMATCHED_REMOTE_READERS;

#ifndef RTPS_CFG_MAX_NUM_READER_CALLBACKS
#define RTPS_CFG_MAX_NUM_READER_CALLBACKS 5
#endif
const uint8_t MAX_NUM_READER_CALLBACKS = RTPS_CFG_MAX_NUM_READER_CALLBACKS;

#ifndef RTPS_CFG_HISTORY_SIZE_STATELESS
#define RTPS_CFG_HISTORY_SIZE_STATELESS 2
#endif
const uint8_t HISTORY_SIZE_STATELESS = RTPS_CFG_HISTORY_SIZE_STATELESS;
#ifndef RTPS_CFG_HISTORY_SIZE_STATEFUL
#define RTPS_CFG_HISTORY_SIZE_STATEFUL 10
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

const int HEARTBEAT_STACKSIZE = 1024 * 6;      // byte
const int THREAD_POOL_WRITER_STACKSIZE = 4096; // byte
// Stack of the transport's worker pool and reactor tasks (EsppTransport),
// which run every reader and matched callback. menuconfig RTPS -> Tasks.
#ifndef RTPS_CFG_THREAD_POOL_READER_STACKSIZE
#define RTPS_CFG_THREAD_POOL_READER_STACKSIZE 6144
#endif
const int THREAD_POOL_READER_STACKSIZE = RTPS_CFG_THREAD_POOL_READER_STACKSIZE; // byte
// Stack of the protocol task (SPDP announcements, heartbeats, liveliness); the
// Domain raises anything below 4096 to 4096. menuconfig RTPS -> Tasks.
#ifndef RTPS_CFG_SPDP_WRITER_STACKSIZE
#define RTPS_CFG_SPDP_WRITER_STACKSIZE 4096
#endif
const uint16_t SPDP_WRITER_STACKSIZE = RTPS_CFG_SPDP_WRITER_STACKSIZE; // byte

// Discovery / reliability timing (menuconfig RTPS -> Discovery and reliability
// timing). All change only WHEN messages are sent, never what is sent.
// Reliable (stateful) writers send a HEARTBEAT every SF_WRITER_HB_PERIOD_MS
// (a quarter of it while a remote reader has unacknowledged changes).
#ifndef RTPS_CFG_SF_WRITER_HB_PERIOD_MS
#define RTPS_CFG_SF_WRITER_HB_PERIOD_MS 4000
#endif
const uint16_t SF_WRITER_HB_PERIOD_MS = RTPS_CFG_SF_WRITER_HB_PERIOD_MS;
// Every participant announces itself (SPDP) every SPDP_RESEND_PERIOD_MS.
#ifndef RTPS_CFG_SPDP_RESEND_PERIOD_MS
#define RTPS_CFG_SPDP_RESEND_PERIOD_MS 2000
#endif
const uint16_t SPDP_RESEND_PERIOD_MS = RTPS_CFG_SPDP_RESEND_PERIOD_MS;
// Remote liveliness is checked every (SPDP_CYCLECOUNT_HEARTBEAT + 1) announce
// rounds, i.e. every (SPDP_CYCLECOUNT_HEARTBEAT + 1) * SPDP_RESEND_PERIOD_MS.
#ifndef RTPS_CFG_SPDP_CYCLECOUNT_HEARTBEAT
#define RTPS_CFG_SPDP_CYCLECOUNT_HEARTBEAT 2
#endif
const uint8_t SPDP_CYCLECOUNT_HEARTBEAT = RTPS_CFG_SPDP_CYCLECOUNT_HEARTBEAT;
// Priority of the protocol task (see SPDP_WRITER_STACKSIZE). menuconfig RTPS -> Tasks.
#ifndef RTPS_CFG_SPDP_WRITER_PRIO
#define RTPS_CFG_SPDP_WRITER_PRIO 5
#endif
const uint8_t SPDP_WRITER_PRIO = RTPS_CFG_SPDP_WRITER_PRIO;
#ifndef RTPS_CFG_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS
#define RTPS_CFG_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS 10
#endif
const uint8_t SPDP_MAX_NUMBER_FOUND_PARTICIPANTS = RTPS_CFG_SPDP_MAX_NUMBER_FOUND_PARTICIPANTS;
// Locators kept per list (metatraffic/default x unicast/multicast: 4 lists) of
// every remote participant proxy, so each slot of
// SPDP_MAX_NUMBER_FOUND_PARTICIPANTS grows by 4 x 12 bytes per locator. A peer
// announcing more keeps the first ones, with one on our network preferred.
#ifndef RTPS_CFG_SPDP_MAX_NUM_LOCATORS
#define RTPS_CFG_SPDP_MAX_NUM_LOCATORS 1
#endif
const uint8_t SPDP_MAX_NUM_LOCATORS = RTPS_CFG_SPDP_MAX_NUM_LOCATORS;
// Lease (seconds) this participant announces for itself, and the lease assumed
// for a remote participant that announces none (usually overwritten by remote
// info). Must comfortably exceed SPDP_RESEND_PERIOD_MS: a peer drops us when it
// misses announcements for a whole lease.
#ifndef RTPS_CFG_SPDP_DEFAULT_REMOTE_LEASE_DURATION_S
#define RTPS_CFG_SPDP_DEFAULT_REMOTE_LEASE_DURATION_S 5
#endif
const Duration_t SPDP_DEFAULT_REMOTE_LEASE_DURATION = {
    RTPS_CFG_SPDP_DEFAULT_REMOTE_LEASE_DURATION_S, 0};
// Absolute maximum lease (seconds) honoured for a remote participant, whatever
// it announces: a silent peer is dropped after min(its lease, this).
#ifndef RTPS_CFG_SPDP_MAX_REMOTE_LEASE_DURATION_S
#define RTPS_CFG_SPDP_MAX_REMOTE_LEASE_DURATION_S 90
#endif
const Duration_t SPDP_MAX_REMOTE_LEASE_DURATION = {RTPS_CFG_SPDP_MAX_REMOTE_LEASE_DURATION_S, 0};

const Duration_t SPDP_LEASE_DURATION = {5, 0};

// Transport channel pool: 2 shared multicast channels (SPDP metatraffic +
// user multicast) + 2 unicast channels per participant (builtin + user), plus
// any dedicated endpoint ports (max_prioritized_endpoint_ports, default 4)
// drawn at runtime. Sized so the profile's MAX_NUM_PARTICIPANTS budget fits
// with dedicated-port headroom; cross-checked at build time.
#ifndef RTPS_CFG_MAX_NUM_UDP_CONNECTIONS
#define RTPS_CFG_MAX_NUM_UDP_CONNECTIONS 10
#endif
const int MAX_NUM_UDP_CONNECTIONS = RTPS_CFG_MAX_NUM_UDP_CONNECTIONS;

const int THREAD_POOL_NUM_WRITERS = 2;
const int THREAD_POOL_NUM_READERS = 2;
const int THREAD_POOL_WRITER_PRIO = 5;
// The transport's worker pool and reactor tasks (EsppTransport). On ESP-IDF,
// menuconfig RTPS -> Transport task priority overrides it.
#ifndef RTPS_CFG_THREAD_POOL_READER_PRIO
#define RTPS_CFG_THREAD_POOL_READER_PRIO 5
#endif
const int THREAD_POOL_READER_PRIO = RTPS_CFG_THREAD_POOL_READER_PRIO;
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH_USERTRAFFIC = 60;
const int THREAD_POOL_WORKLOAD_QUEUE_LENGTH_METATRAFFIC = 60;

constexpr int OVERALL_HEAP_SIZE = THREAD_POOL_NUM_WRITERS * THREAD_POOL_WRITER_STACKSIZE +
                                  THREAD_POOL_NUM_READERS * THREAD_POOL_READER_STACKSIZE +
                                  MAX_NUM_PARTICIPANTS * SPDP_WRITER_STACKSIZE +
                                  NUM_STATEFUL_WRITERS * HEARTBEAT_STACKSIZE;
} // namespace Config
} // namespace rtps

#endif // RTPS_CONFIG_ESP32_H
