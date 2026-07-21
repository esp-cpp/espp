# rtps_embedded

ESPP component that integrates the [embeddedRTPS](https://github.com/embedded-software-laboratory/embeddedRTPS)
RTPS/DDS stack into the ESPP ecosystem.  
Any platform that can build ESPP — including ESP32, Linux, and desktop PCs —
can use this component to discover and exchange typed messages with ROS 2 nodes
or any other DDS participant on the same network using the standard RTPS wire
protocol.

The original embeddedRTPS library has hard dependencies on FreeRTOS and lwIP.
`rtps_embedded` removes those dependencies by replacing all socket, task, and
synchronisation calls with ESPP's platform-agnostic `UdpSocket`, `Task`, and
`ThreadPool` primitives.  When built for ESP32, ESPP uses FreeRTOS and lwIP
under the hood; on other platforms it uses the host OS equivalents — the RTPS
code itself is unchanged in either case.

---

## Architecture

```
user code
    │
    ▼
rtps::Domain          — routes packets to participants; owns discovery threads
    │
    ├── rtps::Participant   — groups writers and readers
    │       ├── rtps::Writer  — publishes CacheChange samples
    │       └── rtps::Reader  — delivers samples to a user callback
    │
    ├── rtps::ThreadPool    — espp::ThreadPool workers that drain the four
    │                         incoming/outgoing meta/user traffic queues
    │
    └── rtps::EsppTransport — one espp::UdpSocket per open UDP port,
                              each with its own receive task
```

`EsppTransport` is the sole platform-specific adapter.  It wraps ESPP's
`UdpSocket` and `Task`, which in turn map to:

| Build target | Socket backend | Task backend |
|---|---|---|
| ESP32 | lwIP (via ESP-IDF) | FreeRTOS |
| Linux / PC | POSIX sockets | `std::thread` |

---

## Quick-start

```cpp
#include "rtps/entities/Domain.h"

// 1. Construct the domain with the local interface IP.
rtps::Domain domain(local_ip);

// 2. Create a participant *before* completeInit().
rtps::Participant *part = domain.createParticipant();

// 3. Add user-defined writer and reader endpoints.
rtps::Writer *writer = domain.createWriter(*part, "my/topic",
                                           "std_msgs::msg::String", false);
rtps::Reader *reader = domain.createReader(*part, "my/topic",
                                           "std_msgs::msg::String", false);

// 4. Register a receive callback on the reader.
reader->registerCallback(
    [](void *, const rtps::ReaderCacheChange &change) {
        // process change.getData() / change.copyInto(...)
    }, nullptr);

// 5. Start discovery (SPDP/SEDP) and worker threads.
domain.completeInit();

// 6. Publish a sample.
const char *payload = "hello";
writer->newChange(rtps::ChangeKind_t::ALIVE,
                  reinterpret_cast<const uint8_t *>(payload),
                  static_cast<rtps::DataSize_t>(strlen(payload) + 1));
```

> **Note**: `createParticipant()` **must** be called before `completeInit()`.
> No new participants can be added after init is complete.

---

## Configuration

Two built-in config headers are provided. Select one by defining
`RTPS_CONFIG_HEADER`, or let `include/rtps/config.h` pick automatically based
on the build target.

| Header | Target |
|---|---|
| [`include/rtps/config_esp32.h`](include/rtps/config_esp32.h) | ESP32 (ESP-IDF) |
| [`include/rtps/config_desktop.h`](include/rtps/config_desktop.h) | Linux / PC |

All tunable constants follow the same layout in both files:

| Constant | Default | Description |
|---|---|---|
| `DOMAIN_ID` | 0 | RTPS domain number (0–230 with UDP) |
| `MAX_NUM_PARTICIPANTS` | 1 | Participant pool size |
| `NUM_STATEFUL_WRITERS` | 5 | User writer endpoint pool |
| `NUM_STATEFUL_READERS` | 5 | User reader endpoint pool |
| `NUM_STATELESS_WRITERS` | 5 | Discovery writer endpoint pool |
| `NUM_STATELESS_READERS` | 5 | Discovery reader endpoint pool |
| `NUM_WRITERS_PER_PARTICIPANT` | 10 | Max writers per participant |
| `NUM_READERS_PER_PARTICIPANT` | 10 | Max readers per participant |
| `HISTORY_SIZE_STATEFUL` | 10 | Per-endpoint history depth |
| `THREAD_POOL_NUM_WRITERS` | 2 | Writer worker threads |
| `THREAD_POOL_NUM_READERS` | 2 | Reader worker threads |
| `THREAD_POOL_WRITER_STACKSIZE` | 4096 B | Writer task stack |
| `THREAD_POOL_READER_STACKSIZE` | 6144 B | Reader / UDP-receive task stack |
| `MAX_NUM_UDP_CONNECTIONS` | 10 | UDP socket pool size |
| `SPDP_RESEND_PERIOD_MS` | 2000 | Discovery announce period |
| `SF_WRITER_HB_PERIOD_MS` | 4000 | Reliable-writer heartbeat period |

The `OVERALL_HEAP_SIZE` constant at the bottom of that file estimates the
total stack RAM consumed by all internal tasks.

---

## ESPP component dependencies

| Component | Purpose |
|---|---|
| `base_component` | ESPP base class with integrated `espp::Logger` |
| `socket` | ESPP `UdpSocket` used by `EsppTransport` |
| `task` | ESPP `Task` for per-port UDP receive loops |
| `thread_pool` | ESPP `ThreadPool` for writer/reader workers |
| `cdr` | CDR serialization helpers |

These components abstract away all OS and network-stack details, so
`rtps_embedded` itself has no direct dependency on FreeRTOS, lwIP, or any
other platform library.

Third-party (vendored in `thirdparty/`):

| Library | Purpose |
|---|---|
| `Micro-CDR` | eProsima CDR (de)serialization for discovery messages |

---

## Example

See [`example/`](example/) for a two-node **initiator / responder** demo.

The same logic runs on any ESPP-supported platform. For ESP32, flash one board
as *Initiator* and a second as *Responder* via menuconfig
(`idf.py menuconfig → RTPS Example Configuration`). The initiator periodically
publishes numbered request messages; the responder echoes each message back on
the response topic.

Key menuconfig options (ESP32 example):

| Option | Description |
|---|---|
| `RTPS_EXAMPLE_ROLE` | `Initiator` or `Responder` |
| `RTPS_EXAMPLE_TOPIC_PREFIX` | Shared topic prefix (e.g. `espp/rtps_example`) |
| `RTPS_EXAMPLE_PUBLISH_PERIOD_MS` | Initiator publish interval |
| `ESP_WIFI_SSID` / `ESP_WIFI_PASSWORD` | Wi-Fi credentials |
