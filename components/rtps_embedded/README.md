# rtps_embedded

ESPP component that integrates the [embeddedRTPS](https://github.com/embedded-software-laboratory/embeddedRTPS)
RTPS/DDS stack into the ESPP ecosystem, behind an idiomatic `espp::RtpsParticipant`
facade. Any platform that can build ESPP — ESP32, Linux, macOS, Windows — can use
it to interoperate with **ROS 2** nodes (rmw_fastrtps) or any DDS participant on
the network over the standard RTPS wire protocol.

It provides three messaging patterns, all validated against live ROS 2:

- **Pub/sub** — topic-based, best-effort or reliable (HEARTBEAT/ACKNACK).
- **Services (RMI)** — request/reply with correlated responses.
- **Actions (AMI)** — long-running goals with feedback, result, and cancellation.

Each has a **typed** layer (reflectable structs, no manual bytes) and a
**byte-level** layer. Services and actions come in a ROS 2-interoperable flavour
and a lean **native** (espp ↔ espp) flavour.

The upstream embeddedRTPS library hard-depends on FreeRTOS and lwIP;
`rtps_embedded` removes those by routing all socket, task, and synchronisation
through ESPP's platform-agnostic `UdpSocket`, `Task`, `ThreadPool`, and
`SocketReactor`. On ESP32 those map to lwIP + FreeRTOS; elsewhere to the host OS.
Micro-CDR is gone — (de)serialization uses ESPP's reflection-driven `cdr`.

---

## Quick-start (typed facade)

```cpp
#include "rtps_participant.hpp"
#include "rtps_pubsub.hpp"   // typed Publisher<T> / Subscriber<T>
#include "rtps_service.hpp"  // typed ServiceServer / ServiceClient
#include "rtps_action.hpp"   // typed ActionServer  / ActionClient

// Any reflectable struct is a message - fields map straight to CDR.
struct StringMsg  { std::string data; };
struct AddReq     { int64_t a, b; };
struct AddResp    { int64_t sum; };

espp::RtpsParticipant participant({.interface_address = "192.168.1.10"});
participant.start();

// Pub/sub
espp::Publisher<StringMsg>  pub(participant, {.topic = "rt/chatter",
                                              .type_name = "std_msgs::msg::dds_::String_",
                                              .reliability = espp::RtpsParticipant::Reliability::RELIABLE});
espp::Subscriber<StringMsg> sub(participant, {.topic = "rt/chatter",
                                              .type_name = "std_msgs::msg::dds_::String_",
                                              .on_message = [](const StringMsg &m) { /* use m.data */ }});
pub.publish(StringMsg{"hello"});

// Service (RMI) - ros2 service call /add_two_ints ... hits this server
espp::ServiceServer<AddReq, AddResp> server(participant, {
    .service = "/add_two_ints", .type_name = "example_interfaces::srv::dds_::AddTwoInts",
    .handler = [](const AddReq &r) { return AddResp{r.a + r.b}; }});
espp::ServiceClient<AddReq, AddResp> client(participant, {
    .service = "/add_two_ints", .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
if (auto resp = client.call(AddReq{7, 35}, std::chrono::seconds(1))) { /* resp->sum == 42 */ }
```

For ROS 2 interop use ROS 2 naming: topic `rt/<name>`, type `<pkg>::msg::dds_::<Type>_`.
The full request/reply + goal APIs (including the three client call styles and the
native protocol) are documented in
[`doc/en/protocols/rtps_rmi_ami.rst`](../../doc/en/protocols/rtps_rmi_ami.rst).

### Byte-level API

The typed wrappers are thin layers over `espp::RtpsParticipant`'s byte-level
methods (`add_writer`/`add_reader`/`publish`, `add_service_server`/`_client`,
`add_action_server`/`_client`, and the `add_native_*` variants), which take/return
CDR-encapsulated `std::span<const uint8_t>`. Use those for dynamic types.

Python bindings expose the same surface via the `espp` module (see
[`python/rtps_rpc_demo.py`](../../python/rtps_rpc_demo.py)).

---

## Architecture

```
user code
    │
    ▼
espp::RtpsParticipant         — the facade: start()/stop(), add_writer/reader,
    │                           publish, add_service_*/add_action_*
    ▼
rtps::Domain                  — routes packets to participants; owns discovery
    │
    ├── rtps::Participant      — groups writers and readers
    │       ├── rtps::Writer   — publishes CacheChange samples
    │       └── rtps::Reader   — delivers samples to a user callback
    │
    └── rtps::EsppTransport    — the sole platform-specific adapter: one
                                 espp::UdpSocket per UDP port, dispatched by an
                                 espp::SocketReactor onto a shared espp::ThreadPool
                                 (also used for async writer work)
```

| Build target | Socket backend | Task backend |
|---|---|---|
| ESP32 | lwIP (via ESP-IDF) | FreeRTOS |
| Linux / macOS / PC | POSIX sockets | `std::thread` |

Services/actions are pure library code over pub/sub — the only wire addition is
a `related_sample_identity` inline QoS on service replies (for ROS 2 correlation).

---

## Configuration

Capacity limits are chosen at build time by a **limits profile** header; storage
policy, fragmentation, and the RPC layer are separate, independent knobs. On
ESP32 these are ESP-IDF menuconfig options (`RTPS (rtps_embedded)`); on host they
default via `include/rtps/config.hpp`.

| Knob | Options / default | Effect |
|---|---|---|
| `RTPS_LIMITS_PROFILE` | `embedded` (default) / `host` / `host_large` | Compile-time endpoint/history capacity caps (`config_esp32.hpp` / `config_desktop.hpp` / `config_host_large.hpp`). Wire-neutral. |
| `RTPS_STORAGE_DYNAMIC` | off on ESP32 / on host | Static `std::array` history (zero-heap, drop-oldest) vs heap-backed `std::deque` (grows). Orthogonal to the profile. |
| `RTPS_ENABLE_FRAGMENTATION` | off on ESP32 / on host | DATA_FRAG for samples > ~64 KB (interoperates with FastDDS/ROS 2). |
| `RTPS_ENABLE_RPC` | on (default) | Compile in services + actions (RMI/AMI). Disable to drop that code + its threads on a pure-pub/sub device. |

The domain id, announcement/heartbeat periods, and pool sizes live in the profile
headers.

---

## ESPP component dependencies

| Component | Purpose |
|---|---|
| `base_component` | ESPP base class with integrated `espp::Logger` |
| `socket` | `UdpSocket` + `SocketReactor` used by `EsppTransport` |
| `task` | `espp::Task` / `espp::Timer` |
| `thread_pool` | shared worker pool for receive dispatch + async writer work |
| `cdr` | reflection-driven CDR/XCDR (de)serialization |

The engine carries no vendored third-party code and has no direct dependency on
FreeRTOS, lwIP, or any platform library.

---

## Example

See [`example/`](example/) — an ESP32 (esp32-ethernet-kit) node that brings up a
participant over Ethernet and exercises the typed APIs: a `Publisher`/`Subscriber`
pair, a `ServiceServer` (`/add_two_ints`) + `ActionServer` (`/fibonacci`) a ROS 2
client can drive, and a `ServiceClient` + `ActionClient`. A `menuconfig` option
adds a second, self-testing participant. See [`example/README.md`](example/README.md).

## Interop & tests

[`interop/`](interop/) runs a dockerised FastDDS / ROS 2 (Jazzy) matrix — golden
byte-for-byte wire tests, in-process loopbacks (pub/sub, services, actions,
native, typed), and live `ros2 service call` / `ros2 action send_goal` both
directions. It is gated in CI (`.github/workflows/rtps_interop.yml`).
