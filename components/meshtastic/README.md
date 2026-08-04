# Meshtastic Component

[![Badge](https://components.espressif.com/components/espp/meshtastic/badge.svg)](https://components.espressif.com/components/espp/meshtastic)

The `MeshtasticNode` component is a minimal, radio-agnostic implementation of
the Meshtastic® over-the-air mesh protocol. It implements enough of the
protocol to interoperate with stock Meshtastic devices on a shared channel
(the public "LongFast" channel by default): framing, AES-CTR encryption with
the well-known default channel key, sending and receiving text messages, node
info, and positions, receive-side deduplication, and optional managed-flood
rebroadcasting.

It is decoupled from any particular radio - you provide a transmit function
and feed it received frames. It pairs naturally with the espp `sx126x`
component (an SX1262 LoRa radio, as found on the LilyGo T-Deck and the M5Stack
Cardputer LoRa+GPS Cap), which is what the example uses. Call `modem_config()`
to get the exact LoRa modulation parameters (frequency, bandwidth, spreading
factor, coding rate, sync word) to apply to your radio for the configured
region / preset / channel.

## Scope

This component targets basic interoperability, not feature parity with the
Meshtastic firmware. It implements:

- The public / PSK-encrypted channel model (default and custom PSK channels)
- Text messages, node info (so your node appears in others' node lists), and
  positions (which you can feed from the espp `gps` component)
- The frequency-slot / channel-hash algorithms for the US, EU868, EU433 and
  ANZ regions
- Receive-side dedup and optional rebroadcasting

It does **not** implement: PKI-encrypted direct messages, MQTT, telemetry,
store-and-forward, the admin protocol, or SNR-weighted rebroadcast timing. A
node in the default receive/originate-only mode (no rebroadcast) still fully
interoperates and will appear in stock devices' node lists.

## Protobuf handling

Meshtastic payloads are Protocol Buffers. Rather than pull in a code
generator, this component includes small hand-written encoders/decoders for
the specific messages it uses (`Data`, `User`, `Position`) in
`meshtastic_protobuf.hpp`. The protobuf wire format is a stable, public
format, and only the handful of fields needed for interop are handled; unknown
fields are skipped on decode.

## Legal / trademark notice

This is an independent, clean-room implementation of the *published*
Meshtastic protocol. It is **not affiliated with, endorsed by, or sponsored
by Meshtastic LLC.** "Meshtastic®" is a registered trademark of Meshtastic
LLC; the name is used here only nominatively, to describe compatibility. Do
not use the Meshtastic name or logo to brand products or services built on
this component. See <https://meshtastic.org/docs/legal/trademark/>.

The Meshtastic firmware and protobuf definitions are licensed GPL-3.0. This
component contains no code copied from those projects - the protocol constants
(default PSK, packet layout, hash algorithms) and the protobuf field
definitions used here are facts about the wire format, gathered from the
public protocol documentation and `.proto` schema. If you intend to
distribute a product based on this component, review the licensing and
trademark implications for your use case.

## Example

The [example](./example) builds a working Meshtastic node on either the LilyGo
T-Deck or the M5Stack Cardputer-Adv (with the LoRa+GPS Cap), selectable via
menuconfig. It initializes the board's SX1262 radio via the BSP, applies the
modem configuration, periodically broadcasts node info and a text ping, and
prints received text messages, node info and positions. Point a stock
Meshtastic device at the same region/channel and the two will see each other.

This has been verified over the air: the example running on a Cardputer-Adv
successfully exchanges text messages with, and appears as a node on, a T-Deck
running the stock Meshtastic firmware on the US LongFast channel.

> [!IMPORTANT]
> The LoRa **region must be set on every device**, including any stock
> Meshtastic node you are talking to. A device with an *unset* region will not
> transmit or receive at all (this is the most common reason a node appears
> "silent"). Set the region on this node via menuconfig (it must be legal
> where you are) and make sure it matches the region configured on the other
> devices.
