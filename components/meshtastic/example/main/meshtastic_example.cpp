#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include "logger.hpp"
#include "meshtastic.hpp"
#include "sx126x.hpp"
#include "task.hpp"

#if CONFIG_EXAMPLE_HARDWARE_TDECK
#include "t-deck.hpp"
#elif CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
#include "m5stack-cardputer.hpp"
#endif

using namespace std::chrono_literals;

#if CONFIG_EXAMPLE_REGION_US
static constexpr auto REGION = espp::meshtastic::Region::US;
#elif CONFIG_EXAMPLE_REGION_EU_868
static constexpr auto REGION = espp::meshtastic::Region::EU_868;
#elif CONFIG_EXAMPLE_REGION_ANZ
static constexpr auto REGION = espp::meshtastic::Region::ANZ;
#endif

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "Meshtastic Example", .level = espp::Logger::Verbosity::INFO});
  logger.info("Starting Meshtastic example!");

  // Compute the modem config first (frequency / SF / BW / CR) so we can set
  // up the radio to match the channel. We use a throwaway node just to derive
  // the config; the real node is created once we know it works. (Cheap - it
  // does no I/O.)
  auto probe =
      espp::MeshtasticNode({.region = REGION, .preset = espp::meshtastic::ModemPreset::LONG_FAST});
  auto modem = probe.modem_config();
  logger.info("Meshtastic LongFast: {:.3f} MHz, SF{}, BW {} kHz, CR 4/{}",
              modem.frequency_hz / 1e6f, modem.spreading_factor, modem.bandwidth_hz / 1000,
              modem.coding_rate);

  // translate the meshtastic modem config into an sx126x radio config
  auto to_bandwidth = [](uint32_t hz) {
    switch (hz) {
    case 125000:
      return espp::Sx126x::Bandwidth::BW_125_KHZ;
    case 250000:
      return espp::Sx126x::Bandwidth::BW_250_KHZ;
    case 500000:
      return espp::Sx126x::Bandwidth::BW_500_KHZ;
    default:
      return espp::Sx126x::Bandwidth::BW_250_KHZ;
    }
  };
  espp::Sx126x::RadioConfig radio_config{
      .frequency_hz = modem.frequency_hz,
      .tx_power_dbm = 22,
      .spreading_factor = (espp::Sx126x::SpreadingFactor)modem.spreading_factor,
      .bandwidth = to_bandwidth(modem.bandwidth_hz),
      .coding_rate = (espp::Sx126x::CodingRate)(modem.coding_rate - 4),
      .preamble_length = modem.preamble_length,
      .crc_enabled = modem.crc_enabled,
      .sync_word = modem.sync_word,
  };

  // bring up the board and its LoRa radio via the BSP
#if CONFIG_EXAMPLE_HARDWARE_TDECK
  auto &board = espp::TDeck::get();
#elif CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
  auto &board = espp::M5StackCardputer::get();
#endif
  if (!board.initialize_lora(radio_config)) {
    logger.error("Failed to initialize the LoRa radio!");
    return;
  }
  auto radio = board.lora();

  //! [meshtastic example]
  // build the mesh node, transmitting through the radio
  auto node = std::make_shared<espp::MeshtasticNode>(espp::MeshtasticNode::Config {
    .long_name = CONFIG_EXAMPLE_LONG_NAME, .short_name = CONFIG_EXAMPLE_SHORT_NAME,
#if CONFIG_EXAMPLE_HARDWARE_TDECK
    .hw_model = espp::meshtastic::HardwareModel::T_DECK,
#elif CONFIG_EXAMPLE_HARDWARE_CARDPUTER_ADV
      .hw_model = espp::meshtastic::HardwareModel::M5STACK_CARDPUTER_ADV,
#endif
    .region = REGION, .preset = espp::meshtastic::ModemPreset::LONG_FAST,
    .transmit = [radio](std::span<const uint8_t> frame) -> bool {
      std::error_code ec;
      bool ok = radio->transmit(frame, 5s, ec);
      if (!ok) {
        // returning to receive is handled by transmit(); log failures
      }
      return ok;
    },
#if CONFIG_EXAMPLE_REBROADCAST
    .rebroadcast = true,
#else
      .rebroadcast = false,
#endif
    .on_text =
        [&](const std::string &text, const espp::meshtastic::PacketMetadata &meta) {
          logger.info("[TEXT] from 0x{:08x} ({} hops, SNR {:.1f}): {}", meta.header.from,
                      meta.hops_taken, meta.snr, text);
        },
    .on_nodeinfo =
        [&](const espp::meshtastic::User &user, const espp::meshtastic::PacketMetadata &meta) {
          logger.info("[NODE] 0x{:08x} is '{}' ({})", meta.header.from, user.long_name,
                      user.short_name);
        },
    .on_position =
        [&](const espp::meshtastic::Position &pos, const espp::meshtastic::PacketMetadata &meta) {
          logger.info("[POS ] 0x{:08x} @ {:.5f}, {:.5f}", meta.header.from, pos.latitude(),
                      pos.longitude());
        },
    .log_level = espp::Logger::Verbosity::INFO,
  });

  // deliver received radio packets to the mesh node
  radio->set_receive_callback([node](const espp::Sx126x::RxPacket &packet) {
    node->handle_frame(packet.data, packet.status.rssi, packet.status.snr);
  });

  // start listening
  std::error_code ec;
  if (!radio->start_receive(ec)) {
    logger.error("Failed to start receiving: {}", ec.message());
    return;
  }
  logger.info("Node {} listening for Meshtastic traffic", node->node_id());

  // announce ourselves so we appear in other nodes' node lists
  node->send_node_info();
  //! [meshtastic example]

#if CONFIG_EXAMPLE_TEXT_INTERVAL_S > 0
  int text_count = 0;
#endif
  auto last_nodeinfo = std::chrono::steady_clock::now();
  while (true) {
#if CONFIG_EXAMPLE_TEXT_INTERVAL_S > 0
    std::this_thread::sleep_for(std::chrono::seconds(CONFIG_EXAMPLE_TEXT_INTERVAL_S));
    std::string message = "hello from " + std::string(CONFIG_EXAMPLE_SHORT_NAME) + " #" +
                          std::to_string(text_count++);
    logger.info("Sending text: {}", message);
    node->send_text(message);
#else
    std::this_thread::sleep_for(10s);
#endif
    // periodically re-broadcast node info
    if (std::chrono::steady_clock::now() - last_nodeinfo >
        std::chrono::seconds(CONFIG_EXAMPLE_NODEINFO_INTERVAL_S)) {
      node->send_node_info();
      last_nodeinfo = std::chrono::steady_clock::now();
    }
  }
}
