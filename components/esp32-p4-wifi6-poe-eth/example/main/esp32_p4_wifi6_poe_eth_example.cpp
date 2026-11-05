#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <numeric>
#include <span>
#include <thread>
#include <vector>

#include <sdkconfig.h>

#include "esp32-p4-wifi6-poe-eth.hpp"
#include "logger.hpp"

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
#include <esp_netif.h>
#endif

using namespace std::chrono_literals;
using DhcpMode = espp::Esp32P4Wifi6PoeEth::DhcpMode;
using EthConfig = espp::Esp32P4Wifi6PoeEth::EthernetConfig;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "P4Wifi6PoeEth", .level = espp::Logger::Verbosity::INFO});
  logger.info("ESP32-P4-WIFI6-POE-ETH example starting");

  // Example 1: get the singleton board instance
  //! [esp32 p4 wifi6 poe eth get instance]
  auto &board = espp::Esp32P4Wifi6PoeEth::get();
  //! [esp32 p4 wifi6 poe eth get instance]

  // Example 2: audio -- a beep through the speaker, then two seconds from the
  // microphone, played back. 16 kHz keeps the recording small (64 KB).
  //! [esp32 p4 wifi6 poe eth audio]
  static constexpr uint32_t audio_rate = 16000;
  if (board.initialize_audio({.sample_rate = audio_rate})) {
    board.volume(60.0f);
    // play() queues what fits and returns how much that was: stream from the
    // returned offset
    auto play_all = [&](std::span<const uint8_t> data) {
      while (!data.empty()) {
        const size_t queued = board.play_audio(data);
        data = data.subspan(queued);
        if (queued == 0) {
          std::this_thread::sleep_for(5ms);
        }
      }
    };
    // a 300 ms, 880 Hz beep (16-bit mono)
    std::vector<int16_t> beep(audio_rate * 3 / 10);
    for (size_t i = 0; i < beep.size(); i++) {
      beep[i] = static_cast<int16_t>(
          8000.0f * std::sin(2.0f * std::numbers::pi_v<float> * 880.0f * i / audio_rate));
    }
    logger.info("Playing a beep");
    play_all({reinterpret_cast<const uint8_t *>(beep.data()), beep.size() * sizeof(int16_t)});
    std::this_thread::sleep_for(500ms);

    // record two seconds of 16-bit mono (the callback runs on the microphone
    // task)
    static std::vector<int16_t> recording;
    recording.reserve(audio_rate * 2);
    logger.info("Recording for 2 s...");
    board.start_audio_recording([](const uint8_t *data, size_t length) {
      const auto *samples = reinterpret_cast<const int16_t *>(data);
      const size_t count =
          std::min(length / sizeof(int16_t), recording.capacity() - recording.size());
      recording.insert(recording.end(), samples, samples + count);
    });
    std::this_thread::sleep_for(2s);
    board.stop_audio_recording();
    const int peak = std::accumulate(recording.begin(), recording.end(), 0, [](int m, int16_t s) {
      return std::max(m, std::abs(static_cast<int>(s)));
    });
    logger.info("Recorded {} samples (peak {}); playing them back", recording.size(), peak);
    play_all(
        {reinterpret_cast<const uint8_t *>(recording.data()), recording.size() * sizeof(int16_t)});
  } else {
    logger.error("Audio initialization failed");
  }
  //! [esp32 p4 wifi6 poe eth audio]

  // NOTE: the board's BOOT key shares GPIO35 with the RMII TXD1 line, so it
  // can only be used to enter the bootloader at reset -- the BSP does not
  // expose it as a runtime button.

#if CONFIG_EXAMPLE_ETH_DHCP_SERVER
  // Example 3: init as DHCP server -- the board assigns IPs to connected hosts.
  // ServerConfig::ip_info zero -> default 192.168.4.1/24. Supply Kconfig
  // values (EXAMPLE_ETH_SERVER_IP / _NETMASK / _GW) for a custom static IP.
  //! [esp32 p4 wifi6 poe eth dhcp server]
  espp::Esp32P4Wifi6PoeEth::ServerConfig srv_cfg;
  if (esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_SERVER_IP, &srv_cfg.ip_info.ip) != ESP_OK ||
      esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_SERVER_NETMASK, &srv_cfg.ip_info.netmask) != ESP_OK ||
      esp_netif_str_to_ip4(CONFIG_EXAMPLE_ETH_SERVER_GW, &srv_cfg.ip_info.gw) != ESP_OK) {
    logger.error("Invalid DHCP-server address in menuconfig (ip='{}' netmask='{}' gw='{}'); "
                 "aborting Ethernet init",
                 CONFIG_EXAMPLE_ETH_SERVER_IP, CONFIG_EXAMPLE_ETH_SERVER_NETMASK,
                 CONFIG_EXAMPLE_ETH_SERVER_GW);
    return;
  }
  srv_cfg.on_client_assigned = [&](esp_ip4_addr_t ip, std::array<uint8_t, 6> mac) {
    logger.info("Client assigned {}.{}.{}.{} (mac {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x})",
                esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip),
                esp_ip4_addr4_16(&ip), mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
  };
  bool eth_ok = board.initialize_ethernet({
      .mode = DhcpMode::SERVER,
      .server_config = srv_cfg,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            logger.info("DHCP server up at {}.{}.{}.{}", esp_ip4_addr1_16(&ip),
                        esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
          },
      .on_lost_ip = [&]() { logger.warn("Ethernet lost IP"); },
  });
  //! [esp32 p4 wifi6 poe eth dhcp server]
#else
  // Example 4: init as DHCP client -- the board acquires an IP from the network.
  //! [esp32 p4 wifi6 poe eth dhcp client]
  bool eth_ok = board.initialize_ethernet({
      .mode = DhcpMode::CLIENT,
      .on_link_up = [&]() { logger.info("Ethernet link up"); },
      .on_link_down = [&]() { logger.warn("Ethernet link down"); },
      .on_got_ip =
          [&](esp_ip4_addr_t ip) {
            logger.info("DHCP lease acquired: {}.{}.{}.{}", esp_ip4_addr1_16(&ip),
                        esp_ip4_addr2_16(&ip), esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
          },
      .on_lost_ip = [&]() { logger.warn("Ethernet lost IP"); },
  });
  //! [esp32 p4 wifi6 poe eth dhcp client]
#endif

  if (!eth_ok) {
    logger.error("Ethernet initialization failed");
    return;
  }

  // SERVER mode: connected as soon as the cable is plugged in (static IP).
  // CLIENT mode: connected once the DHCP lease is granted (up to ~30 s).
  logger.info("Waiting for Ethernet...");
  for (int i = 0; i < 60 && !board.is_ethernet_connected(); ++i) {
    std::this_thread::sleep_for(500ms);
  }

  if (board.is_ethernet_connected()) {
    auto ip = board.ethernet_ip();
    logger.info("Connected. IP: {}.{}.{}.{}", esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
  } else {
    logger.warn("Ethernet not ready within 30 s");
  }

  while (true) {
    std::this_thread::sleep_for(5s);
    if (board.is_ethernet_connected()) {
      auto ip = board.ethernet_ip();
      logger.info("Ethernet up, IP: {}.{}.{}.{}", esp_ip4_addr1_16(&ip), esp_ip4_addr2_16(&ip),
                  esp_ip4_addr3_16(&ip), esp_ip4_addr4_16(&ip));
    } else {
      logger.warn("Ethernet not connected");
    }
  }
}
