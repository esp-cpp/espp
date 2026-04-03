#include <algorithm>
#include <chrono>
#include <functional>
#include <iterator>
#include <thread>

#include "sdkconfig.h"

#if CONFIG_ESP32_WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif

#include "logger.hpp"
#include "task.hpp"
#include "wifi_sta.hpp"

#include "rtsp_client.hpp"
#include "rtsp_server.hpp"

// Packetizer/depacketizer headers
#include "generic_depacketizer.hpp"
#include "generic_packetizer.hpp"
#include "h264_depacketizer.hpp"
#include "h264_packetizer.hpp"
#include "mjpeg_depacketizer.hpp"
#include "mjpeg_packetizer.hpp"

#include "jpeg_image.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

#if CONFIG_RTSP_EXAMPLE_RUN_API_TESTS

/// Run offline API tests that exercise packetizer/depacketizer classes and
/// server/client construction. These tests run before networking and catch
/// breaking API changes at build/boot time.
static bool run_api_tests(espp::Logger &logger) {
  bool all_passed = true;
  int tests_run = 0;
  int tests_passed = 0;

  auto check = [&](bool condition, const std::string &test_name) {
    tests_run++;
    if (condition) {
      tests_passed++;
      logger.info("[PASS] {}", test_name);
    } else {
      all_passed = false;
      logger.error("[FAIL] {}", test_name);
    }
  };

  logger.info("=== Running RTSP component API tests ===");

  // ---- MjpegPacketizer tests ----
  {
    //! [mjpeg_packetizer_test]
    espp::MjpegPacketizer mjpeg_packer({.max_payload_size = 1000});
    check(mjpeg_packer.get_payload_type() == 26, "MjpegPacketizer: payload type is 26");
    check(mjpeg_packer.get_clock_rate() == 90000, "MjpegPacketizer: clock rate is 90000");
    check(!mjpeg_packer.get_sdp_media_line().empty(), "MjpegPacketizer: SDP media line non-empty");
    check(!mjpeg_packer.get_sdp_media_attributes().empty(),
          "MjpegPacketizer: SDP attributes non-empty");

    // Packetize the embedded JPEG image
    std::span<const uint8_t> jpeg_data_span(reinterpret_cast<const uint8_t *>(jpeg_data),
                                            sizeof(jpeg_data));
    auto chunks = mjpeg_packer.packetize(jpeg_data_span);
    check(!chunks.empty(), "MjpegPacketizer: produced chunks from JPEG data");
    check(chunks.back().marker, "MjpegPacketizer: last chunk has marker bit set");

    // Verify chunk sizes respect MTU
    bool all_chunks_within_mtu = true;
    for (auto &c : chunks) {
      if (c.data.size() > 1000 + 200) { // payload + MJPEG header overhead
        all_chunks_within_mtu = false;
      }
    }
    check(all_chunks_within_mtu, "MjpegPacketizer: chunk sizes within expected bounds");
    //! [mjpeg_packetizer_test]
  }

  // ---- MjpegDepacketizer tests ----
  {
    //! [mjpeg_depacketizer_test]
    espp::MjpegDepacketizer mjpeg_depacker(espp::MjpegDepacketizer::Config{});

    bool generic_frame_received = false;
    bool jpeg_frame_received = false;

    mjpeg_depacker.set_frame_callback([&](std::vector<uint8_t> &&data) {
      generic_frame_received = true;
      logger.info("MjpegDepacketizer: generic callback got {} bytes", data.size());
    });
    mjpeg_depacker.set_jpeg_frame_callback([&](std::shared_ptr<espp::JpegFrame> frame) {
      jpeg_frame_received = true;
      logger.info("MjpegDepacketizer: JPEG callback got {}x{} frame", frame->get_width(),
                  frame->get_height());
    });

    // Create RTP packets from the MJPEG packetizer output and feed them
    // through the depacketizer to test the full round-trip
    espp::MjpegPacketizer mjpeg_packer({.max_payload_size = 1000});
    std::span<const uint8_t> jpeg_data_span(reinterpret_cast<const uint8_t *>(jpeg_data),
                                            sizeof(jpeg_data));
    auto chunks = mjpeg_packer.packetize(jpeg_data_span);

    uint16_t seq = 0;
    for (auto &chunk : chunks) {
      espp::RtpPacket pkt(chunk.data.size());
      pkt.set_version(2);
      pkt.set_payload_type(26);
      pkt.set_sequence_number(seq++);
      pkt.set_timestamp(0);
      pkt.set_ssrc(12345);
      pkt.set_marker(chunk.marker);
      pkt.set_payload(std::span<const uint8_t>(chunk.data));
      pkt.serialize();
      mjpeg_depacker.process_packet(pkt);
    }

    check(generic_frame_received, "MjpegDepacketizer: generic frame callback invoked");
    check(jpeg_frame_received, "MjpegDepacketizer: JPEG frame callback invoked");
    //! [mjpeg_depacketizer_test]
  }

  // ---- H264Packetizer tests ----
  {
    //! [h264_packetizer_test]
    // Synthetic SPS and PPS (minimal valid-ish NAL units)
    std::vector<uint8_t> sps = {0x67, 0x42, 0xC0, 0x1E, 0xD9, 0x00, 0xA0, 0x47, 0xFE, 0xC8};
    std::vector<uint8_t> pps = {0x68, 0xCE, 0x38, 0x80};

    espp::H264Packetizer h264_packer({
        .max_payload_size = 1400,
        .payload_type = 96,
        .profile_level_id = "42C01E",
        .packetization_mode = 1,
        .sps = sps,
        .pps = pps,
    });

    check(h264_packer.get_payload_type() == 96, "H264Packetizer: payload type is 96");
    check(h264_packer.get_clock_rate() == 90000, "H264Packetizer: clock rate is 90000");

    auto sdp_attrs = h264_packer.get_sdp_media_attributes();
    check(sdp_attrs.find("H264/90000") != std::string::npos,
          "H264Packetizer: SDP contains H264/90000");
    check(sdp_attrs.find("profile-level-id=42C01E") != std::string::npos,
          "H264Packetizer: SDP contains profile-level-id");
    check(sdp_attrs.find("sprop-parameter-sets=") != std::string::npos,
          "H264Packetizer: SDP contains SPS/PPS base64");

    // Create a synthetic H.264 access unit in Annex B format:
    // Start code + SPS + Start code + PPS + Start code + small IDR slice
    std::vector<uint8_t> annex_b_frame;
    // SPS NAL
    annex_b_frame.insert(annex_b_frame.end(), {0x00, 0x00, 0x00, 0x01});
    annex_b_frame.insert(annex_b_frame.end(), sps.begin(), sps.end());
    // PPS NAL
    annex_b_frame.insert(annex_b_frame.end(), {0x00, 0x00, 0x00, 0x01});
    annex_b_frame.insert(annex_b_frame.end(), pps.begin(), pps.end());
    // IDR slice NAL (type 5) — fill with dummy data
    annex_b_frame.insert(annex_b_frame.end(), {0x00, 0x00, 0x00, 0x01});
    annex_b_frame.push_back(0x65); // NAL header: type=5 (IDR)
    for (int i = 0; i < 100; i++) {
      annex_b_frame.push_back(static_cast<uint8_t>(i & 0xFF));
    }

    auto chunks =
        h264_packer.packetize(std::span<const uint8_t>(annex_b_frame.data(), annex_b_frame.size()));
    check(!chunks.empty(), "H264Packetizer: produced chunks from Annex B data");
    check(chunks.back().marker, "H264Packetizer: last chunk has marker bit");

    // With small NALs, all should be single NAL mode (no FU-A needed)
    check(chunks.size() == 3, "H264Packetizer: 3 chunks for SPS+PPS+IDR");
    //! [h264_packetizer_test]
  }

  // ---- H264Depacketizer tests ----
  {
    //! [h264_depacketizer_test]
    espp::H264Depacketizer h264_depacker(espp::H264Depacketizer::Config{});

    bool frame_received = false;
    size_t frame_size = 0;

    h264_depacker.set_frame_callback([&](std::vector<uint8_t> &&data) {
      frame_received = true;
      frame_size = data.size();
      logger.info("H264Depacketizer: got frame of {} bytes", data.size());
      // Verify Annex B start codes are present
      if (data.size() >= 4) {
        bool has_start_code =
            (data[0] == 0x00 && data[1] == 0x00 && data[2] == 0x00 && data[3] == 0x01);
        logger.info("H264Depacketizer: Annex B start code present: {}", has_start_code);
      }
    });

    // Create synthetic single NAL packets and feed them
    std::vector<uint8_t> sps = {0x67, 0x42, 0xC0, 0x1E, 0xD9};
    std::vector<uint8_t> pps = {0x68, 0xCE, 0x38, 0x80};
    std::vector<uint8_t> idr = {0x65, 0x01, 0x02, 0x03, 0x04};

    auto make_rtp = [](const std::vector<uint8_t> &payload, int pt, uint16_t seq, bool marker) {
      espp::RtpPacket pkt(payload.size());
      pkt.set_version(2);
      pkt.set_payload_type(pt);
      pkt.set_sequence_number(seq);
      pkt.set_timestamp(0);
      pkt.set_ssrc(54321);
      pkt.set_marker(marker);
      pkt.set_payload(std::span<const uint8_t>(payload));
      pkt.serialize();
      return pkt;
    };

    h264_depacker.process_packet(make_rtp(sps, 96, 0, false));
    h264_depacker.process_packet(make_rtp(pps, 96, 1, false));
    h264_depacker.process_packet(make_rtp(idr, 96, 2, true)); // marker = end of AU

    check(frame_received, "H264Depacketizer: frame callback invoked");
    // Expected: 3 NALs with start codes = 3*(4) + 5+4+5 = 26 bytes
    check(frame_size > 0, "H264Depacketizer: frame has data");
    //! [h264_depacketizer_test]
  }

  // ---- H264 FU-A round-trip test ----
  {
    //! [h264_fua_roundtrip_test]
    // Create a large NAL that requires FU-A fragmentation
    std::vector<uint8_t> large_nal;
    large_nal.push_back(0x65); // IDR slice (type 5)
    for (int i = 0; i < 3000; i++) {
      large_nal.push_back(static_cast<uint8_t>(i & 0xFF));
    }

    // Annex B frame with one large NAL
    std::vector<uint8_t> annex_b;
    annex_b.insert(annex_b.end(), {0x00, 0x00, 0x00, 0x01});
    annex_b.insert(annex_b.end(), large_nal.begin(), large_nal.end());

    espp::H264Packetizer packer({.max_payload_size = 1000,
                                 .payload_type = 96,
                                 .profile_level_id = {},
                                 .packetization_mode = 1,
                                 .sps = {},
                                 .pps = {}});
    auto chunks = packer.packetize(std::span<const uint8_t>(annex_b));
    check(chunks.size() > 1, "H264 FU-A: large NAL produces multiple chunks");

    // Feed chunks through depacketizer
    espp::H264Depacketizer depacker(espp::H264Depacketizer::Config{});
    bool fua_frame_received = false;
    std::vector<uint8_t> received_frame;
    depacker.set_frame_callback([&](std::vector<uint8_t> &&data) {
      fua_frame_received = true;
      received_frame = std::move(data);
    });

    uint16_t seq = 0;
    for (auto &chunk : chunks) {
      espp::RtpPacket pkt(chunk.data.size());
      pkt.set_version(2);
      pkt.set_payload_type(96);
      pkt.set_sequence_number(seq++);
      pkt.set_timestamp(0);
      pkt.set_ssrc(99999);
      pkt.set_marker(chunk.marker);
      pkt.set_payload(std::span<const uint8_t>(chunk.data));
      pkt.serialize();
      depacker.process_packet(pkt);
    }

    check(fua_frame_received, "H264 FU-A: round-trip frame received");
    // Output should be: start code (4) + original NAL (3001)
    check(received_frame.size() == 4 + large_nal.size(),
          "H264 FU-A: round-trip frame size matches");
    // Verify the NAL content matches (skip start code)
    if (received_frame.size() >= 4 + large_nal.size()) {
      bool content_matches =
          std::equal(large_nal.begin(), large_nal.end(), received_frame.begin() + 4);
      check(content_matches, "H264 FU-A: round-trip content matches");
    }
    //! [h264_fua_roundtrip_test]
  }

  // ---- GenericPacketizer tests ----
  {
    //! [generic_packetizer_test]
    espp::GenericPacketizer generic_packer({
        .max_payload_size = 500,
        .payload_type = 97,
        .clock_rate = 48000,
        .encoding_name = "opus",
        .channels = 2,
        .fmtp = {},
        .media_type = espp::MediaType::AUDIO,
    });

    check(generic_packer.get_payload_type() == 97, "GenericPacketizer: payload type is 97");
    check(generic_packer.get_clock_rate() == 48000, "GenericPacketizer: clock rate is 48000");

    auto sdp_line = generic_packer.get_sdp_media_line();
    check(sdp_line.find("m=audio") != std::string::npos,
          "GenericPacketizer: SDP media line is audio");

    auto sdp_attrs = generic_packer.get_sdp_media_attributes();
    check(sdp_attrs.find("opus/48000/2") != std::string::npos,
          "GenericPacketizer: SDP has encoding/rate/channels");

    // Packetize 1200 bytes of synthetic audio
    std::vector<uint8_t> audio_data(1200, 0xAB);
    auto chunks =
        generic_packer.packetize(std::span<const uint8_t>(audio_data.data(), audio_data.size()));
    check(chunks.size() == 3, "GenericPacketizer: 1200 bytes @ 500 MTU = 3 chunks");
    check(chunks.back().marker, "GenericPacketizer: last chunk has marker");
    check(!chunks.front().marker, "GenericPacketizer: first chunk has no marker");
    //! [generic_packetizer_test]
  }

  // ---- GenericDepacketizer round-trip test ----
  {
    //! [generic_depacketizer_test]
    espp::GenericDepacketizer generic_depacker(espp::GenericDepacketizer::Config{});

    bool audio_frame_received = false;
    size_t audio_frame_size = 0;

    generic_depacker.set_frame_callback([&](std::vector<uint8_t> &&data) {
      audio_frame_received = true;
      audio_frame_size = data.size();
    });

    // Packetize and depacketize audio data
    espp::GenericPacketizer generic_packer({.max_payload_size = 500,
                                            .payload_type = 97,
                                            .clock_rate = 48000,
                                            .encoding_name = "L16",
                                            .channels = 1,
                                            .fmtp = {},
                                            .media_type = espp::MediaType::AUDIO});
    std::vector<uint8_t> audio_data(1200, 0xCD);
    auto chunks =
        generic_packer.packetize(std::span<const uint8_t>(audio_data.data(), audio_data.size()));

    uint16_t seq = 0;
    for (auto &chunk : chunks) {
      espp::RtpPacket pkt(chunk.data.size());
      pkt.set_version(2);
      pkt.set_payload_type(97);
      pkt.set_sequence_number(seq++);
      pkt.set_timestamp(1000);
      pkt.set_ssrc(77777);
      pkt.set_marker(chunk.marker);
      pkt.set_payload(std::span<const uint8_t>(chunk.data));
      pkt.serialize();
      generic_depacker.process_packet(pkt);
    }

    check(audio_frame_received, "GenericDepacketizer: frame callback invoked");
    check(audio_frame_size == 1200, "GenericDepacketizer: round-trip frame size matches");
    //! [generic_depacketizer_test]
  }

  // ---- RtspServer construction and track API tests ----
  {
    //! [rtsp_server_api_test]
    espp::RtspServer server({
        .server_address = "127.0.0.1",
        .port = 8554,
        .path = "/test",
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // Test add_track with MJPEG packetizer
    auto mjpeg_packer = std::make_shared<espp::MjpegPacketizer>(
        espp::MjpegPacketizer::Config{.max_payload_size = 1400});
    server.add_track(espp::RtspServer::TrackConfig{.track_id = 0, .packetizer = mjpeg_packer});

    // Test add_track with H264 packetizer
    auto h264_packer = std::make_shared<espp::H264Packetizer>(espp::H264Packetizer::Config{
        .max_payload_size = 1400,
        .payload_type = 96,
        .profile_level_id = "42C01E",
        .sps = {0x67, 0x42, 0xC0, 0x1E},
        .pps = {0x68, 0xCE, 0x38, 0x80},
    });
    server.add_track(espp::RtspServer::TrackConfig{.track_id = 1, .packetizer = h264_packer});

    // Test add_track with generic audio packetizer
    auto audio_packer = std::make_shared<espp::GenericPacketizer>(espp::GenericPacketizer::Config{
        .max_payload_size = 1400,
        .payload_type = 97,
        .clock_rate = 48000,
        .encoding_name = "opus",
        .channels = 2,
        .fmtp = {},
        .media_type = espp::MediaType::AUDIO,
    });
    server.add_track(espp::RtspServer::TrackConfig{.track_id = 2, .packetizer = audio_packer});

    check(true, "RtspServer: construction with multi-track succeeded");

    // Test set_session_log_level
    server.set_session_log_level(espp::Logger::Verbosity::DEBUG);
    check(true, "RtspServer: set_session_log_level succeeded");
    //! [rtsp_server_api_test]
  }

  // ---- RtspClient construction and depacketizer API tests ----
  {
    //! [rtsp_client_api_test]
    // Test backward-compatible construction with on_jpeg_frame
    bool jpeg_cb_set = false;
    espp::RtspClient client({
        .server_address = "127.0.0.1",
        .rtsp_port = 8554,
        .path = "/test",
        .on_jpeg_frame = [&jpeg_cb_set](std::shared_ptr<espp::JpegFrame>) { jpeg_cb_set = true; },
        .log_level = espp::Logger::Verbosity::WARN,
    });

    check(true, "RtspClient: construction with on_jpeg_frame succeeded");

    // Test add_depacketizer for H264
    auto h264_depacker = std::make_shared<espp::H264Depacketizer>(espp::H264Depacketizer::Config{});
    h264_depacker->set_frame_callback([](std::vector<uint8_t> &&data) {
      // would receive Annex B H.264 access units
    });
    client.add_depacketizer(96, h264_depacker);

    // Test add_depacketizer for generic audio
    auto audio_depacker =
        std::make_shared<espp::GenericDepacketizer>(espp::GenericDepacketizer::Config{});
    audio_depacker->set_frame_callback([](std::vector<uint8_t> &&data) {
      // would receive reassembled audio frames
    });
    client.add_depacketizer(97, audio_depacker);

    check(true, "RtspClient: add_depacketizer for H264 and audio succeeded");
    //! [rtsp_client_api_test]
  }

  // ---- Legacy JpegFrame send_frame backward-compat test ----
  {
    //! [legacy_send_frame_test]
    espp::RtspServer server({
        .server_address = "127.0.0.1",
        .port = 8556,
        .path = "/mjpeg/1",
        .max_data_size = 1000,
        .log_level = espp::Logger::Verbosity::WARN,
    });

    // send_frame(JpegFrame) should lazily create MJPEG track 0
    std::span<const uint8_t> jpeg_data_span(reinterpret_cast<const uint8_t *>(jpeg_data),
                                            sizeof(jpeg_data));
    espp::JpegFrame frame(jpeg_data_span);
    server.send_frame(frame);

    check(true, "Legacy send_frame(JpegFrame): lazy MJPEG track creation succeeded");
    //! [legacy_send_frame_test]
  }

  logger.info("=== API tests complete: {}/{} passed ===", tests_passed, tests_run);
  return all_passed;
}

#endif // CONFIG_RTSP_EXAMPLE_RUN_API_TESTS

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "main", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting RTSP example!");

#if CONFIG_RTSP_EXAMPLE_RUN_API_TESTS
  if (!run_api_tests(logger)) {
    logger.error("API tests FAILED — check output above");
  } else {
    logger.info("All API tests passed!");
  }
#endif

#if CONFIG_ESP32_WIFI_NVS_ENABLED
  // Initialize NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
#endif

  std::string ip_address;
  espp::WifiSta wifi_sta({.ssid = CONFIG_ESP_WIFI_SSID,
                          .password = CONFIG_ESP_WIFI_PASSWORD,
                          .num_connect_retries = CONFIG_ESP_MAXIMUM_RETRY,
                          .on_connected = nullptr,
                          .on_disconnected = nullptr,
                          .on_got_ip = [&ip_address](ip_event_got_ip_t *eventdata) {
                            ip_address = fmt::format("{}.{}.{}.{}", IP2STR(&eventdata->ip_info.ip));
                            fmt::print("got IP: {}\n", ip_address);
                          }});

  while (!wifi_sta.is_connected()) {
    std::this_thread::sleep_for(100ms);
  }

  const int server_port = CONFIG_RTSP_SERVER_PORT;

  // --------------------------------------------------------------------------
  // Mode: Legacy MJPEG (server + client on the same device)
  // --------------------------------------------------------------------------
#if CONFIG_RTSP_EXAMPLE_MODE_LEGACY

  //! [rtsp_server_example]
  const std::string server_uri = fmt::format("rtsp://{}:{}/mjpeg/1", ip_address, server_port);
  logger.info("Starting RTSP Server on port {}", server_port);
  logger.info("RTSP URI: {}", server_uri);

  espp::RtspServer rtsp_server({
      .server_address = ip_address,
      .port = server_port,
      .path = "/mjpeg/1",
      .log_level = espp::Logger::Verbosity::INFO,
  });
  rtsp_server.start();

  std::span<const uint8_t> frame_data(reinterpret_cast<const uint8_t *>(jpeg_data),
                                      sizeof(jpeg_data));
  espp::JpegFrame jpeg_frame(frame_data);

  logger.info("Parsed JPEG image, num bytes: {}", jpeg_frame.get_data().size());
  logger.info("Created frame of size {}x{}", jpeg_frame.get_width(), jpeg_frame.get_height());
  rtsp_server.send_frame(jpeg_frame);
  //! [rtsp_server_example]

  //! [rtsp_client_example]
  espp::RtspClient rtsp_client({
      .server_address = ip_address,
      .rtsp_port = CONFIG_RTSP_SERVER_PORT,
      .path = "/mjpeg/1",
      .on_jpeg_frame =
          [](std::shared_ptr<espp::JpegFrame> jpeg_frame) {
            fmt::print("Got JPEG frame of size {}x{}\n", jpeg_frame->get_width(),
                       jpeg_frame->get_height());
          },
      .log_level = espp::Logger::Verbosity::ERROR,
  });

  std::error_code ec;

  do {
    ec.clear();
    rtsp_client.connect(ec);
    if (ec) {
      logger.error("Error connecting to server: {}", ec.message());
      logger.info("Retrying in 1s...");
      std::this_thread::sleep_for(1s);
    }
  } while (ec);

  rtsp_client.describe(ec);
  if (ec) {
    logger.error("Error describing server: {}", ec.message());
  }

  rtsp_client.setup(ec);
  if (ec) {
    logger.error("Error setting up server: {}", ec.message());
  }

  rtsp_client.play(ec);
  if (ec) {
    logger.error("Error playing server: {}", ec.message());
  }
  //! [rtsp_client_example]

  while (true) {
    rtsp_server.send_frame(jpeg_frame);
    std::this_thread::sleep_for(100ms);
  }

  // --------------------------------------------------------------------------
  // Mode: Server only (MJPEG)
  // --------------------------------------------------------------------------
#elif CONFIG_RTSP_EXAMPLE_MODE_SERVER_ONLY

  const std::string server_uri = fmt::format("rtsp://{}:{}/mjpeg/1", ip_address, server_port);
  logger.info("Starting RTSP Server (server-only mode) on port {}", server_port);
  logger.info("RTSP URI: {}", server_uri);

  espp::RtspServer rtsp_server({
      .server_address = ip_address,
      .port = server_port,
      .path = "/mjpeg/1",
      .log_level = espp::Logger::Verbosity::INFO,
  });
  rtsp_server.start();

  std::span<const uint8_t> frame_data(reinterpret_cast<const uint8_t *>(jpeg_data),
                                      sizeof(jpeg_data));
  espp::JpegFrame jpeg_frame(frame_data);

  logger.info("Streaming JPEG frame ({}x{}), connect with an RTSP client...",
              jpeg_frame.get_width(), jpeg_frame.get_height());

  while (true) {
    rtsp_server.send_frame(jpeg_frame);
    std::this_thread::sleep_for(100ms);
  }

  // --------------------------------------------------------------------------
  // Mode: Client only
  // --------------------------------------------------------------------------
#elif CONFIG_RTSP_EXAMPLE_MODE_CLIENT_ONLY

  logger.info("Starting RTSP Client (client-only mode)");
  logger.info("Connecting to {}:{}", CONFIG_RTSP_CLIENT_SERVER_ADDRESS, server_port);

  espp::RtspClient rtsp_client({
      .server_address = CONFIG_RTSP_CLIENT_SERVER_ADDRESS,
      .rtsp_port = CONFIG_RTSP_SERVER_PORT,
      .path = "/mjpeg/1",
      .on_jpeg_frame =
          [&logger](std::shared_ptr<espp::JpegFrame> jpeg_frame) {
            logger.info("Got JPEG frame: {}x{}", jpeg_frame->get_width(), jpeg_frame->get_height());
          },
      .log_level = espp::Logger::Verbosity::INFO,
  });

  std::error_code ec;

  do {
    ec.clear();
    rtsp_client.connect(ec);
    if (ec) {
      logger.error("Error connecting: {}", ec.message());
      logger.info("Retrying in 1s...");
      std::this_thread::sleep_for(1s);
    }
  } while (ec);

  rtsp_client.describe(ec);
  if (ec) {
    logger.error("Error describing: {}", ec.message());
  }

  rtsp_client.setup(ec);
  if (ec) {
    logger.error("Error setting up: {}", ec.message());
  }

  rtsp_client.play(ec);
  if (ec) {
    logger.error("Error playing: {}", ec.message());
  }

  logger.info("Client is playing, receiving frames...");
  while (true) {
    std::this_thread::sleep_for(1s);
  }

  // --------------------------------------------------------------------------
  // Mode: Multi-track server (MJPEG video + generic audio)
  // --------------------------------------------------------------------------
#elif CONFIG_RTSP_EXAMPLE_MODE_MULTITRACK

  //! [rtsp_server_multitrack_example]
  const std::string server_uri = fmt::format("rtsp://{}:{}/stream", ip_address, server_port);
  logger.info("Starting multi-track RTSP Server on port {}", server_port);
  logger.info("RTSP URI: {}", server_uri);

  espp::RtspServer rtsp_server({
      .server_address = ip_address,
      .port = server_port,
      .path = "/stream",
      .log_level = espp::Logger::Verbosity::INFO,
  });

  // Track 0: MJPEG video
  auto mjpeg_packetizer = std::make_shared<espp::MjpegPacketizer>(
      espp::MjpegPacketizer::Config{.max_payload_size = 1400});
  rtsp_server.add_track(
      espp::RtspServer::TrackConfig{.track_id = 0, .packetizer = mjpeg_packetizer});

  // Track 1: Generic audio (e.g., 16-bit PCM at 16kHz)
  auto audio_packetizer = std::make_shared<espp::GenericPacketizer>(espp::GenericPacketizer::Config{
      .max_payload_size = 1400,
      .payload_type = 97,
      .clock_rate = 16000,
      .encoding_name = "L16",
      .channels = 1,
      .media_type = espp::MediaType::AUDIO,
  });
  rtsp_server.add_track(
      espp::RtspServer::TrackConfig{.track_id = 1, .packetizer = audio_packetizer});

  rtsp_server.start();

  // Prepare video frame
  std::span<const uint8_t> jpeg_raw(reinterpret_cast<const uint8_t *>(jpeg_data),
                                    sizeof(jpeg_data));

  // Prepare synthetic audio frame (320 samples of 16-bit PCM = 640 bytes = 20ms @ 16kHz)
  std::vector<uint8_t> audio_frame(640, 0x00);
  for (size_t i = 0; i < audio_frame.size(); i += 2) {
    // Generate a simple 440Hz sine-like pattern for audibility
    uint16_t sample = static_cast<uint16_t>(128 * (i % 73)); // pseudo pattern
    audio_frame[i] = sample & 0xFF;
    audio_frame[i + 1] = (sample >> 8) & 0xFF;
  }

  logger.info("Streaming MJPEG video (track 0) + audio (track 1)...");

  while (true) {
    // Send video frame using generic send_frame(track_id, data)
    rtsp_server.send_frame(0, jpeg_raw);
    // Send audio frame
    rtsp_server.send_frame(1, std::span<const uint8_t>(audio_frame.data(), audio_frame.size()));
    std::this_thread::sleep_for(100ms);
  }
  //! [rtsp_server_multitrack_example]

#endif // CONFIG_RTSP_EXAMPLE_MODE_*
}
