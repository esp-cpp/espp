#pragma once

#include "socket_msvc.hpp"

#include <memory>
#include <span>
#include <string>
#include <system_error>
#include <vector>

#if defined(ESP_PLATFORM)
#include <esp_random.h>
#else
#include <random>
#endif

#include "base_component.hpp"
#include "task.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

#include "jpeg_frame.hpp"
#include "mjpeg_packetizer.hpp"
#include "rtcp_packet.hpp"
#include "rtp_jpeg_packet.hpp"
#include "rtp_packet.hpp"
#include "rtp_packetizer.hpp"
#include "rtp_types.hpp"

#include "rtsp_session.hpp"

namespace espp {
/// Class for streaming MJPEG data from a camera using RTSP + RTP
/// Starts a TCP socket to listen for RTSP connections, and then spawns off a
/// new RTSP session for each connection.
/// @see RtspSession
/// @note This class does not currently send RTCP packets
///
/// \section rtsp_server_ex1 RtspServer example
/// \snippet rtsp_example.cpp rtsp_server_example
class RtspServer : public BaseComponent {
public:
  /// @brief Configuration for the RTSP server
  struct Config {
    std::string server_address; ///< The ip address of the server
    int port;                   ///< The port to listen on
    std::string path;           ///< The path to the RTSP stream
    size_t max_data_size =
        1000; ///< The maximum size of RTP packet data for the MJPEG stream. Frames will be broken
              ///< up into multiple packets if they are larger than this. It seems that 1500 works
              ///< well for sending, but is too large for the esp32 (camera-display) to receive
              ///< properly.
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::WARN; ///< The log level for the RTSP server
  };

  /// Configuration for a media track to be registered with the server
  struct TrackConfig {
    int track_id{0};                                 ///< Track identifier
    std::shared_ptr<espp::RtpPacketizer> packetizer; ///< Codec-specific packetizer
  };

  /// @brief Construct an RTSP server
  /// @param config The configuration for the RTSP server
  explicit RtspServer(const espp::RtspServer::Config &config);

  /// @brief Destroy the RTSP server
  ~RtspServer();

  /// @brief Sets the log level for the RTSP sessions created by this server
  /// @note This does not affect the log level of the RTSP server itself
  /// @note This does not change the log level of any sessions that have
  ///       already been created
  /// @param log_level The log level to set
  void set_session_log_level(espp::Logger::Verbosity log_level);

  /// @brief Start the RTSP server
  /// Starts the accept task, session task, and binds the RTSP socket
  /// @param accept_timeout The timeout for accepting new connections
  /// @return True if the server was started successfully, false otherwise
  bool start(const std::chrono::duration<float> &accept_timeout = std::chrono::seconds(5));

  /// @brief Stop the FTP server
  /// Stops the accept task, session task, and closes the RTSP socket
  void stop();

  /// @brief Register a media track with the server.
  /// Each track has its own packetizer, SSRC, and sequence number.
  /// @param config Track configuration including the packetizer.
  void add_track(const TrackConfig &config);

  /// @brief Send a frame on a specific track.
  /// The track's packetizer splits the frame into RTP payload chunks,
  /// which are then wrapped with RTP headers and queued for delivery.
  /// @note Overwrites any existing pending packets for this track.
  /// @param track_id The track to send on.
  /// @param frame_data Raw encoded frame data.
  void send_frame(int track_id, std::span<const uint8_t> frame_data);

  /// @brief Send a JPEG frame over the RTSP connection (backward compatible).
  /// If no tracks have been added, lazily creates a default MJPEG track on
  /// track 0. Uses the legacy RtpJpegPacket packetization to preserve the
  /// exact wire format for existing MJPEG users.
  /// @note Overwrites any existing frame that has not been sent.
  /// @param frame The frame to send.
  void send_frame(const espp::JpegFrame &frame);

protected:
  /// Per-track state holding packetizer, RTP sequencing, and pending packets
  struct TrackState {
    int track_id{0};
    std::shared_ptr<RtpPacketizer> packetizer;
    uint32_t ssrc{0};
    uint16_t sequence_number{0};
    std::mutex packets_mutex;
    std::vector<std::unique_ptr<RtpPacket>> pending_packets;
  };

  bool accept_task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified);
  bool session_task_function(std::mutex &m, std::condition_variable &cv, bool &task_notified);

  /// Generate combined SDP from all registered tracks.
  /// @param session_path Full RTSP URL path
  /// @param session_id The session ID
  /// @param server_address The server address (ip:port)
  /// @return SDP body string
  std::string generate_sdp(const std::string &session_path, uint32_t session_id,
                           const std::string &server_address) const;

  /// Generate a random SSRC value
  /// @return A random 32-bit SSRC
  static uint32_t generate_ssrc() {
#if defined(ESP_PLATFORM)
    return esp_random();
#else
    static std::random_device rd;
    static std::mt19937 gen(rd());
    static std::uniform_int_distribution<uint32_t> dis;
    return dis(gen);
#endif
  }

  std::string server_address_; ///< the address of the server
  int port_;                   ///< the port of the RTSP server
  std::string path_; ///< the path of the RTSP server, e.g. rtsp:://\<ip\>:\<port\>/\<path\>

  espp::TcpSocket rtsp_socket_;

  size_t max_data_size_;

  std::vector<std::shared_ptr<TrackState>> tracks_;
  bool default_mjpeg_track_created_{false};

  espp::Logger::Verbosity session_log_level_{espp::Logger::Verbosity::WARN};
  std::mutex session_mutex_;
  std::unordered_map<int, std::unique_ptr<espp::RtspSession>> sessions_;

  std::unique_ptr<Task> accept_task_;
  std::unique_ptr<Task> session_task_;
};
} // namespace espp
