#pragma once

#include "socket_msvc.hpp"

#include <memory>
#include <string>
#include <system_error>
#include <unordered_map>
#include <vector>

#include "base_component.hpp"
#include "tcp_socket.hpp"
#include "udp_socket.hpp"

#include "jpeg_frame.hpp"

namespace espp {

/// A class for interacting with an RTSP server using RTP and RTCP over UDP
///
/// This class is used to connect to an RTSP server and receive JPEG frames
/// over RTP. It uses the TCP socket to send RTSP requests and receive RTSP
/// responses. It uses the UDP socket to receive RTP and RTCP packets.
///
/// The RTSP client is designed to be used with the RTSP server in the
/// [camera-streamer]https://github.com/esp-cpp/camera-streamer) project, but it
/// should work with any RTSP server that sends JPEG frames over RTP.
///
/// \section rtsp_client_ex1 RtspClient Example
/// \snippet rtsp_example.cpp rtsp_client_example
class RtspClient : public BaseComponent {
public:
  /// Function type for the callback to call when a JPEG frame is received
  using jpeg_frame_callback_t = std::function<void(std::unique_ptr<JpegFrame> jpeg_frame)>;

  /// Configuration for the RTSP client
  struct Config {
    std::string server_address;   ///< The server IP Address to connect to
    int rtsp_port{8554};          ///< The port of the RTSP server
    std::string path{"/mjpeg/1"}; ///< The path to the RTSP stream on the server. Will be appended
                                  ///< to the server address and port to form the full path of the
                                  ///< form "rtsp://<server_address>:<rtsp_port><path>"
    espp::RtspClient::jpeg_frame_callback_t
        on_jpeg_frame; ///< The callback to call when a JPEG frame is received
    espp::Logger::Verbosity log_level =
        espp::Logger::Verbosity::INFO; ///< The verbosity of the logger
  };

  /// Constructor
  /// \param config The configuration for the RTSP client
  explicit RtspClient(const Config &config);

  /// Destructor
  /// Disconnects from the RTSP server
  ~RtspClient();

  /// Send an RTSP request to the server
  /// \note This is a blocking call
  /// \note This will parse the response and set the session ID if it is
  ///      present in the response. If the response is not a 200 OK, then
  ///      an error code will be set and the response will be returned.
  ///      If the response is a 200 OK, then the response will be returned
  ///      and the error code will be set to success.
  /// \param method The method to use for connecting.
  ///       Options are "OPTIONS", "DESCRIBE", "SETUP", "PLAY", and "TEARDOWN"
  /// \param path The path to the RTSP stream on the server.
  /// \param extra_headers Any extra headers to send with the request. These
  ///      will be added to the request after the CSeq and Session headers. The
  ///      key is the header name and the value is the header value. For example,
  ///      {"Accept": "application/sdp"} will add "Accept: application/sdp" to the
  ///      request. The "User-Agent" header will be added automatically. The
  ///      "CSeq" and "Session" headers will be added automatically.
  ///      The "Accept" header will be added automatically. The "Transport"
  ///      header will be added automatically for the "SETUP" method. Defaults to
  ///      an empty map.
  /// \param ec The error code to set if an error occurs
  /// \return The response from the server
  std::string send_request(const std::string &method, const std::string &path,
                           const std::unordered_map<std::string, std::string> &extra_headers,
                           std::error_code &ec);

  /// Connect to the RTSP server
  /// Connects to the RTSP server and sends the OPTIONS request.
  /// \param ec The error code to set if an error occurs
  void connect(std::error_code &ec);

  /// Disconnect from the RTSP server
  /// Disconnects from the RTSP server and sends the TEARDOWN request.
  /// \param ec The error code to set if an error occurs
  void disconnect(std::error_code &ec);

  /// Describe the RTSP stream
  /// Sends the DESCRIBE request to the RTSP server and parses the response.
  /// \param ec The error code to set if an error occurs
  void describe(std::error_code &ec);

  /// Setup the RTSP stream
  /// \note Starts the RTP and RTCP threads.
  /// Sends the SETUP request to the RTSP server and parses the response.
  /// \note The default ports are 5000 and 5001 for RTP and RTCP respectively.
  /// \note The default receive timeout is 5 seconds.
  /// \param ec The error code to set if an error occurs
  void setup(std::error_code &ec);

  /// Setup the RTSP stream
  /// Sends the SETUP request to the RTSP server and parses the response.
  /// \note Starts the RTP and RTCP threads.
  /// \param rtp_port The RTP client port
  /// \param rtcp_port The RTCP client port
  /// \param receive_timeout The timeout for receiving RTP and RTCP packets
  /// \param ec The error code to set if an error occurs
  void setup(size_t rtp_port, size_t rtcp_port, const std::chrono::duration<float> &receive_timeout,
             std::error_code &ec);

  /// Play the RTSP stream
  /// Sends the PLAY request to the RTSP server and parses the response.
  /// \param ec The error code to set if an error occurs
  void play(std::error_code &ec);

  /// Pause the RTSP stream
  /// Sends the PAUSE request to the RTSP server and parses the response.
  /// \param ec The error code to set if an error occurs
  void pause(std::error_code &ec);

  /// Teardown the RTSP stream
  /// Sends the TEARDOWN request to the RTSP server and parses the response.
  /// \param ec The error code to set if an error occurs
  void teardown(std::error_code &ec);

protected:
  /// Parse the RTSP response
  /// \note Parses response data for the following fields:
  ///  - Status code
  ///  - Status message
  ///  - Session
  /// \note Increments the sequence number on success.
  /// \param response_data The response data to parse
  /// \param ec The error code to set if an error occurs
  /// \return True if the response was parsed successfully, false otherwise
  bool parse_response(const std::string &response_data, std::error_code &ec);

  /// Initialize the RTP socket
  /// \note Starts the RTP socket task.
  /// \param rtp_port The RTP client port
  /// \param receive_timeout The timeout for receiving RTP packets
  /// \param ec The error code to set if an error occurs
  void init_rtp(size_t rtp_port, const std::chrono::duration<float> &receive_timeout,
                std::error_code &ec);

  /// Initialize the RTCP socket
  /// \note Starts the RTCP socket task.
  /// \param rtcp_port The RTCP client port
  /// \param receive_timeout The timeout for receiving RTCP packets
  /// \param ec The error code to set if an error occurs
  void init_rtcp(size_t rtcp_port, const std::chrono::duration<float> &receive_timeout,
                 std::error_code &ec);

  /// Handle an RTP packet
  /// \note Parses the RTP packet and appends it to the current JPEG frame.
  /// \note If the packet is the last fragment of the JPEG frame, the frame is sent to the
  /// on_jpeg_frame callback. \note This function is called by the RTP socket task. \param data The
  /// data to handle \param sender_info The sender info \return Optional data to send back to the
  /// sender
  std::optional<std::vector<uint8_t>> handle_rtp_packet(std::vector<uint8_t> &data,
                                                        const espp::Socket::Info &sender_info);

  /// Handle an RTCP packet
  /// \note Parses the RTCP packet and sends a response if necessary.
  /// \note This function is called by the RTCP socket task.
  /// \param data The data to handle
  /// \param sender_info The sender info
  /// \return Optional data to send back to the sender
  std::optional<std::vector<uint8_t>> handle_rtcp_packet(std::vector<uint8_t> &data,
                                                         const espp::Socket::Info &sender_info);
  std::string server_address_;
  int rtsp_port_;

  espp::TcpSocket rtsp_socket_;
  espp::UdpSocket rtp_socket_;
  espp::UdpSocket rtcp_socket_;

  jpeg_frame_callback_t on_jpeg_frame_{nullptr};

  int cseq_ = 0;
  int video_port_ = 0;
  int video_payload_type_ = 0;
  std::string path_;
  std::string session_id_;
};

} // namespace espp
