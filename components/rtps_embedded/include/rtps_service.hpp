#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <vector>

#include "rtps_message.hpp" // RtpsMessage, RtpsProtocol, detail::rtps_(de)serialize
#include "rtps_participant.hpp"

namespace espp {

#ifdef RTPS_WITH_RPC

/// @brief Typed service server (RMI): answers Request messages with Response
/// messages, with no manual CDR handling.
///
/// A thin, header-only wrapper over espp::RtpsParticipant that (de)serializes the
/// reflectable Request/Response structs around the byte-level service API. Works
/// for both the ROS 2-interoperable and the native protocol (see Config::protocol).
///
/// @code
/// struct AddReq { int64_t a, b; };
/// struct AddResp { int64_t sum; };
/// espp::ServiceServer<AddReq, AddResp> server(participant, {
///     .service = "/add_two_ints",
///     .type_name = "example_interfaces::srv::dds_::AddTwoInts",
///     .handler = [](const AddReq &r) { return AddResp{r.a + r.b}; }});
/// @endcode
///
/// @tparam Request  Reflectable request message type (the service Request).
/// @tparam Response Reflectable response message type (the service Response).
template <RtpsMessage Request, RtpsMessage Response> class ServiceServer {
public:
  /// Handler: given a typed request, return the typed response. Runs on an
  /// engine worker thread - return promptly.
  using handler_t = std::function<Response(const Request &)>;

  /// Configuration for a typed service server.
  struct Config {
    std::string service;   ///< Service name, e.g. "/add_two_ints".
    std::string type_name; ///< Base DDS type (ROS 2), or any matching name (native).
    handler_t handler;     ///< Request -> Response.
    RtpsProtocol protocol{RtpsProtocol::ROS2}; ///< Wire protocol.
  };

  /// Construct and register the server on a started participant, which must
  /// outlive this object. Check is_valid().
  /// \param participant The started participant to serve through.
  /// \param config The server configuration (service name, type, handler).
  ServiceServer(RtpsParticipant &participant, const Config &config) {
    auto handler = config.handler;
    auto byte_handler = [handler](std::span<const uint8_t> req_bytes) -> std::vector<uint8_t> {
      auto req = detail::rtps_deserialize<Request>(req_bytes);
      if (!req || !handler) {
        return {};
      }
      return detail::rtps_serialize<Response>(handler(*req));
    };
    if (config.protocol == RtpsProtocol::NATIVE) {
      valid_ = participant.add_native_service_server({config.service, config.type_name},
                                                     std::move(byte_handler));
    } else {
      valid_ = participant.add_service_server({config.service, config.type_name},
                                              std::move(byte_handler));
    }
  }

  /// \return True if the server registered successfully.
  [[nodiscard]] bool is_valid() const { return valid_; }

private:
  bool valid_{false};
};

/// @brief Typed service client (RMI): call a service with a Request and get a
/// Response, with no manual CDR handling. Blocking, callback, and future styles.
///
/// @code
/// espp::ServiceClient<AddReq, AddResp> client(participant, {
///     .service = "/add_two_ints",
///     .type_name = "example_interfaces::srv::dds_::AddTwoInts"});
/// if (auto resp = client.call(AddReq{7, 35}, 1s)) use(resp->sum);
/// @endcode
///
/// @tparam Request  Reflectable request message type (the service Request).
/// @tparam Response Reflectable response message type (the service Response).
template <RtpsMessage Request, RtpsMessage Response> class ServiceClient {
public:
  /// Callback delivering the typed response for a call_async() request. Runs on
  /// an engine worker thread - return promptly.
  using response_callback_t = std::function<void(const Response &)>;

  /// Configuration for a typed service client.
  struct Config {
    std::string service;   ///< Service name, e.g. "/add_two_ints".
    std::string type_name; ///< Base DDS type (ROS 2), or any matching name (native).
    RtpsProtocol protocol{RtpsProtocol::ROS2}; ///< Wire protocol.
  };

  /// Construct and register the client on a started participant, which must
  /// outlive this object. Check is_valid().
  /// \param participant The started participant to call through.
  /// \param config The client configuration (service name, type, protocol).
  ServiceClient(RtpsParticipant &participant, const Config &config) {
    if (config.protocol == RtpsProtocol::NATIVE) {
      native_ = participant.add_native_service_client({config.service, config.type_name});
    } else {
      ros_ = participant.add_service_client({config.service, config.type_name});
    }
  }

  /// \return True if the client registered successfully.
  [[nodiscard]] bool is_valid() const { return ros_ != nullptr || native_ != nullptr; }

  /// Blocking call (RMI): send the request and wait for the correlated reply.
  /// \param request The typed request.
  /// \param timeout How long to wait for the reply.
  /// \return The Response, or std::nullopt on timeout / failure. Do not call
  ///         from within an engine callback (it would deadlock).
  std::optional<Response> call(const Request &request, std::chrono::milliseconds timeout) {
    const auto req = detail::rtps_serialize<Request>(request);
    std::optional<std::vector<uint8_t>> reply;
    if (ros_) {
      reply = ros_->call(req, timeout);
    } else if (native_) {
      reply = native_->call(req, timeout);
    }
    if (!reply) {
      return std::nullopt;
    }
    return detail::rtps_deserialize<Response>(*reply);
  }

  /// Async call (AMI): on_response(Response) is invoked when the correlated reply
  /// arrives (on an engine worker thread).
  /// \param request The typed request.
  /// \param on_response Called once with the typed response.
  /// \return False if the request could not be queued.
  bool call_async(const Request &request, response_callback_t on_response) {
    const auto req = detail::rtps_serialize<Request>(request);
    auto cb = [on_response](std::span<const uint8_t> reply_bytes) {
      auto resp = detail::rtps_deserialize<Response>(reply_bytes);
      if (resp && on_response) {
        on_response(*resp);
      }
    };
    if (ros_) {
      return ros_->call_async(req, std::move(cb));
    }
    if (native_) {
      return native_->call_async(req, std::move(cb));
    }
    return false;
  }

  /// Future-based call (AMI): the future becomes ready with the Response
  /// (std::nullopt if the request could not be queued). Works for both the ROS 2
  /// and native protocols (built on call_async). Wait on the future - or
  /// wait_for a timeout - from the caller; do not block a worker thread.
  std::future<std::optional<Response>> call_future(const Request &request) {
    auto promise = std::make_shared<std::promise<std::optional<Response>>>();
    auto future = promise->get_future();
    if (!call_async(request, [promise](const Response &r) { promise->set_value(r); })) {
      promise->set_value(std::nullopt);
    }
    return future;
  }

private:
  std::shared_ptr<RtpsParticipant::ServiceClient> ros_;
  std::shared_ptr<RtpsParticipant::NativeServiceClient> native_;
};

#endif // RTPS_WITH_RPC

} // namespace espp
