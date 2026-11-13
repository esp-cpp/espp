#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <span>
#include <string>

#include "rtps_message.hpp" // RtpsMessage, RtpsProtocol, detail::rtps_(de)serialize
#include "rtps_participant.hpp"
#include "rtps_service.hpp" // typed ServiceClient/Server (shares the CDR helpers)

namespace espp {

#ifdef RTPS_WITH_RPC

/// @brief Terminal goal status (mirrors action_msgs/msg/GoalStatus).
enum class GoalStatus : int8_t {
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6,
};

/// @brief The ROS 2 GetResult_Response message for a Result type: `status`
/// (a GoalStatus value, UNKNOWN 0 when the server does not know the goal) then
/// the result. Serialized as ONE message so every field of the Result is
/// aligned relative to the response (a result spliced in behind the status
/// afterwards has its int64 / float64 fields off by 4 bytes, see
/// rtps/rpc/action_types.hpp); the typed server and client exchange exactly
/// these bytes with ROS 2 clients and servers.
template <RtpsMessage Result> struct GetResultResponse {
  int8_t status{0};
  Result result{};
};

/// @brief Server-side handle to a running typed goal, passed to the execute
/// callback (which runs on its own thread). Publish feedback and terminate the
/// goal through it, all with typed messages (no manual CDR). Exactly one
/// terminator (succeed / abort) should be called per goal.
///
/// @tparam Goal     Reflectable goal message type.
/// @tparam Result   Reflectable result message type.
/// @tparam Feedback Reflectable feedback message type.
///
/// \section rtps_goal_handle_ex1 ActionGoalHandle Example
/// The handle is the `h` passed to the ActionServer execute callback:
/// \snippet rtps_example.cpp rtps action server example
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionGoalHandle {
public:
  /// \return The typed goal being executed.
  const Goal &goal() const { return goal_; }
  /// \return True if the client has requested cancellation of this goal (both
  ///         the ROS 2 and native protocols). A long-running execute callback
  ///         should poll this and wind the goal down - calling canceled() - when
  ///         it becomes true.
  bool is_canceling() const { return is_canceling_ ? is_canceling_() : false; }
  /// Publish a typed feedback message for this goal.
  /// \param feedback The feedback to send to the client.
  void publish_feedback(const Feedback &feedback) const {
    if (publish_feedback_) {
      publish_feedback_(detail::rtps_serialize<Feedback>(feedback));
    }
  }
  /// Terminate the goal as SUCCEEDED and deliver the result to the client.
  /// \param result The final result.
  void succeed(const Result &result) const { terminate(GoalStatus::SUCCEEDED, result, succeed_); }
  /// Terminate the goal as ABORTED and deliver the result to the client.
  /// \param result The (partial/error) result.
  void abort(const Result &result) const { terminate(GoalStatus::ABORTED, result, abort_); }
  /// Terminate the goal as CANCELED (in response to is_canceling()) and deliver
  /// the (partial) result to the client.
  /// \param result The result gathered before cancellation.
  void canceled(const Result &result) const { terminate(GoalStatus::CANCELED, result, canceled_); }

  /// @cond INTERNAL
  // Populated by ActionServer from a byte-level goal handle; not user-facing.
  Goal goal_{};
  std::function<void(std::span<const uint8_t>)> publish_feedback_;
  // Native protocol: the result alone (its own framing).
  std::function<void(std::span<const uint8_t>)> succeed_;
  std::function<void(std::span<const uint8_t>)> abort_;
  std::function<void(std::span<const uint8_t>)> canceled_;
  // ROS 2 protocol: the whole GetResult_Response (see GetResultResponse).
  std::function<void(int8_t, std::span<const uint8_t>)> terminate_response_;
  std::function<bool()> is_canceling_;
  /// @endcond

private:
  void terminate(GoalStatus status, const Result &result,
                 const std::function<void(std::span<const uint8_t>)> &native) const {
    if (terminate_response_) {
      terminate_response_(static_cast<int8_t>(status),
                          detail::rtps_serialize<GetResultResponse<Result>>(
                              GetResultResponse<Result>{static_cast<int8_t>(status), result}));
    } else if (native) {
      native(detail::rtps_serialize<Result>(result));
    }
  }
};

/// @brief Typed action server (AMI): runs long goals with typed Goal / Result /
/// Feedback messages, no manual CDR handling.
///
/// @code
/// struct Goal { int32_t order; };
/// struct Seq  { std::vector<int32_t> sequence; };  // Result + Feedback
/// espp::ActionServer<Goal, Seq, Seq> server(participant, {
///     .action = "/fibonacci",
///     .type_name = "example_interfaces::action::dds_::Fibonacci",
///     .on_goal = [](const Goal &g) { return g.order > 0; },
///     .execute = [](auto &h) {
///        h.publish_feedback(...); h.succeed(...); }});
/// @endcode
///
/// @tparam Goal     Reflectable goal message type.
/// @tparam Result   Reflectable result message type.
/// @tparam Feedback Reflectable feedback message type.
///
/// \section rtps_action_server_ex1 ActionServer Example
/// \snippet rtps_example.cpp rtps action server example
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionServer {
public:
  /// The typed goal handle passed to the execute callback.
  using Handle = ActionGoalHandle<Goal, Result, Feedback>;
  /// Called when a goal arrives; return true to accept it, false to reject.
  /// Runs on an engine worker thread - return promptly.
  using goal_callback_t = std::function<bool(const Goal &)>;
  /// Called (on one of the server's execute workers, see Config) to run an
  /// accepted goal to completion via the handle (publish_feedback / succeed /
  /// abort).
  using execute_callback_t = std::function<void(Handle &)>;

  /// Configuration for a typed action server.
  struct Config {
    std::string action;         ///< Action name, e.g. "/fibonacci".
    std::string type_name;      ///< Base DDS type (ROS 2), or any matching name (native).
    goal_callback_t on_goal;    ///< Accept/reject each incoming goal.
    execute_callback_t execute; ///< Run each accepted goal (on an execute worker).
    RtpsProtocol protocol{RtpsProtocol::ROS2}; ///< Wire protocol.
    /// Goals held at once (executing or waiting for a worker); see
    /// RtpsParticipant::ActionConfig::max_goals_in_flight. 0 = unlimited.
    std::size_t max_goals_in_flight{8};
    /// Execute workers (see RtpsParticipant::ActionConfig::execute_workers)
    /// and their task configuration (stack size, priority, core id; see
    /// RtpsParticipant::ActionConfig::execute_task).
    std::size_t execute_workers{1};
    espp::Task::BaseConfig execute_task{
        .name = "",
        .stack_size_bytes = 4096,
        .priority = 5,
    };
    /// Priority band inherited by all of the action's underlying endpoints
    /// (see RtpsParticipant::ActionConfig::band, incl. the ration note).
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP marking for the traffic the server sends (see
    /// RtpsParticipant::ActionConfig::dscp).
    std::optional<espp::Dscp> dscp{};
  };

  /// Construct and register the action server on a started participant, which
  /// must outlive this object. Check is_valid().
  /// \param participant The started participant to serve through.
  /// \param config The server configuration.
  ActionServer(RtpsParticipant &participant, const Config &config) {
    auto on_goal = config.on_goal;
    auto execute = config.execute;
    if (config.protocol == RtpsProtocol::NATIVE) {
      valid_ = participant.add_native_action_server(
          {.action = config.action,
           .type_name = config.type_name,
           .band = config.band,
           .dscp = config.dscp,
           .max_goals_in_flight = config.max_goals_in_flight,
           .execute_workers = config.execute_workers,
           .execute_task = config.execute_task},
          [on_goal](std::span<const uint8_t> goal_bytes) -> bool {
            auto g = detail::rtps_deserialize<Goal>(goal_bytes);
            return g && (!on_goal || on_goal(*g));
          },
          [execute](RtpsParticipant::NativeGoalHandle bh) {
            auto g = detail::rtps_deserialize<Goal>(bh.goal());
            if (!g) {
              return;
            }
            Handle h;
            h.goal_ = std::move(*g);
            h.publish_feedback_ = [bh](std::span<const uint8_t> b) { bh.publish_feedback(b); };
            h.succeed_ = [bh](std::span<const uint8_t> b) { bh.succeed(b); };
            h.abort_ = [bh](std::span<const uint8_t> b) { bh.abort(b); };
            h.canceled_ = [bh](std::span<const uint8_t> b) { bh.canceled(b); };
            h.is_canceling_ = [bh]() { return bh.is_canceling(); };
            if (execute) {
              execute(h);
            }
          });
    } else {
      // A ROS 2 client asking for a goal this server does not know gets a
      // well-formed response: status UNKNOWN with a default Result.
      valid_ = participant.add_action_server(
          {.action = config.action,
           .type_name = config.type_name,
           .band = config.band,
           .dscp = config.dscp,
           .max_goals_in_flight = config.max_goals_in_flight,
           .execute_workers = config.execute_workers,
           .execute_task = config.execute_task,
           .unknown_goal_response = detail::rtps_serialize<GetResultResponse<Result>>(
               GetResultResponse<Result>{0, Result{}})},
          [on_goal](const RtpsParticipant::GoalId &, std::span<const uint8_t> goal_bytes) -> bool {
            auto g = detail::rtps_deserialize<Goal>(goal_bytes);
            return g && (!on_goal || on_goal(*g));
          },
          [execute](RtpsParticipant::ActionGoalHandle bh) {
            auto g = detail::rtps_deserialize<Goal>(bh.goal());
            if (!g) {
              return;
            }
            Handle h;
            h.goal_ = std::move(*g);
            h.publish_feedback_ = [bh](std::span<const uint8_t> b) { bh.publish_feedback(b); };
            // The result travels inside the GetResult_Response, serialized as
            // one message (GetResultResponse), never spliced in afterwards.
            h.terminate_response_ = [bh](int8_t status, std::span<const uint8_t> response) {
              bh.terminate_with_response(status, response);
            };
            h.is_canceling_ = [bh]() { return bh.is_canceling(); };
            if (execute) {
              execute(h);
            }
          });
    }
  }

  /// \return True if the action server registered successfully.
  [[nodiscard]] bool is_valid() const { return valid_; }

private:
  bool valid_{false};
};

/// @brief Typed action client (AMI): send typed goals and receive typed feedback
/// + result, no manual CDR handling.
///
/// @code
/// espp::ActionClient<Goal, Seq, Seq> client(participant, {
///     .action = "/fibonacci",
///     .type_name = "example_interfaces::action::dds_::Fibonacci"});
/// client.send_goal(Goal{5},
///     [](const Seq &fb) { ... },
///     [](espp::GoalStatus st, const Seq &res) { ... });
/// @endcode
///
/// @tparam Goal     Reflectable goal message type.
/// @tparam Result   Reflectable result message type.
/// @tparam Feedback Reflectable feedback message type.
///
/// \section rtps_action_client_ex1 ActionClient Example
/// \snippet rtps_example.cpp rtps rpc client example
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionClient {
public:
  /// Callback delivering one typed feedback message (on an engine worker thread).
  using feedback_callback_t = std::function<void(const Feedback &)>;
  /// Callback delivering the terminal status + typed result, once per goal.
  using result_callback_t = std::function<void(GoalStatus, const Result &)>;

  /// Configuration for a typed action client.
  struct Config {
    std::string action;    ///< Action name, e.g. "/fibonacci".
    std::string type_name; ///< Base DDS type (ROS 2), or any matching name (native).
    RtpsProtocol protocol{RtpsProtocol::ROS2}; ///< Wire protocol.
    /// Priority band inherited by all of the action's underlying endpoints
    /// (see RtpsParticipant::ActionConfig::band, incl. the ration note).
    espp::QosBand band{espp::QosBand::Normal};
    /// Optional DSCP marking for the traffic the client sends (see
    /// RtpsParticipant::ActionConfig::dscp).
    std::optional<espp::Dscp> dscp{};
  };

  /// Construct and register the action client on a started participant, which
  /// must outlive this object. Check is_valid().
  /// \param participant The started participant to drive the action through.
  /// \param config The client configuration.
  ActionClient(RtpsParticipant &participant, const Config &config) {
    if (config.protocol == RtpsProtocol::NATIVE) {
      native_ = participant.add_native_action_client(
          {config.action, config.type_name, config.band, config.dscp});
    } else {
      ros_ = participant.add_action_client(
          {config.action, config.type_name, config.band, config.dscp});
    }
  }

  /// \return True if the action client registered successfully.
  [[nodiscard]] bool is_valid() const { return ros_ != nullptr || native_ != nullptr; }

  /// ROS 2 protocol: action servers matched right now (see
  /// RtpsParticipant::ActionClient::server_count()). Native protocol: always 1,
  /// a native goal is queued and delivered once its server appears.
  [[nodiscard]] std::size_t server_count() const {
    return ros_ ? ros_->server_count() : (native_ ? 1 : 0);
  }

  /// ROS 2 protocol: block until an action server is matched, like
  /// rclcpp_action's wait_for_action_server(); a goal sent before that is
  /// refused. Native protocol: returns true at once. \return False on timeout.
  bool wait_for_server(std::chrono::milliseconds timeout) const {
    return ros_ ? ros_->wait_for_server(timeout) : is_valid();
  }

  /// Send a typed goal to the server.
  /// \param goal The typed goal.
  /// \param on_feedback Invoked for each feedback message during execution.
  /// \param on_result Invoked once with the terminal status + result (an empty
  ///        Result and non-SUCCEEDED status if the goal was rejected).
  /// \return True if the goal was queued.
  bool send_goal(const Goal &goal, feedback_callback_t on_feedback, result_callback_t on_result) {
    const auto goal_bytes = detail::rtps_serialize<Goal>(goal);
    auto fb_cb = [on_feedback](std::span<const uint8_t> b) {
      auto fb = detail::rtps_deserialize<Feedback>(b);
      if (fb && on_feedback) {
        on_feedback(*fb);
      }
    };
    auto res_cb = [on_result](int8_t status, std::span<const uint8_t> b) {
      auto res = detail::rtps_deserialize<Result>(b);
      if (on_result) {
        on_result(static_cast<GoalStatus>(status), res ? *res : Result{});
      }
    };
    if (ros_) {
      // The raw GetResult_Response, read as one message (GetResultResponse):
      // a Result with int64 / float64 fields only decodes correctly this way.
      // An empty / unreadable response is a rejected or abandoned goal.
      auto response_cb = [on_result](std::span<const uint8_t> response) {
        auto res = detail::rtps_deserialize<GetResultResponse<Result>>(response);
        if (on_result) {
          if (res) {
            on_result(static_cast<GoalStatus>(res->status), res->result);
          } else {
            on_result(GoalStatus::ABORTED, Result{});
          }
        }
      };
      auto id = ros_->send_goal_raw(goal_bytes, std::move(fb_cb), std::move(response_cb));
      if (id) {
        std::lock_guard<std::mutex> lock(latest_->m);
        latest_->ros_id = *id;
      }
      return id.has_value();
    }
    if (native_) {
      auto lat = latest_;
      return native_->send_goal(
          goal_bytes, std::move(fb_cb),
          [res_cb](uint8_t status, std::span<const uint8_t> b) {
            res_cb(static_cast<int8_t>(status), b);
          },
          [lat](uint32_t handle) {
            std::lock_guard<std::mutex> lock(lat->m);
            lat->native_handle = handle;
          });
    }
    return false;
  }

  /// The server's verdict on a cancel request (ROS 2 protocol; see
  /// RtpsParticipant::ActionClient::cancel_callback_t).
  using cancel_callback_t = RtpsParticipant::ActionClient::cancel_callback_t;

  /// Request cancellation of the most recently accepted goal (works on both the
  /// ROS 2 and native protocols). The server observes the cancel via its goal
  /// handle's is_canceling() and should wind the goal down cooperatively.
  /// \param on_response ROS 2 protocol only: the server's return code and how
  ///        many goals are now canceling (the native protocol has no verdict;
  ///        the callback is not invoked there).
  /// \return True if the cancel request was queued.
  bool cancel_goal(cancel_callback_t on_response = nullptr) {
    std::lock_guard<std::mutex> lock(latest_->m);
    if (native_ && latest_->native_handle) {
      return native_->cancel_goal(*latest_->native_handle);
    }
    if (ros_ && latest_->ros_id) {
      return ros_->cancel_goal(*latest_->ros_id, std::move(on_response));
    }
    return false;
  }

private:
  // Tracks the most recently accepted goal so cancel_goal() can target it. A
  // shared_ptr so the native on_accepted callback (which runs later, on an engine
  // thread) can record the server-assigned handle here.
  struct Latest {
    std::mutex m;
    std::optional<uint32_t> native_handle;
    std::optional<RtpsParticipant::GoalId> ros_id;
  };
  std::shared_ptr<Latest> latest_ = std::make_shared<Latest>();
  std::shared_ptr<RtpsParticipant::ActionClient> ros_;
  std::shared_ptr<RtpsParticipant::NativeActionClient> native_;
};

#endif // RTPS_WITH_RPC

} // namespace espp
