#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string>

#include "cdr.hpp"
#include "rtps_participant.hpp"
#include "rtps_service.hpp" // RtpsProtocol + detail::rtps_serialize/deserialize

namespace espp {

#ifdef RTPS_WITH_RPC

/// @brief Terminal goal status (mirrors action_msgs/msg/GoalStatus).
enum class GoalStatus : int8_t {
  SUCCEEDED = 4,
  CANCELED = 5,
  ABORTED = 6,
};

/// @brief Server-side handle to a running typed goal, passed to the execute
/// callback (which runs on its own thread). Publish feedback and terminate the
/// goal through it, all with typed messages (no manual CDR).
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionGoalHandle {
public:
  /// The typed goal for this handle.
  const Goal &goal() const { return goal_; }
  /// True if a cancel has been requested (ROS 2 protocol only; native: false).
  bool is_canceling() const { return is_canceling_ ? is_canceling_() : false; }
  /// Publish a typed feedback message for this goal.
  void publish_feedback(const Feedback &feedback) const {
    if (publish_feedback_) {
      publish_feedback_(detail::rtps_serialize<Feedback>(feedback));
    }
  }
  /// Terminate the goal SUCCEEDED with a typed result.
  void succeed(const Result &result) const {
    if (succeed_) {
      succeed_(detail::rtps_serialize<Result>(result));
    }
  }
  /// Terminate the goal ABORTED with a typed result.
  void abort(const Result &result) const {
    if (abort_) {
      abort_(detail::rtps_serialize<Result>(result));
    }
  }

  // Internal: constructed by ActionServer from a byte-level goal handle.
  Goal goal_{};
  std::function<void(std::span<const uint8_t>)> publish_feedback_;
  std::function<void(std::span<const uint8_t>)> succeed_;
  std::function<void(std::span<const uint8_t>)> abort_;
  std::function<bool()> is_canceling_;
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
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionServer {
public:
  using Handle = ActionGoalHandle<Goal, Result, Feedback>;
  /// Called when a goal arrives; return true to accept.
  using goal_callback_t = std::function<bool(const Goal &)>;
  /// Called (own thread) to run an accepted goal to completion.
  using execute_callback_t = std::function<void(Handle &)>;

  /// Configuration for a typed action server.
  struct Config {
    std::string action;    ///< Action name, e.g. "/fibonacci".
    std::string type_name; ///< Base DDS type (ROS 2), or any matching name (native).
    goal_callback_t on_goal;
    execute_callback_t execute;
    RtpsProtocol protocol{RtpsProtocol::ROS2};
  };

  ActionServer(RtpsParticipant &participant, const Config &config) {
    auto on_goal = config.on_goal;
    auto execute = config.execute;
    if (config.protocol == RtpsProtocol::NATIVE) {
      valid_ = participant.add_native_action_server(
          {config.action, config.type_name},
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
            if (execute) {
              execute(h);
            }
          });
    } else {
      valid_ = participant.add_action_server(
          {config.action, config.type_name},
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
            h.succeed_ = [bh](std::span<const uint8_t> b) { bh.succeed(b); };
            h.abort_ = [bh](std::span<const uint8_t> b) { bh.abort(b); };
            h.is_canceling_ = [bh]() { return bh.is_canceling(); };
            if (execute) {
              execute(h);
            }
          });
    }
  }

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
template <RtpsMessage Goal, RtpsMessage Result, RtpsMessage Feedback> class ActionClient {
public:
  using feedback_callback_t = std::function<void(const Feedback &)>;
  using result_callback_t = std::function<void(GoalStatus, const Result &)>;

  /// Configuration for a typed action client.
  struct Config {
    std::string action;    ///< Action name, e.g. "/fibonacci".
    std::string type_name; ///< Base DDS type (ROS 2), or any matching name (native).
    RtpsProtocol protocol{RtpsProtocol::ROS2};
  };

  ActionClient(RtpsParticipant &participant, const Config &config) {
    if (config.protocol == RtpsProtocol::NATIVE) {
      native_ = participant.add_native_action_client({config.action, config.type_name});
    } else {
      ros_ = participant.add_action_client({config.action, config.type_name});
    }
  }

  [[nodiscard]] bool is_valid() const { return ros_ != nullptr || native_ != nullptr; }

  /// Send a typed goal. on_feedback is invoked for each feedback message,
  /// on_result once when the goal terminates. \return True if queued.
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
      return ros_->send_goal(goal_bytes, std::move(fb_cb), std::move(res_cb)).has_value();
    }
    if (native_) {
      return native_->send_goal(goal_bytes, std::move(fb_cb),
                                [res_cb](uint8_t status, std::span<const uint8_t> b) {
                                  res_cb(static_cast<int8_t>(status), b);
                                });
    }
    return false;
  }

private:
  std::shared_ptr<RtpsParticipant::ActionClient> ros_;
  std::shared_ptr<RtpsParticipant::NativeActionClient> native_;
};

#endif // RTPS_WITH_RPC

} // namespace espp
