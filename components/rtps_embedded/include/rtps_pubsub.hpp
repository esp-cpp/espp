#pragma once

#include <cstdint>
#include <functional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include "cdr.hpp"
#include "rtps_participant.hpp"

namespace espp {

/// @brief A type usable with the typed RTPS pub/sub layer.
///
/// Any reflectable struct the `cdr` component can serialize and deserialize
/// qualifies - no base class, macros, or member functions required. This mirrors
/// the ROS 2 / DDS message model: a plain data struct whose fields map to CDR.
template <typename T>
concept RtpsMessage = requires(const T &value, std::span<const std::byte> bytes) {
  { cdr::serialized_size<cdr::xcdr1>(value) } -> std::convertible_to<std::size_t>;
  {cdr::deserialize<T>(bytes)};
};

/// @brief Typed publisher: publish reflectable message structs on a topic.
///
/// A thin, header-only wrapper over espp::RtpsParticipant that removes the manual
/// CDR (de)serialization + byte-span handling of the untyped API. Serialization
/// uses the reflection-driven `cdr` component in ROS 2 / classic-CDR (XCDR1) wire
/// format, into a reused buffer so steady-state publishing does not allocate.
///
/// @code
/// struct Imu { float ax, ay, az; };   // any reflectable struct
/// espp::Publisher<Imu> pub(participant, {.topic = "rt/imu",
///                                        .type_name = "sensor_msgs::msg::dds_::Imu_",
///                                        .reliability = Reliability::RELIABLE});
/// pub.publish(Imu{0.1f, 0.2f, 9.8f});
/// @endcode
///
/// \note For ROS 2 interop use ROS 2 naming: topic "rt/<name>" and type
///       "<pkg>::msg::dds_::<Type>_" (e.g. "rt/chatter" +
///       "std_msgs::msg::dds_::String_").
template <RtpsMessage T> class Publisher {
public:
  /// Configuration for a typed publisher.
  struct Config {
    std::string topic;     ///< DDS topic name.
    std::string type_name; ///< DDS type name (must match the peer for interop).
    RtpsParticipant::Reliability reliability{
        RtpsParticipant::Reliability::BEST_EFFORT}; ///< Reliability QoS.
  };

  /// Construct and register a writer on the participant. The participant must
  /// already be started and must outlive this publisher. Check is_valid() (or
  /// the return of publish()) to detect registration failure.
  /// \param participant The started participant to publish through.
  /// \param config The publisher configuration.
  Publisher(RtpsParticipant &participant, const Config &config)
      : participant_(&participant)
      , topic_(config.topic) {
    valid_ = participant_->add_writer({
        .topic = config.topic,
        .type_name = config.type_name,
        .reliability = config.reliability,
    });
  }

  /// \return True if the writer was registered successfully.
  [[nodiscard]] bool is_valid() const { return valid_; }

  /// Publish one sample. Serializes into a reused buffer (no steady-state
  /// allocation) and hands the CDR bytes to the participant.
  /// \param sample The message to publish.
  /// \return True on success; false if invalid, serialization failed, or the
  ///         writer history was full.
  bool publish(const T &sample) {
    if (!valid_) {
      return false;
    }
    const std::size_t needed = cdr::serialized_size<cdr::xcdr1>(sample);
    if (buffer_.size() < needed) {
      buffer_.resize(needed);
    }
    const auto written = cdr::serialize_into<cdr::xcdr1>(sample, buffer_);
    if (!written) {
      return false;
    }
    return participant_->publish(
        topic_,
        std::span<const uint8_t>(reinterpret_cast<const uint8_t *>(buffer_.data()), *written));
  }

private:
  RtpsParticipant *participant_{nullptr};
  std::string topic_;
  std::vector<std::byte> buffer_; ///< reused serialization scratch (grows once)
  bool valid_{false};
};

/// @brief Typed subscriber: receive reflectable message structs from a topic.
///
/// A thin, header-only wrapper over espp::RtpsParticipant that deserializes each
/// CDR sample into a T and delivers it to a typed callback, removing the manual
/// byte-span + cdr::deserialize handling of the untyped API.
///
/// @code
/// espp::Subscriber<Imu> sub(participant, {.topic = "rt/imu",
///                                         .type_name = "sensor_msgs::msg::dds_::Imu_",
///                                         .on_message = [](const Imu &m) { use(m); }});
/// @endcode
template <RtpsMessage T> class Subscriber {
public:
  /// Called for each successfully deserialized sample.
  /// \note Runs on an engine worker thread - return quickly, do not block.
  using message_callback_t = std::function<void(const T &)>;

  /// Configuration for a typed subscriber.
  struct Config {
    std::string topic;     ///< DDS topic name.
    std::string type_name; ///< DDS type name (must match the peer for interop).
    RtpsParticipant::Reliability reliability{
        RtpsParticipant::Reliability::BEST_EFFORT}; ///< Reliability QoS.
    message_callback_t on_message{nullptr};         ///< Typed sample callback.
  };

  /// Construct and register a reader on the participant. The participant must
  /// already be started and must outlive this subscriber.
  /// \param participant The started participant to subscribe through.
  /// \param config The subscriber configuration.
  Subscriber(RtpsParticipant &participant, const Config &config)
      : on_message_(config.on_message) {
    valid_ = participant.add_reader({
        .topic = config.topic,
        .type_name = config.type_name,
        .reliability = config.reliability,
        .on_sample =
            [this](std::span<const uint8_t> cdr_payload) {
              if (!on_message_) {
                return;
              }
              auto sample = cdr::deserialize<T>(std::span<const std::byte>(
                  reinterpret_cast<const std::byte *>(cdr_payload.data()), cdr_payload.size()));
              if (sample) {
                on_message_(*sample);
              }
            },
    });
  }

  /// \return True if the reader was registered successfully.
  [[nodiscard]] bool is_valid() const { return valid_; }

private:
  message_callback_t on_message_;
  bool valid_{false};
};

/// @brief Helpers for ROS 2 name mangling, so typed interop is turnkey.
namespace ros2 {
/// Map a ROS 2 topic (e.g. "chatter") to its DDS topic name ("rt/chatter").
inline std::string topic_name(std::string_view ros_topic) {
  std::string out = "rt/";
  out += ros_topic;
  return out;
}
} // namespace ros2

} // namespace espp
