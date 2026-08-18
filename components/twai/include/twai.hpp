#pragma once

#include <array>
#include <atomic>
#include <cstring>
#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>

#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_twai_types.h"

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#include "base_component.hpp"
#include "format.hpp"
#include "task.hpp"

namespace espp {
/// \brief A class to interface with the ESP TWAI (CAN 2.0) peripheral.
/// \details This class wraps the modern node-based ESP-IDF TWAI driver
///          (\c esp_driver_twai) in an idiomatic C++ interface. It creates an
///          on-chip TWAI node, registers ISR event callbacks, and marshals
///          received frames (and optional error / state-change events) from ISR
///          context into a task-context callback using an internal FreeRTOS
///          queue and an \c espp::Task.
///
///          All user callbacks (on_receive, on_error, on_state_change) are
///          invoked from the internal task context - never from the ISR - so
///          they may safely call blocking / non-IRAM-safe APIs. No lock is held
///          while a user callback is invoked.
///
///          The class supports the classic CAN 2.0 frame format (up to 8 data
///          bytes). CAN-FD (up to 64 data bytes, bit-rate switching) is a
///          possible future extension; the underlying driver and frame types
///          support it, but this wrapper intentionally keeps to classic CAN for
///          a small, clean surface.
///
/// \section twai_ex0 TWAI (loopback) Example
/// \snippet twai_example.cpp twai example
class Twai : public BaseComponent {
public:
  /// \brief Maximum number of data bytes in a classic CAN 2.0 frame.
  static constexpr size_t MAX_DATA_LEN = 8;

  /// \brief Default timeout (ms) used when transmitting a frame.
  static constexpr int DEFAULT_TX_TIMEOUT_MS = 100;

  /// \brief The operating mode of the TWAI node.
  enum class Mode {
    NORMAL,      ///< Normal mode: transmit, receive, and acknowledge frames on the bus. Requires a
                 ///< transceiver and at least one other acknowledging node.
    LISTEN_ONLY, ///< Listen-only mode: the node only monitors the bus and never transmits or
                 ///< acknowledges. Useful for passive bus monitoring / sniffing.
    LOOPBACK,    ///< Loopback self-test mode: the controller receives back the frames it transmits
                 ///< and does not require acknowledgement. This lets the node run with no
                 ///< transceiver and no other node on the bus (used by the example).
  };

  /// \brief A classic CAN 2.0 message / frame.
  struct Message {
    uint32_t id{0};       ///< Arbitration ID (11-bit standard, or 29-bit if \c extended is true).
    bool extended{false}; ///< True for an extended (29-bit) ID, false for standard (11-bit) ID.
    bool rtr{false};      ///< True if this is a Remote Transmission Request frame (no data).
    uint8_t dlc{0};       ///< Data Length Code / number of valid data bytes (0-8).
    std::array<uint8_t, MAX_DATA_LEN>
        data{}; ///< Frame payload (only the first \c dlc bytes valid).

    /// \brief Convert this Message into a driver \c twai_frame_t for transmission.
    /// \note The returned frame's \c buffer points into this Message's \c data
    ///       array, so the Message must outlive the returned frame (which it
    ///       does for the duration of a synchronous transmit call).
    /// \return A \c twai_frame_t describing this message.
    twai_frame_t to_twai_frame() const {
      // classic CAN carries at most MAX_DATA_LEN (8) data bytes; clamp defensively
      // so buffer_len never exceeds the backing data array. transmit() rejects an
      // out-of-range DLC up front, so this is only a safety net.
      const uint8_t len = dlc > MAX_DATA_LEN ? static_cast<uint8_t>(MAX_DATA_LEN) : dlc;
      twai_frame_t frame = {};
      frame.header.id = id;
      frame.header.ide = extended ? 1 : 0;
      frame.header.rtr = rtr ? 1 : 0;
      frame.header.fdf = 0; // classic CAN, not FD
      frame.header.dlc = len;
      frame.buffer = const_cast<uint8_t *>(data.data());
      frame.buffer_len = len;
      return frame;
    }

    /// \brief Build a Message from a driver \c twai_frame_t.
    /// \param frame The driver frame (as returned by \c twai_node_receive_from_isr).
    /// \return A Message copy of the frame's header + data.
    static Message from_twai_frame(const twai_frame_t &frame) {
      Message msg;
      msg.id = frame.header.id;
      msg.extended = frame.header.ide;
      msg.rtr = frame.header.rtr;
      // This wrapper is classic CAN only. A CAN-FD frame (fdf set) or a decoded
      // length above MAX_DATA_LEN (8) would let a Message claim a DLC larger than
      // the bytes actually copied, so normalize the length to what we can hold and
      // set msg.dlc to that copied length.
      size_t len = frame.header.fdf ? MAX_DATA_LEN : twaifd_dlc2len(frame.header.dlc);
      if (len > MAX_DATA_LEN) {
        len = MAX_DATA_LEN;
      }
      // never copy more than the driver says it filled (a header/buffer
      // mismatch would otherwise read uninitialized tail bytes)
      if (len > frame.buffer_len) {
        len = frame.buffer_len;
      }
      msg.dlc = static_cast<uint8_t>(len);
      if (frame.buffer && !frame.header.rtr) {
        memcpy(msg.data.data(), frame.buffer, len);
      }
      return msg;
    }
  };

  /// \brief An acceptance (hardware) filter for received frames.
  /// \details A frame is accepted if <tt>(received_id & mask) == (id & mask)</tt>.
  ///          A mask bit of 1 means the corresponding ID bit must match; a mask
  ///          bit of 0 means "don't care". Therefore \c id = 0, \c mask = 0
  ///          accepts all frames.
  struct Filter {
    uint32_t id{0};       ///< The base ID to match.
    uint32_t mask{0};     ///< The mask (1 = bit must match, 0 = don't care).
    bool extended{false}; ///< True to filter on extended (29-bit) IDs, false for standard (11-bit).
    bool dual{false};     ///< Configure the filter as a dual 16-bit filter (advanced).
  };

  /// \brief Data passed to the state-change callback.
  struct StateChange {
    twai_error_state_t old_state; ///< The previous error state.
    twai_error_state_t new_state; ///< The new error state.
  };

  typedef std::function<void(const Message &)> receive_callback_fn; ///< Receive callback type.
  typedef std::function<void(const StateChange &)>
      state_change_callback_fn; ///< State-change callback type.
  typedef std::function<void(twai_error_flags_t)> error_callback_fn; ///< Error callback type.

  /// \brief Configuration for the TWAI node.
  struct Config {
    int tx_gpio{-1};                ///< GPIO number for TWAI TX. Must be set (validated).
    int rx_gpio{-1};                ///< GPIO number for TWAI RX. Must be set (validated).
    uint32_t baudrate{500000};      ///< Bus baud rate / bit rate in bits/second (e.g. 500000).
    Mode mode{Mode::NORMAL};        ///< Operating mode of the node.
    size_t tx_queue_depth{5};       ///< Depth of the hardware transmit queue.
    std::optional<Filter> filter{}; ///< Optional acceptance filter (default: accept all).
    receive_callback_fn on_receive{nullptr}; ///< Called (in task context) for each received frame.
    error_callback_fn on_error{nullptr};     ///< Optional: called (in task context) on a bus error.
    state_change_callback_fn on_state_change{
        nullptr};             ///< Optional: called (in task context) on an error-state change.
    bool auto_start{true};    ///< If true, the node is enabled at the end of initialize().
    size_t rx_queue_size{16}; ///< Size (number of events) of the internal ISR->task event queue.
    Task::BaseConfig task_config{
        .name = "Twai Task",
        .stack_size_bytes = 4096,
        .priority = 10,
    }; ///< Configuration for the internal receive task.
    espp::Logger::Verbosity log_level{espp::Logger::Verbosity::WARN}; ///< Log verbosity.
  };

  /// \brief Construct a new Twai object.
  /// \param config The configuration for the TWAI node.
  /// \note This does not touch the hardware; call initialize() to create the
  ///       node and start the receive task.
  explicit Twai(const Config &config)
      : BaseComponent("Twai", config.log_level)
      , config_(config) {}

  /// \brief Destructor. Stops the receive task and disables + deletes the node.
  ~Twai() {
    std::error_code ec;
    stop(ec); // disable the node if it is enabled (ignore errors)
    teardown();
  }

  Twai(const Twai &) = delete;
  Twai &operator=(const Twai &) = delete;

  /// \brief Initialize the TWAI node.
  /// \details Creates the on-chip node, registers ISR event callbacks, applies
  ///          the optional acceptance filter, creates the internal event queue
  ///          and receive task, and (if Config::auto_start is true) enables the
  ///          node.
  /// \param ec The error code, set if initialization fails.
  /// \return True on success, false on failure.
  bool initialize(std::error_code &ec) {
    ec.clear();
    std::unique_lock<std::recursive_mutex> lock(mutex_);
    if (node_) {
      logger_.warn("Already initialized");
      return true;
    }
    if (config_.tx_gpio < 0 || config_.rx_gpio < 0) {
      logger_.error("tx_gpio and rx_gpio must be set (tx={}, rx={})", config_.tx_gpio,
                    config_.rx_gpio);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // build the node configuration
    twai_onchip_node_config_t node_cfg = {};
    node_cfg.io_cfg.tx = static_cast<gpio_num_t>(config_.tx_gpio);
    node_cfg.io_cfg.rx = static_cast<gpio_num_t>(config_.rx_gpio);
    node_cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    node_cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    node_cfg.bit_timing.bitrate = config_.baudrate;
    node_cfg.tx_queue_depth = config_.tx_queue_depth;
    switch (config_.mode) {
    case Mode::NORMAL:
      break;
    case Mode::LISTEN_ONLY:
      node_cfg.flags.enable_listen_only = 1;
      break;
    case Mode::LOOPBACK:
      // internal loopback + self-test: receive our own frames and don't require
      // an acknowledgement, so we can run with no transceiver / no other node.
      node_cfg.flags.enable_loopback = 1;
      node_cfg.flags.enable_self_test = 1;
      break;
    }

    esp_err_t err = twai_new_node_onchip(&node_cfg, &node_);
    if (err != ESP_OK) {
      logger_.error("Failed to create TWAI node: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      node_ = nullptr;
      return false;
    }

    // register the ISR event callbacks
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = &Twai::on_rx_done_cb;
    if (config_.on_state_change) {
      cbs.on_state_change = &Twai::on_state_change_cb;
    }
    if (config_.on_error) {
      cbs.on_error = &Twai::on_error_cb;
    }
    err = twai_node_register_event_callbacks(node_, &cbs, this);
    if (err != ESP_OK) {
      logger_.error("Failed to register event callbacks: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      twai_node_delete(node_);
      node_ = nullptr;
      return false;
    }

    // apply the acceptance filter if provided (node must be disabled - it is,
    // since a freshly created node is disabled)
    if (config_.filter.has_value()) {
      const auto &f = config_.filter.value();
      twai_mask_filter_config_t mask_cfg = {};
      mask_cfg.id = f.id;
      mask_cfg.mask = f.mask;
      mask_cfg.is_ext = f.extended ? 1 : 0;
      mask_cfg.dual_filter = f.dual ? 1 : 0;
      err = twai_node_config_mask_filter(node_, 0, &mask_cfg);
      if (err != ESP_OK) {
        logger_.error("Failed to configure mask filter: {}", esp_err_to_name(err));
        ec = std::make_error_code(std::errc::invalid_argument);
        twai_node_delete(node_);
        node_ = nullptr;
        return false;
      }
    }

    // create the internal ISR->task event queue
    queue_ = xQueueCreate(config_.rx_queue_size, sizeof(EventData));
    if (!queue_) {
      logger_.error("Failed to create event queue");
      ec = std::make_error_code(std::errc::not_enough_memory);
      twai_node_delete(node_);
      node_ = nullptr;
      return false;
    }

    // create and start the receive task
    task_ = espp::Task::make_unique({
        .callback = std::bind(&Twai::task_callback, this, std::placeholders::_1,
                              std::placeholders::_2, std::placeholders::_3),
        .task_config = config_.task_config,
        .log_level = config_.log_level,
    });
    task_->start();

    // enable the node if requested
    if (config_.auto_start) {
      err = twai_node_enable(node_);
      if (err != ESP_OK) {
        logger_.error("Failed to enable TWAI node: {}", esp_err_to_name(err));
        ec = std::make_error_code(std::errc::io_error);
        // tear down every partially-created resource (task, queue, node) so a
        // failed initialize() leaves no dangling handles and does not report
        // "Already initialized" on a subsequent call. Release the lock first:
        // teardown() joins the receive task (like the destructor, which also
        // calls it unlocked), and joining while holding the mutex could
        // deadlock if the task were in a user callback taking a locking method.
        lock.unlock();
        teardown();
        return false;
      }
      enabled_ = true;
    }

    logger_.info("Initialized TWAI node (tx={}, rx={}, baud={}, mode={})", config_.tx_gpio,
                 config_.rx_gpio, config_.baudrate, static_cast<int>(config_.mode));
    return true;
  }

  /// \brief Enable (start) the TWAI node so it participates on the bus.
  /// \param ec The error code, set if the node could not be enabled.
  /// \return True on success, false otherwise.
  bool start(std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!node_) {
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    if (enabled_) {
      return true;
    }
    esp_err_t err = twai_node_enable(node_);
    if (err != ESP_OK) {
      logger_.error("Failed to enable TWAI node: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    enabled_ = true;
    return true;
  }

  /// \brief Disable (stop) the TWAI node.
  /// \param ec The error code, set if the node could not be disabled.
  /// \return True on success, false otherwise.
  bool stop(std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!node_ || !enabled_) {
      return true;
    }
    esp_err_t err = twai_node_disable(node_);
    if (err != ESP_OK) {
      logger_.error("Failed to disable TWAI node: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    enabled_ = false;
    return true;
  }

  /// \brief Transmit a CAN message.
  /// \param message The message to transmit.
  /// \param ec The error code, set if the transmission could not be queued.
  /// \param timeout_ms Max time (ms) to wait if the TX queue is full (-1 = forever).
  /// \return True if the message was queued for transmission, false otherwise.
  bool transmit(const Message &message, std::error_code &ec,
                int timeout_ms = DEFAULT_TX_TIMEOUT_MS) {
    ec.clear();
    // Snapshot the handle under the lock, then call the (potentially blocking,
    // up to timeout_ms) driver transmit with the lock RELEASED so a congested
    // TX queue cannot stall is_enabled()/get_status()/stop()/the destructor.
    // The driver's transmit path is internally thread-safe.
    twai_node_handle_t node = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!node_) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
      }
      if (!enabled_) {
        logger_.error("Cannot transmit: node not enabled");
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
      }
      // classic CAN 2.0 carries at most MAX_DATA_LEN (8) data bytes; reject anything larger
      if (message.dlc > MAX_DATA_LEN) {
        logger_.error("Cannot transmit: DLC {} exceeds classic CAN max ({})", message.dlc,
                      MAX_DATA_LEN);
        ec = std::make_error_code(std::errc::invalid_argument);
        return false;
      }
      node = node_;
    }
    twai_frame_t frame = message.to_twai_frame();
    esp_err_t err = twai_node_transmit(node, &frame, timeout_ms);
    if (err != ESP_OK) {
      logger_.error("Failed to transmit frame: {}", esp_err_to_name(err));
      if (err == ESP_ERR_TIMEOUT) {
        ec = std::make_error_code(std::errc::timed_out);
      } else if (err == ESP_ERR_INVALID_STATE) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
      } else if (err == ESP_ERR_NOT_SUPPORTED) {
        ec = std::make_error_code(std::errc::operation_not_supported);
      } else {
        ec = std::make_error_code(std::errc::io_error);
      }
      return false;
    }
    return true;
  }

  /// \brief Begin bus-off recovery.
  /// \details Starts the recovery process for a node in the bus-off state. Use
  ///          the on_state_change callback or get_status() to know when recovery
  ///          has finished.
  /// \param ec The error code, set if recovery could not be started.
  /// \return True if recovery was started, false otherwise.
  bool recover(std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!node_) {
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    esp_err_t err = twai_node_recover(node_);
    if (err != ESP_OK) {
      logger_.error("Failed to start bus-off recovery: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return true;
  }

  /// \brief Get the current node status (error state, error counters, TX queue space).
  /// \param status The status structure to fill in.
  /// \param ec The error code, set on failure.
  /// \return True on success, false otherwise.
  bool get_status(twai_node_status_t &status, std::error_code &ec) {
    twai_node_record_t record;
    return get_info(status, record, ec);
  }

  /// \brief Get the node statistics (cumulative bus error count).
  /// \param record The statistics structure to fill in.
  /// \param ec The error code, set on failure.
  /// \return True on success, false otherwise.
  bool get_statistics(twai_node_record_t &record, std::error_code &ec) {
    twai_node_status_t status;
    return get_info(status, record, ec);
  }

  /// \brief Get both the node status and statistics.
  /// \param status The status structure to fill in.
  /// \param record The statistics structure to fill in.
  /// \param ec The error code, set on failure.
  /// \return True on success, false otherwise.
  bool get_info(twai_node_status_t &status, twai_node_record_t &record, std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!node_) {
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    esp_err_t err = twai_node_get_info(node_, &status, &record);
    if (err != ESP_OK) {
      logger_.error("Failed to get node info: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    return true;
  }

  /// \brief Whether the node is currently enabled (started).
  /// \return True if the node is enabled, false otherwise.
  bool is_enabled() const {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    return enabled_;
  }

protected:
  enum class EventType { RX, ERROR, STATE_CHANGE, STOP };

  struct EventData {
    EventType type;
    Message message;              ///< Valid for EventType::RX
    twai_error_flags_t err_flags; ///< Valid for EventType::ERROR
    twai_error_state_t old_state; ///< Valid for EventType::STATE_CHANGE
    twai_error_state_t new_state; ///< Valid for EventType::STATE_CHANGE
  };

  /// \brief Tear down every internal resource (receive task, event queue, node)
  ///        and clear the handles. Safe to call with any subset created, so it is
  ///        used both by the destructor and by initialize()'s failure paths.
  /// \note Does not disable the node; callers that may have enabled it should
  ///       call stop() first.
  void teardown() {
    // stop and delete the receive task
    if (task_) {
      if (queue_) {
        EventData stop_event{};
        stop_event.type = EventType::STOP;
        xQueueSend(queue_, &stop_event, 0);
      }
      task_->stop();
      task_.reset();
    }
    // delete the node
    if (node_) {
      // twai_node_delete() requires the node to be disabled first. If a prior
      // stop() failed (or was never called) and the node is still enabled,
      // deleting it would fail and leak the driver resource -- so make a
      // best-effort disable here before deleting.
      if (enabled_) {
        esp_err_t err = twai_node_disable(node_);
        if (err != ESP_OK)
          logger_.warn("Could not disable TWAI node before delete: {}", esp_err_to_name(err));
        enabled_ = false;
      }
      twai_node_delete(node_);
      node_ = nullptr;
    }
    // delete the queue
    if (queue_) {
      vQueueDelete(queue_);
      queue_ = nullptr;
    }
    enabled_ = false;
  }

  // ---- ISR callbacks (run in ISR context; only marshal to the queue) ----

  static bool on_rx_done_cb(twai_node_handle_t handle, const twai_rx_done_event_data_t *edata,
                            void *user_ctx) {
    (void)edata;
    auto *self = static_cast<Twai *>(user_ctx);
    BaseType_t higher_priority_task_woken = pdFALSE;
    uint8_t buffer[MAX_DATA_LEN];
    twai_frame_t rx_frame = {};
    rx_frame.buffer = buffer;
    rx_frame.buffer_len = sizeof(buffer);
    if (twai_node_receive_from_isr(handle, &rx_frame) == ESP_OK) {
      EventData event{};
      event.type = EventType::RX;
      event.message = Message::from_twai_frame(rx_frame);
      xQueueSendFromISR(self->queue_, &event, &higher_priority_task_woken);
    }
    return higher_priority_task_woken == pdTRUE;
  }

  static bool on_state_change_cb(twai_node_handle_t handle,
                                 const twai_state_change_event_data_t *edata, void *user_ctx) {
    (void)handle;
    auto *self = static_cast<Twai *>(user_ctx);
    BaseType_t higher_priority_task_woken = pdFALSE;
    EventData event{};
    event.type = EventType::STATE_CHANGE;
    event.old_state = edata->old_sta;
    event.new_state = edata->new_sta;
    xQueueSendFromISR(self->queue_, &event, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
  }

  static bool on_error_cb(twai_node_handle_t handle, const twai_error_event_data_t *edata,
                          void *user_ctx) {
    (void)handle;
    auto *self = static_cast<Twai *>(user_ctx);
    BaseType_t higher_priority_task_woken = pdFALSE;
    EventData event{};
    event.type = EventType::ERROR;
    event.err_flags = edata->err_flags;
    xQueueSendFromISR(self->queue_, &event, &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
  }

  // ---- Task context: drain the queue and invoke user callbacks ----

  bool task_callback(std::mutex &, std::condition_variable &, bool &) {
    EventData event;
    // Use a bounded receive timeout so this callback periodically returns to the
    // espp::Task loop, which re-checks its running flag. That lets Task::stop()
    // exit even if the STOP event below could not be enqueued (e.g. the RX queue
    // was full) -- the STOP event is a fast-wake optimization, not the sole stop
    // mechanism, so teardown() can never deadlock waiting on the task.
    if (xQueueReceive(queue_, &event, pdMS_TO_TICKS(100))) {
      switch (event.type) {
      case EventType::STOP:
        return true; // stop the task
      case EventType::RX:
        logger_.debug("Received frame id=0x{:X} dlc={}", event.message.id, event.message.dlc);
        if (config_.on_receive) {
          config_.on_receive(event.message);
        }
        break;
      case EventType::ERROR:
        logger_.debug("Bus error, flags=0x{:X}", event.err_flags.val);
        if (config_.on_error) {
          config_.on_error(event.err_flags);
        }
        break;
      case EventType::STATE_CHANGE:
        logger_.debug("State change {} -> {}", static_cast<int>(event.old_state),
                      static_cast<int>(event.new_state));
        if (config_.on_state_change) {
          config_.on_state_change(StateChange{event.old_state, event.new_state});
        }
        break;
      }
    }
    return false; // keep running
  }

  Config config_;
  mutable std::recursive_mutex mutex_;
  twai_node_handle_t node_{nullptr};
  QueueHandle_t queue_{nullptr};
  std::unique_ptr<espp::Task> task_;
  bool enabled_{false};
};
} // namespace espp

// for printing an espp::Twai::Message using libfmt
template <> struct fmt::formatter<espp::Twai::Message> {
  constexpr auto parse(format_parse_context &ctx) const { return ctx.begin(); }
  template <typename FormatContext>
  auto format(const espp::Twai::Message &m, FormatContext &ctx) const {
    auto out = fmt::format_to(ctx.out(), "Message{{id=0x{:X}, {}, {}, dlc={}, data=[", m.id,
                              m.extended ? "ext" : "std", m.rtr ? "rtr" : "data", m.dlc);
    for (uint8_t i = 0; i < m.dlc && i < espp::Twai::MAX_DATA_LEN; ++i) {
      out = fmt::format_to(out, "{}0x{:02X}", i == 0 ? "" : " ", m.data[i]);
    }
    return fmt::format_to(out, "]}}");
  }
};
