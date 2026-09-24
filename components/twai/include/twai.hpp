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
#include "freertos/semphr.h"

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
    ///       array, and the driver reads both the frame and the buffer from the
    ///       TX ISR after \c twai_node_transmit() queues them -- so both must
    ///       stay valid until the on_tx_done callback (transmit() guarantees
    ///       this by using member storage and waiting for completion).
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
    int tx_gpio{-1};           ///< GPIO number for TWAI TX. Must be set (validated).
    int rx_gpio{-1};           ///< GPIO number for TWAI RX. Must be set (validated).
    uint32_t baudrate{500000}; ///< Bus baud rate / bit rate in bits/second (e.g. 500000).
    Mode mode{Mode::NORMAL};   ///< Operating mode of the node.
    size_t tx_queue_depth{5};  ///< Depth of the hardware transmit queue.
    /** Hardware retransmission limit for a frame that fails (no ACK, bit error,
     *  arbitration lost): 0 = single shot (the default: one attempt, then the
     *  frame is dropped and transmit() reports the failure -- the behaviour
     *  espp::Twai has always had), 1..15 = that many retries, -1 = retransmit
     *  until it succeeds (standard CAN behaviour; transmit() bounds the wait
     *  with its timeout). */
    int8_t tx_retry_count{0};
    /** What transmit() does with a frame the controller has not finished with
     *  when the completion wait times out (typically: tx_retry_count = -1 and
     *  nothing on the bus acknowledges). true (the default): the pending
     *  transmission is dropped with abort_pending(), so the controller stops
     *  retransmitting it and the next transmit() proceeds at once. false: the
     *  frame is left with the controller (it may still go out when the bus
     *  comes back); the next transmit() first flush()es -- waits, up to its
     *  own timeout, for that frame to finish -- and fails with timed_out while
     *  it is still pending. Call abort_pending() to drop it explicitly. */
    bool auto_abort_on_timeout{true};
    std::optional<Filter> filter{};          ///< Optional acceptance filter (default: accept all).
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
    if (config_.tx_retry_count < -1 || config_.tx_retry_count > 15) {
      logger_.error("tx_retry_count must be 0..15 or -1 (retransmit until acknowledged), got {}",
                    config_.tx_retry_count);
      ec = std::make_error_code(std::errc::invalid_argument);
      return false;
    }

    // create the driver node (with its callbacks and filter); it is disabled
    if (!create_node(ec)) {
      return false;
    }

    // create the internal ISR->task event queue (it, the task and the
    // semaphore below survive a failed abort_pending() re-creation, in which
    // case only the node is missing here)
    if (!queue_) {
      queue_ = xQueueCreate(config_.rx_queue_size, sizeof(EventData));
      if (!queue_) {
        logger_.error("Failed to create event queue");
        ec = std::make_error_code(std::errc::not_enough_memory);
        twai_node_delete(node_);
        node_ = nullptr;
        return false;
      }
    }

    // create and start the receive task
    if (!task_) {
      task_ = espp::Task::make_unique({
          .callback = std::bind(&Twai::task_callback, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3),
          .task_config = config_.task_config,
          .log_level = config_.log_level,
      });
      task_->start();
    }

    // Transmit-completion semaphore (given from the on_tx_done ISR callback).
    // Created here, after every fallible setup step above has succeeded and
    // before the node is enabled: no earlier failure path (which delete node_
    // inline) can leak it, and no TX -- hence no on_tx_done -- can fire before
    // it exists, since transmitting requires an enabled node. On failure,
    // teardown() (below / in the destructor) releases task, queue and node.
    if (!tx_done_sem_) {
      tx_done_sem_ = xSemaphoreCreateBinary();
      if (!tx_done_sem_) {
        logger_.error("Failed to create TX-done semaphore");
        ec = std::make_error_code(std::errc::not_enough_memory);
        lock.unlock();
        teardown();
        return false;
      }
    }

    // enable the node if requested
    if (config_.auto_start) {
      const esp_err_t err = twai_node_enable(node_);
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

  /// \brief Wait until the controller has no pending transmission: its TX
  ///        queue is empty and the frame in progress (if any) has completed or
  ///        failed.
  /// \details transmit() already waits for its own frame, so normally nothing
  ///          is pending once it returns. What can be pending is a frame whose
  ///          transmit() timed out with Config::auto_abort_on_timeout off (the
  ///          controller keeps retransmitting it, tx_retry_count = -1, until
  ///          something acknowledges); flush() is how a caller waits for such a
  ///          frame to go out -- e.g. before a stop that must not be followed
  ///          by anything older. Note the driver offers no way to *drop* a
  ///          queued frame short of re-creating the node, which is what
  ///          abort_pending() does.
  /// \param ec The error code, set if the wait failed: \c operation_not_permitted
  ///        (node not initialized, or not enabled while a frame is pending, or
  ///        bus-off), \c timed_out (still pending after \p timeout_ms).
  /// \param timeout_ms Max time (ms) to wait, -1 = forever.
  /// \return True once nothing is pending, false otherwise.
  bool flush(std::error_code &ec, int timeout_ms = DEFAULT_TX_TIMEOUT_MS) {
    ec.clear();
    twai_node_handle_t node = nullptr;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (!node_) {
        ec = std::make_error_code(std::errc::operation_not_permitted);
        return false;
      }
      if (!enabled_) {
        // a disabled node transmits nothing: a pending frame can only go out
        // once start() is called, or be dropped with abort_pending()
        if (tx_pending_.load()) {
          logger_.error("Cannot flush: node not enabled while a frame is pending");
          ec = std::make_error_code(std::errc::operation_not_permitted);
          return false;
        }
        return true;
      }
      node = node_;
    }
    return flush_node(node, ec, timeout_ms);
  }

  /// \brief Drop every pending transmission: the frame in progress and any
  ///        queued behind it.
  /// \details The driver has no abort: disabling the node only pauses the
  ///          transmission in progress and re-enabling resumes it. The only
  ///          way to make the controller forget a frame is to delete the node
  ///          and create it again, which is what this does (same
  ///          configuration, callbacks and filter; the node comes back enabled
  ///          if it was). The receive task and event queue are untouched, so
  ///          nothing already received is lost; frames arriving during the few
  ///          hundred microseconds the node is gone are. Waits for a
  ///          transmit() in progress on another task to finish (or time out)
  ///          first, so it never pulls the node from under one.
  /// \param ec The error code, set if the node could not be re-created (it is
  ///        then gone: transmit() fails with \c operation_not_permitted until
  ///        initialize() is called again, which re-creates just the node) or
  ///        re-enabled.
  /// \return True if nothing is pending any more, false otherwise.
  bool abort_pending(std::error_code &ec) {
    ec.clear();
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
    return abort_pending_locked(ec);
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

  /// \brief Transmit a CAN message and wait for it to complete.
  /// \details \c twai_node_transmit() only QUEUES the transmission: the driver
  ///          keeps the passed \c twai_frame_t pointer (and its data buffer)
  ///          and formats the frame later, in the TX ISR. The descriptor must
  ///          therefore stay valid until the on_tx_done callback fires -- a
  ///          stack-local frame is read after the caller's stack is gone,
  ///          transmitting garbage (or tripping the driver's DLC assert). This
  ///          method copies the message into member storage that outlives the
  ///          call, serializes transmitters, and blocks until the driver
  ///          reports transmission complete.
  /// \param message The message to transmit.
  /// \param ec The error code, set if the message was not transmitted:
  ///        \c operation_not_permitted (node not initialized / enabled),
  ///        \c invalid_argument (DLC > 8), \c timed_out (the frame could not be
  ///        queued, or it was not acknowledged within the timeout while the
  ///        controller kept retransmitting -- Config::tx_retry_count = -1 --,
  ///        or an earlier frame left pending by such a timeout, with
  ///        Config::auto_abort_on_timeout off, is still pending), or
  ///        \c io_error (the controller gave up on the frame: the retries of a
  ///        bounded Config::tx_retry_count were exhausted, a bit error, or
  ///        arbitration lost; Config::on_error carries the reason). On a
  ///        completion timeout the pending frame is dropped
  ///        (Config::auto_abort_on_timeout, the default) or left with the
  ///        controller; see flush() / abort_pending().
  /// \param timeout_ms Max time (ms) to wait to queue the frame (-1 = forever
  ///        for the queueing step). The subsequent wait for transmit
  ///        completion is always bounded (by this value when >= 0, else by
  ///        DEFAULT_TX_TIMEOUT_MS) so an unacknowledged frame cannot hang the
  ///        caller.
  /// \return True if the message was transmitted (in Mode::NORMAL: acknowledged
  ///         by another node), false otherwise.
  bool transmit(const Message &message, std::error_code &ec,
                int timeout_ms = DEFAULT_TX_TIMEOUT_MS) {
    ec.clear();
    // Serialize the whole transmit -- node access, the driver call, and the
    // completion wait -- under tx_mutex_. The driver keeps a pointer to
    // tx_frame_ / tx_message_.data until on_tx_done, so only one frame may be
    // in flight, and teardown() takes tx_mutex_ before deleting the node /
    // semaphore, so neither can be freed while a transmit is using them. Note
    // tx_mutex_ is distinct from mutex_ (which guards node_/enabled_ and is
    // held only briefly below), so a congested TX queue does not stall
    // is_enabled() / get_status() / stop().
    std::lock_guard<std::mutex> tx_lock(tx_mutex_);
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
    // A frame left pending by an earlier completion timeout (with
    // auto_abort_on_timeout off) still owns tx_frame_ / tx_message_: the driver
    // reads them from the TX ISR when it finally sends it, so they cannot be
    // overwritten until it is done. Wait for it, bounded like the completion
    // wait below.
    if (tx_pending_.load()) {
      const int pending_timeout_ms = timeout_ms < 0 ? DEFAULT_TX_TIMEOUT_MS : timeout_ms;
      if (!flush_node(node, ec, pending_timeout_ms)) {
        logger_.error(
            "Cannot transmit: an earlier frame is still pending (abort_pending() drops it)");
        return false;
      }
    }
    // drain a stale completion (e.g. from a prior transmit whose frame we
    // aborted below after a timeout) so the wait sees only our own
    xSemaphoreTake(tx_done_sem_, 0);
    tx_success_.store(false);
    tx_message_ = message;
    tx_frame_ = tx_message_.to_twai_frame();
    esp_err_t err = twai_node_transmit(node, &tx_frame_, timeout_ms);
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
    // Wait for the on_tx_done callback; on a healthy bus a classic frame
    // completes in well under a millisecond, but an unacknowledged frame is
    // retransmitted indefinitely, so ALWAYS bound the completion wait -- even
    // when timeout_ms < 0 (which means "wait forever to queue", above): an
    // unbounded completion wait would hang the caller on a bus with no ACK.
    const TickType_t wait_ticks =
        timeout_ms < 0 ? pdMS_TO_TICKS(DEFAULT_TX_TIMEOUT_MS) : pdMS_TO_TICKS(timeout_ms);
    if (xSemaphoreTake(tx_done_sem_, wait_ticks) != pdTRUE) {
      // The frame was queued but did not complete (e.g. nothing ACKed it, so
      // the controller keeps retransmitting). The driver still references
      // tx_frame_ / tx_message_.data from its TX ISR. Either drop the pending
      // transmission now (the only way is to re-create the node: disabling it
      // merely pauses the transmission and re-enabling resumes it), or leave
      // the frame with the controller and remember that the storage is taken
      // until it goes out, which the next transmit() (or flush()) waits for.
      if (config_.auto_abort_on_timeout) {
        std::error_code abort_ec;
        if (abort_pending_locked(abort_ec)) {
          logger_.error("Timed out waiting for transmit completion (no ACK on the bus?); "
                        "the pending frame was dropped");
        } else {
          logger_.error("Timed out waiting for transmit completion (no ACK on the bus?), and "
                        "dropping the pending frame failed: {}",
                        abort_ec.message());
        }
      } else {
        tx_pending_.store(true);
        logger_.error("Timed out waiting for transmit completion (no ACK on the bus?); the frame "
                      "stays with the controller (flush() waits for it, abort_pending() drops it)");
      }
      ec = std::make_error_code(std::errc::timed_out);
      return false;
    }
    // The controller finished with the frame, but not necessarily by sending it:
    // with a bounded tx_retry_count it gives up after the retries (no ACK, bit
    // error, arbitration lost -- see on_error) and reports the failure here.
    if (!tx_success_.load()) {
      logger_.error("Transmit failed: the frame was not acknowledged / could not be sent "
                    "(check the transceiver, bit rate and that another node is on the bus)");
      ec = std::make_error_code(std::errc::io_error);
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

  /// \brief Create the driver node from config_ (disabled), register the ISR
  ///        callbacks and apply the acceptance filter. On failure node_ is left
  ///        null and ec set. Requires mutex_ (recursive) held or single-threaded
  ///        use (initialize()).
  bool create_node(std::error_code &ec) {
    twai_onchip_node_config_t node_cfg = {};
    node_cfg.io_cfg.tx = static_cast<gpio_num_t>(config_.tx_gpio);
    node_cfg.io_cfg.rx = static_cast<gpio_num_t>(config_.rx_gpio);
    node_cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;
    node_cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    node_cfg.bit_timing.bitrate = config_.baudrate;
    node_cfg.tx_queue_depth = config_.tx_queue_depth;
    // NOTE: the driver treats every value but -1 as a bounded retry count with
    // the controller's single-shot bit set; a frame that exhausts it arrives in
    // on_tx_done with is_tx_success == false, which transmit() reports.
    node_cfg.fail_retry_cnt = config_.tx_retry_count;
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
    cbs.on_tx_done = &Twai::on_tx_done_cb;
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
    return true;
  }

  /// \brief The wait behind flush(): block until the driver reports the node
  ///        idle with an empty TX queue. Clears tx_pending_ on success.
  bool flush_node(twai_node_handle_t node, std::error_code &ec, int timeout_ms) {
    esp_err_t err = twai_node_transmit_wait_all_done(node, timeout_ms);
    if (err == ESP_OK) {
      tx_pending_.store(false);
      return true;
    }
    if (err == ESP_ERR_TIMEOUT) {
      logger_.error("Timed out waiting for the pending transmission(s) to finish");
      ec = std::make_error_code(std::errc::timed_out);
    } else if (err == ESP_ERR_INVALID_STATE) {
      logger_.error("Cannot wait for pending transmissions: node is bus-off / disabled");
      ec = std::make_error_code(std::errc::operation_not_permitted);
    } else {
      logger_.error("Waiting for pending transmissions failed: {}", esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
    }
    return false;
  }

  /// \brief abort_pending() with tx_mutex_ already held (transmit()'s timeout
  ///        path calls it from inside its own critical section).
  bool abort_pending_locked(std::error_code &ec) {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!node_) {
      ec = std::make_error_code(std::errc::operation_not_permitted);
      return false;
    }
    const bool was_enabled = enabled_;
    if (enabled_) {
      // ESP_ERR_INVALID_STATE here means the node is already stopped (bus-off),
      // which is exactly the state twai_node_delete() needs
      esp_err_t err = twai_node_disable(node_);
      if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        logger_.error("Failed to disable TWAI node to drop its pending frame: {}",
                      esp_err_to_name(err));
        ec = std::make_error_code(std::errc::io_error);
        return false;
      }
      enabled_ = false;
    }
    esp_err_t err = twai_node_delete(node_);
    node_ = nullptr;
    if (err != ESP_OK) {
      logger_.error("Failed to delete TWAI node to drop its pending frame: {}",
                    esp_err_to_name(err));
      ec = std::make_error_code(std::errc::io_error);
      return false;
    }
    // the deleted node's ISR is gone: nothing references tx_frame_ any more
    tx_pending_.store(false);
    if (!create_node(ec)) {
      logger_.error("TWAI node could not be re-created after dropping its pending frame; it is "
                    "gone until initialize() is called again");
      return false;
    }
    if (was_enabled) {
      err = twai_node_enable(node_);
      if (err != ESP_OK) {
        logger_.error("Failed to re-enable TWAI node after dropping its pending frame: {}",
                      esp_err_to_name(err));
        ec = std::make_error_code(std::errc::io_error);
        return false;
      }
      enabled_ = true;
    }
    // absorb a completion that may have raced in just before the node went
    xSemaphoreTake(tx_done_sem_, 0);
    return true;
  }

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
    // Disable the node first (best-effort, under mutex_). twai_node_delete()
    // requires a disabled node anyway, and disabling also aborts / unblocks any
    // in-flight twai_node_transmit() so a concurrent transmit() blocked on a
    // full queue (timeout_ms < 0) can return and release tx_mutex_ -- otherwise
    // acquiring tx_mutex_ below could deadlock against it. Done in its own
    // scope so mutex_ is released before we take tx_mutex_ then mutex_ (keeping
    // the tx_mutex_ -> mutex_ order that transmit() uses).
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (node_ && enabled_) {
        esp_err_t err = twai_node_disable(node_);
        if (err != ESP_OK)
          logger_.warn("Could not disable TWAI node before delete: {}", esp_err_to_name(err));
        enabled_ = false;
      }
    }
    // Delete the node, queue and TX-done semaphore under tx_mutex_ so a
    // transmit() that started before teardown -- which holds tx_mutex_ across
    // the (now always bounded) driver call and completion wait, and references
    // node_ and tx_done_sem_ -- has finished before we free them. transmit()
    // takes tx_mutex_ then mutex_, so acquire them in the same order here.
    {
      std::lock_guard<std::mutex> tx_lock(tx_mutex_);
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (node_) {
        twai_node_delete(node_);
        node_ = nullptr;
      }
      if (queue_) {
        vQueueDelete(queue_);
        queue_ = nullptr;
      }
      if (tx_done_sem_) {
        vSemaphoreDelete(tx_done_sem_);
        tx_done_sem_ = nullptr;
      }
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

  static bool on_tx_done_cb(twai_node_handle_t handle, const twai_tx_done_event_data_t *edata,
                            void *user_ctx) {
    (void)handle;
    auto *self = static_cast<Twai *>(user_ctx);
    self->tx_success_.store(edata && edata->is_tx_success);
    // Guard against a lifecycle race: the semaphore may not exist yet (a TX
    // completing during a partial init) or may already be freed (teardown), so
    // never dereference a null handle from ISR context.
    if (!self->tx_done_sem_) {
      return false;
    }
    BaseType_t higher_priority_task_woken = pdFALSE;
    xSemaphoreGiveFromISR(self->tx_done_sem_, &higher_priority_task_woken);
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

  // Transmit path: the driver holds a pointer to tx_frame_ (whose buffer
  // points into tx_message_.data) from twai_node_transmit() until the
  // on_tx_done ISR callback, so both live here rather than on the stack;
  // tx_mutex_ keeps a single frame in flight and tx_done_sem_ signals
  // completion from the ISR.
  std::mutex tx_mutex_;
  SemaphoreHandle_t tx_done_sem_{nullptr};
  std::atomic<bool> tx_success_{false}; // is_tx_success of the last on_tx_done
  // a frame left with the controller by a completion timeout (with
  // auto_abort_on_timeout off) still owns tx_frame_ / tx_message_
  std::atomic<bool> tx_pending_{false};
  Message tx_message_{};
  twai_frame_t tx_frame_{};
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
