#include <sdkconfig.h>

#if defined(CONFIG_ESPP_I2C_USE_NEW_API)

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <esp_attr.h>
#include <esp_err.h>

#include "i2c_slave.hpp"

namespace espp {
namespace {
std::error_code make_error_code(esp_err_t err) {
  switch (err) {
  case ESP_OK:
    return {};
  case ESP_ERR_INVALID_ARG:
    return std::make_error_code(std::errc::invalid_argument);
  case ESP_ERR_NO_MEM:
    return std::make_error_code(std::errc::not_enough_memory);
  case ESP_ERR_TIMEOUT:
    return std::make_error_code(std::errc::timed_out);
  case ESP_ERR_INVALID_STATE:
    return std::make_error_code(std::errc::operation_not_permitted);
  case ESP_ERR_NOT_FOUND:
    return std::make_error_code(std::errc::no_such_device_or_address);
  default:
    return std::make_error_code(std::errc::io_error);
  }
}
} // namespace

I2cSlaveDevice::I2cSlaveDevice(const Config &config)
    : BaseComponent("I2cSlaveDevice", config.log_level)
    , config_(config) {}

I2cSlaveDevice::~I2cSlaveDevice() {
  std::error_code ec;
  deinit(ec);
  if (ec) {
    logger_.error("failed to deinitialize slave device: {}", ec.message());
  }
}

size_t I2cSlaveDevice::message_buffer_capacity(size_t max_message_size, size_t max_messages) {
  size_t message_size = std::max<size_t>(1, max_message_size);
  size_t num_messages = std::max<size_t>(1, max_messages);
  return num_messages * (message_size + sizeof(size_t));
}

TickType_t I2cSlaveDevice::timeout_ticks(int timeout_ms) {
  if (timeout_ms < 0) {
    return portMAX_DELAY;
  }
  return pdMS_TO_TICKS(timeout_ms);
}

bool I2cSlaveDevice::init(std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (initialized_) {
    ec = std::make_error_code(std::errc::already_connected);
    return false;
  }
  if (config_.sda_io_num < 0 || config_.scl_io_num < 0 || config_.receive_buffer_depth == 0 ||
      config_.send_buffer_depth == 0 || config_.event_queue_depth == 0) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  event_queue_ = xQueueCreate(config_.event_queue_depth, sizeof(Event));
  if (!event_queue_) {
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }

  read_buffer_ = xMessageBufferCreate(
      message_buffer_capacity(config_.receive_buffer_depth, config_.event_queue_depth));
  callback_buffer_ = xMessageBufferCreate(
      message_buffer_capacity(config_.receive_buffer_depth, config_.event_queue_depth));
  if (!read_buffer_ || !callback_buffer_) {
    if (read_buffer_) {
      vMessageBufferDelete(read_buffer_);
      read_buffer_ = nullptr;
    }
    if (callback_buffer_) {
      vMessageBufferDelete(callback_buffer_);
      callback_buffer_ = nullptr;
    }
    vQueueDelete(event_queue_);
    event_queue_ = nullptr;
    ec = std::make_error_code(std::errc::not_enough_memory);
    return false;
  }

  i2c_slave_config_t slave_cfg = {};
  slave_cfg.i2c_port = config_.port;
  slave_cfg.sda_io_num = static_cast<gpio_num_t>(config_.sda_io_num);
  slave_cfg.scl_io_num = static_cast<gpio_num_t>(config_.scl_io_num);
  slave_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  slave_cfg.send_buf_depth = config_.send_buffer_depth;
#if ESPP_I2C_SLAVE_V2_API
  slave_cfg.receive_buf_depth = config_.receive_buffer_depth;
#endif
  slave_cfg.slave_addr = config_.slave_address;
  slave_cfg.addr_bit_len = config_.addr_bit_len;
  slave_cfg.intr_priority = config_.intr_priority;
  slave_cfg.flags.allow_pd = 0;
#if ESPP_I2C_SLAVE_V2_API
  slave_cfg.flags.enable_internal_pullup = config_.enable_internal_pullup;
#endif

  esp_err_t err = i2c_new_slave_device(&slave_cfg, &dev_handle_);
  if (err != ESP_OK) {
    logger_.error("could not create I2C slave device: {}", esp_err_to_name(err));
    vMessageBufferDelete(read_buffer_);
    vMessageBufferDelete(callback_buffer_);
    vQueueDelete(event_queue_);
    read_buffer_ = nullptr;
    callback_buffer_ = nullptr;
    event_queue_ = nullptr;
    ec = make_error_code(err);
    return false;
  }

  i2c_slave_event_callbacks_t cbs = {};
#if ESPP_I2C_SLAVE_V2_API
  cbs.on_request = &I2cSlaveDevice::request_callback_trampoline;
  cbs.on_receive = &I2cSlaveDevice::receive_callback_trampoline;
#else
  cbs.on_recv_done = &I2cSlaveDevice::receive_callback_trampoline;
#endif
  err = i2c_slave_register_event_callbacks(dev_handle_, &cbs, this);
  if (err != ESP_OK) {
    logger_.error("could not register I2C slave callbacks: {}", esp_err_to_name(err));
    i2c_del_slave_device(dev_handle_);
    dev_handle_ = nullptr;
    vMessageBufferDelete(read_buffer_);
    vMessageBufferDelete(callback_buffer_);
    vQueueDelete(event_queue_);
    read_buffer_ = nullptr;
    callback_buffer_ = nullptr;
    event_queue_ = nullptr;
    ec = make_error_code(err);
    return false;
  }

  event_task_ = espp::Task::make_unique(
      {.callback = std::bind(&I2cSlaveDevice::event_task_callback, this, std::placeholders::_1,
                             std::placeholders::_2, std::placeholders::_3),
       .task_config = config_.task_config,
       .log_level = config_.log_level});
  if (!event_task_ || !event_task_->start()) {
    logger_.error("could not start I2C slave event task");
    i2c_del_slave_device(dev_handle_);
    dev_handle_ = nullptr;
    event_task_.reset();
    vMessageBufferDelete(read_buffer_);
    vMessageBufferDelete(callback_buffer_);
    vQueueDelete(event_queue_);
    read_buffer_ = nullptr;
    callback_buffer_ = nullptr;
    event_queue_ = nullptr;
    ec = std::make_error_code(std::errc::resource_unavailable_try_again);
    return false;
  }

  read_buffer_overflowed_ = false;
  callback_buffer_overflowed_ = false;
  event_queue_overflowed_ = false;
  legacy_receive_buffer_.assign(config_.receive_buffer_depth, 0);
  legacy_receive_length_ = 0;
  legacy_receive_armed_ = false;
  initialized_ = true;
  ec.clear();
  return true;
}

bool I2cSlaveDevice::deinit(std::error_code &ec) {
  i2c_slave_dev_handle_t dev_handle = nullptr;
  {
    std::lock_guard<std::recursive_mutex> lock(mutex_);
    if (!initialized_) {
      ec.clear();
      return true;
    }
    initialized_ = false;
    dev_handle = dev_handle_;
    dev_handle_ = nullptr;
  }

  if (dev_handle) {
    esp_err_t err = i2c_del_slave_device(dev_handle);
    if (err != ESP_OK) {
      logger_.error("could not delete I2C slave device: {}", esp_err_to_name(err));
      ec = make_error_code(err);
      return false;
    }
  }

  if (event_queue_) {
    Event stop_event{EventType::STOP};
    xQueueReset(event_queue_);
    if (xQueueSend(event_queue_, &stop_event, timeout_ticks(config_.timeout_ms)) != pdPASS) {
      logger_.warn("failed to queue I2C slave stop event during deinit");
    }
  }
  if (event_task_) {
    event_task_->stop();
    event_task_.reset();
  }

  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (event_queue_) {
    vQueueDelete(event_queue_);
    event_queue_ = nullptr;
  }
  if (read_buffer_) {
    vMessageBufferDelete(read_buffer_);
    read_buffer_ = nullptr;
  }
  if (callback_buffer_) {
    vMessageBufferDelete(callback_buffer_);
    callback_buffer_ = nullptr;
  }
  callbacks_ = {};
  legacy_receive_buffer_.clear();
  legacy_receive_length_ = 0;
  legacy_receive_armed_ = false;
  ec.clear();
  return true;
}

bool I2cSlaveDevice::write(const uint8_t *data, size_t len, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !dev_handle_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (len == 0) {
    ec.clear();
    return true;
  }
  if (!data) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

#if ESPP_I2C_SLAVE_V2_API
  uint32_t write_len = 0;
  esp_err_t err = i2c_slave_write(dev_handle_, data, len, &write_len, config_.timeout_ms);
  if (err != ESP_OK) {
    logger_.error("I2C slave write failed: {}", esp_err_to_name(err));
    ec = make_error_code(err);
    return false;
  }
  if (write_len != len) {
    logger_.error("I2C slave write was truncated: wrote {} of {} bytes", write_len, len);
    ec = std::make_error_code(std::errc::io_error);
    return false;
  }

  ec.clear();
  return true;
#else
  esp_err_t err = i2c_slave_transmit(dev_handle_, data, len, config_.timeout_ms);
  if (err != ESP_OK) {
    logger_.error("I2C slave write failed: {}", esp_err_to_name(err));
    ec = make_error_code(err);
    return false;
  }

  ec.clear();
  return true;
#endif
}

bool I2cSlaveDevice::read(uint8_t *data, size_t len, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  if (!initialized_ || !read_buffer_) {
    ec = std::make_error_code(std::errc::not_connected);
    return false;
  }
  if (len == 0) {
    ec.clear();
    return true;
  }
  if (!data) {
    ec = std::make_error_code(std::errc::invalid_argument);
    return false;
  }

  log_pending_overflows();

#if !ESPP_I2C_SLAVE_V2_API
  size_t next_length = xMessageBufferNextLengthBytes(read_buffer_);
  if (next_length == 0 && !legacy_receive_armed_) {
    if (len > config_.receive_buffer_depth) {
      logger_.error(
          "I2C slave read buffer too small for configured receive depth (need {}, have {})",
          config_.receive_buffer_depth, len);
      ec = std::make_error_code(std::errc::message_size);
      return false;
    }
    std::fill(legacy_receive_buffer_.begin(), legacy_receive_buffer_.end(), 0);
    legacy_receive_length_ = len;
    legacy_receive_armed_ = true;
    esp_err_t err = i2c_slave_receive(dev_handle_, legacy_receive_buffer_.data(), len);
    if (err != ESP_OK) {
      legacy_receive_armed_ = false;
      legacy_receive_length_ = 0;
      logger_.error("I2C slave read failed to arm receive: {}", esp_err_to_name(err));
      ec = make_error_code(err);
      return false;
    }
  }
#endif

  size_t read_len =
      xMessageBufferReceive(read_buffer_, data, len, timeout_ticks(config_.timeout_ms));
  if (read_len == 0) {
    size_t next_length = xMessageBufferNextLengthBytes(read_buffer_);
    if (next_length > len) {
      logger_.error("I2C slave read buffer too small for next transaction (need {}, have {})",
                    next_length, len);
      ec = std::make_error_code(std::errc::message_size);
#if !ESPP_I2C_SLAVE_V2_API
    } else if (legacy_receive_armed_) {
      ec = std::make_error_code(std::errc::timed_out);
#endif
    } else if (read_buffer_overflowed_.exchange(false)) {
      logger_.error("I2C slave dropped received data because the read queue overflowed");
      ec = std::make_error_code(std::errc::no_buffer_space);
    } else {
      ec = std::make_error_code(std::errc::timed_out);
    }
    return false;
  }

  ec.clear();
  return true;
}

bool I2cSlaveDevice::register_callbacks(const Callbacks &cb, std::error_code &ec) {
  std::lock_guard<std::recursive_mutex> lock(mutex_);
  callbacks_ = cb;
  ec.clear();
  return true;
}

#if ESPP_I2C_SLAVE_V2_API
bool IRAM_ATTR I2cSlaveDevice::request_callback_trampoline(i2c_slave_dev_handle_t,
                                                           const i2c_slave_request_event_data_t *,
                                                           void *user_data) {
  auto *device = static_cast<I2cSlaveDevice *>(user_data);
  if (!device || !device->event_queue_) {
    return false;
  }

  BaseType_t task_woken = pdFALSE;
  Event event{EventType::REQUEST};
  if (xQueueSendFromISR(device->event_queue_, &event, &task_woken) != pdPASS) {
    device->event_queue_overflowed_ = true;
  }
  return task_woken == pdTRUE;
}
#endif

bool IRAM_ATTR I2cSlaveDevice::receive_callback_trampoline(
    i2c_slave_dev_handle_t, const i2c_slave_rx_done_event_data_t *evt_data, void *user_data) {
  auto *device = static_cast<I2cSlaveDevice *>(user_data);
  if (!device || !device->event_queue_ || !evt_data || !evt_data->buffer) {
    return false;
  }

  BaseType_t task_woken = pdFALSE;
#if ESPP_I2C_SLAVE_V2_API
  size_t length = evt_data->length;
#else
  size_t length = device->legacy_receive_length_.exchange(0);
  device->legacy_receive_armed_ = false;
#endif
  if (length == 0) {
    return false;
  }

  size_t sent =
      xMessageBufferSendFromISR(device->read_buffer_, evt_data->buffer, length, &task_woken);
  if (sent != length) {
    device->read_buffer_overflowed_ = true;
  }

  sent = xMessageBufferSendFromISR(device->callback_buffer_, evt_data->buffer, length, &task_woken);
  if (sent != length) {
    device->callback_buffer_overflowed_ = true;
  }

  Event event{EventType::RECEIVE};
  if (xQueueSendFromISR(device->event_queue_, &event, &task_woken) != pdPASS) {
    device->event_queue_overflowed_ = true;
  }
  return task_woken == pdTRUE;
}

void I2cSlaveDevice::log_pending_overflows() {
  if (read_buffer_overflowed_.exchange(false)) {
    logger_.error("I2C slave dropped received data because the read queue overflowed");
  }
  if (callback_buffer_overflowed_.exchange(false)) {
    logger_.error("I2C slave dropped callback data because the callback queue overflowed");
  }
  if (event_queue_overflowed_.exchange(false)) {
    logger_.error("I2C slave dropped events because the event queue overflowed");
  }
}

bool I2cSlaveDevice::event_task_callback(std::mutex &m, std::condition_variable &, bool &notified) {
  {
    std::lock_guard<std::mutex> lock(m);
    if (notified) {
      notified = false;
      return true;
    }
  }

  Event event{};
  static constexpr TickType_t stop_poll_ticks = pdMS_TO_TICKS(50);
  if (!event_queue_) {
    return true;
  }
  if (!xQueueReceive(event_queue_, &event, stop_poll_ticks)) {
    std::lock_guard<std::mutex> lock(m);
    bool stop_requested = notified;
    notified = false;
    return stop_requested;
  }

  {
    std::lock_guard<std::mutex> lock(m);
    if (notified) {
      notified = false;
      return true;
    }
  }

  if (!initialized_ && event.type != EventType::STOP) {
    return false;
  }

  log_pending_overflows();

  switch (event.type) {
  case EventType::STOP:
    return true;
  case EventType::REQUEST: {
    RequestCallback callback;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      callback = callbacks_.on_request;
    }
    if (callback) {
      callback();
    }
    break;
  }
  case EventType::RECEIVE: {
    std::vector<uint8_t> data(config_.receive_buffer_depth);
    size_t length =
        callback_buffer_ ? xMessageBufferReceive(callback_buffer_, data.data(), data.size(), 0) : 0;
    if (length == 0) {
      break;
    }
    ReceiveCallback callback;
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      callback = callbacks_.on_receive;
    }
    if (callback) {
      callback(data.data(), length);
    }
    break;
  }
  }

  return false;
}

} // namespace espp

#endif // CONFIG_ESPP_I2C_USE_NEW_API
