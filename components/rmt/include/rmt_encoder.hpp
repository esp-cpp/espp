#pragma once

#include <functional>

#include "driver/rmt_encoder.h"

namespace espp {
/// \brief Class representing an RMT encoder
/// \details This class is used to encode data for the RMT peripheral.
/// It is used by the Rmt class to encode data for transmission.
/// \sa
/// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/rmt.html#rmt-encoder
///
///  \section rmt_encoder_ex1 Example 1: WS2812 encoder
///  \snippet rmt_example.cpp rmt encoder example
class RmtEncoder {
public:
  /// \brief Configuration for the byte encoding for SK6805 LEDs
  /// \details This configuration is used to encode bytes for SK6085 LEDs.
  /// \note This configuration can be provided to the configuration for this
  /// class. These values are based on the timing values provided in the
  /// SK6805 datasheet (https://cdn-shop.adafruit.com/product-files/3484/3484_Datasheet.pdf)
  /// \sa Config
  static constexpr rmt_bytes_encoder_config_t sk6805_10mhz_bytes_encoder_config = {
      .bit0 =
          {
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration0 = static_cast<uint16_t>(0.3 * 10000000 / 1000000), // T0H=0.3us
              .level0 = 1,
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration1 = static_cast<uint16_t>(0.9 * 10000000 / 1000000), // T0L=0.9us
              .level1 = 0,
          },
      .bit1 =
          {
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration0 = static_cast<uint16_t>(0.6 * 10000000 / 1000000), // T1H=0.6us
              .level0 = 1,
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration1 = static_cast<uint16_t>(0.6 * 10000000 / 1000000), // T1L=0.6us
              .level1 = 0,
          },
      .flags =
          {
              .msb_first = 1 // SK6805 transfer bit order: G7...G0R7...R0B7...B0
          },
  };

  /// \brief Configuration for the byte encoding for WS2812 LEDs
  /// \details This configuration is used to encode bytes for WS2812 LEDs.
  /// \note This configuration can be provided to the configuration for this
  /// class.
  /// \sa Config
  static constexpr rmt_bytes_encoder_config_t ws2812_10mhz_bytes_encoder_config = {
      .bit0 =
          {
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration0 = static_cast<uint16_t>(0.3 * 10000000 / 1000000), // T0H=0.3us
              .level0 = 1,
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration1 = static_cast<uint16_t>(0.9 * 10000000 / 1000000), // T0L=0.9us
              .level1 = 0,
          },
      .bit1 =
          {
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration0 = static_cast<uint16_t>(0.9 * 10000000 / 1000000), // T1H=0.9us
              .level0 = 1,
              // divide the rmt transmit resolution (10 MHz) by 1,000,000
              .duration1 = static_cast<uint16_t>(0.3 * 10000000 / 1000000), // T1L=0.3us
              .level1 = 0,
          },
      .flags =
          {
              .msb_first = 1 // WS2812 transfer bit order: G7...G0R7...R0B7...B0
          },
  };

  /// \brief Function to encode data for the RMT peripheral
  /// \param channel RMT channel to use for encoding
  /// \param copy_encoder RMT encoder to use for copying data
  /// \param bytes_encoder RMT encoder to use for encoding bytes
  /// \param primary_data Pointer to the primary data to encode
  /// \param data_size Size of the primary data to encode
  /// \param ret_state Pointer to the RMT encoder state to return
  /// \return Number of bytes encoded
  /// \note This function is called by the Rmt class to encode data for
  /// transmission. It is called repeatedly until all data has been encoded.
  /// \note This function should return the number of bytes encoded.
  /// \note This function should set the RMT encoder state to the next state
  /// to be used for encoding.
  typedef std::function<size_t(rmt_channel_handle_t channel, rmt_encoder_t *copy_encoder,
                               rmt_encoder_t *bytes_encoder, const void *primary_data,
                               size_t data_size, rmt_encode_state_t *ret_state)>
      encode_fn;

  /// \brief Function to delete an RMT encoder
  /// \param encoder RMT encoder to delete
  /// \return ESP_OK if the encoder was successfully deleted, an error code
  typedef std::function<esp_err_t(rmt_encoder_t *)> delete_fn;

  /// \brief Function to reset an RMT encoder
  /// \param encoder RMT encoder to reset
  /// \return ESP_OK if the encoder was successfully reset, an error code
  typedef std::function<esp_err_t(rmt_encoder_t *)> reset_fn;

  /// \brief Configuration for this class
  struct Config {
    rmt_bytes_encoder_config_t bytes_encoder_config; ///< Configuration for the RMT bytes encoder
    encode_fn encode; ///< Function to encode data for the RMT peripheral
    delete_fn del;    ///< Function to delete an RMT encoder
    reset_fn reset;   ///< Function to reset an RMT encoder
  };

  /// \brief Constructor
  /// \param config Configuration for this class
  explicit RmtEncoder(const Config &config) { init(config); }

  /// \brief Destructor
  ~RmtEncoder() {
    if (handle_ != nullptr)
      rmt_del_encoder(handle_);
  }

  /// \brief Get the RMT encoder handle
  /// \return RMT encoder handle
  rmt_encoder_handle_t handle() const { return handle_; }

protected:
  /// \brief RMT encoder wrapper
  struct Wrapper {
    rmt_encoder_t base_;           ///< Base RMT encoder
    rmt_encoder_t *bytes_encoder_; ///< RMT encoder for encoding bytes
    rmt_encoder_t *copy_encoder_;  ///< RMT encoder for copying data
    RmtEncoder *self_;             ///< Pointer to the RmtEncoder object
  };

  static size_t encoder_encode(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                               const void *primary_data, size_t data_size,
                               rmt_encode_state_t *ret_state) {
    // get the encoders from the wrapper
    Wrapper *wrapper = __containerof(encoder, Wrapper, base_);
    if (wrapper == nullptr) {
      return 0;
    }
    RmtEncoder *self = wrapper->self_;
    if (self == nullptr) {
      return 0;
    }
    rmt_encoder_t *copy_encoder = wrapper->copy_encoder_;
    rmt_encoder_t *bytes_encoder = wrapper->bytes_encoder_;
    // call the object's encode function
    auto bytes_encoded =
        self->encode_(channel, copy_encoder, bytes_encoder, primary_data, data_size, ret_state);
    return bytes_encoded;
  }

  static esp_err_t encoder_reset(rmt_encoder_t *encoder) {
    // get the encoders from the wrapper
    Wrapper *wrapper = __containerof(encoder, Wrapper, base_);
    RmtEncoder *self = wrapper->self_;
    // call the object's reset runction
    self->reset_(encoder);
    // now reset the encoders
    rmt_encoder_reset(wrapper->bytes_encoder_);
    rmt_encoder_reset(wrapper->copy_encoder_);
    return ESP_OK;
  }

  static esp_err_t encoder_delete(rmt_encoder_t *encoder) {
    // get the encoders from the wrapper
    Wrapper *wrapper = __containerof(encoder, Wrapper, base_);
    if (wrapper == nullptr) {
      return ESP_OK;
    }
    RmtEncoder *self = wrapper->self_;
    // call the object's delete function
    if (self->del_ != nullptr)
      self->del_(encoder);
    // delete the encoders
    rmt_del_encoder(wrapper->bytes_encoder_);
    rmt_del_encoder(wrapper->copy_encoder_);
    // delete the wrapper
    delete wrapper;
    // now delete the RmtEncoder object
    delete self;
    return ESP_OK;
  }

  void init(const Config &config) {
    encode_ = config.encode;
    del_ = config.del;
    reset_ = config.reset;
    // create the wrapper
    Wrapper *wrapper = new Wrapper;
    wrapper->self_ = this;
    // create the encoders
    wrapper->base_.encode = encoder_encode;
    wrapper->base_.del = encoder_delete;
    wrapper->base_.reset = encoder_reset;
    rmt_new_bytes_encoder(&config.bytes_encoder_config, &wrapper->bytes_encoder_);
    rmt_copy_encoder_config_t copy_encoder_config = {};
    rmt_new_copy_encoder(&copy_encoder_config, &wrapper->copy_encoder_);
    // store the handle
    handle_ = &wrapper->base_;
  }

  encode_fn encode_;
  delete_fn del_;
  reset_fn reset_;
  rmt_encoder_handle_t handle_{nullptr};
};
} // namespace espp
