#include "xiao-esp32s3-sense.hpp"

#include <cmath>

#include <driver/spi_master.h>

using namespace espp;

XiaoEsp32S3Sense::XiaoEsp32S3Sense()
    : BaseComponent("XiaoEsp32S3Sense") {}

XiaoEsp32S3Sense::CameraPins XiaoEsp32S3Sense::camera_pins() const {
  return {
      .pwdn = camera_pwdn_pin(),
      .reset = camera_reset_pin(),
      .xclk = camera_xclk_pin(),
      .sccb_sda = camera_sccb_sda_pin(),
      .sccb_scl = camera_sccb_scl_pin(),
      .d7 = camera_d7_pin(),
      .d6 = camera_d6_pin(),
      .d5 = camera_d5_pin(),
      .d4 = camera_d4_pin(),
      .d3 = camera_d3_pin(),
      .d2 = camera_d2_pin(),
      .d1 = camera_d1_pin(),
      .d0 = camera_d0_pin(),
      .vsync = camera_vsync_pin(),
      .href = camera_href_pin(),
      .pclk = camera_pclk_pin(),
  };
}

XiaoEsp32S3Sense::MicrophonePins XiaoEsp32S3Sense::microphone_pins() const {
  return {
      .clk = microphone_clk_pin(),
      .data = microphone_data_pin(),
  };
}

XiaoEsp32S3Sense::SdCardPins XiaoEsp32S3Sense::sd_card_pins() const {
  return {
      .clk = sd_card_clk_pin(),
      .mosi = sd_card_mosi_pin(),
      .miso = sd_card_miso_pin(),
      .cs = sd_card_cs_pin(),
  };
}

bool XiaoEsp32S3Sense::initialize_sdcard() { return initialize_sdcard(SdCardConfig{}); }

bool XiaoEsp32S3Sense::initialize_sdcard(const XiaoEsp32S3Sense::SdCardConfig &config) {
  if (sdcard_) {
    logger_.error("SD card already initialized");
    return false;
  }

  logger_.info("Initializing microSD card");

  esp_vfs_fat_sdmmc_mount_config_t mount_config{};
  mount_config.format_if_mount_failed = config.format_if_mount_failed;
  mount_config.max_files = config.max_files;
  mount_config.allocation_unit_size = config.allocation_unit_size;

  spi_bus_config_t bus_config{};
  bus_config.mosi_io_num = sd_card_mosi_pin();
  bus_config.miso_io_num = sd_card_miso_pin();
  bus_config.sclk_io_num = sd_card_clk_pin();
  bus_config.quadwp_io_num = -1;
  bus_config.quadhd_io_num = -1;
  bus_config.max_transfer_sz = sd_card_spi_max_transfer_bytes_;

  esp_err_t ret = spi_bus_initialize(sd_card_spi_num_, &bus_config, SDSPI_DEFAULT_DMA);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize microSD SPI bus: {}", esp_err_to_name(ret));
    return false;
  }

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  host.slot = sd_card_spi_num_;

  sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
  slot_config.gpio_cs = sd_card_cs_pin();
  slot_config.host_id = sd_card_spi_num_;

  ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &sdcard_);
  if (ret != ESP_OK) {
    if (ret == ESP_FAIL) {
      logger_.error("Failed to mount microSD filesystem");
    } else {
      logger_.error("Failed to initialize microSD card ({}). Make sure the card is inserted and "
                    "the bus has pull-ups.",
                    esp_err_to_name(ret));
    }
    spi_bus_free(sd_card_spi_num_);
    sdcard_ = nullptr;
    return false;
  }

  logger_.info("microSD filesystem mounted at {}", mount_point);
  sdmmc_card_print_info(stdout, sdcard_);
  return true;
}

bool XiaoEsp32S3Sense::initialize_led(float breathing_period) {
  if (led_) {
    logger_.error("LED already initialized");
    return false;
  }
  led_ = std::make_shared<espp::Led>(espp::Led::Config{
      .timer = LEDC_TIMER_1,
      .frequency_hz = 5000,
      .channels = led_channels_,
      .duty_resolution = LEDC_TIMER_10_BIT,
  });
  using namespace std::placeholders;
  led_task_ = espp::Task::make_unique(
      {.callback = std::bind(&XiaoEsp32S3Sense::led_task_callback, this, _1, _2, _3),
       .task_config = {.name = "xiao_led_breathe"}});
  if (!led_task_) {
    logger_.error("Could not create LED breathing task");
    led_.reset();
    return false;
  }
  if (!set_led_breathing_period(breathing_period)) {
    led_task_.reset();
    led_.reset();
    return false;
  }
  return true;
}

void XiaoEsp32S3Sense::start_led_breathing() {
  if (led_ == nullptr || led_task_ == nullptr) {
    logger_.error("LED not initialized");
    return;
  }
  breathing_start_ = std::chrono::high_resolution_clock::now();
  led_task_->start();
}

void XiaoEsp32S3Sense::stop_led_breathing() {
  if (led_ == nullptr || led_task_ == nullptr) {
    logger_.error("LED not initialized");
    return;
  }
  led_task_->stop();
}

bool XiaoEsp32S3Sense::set_led_brightness(float brightness) {
  if (led_ == nullptr || led_task_ == nullptr) {
    logger_.error("LED not initialized");
    return false;
  }
  if (led_task_->is_running()) {
    logger_.error("Cannot set brightness while breathing");
    return false;
  }
  brightness = std::clamp(brightness, 0.0f, 1.0f);
  led_->set_duty(led_channels_[0].channel, 100.0f * brightness);
  return true;
}

float XiaoEsp32S3Sense::get_led_brightness() {
  if (led_ == nullptr) {
    logger_.error("LED not initialized");
    return 0.0f;
  }
  auto maybe_duty = led_->get_duty(led_channels_[0].channel);
  if (!maybe_duty.has_value()) {
    logger_.error("Failed to get LED duty");
    return 0.0f;
  }
  return maybe_duty.value() / 100.0f;
}

bool XiaoEsp32S3Sense::set_led_breathing_period(float breathing_period) {
  if (breathing_period <= 0.0f) {
    logger_.error("Invalid breathing period: {}", breathing_period);
    return false;
  }
  breathing_period_ = breathing_period;
  return true;
}

float XiaoEsp32S3Sense::get_led_breathing_period() { return breathing_period_; }

std::shared_ptr<espp::Led> XiaoEsp32S3Sense::led() { return led_; }

espp::Gaussian &XiaoEsp32S3Sense::gaussian() { return gaussian_; }

float XiaoEsp32S3Sense::led_breathe() {
  auto now = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration<float>(now - breathing_start_).count();
  float t = std::fmod(elapsed, breathing_period_) / breathing_period_;
  return gaussian_(t);
}

bool XiaoEsp32S3Sense::led_task_callback(std::mutex &m, std::condition_variable &cv,
                                         bool &task_notified) {
  using namespace std::chrono_literals;
  led_->set_duty(led_channels_[0].channel, 100.0f * led_breathe());
  std::unique_lock<std::mutex> lk(m);
  cv.wait_for(lk, 10ms, [&task_notified] { return task_notified; });
  task_notified = false;
  return false;
}

i2s_pdm_rx_clk_config_t XiaoEsp32S3Sense::microphone_clock_config(uint32_t sample_rate_hz) const {
  i2s_pdm_rx_clk_config_t config = I2S_PDM_RX_CLK_DEFAULT_CONFIG(sample_rate_hz);
  return config;
}

i2s_pdm_rx_slot_config_t XiaoEsp32S3Sense::microphone_slot_config(i2s_slot_mode_t slot_mode) const {
  i2s_pdm_rx_slot_config_t config =
      I2S_PDM_RX_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, slot_mode);
  return config;
}

i2s_pdm_rx_gpio_config_t XiaoEsp32S3Sense::microphone_gpio_config(bool clock_inverted) const {
  i2s_pdm_rx_gpio_config_t config = {};
  config.clk = microphone_clk_pin();
  config.din = microphone_data_pin();
  config.invert_flags.clk_inv = clock_inverted;
  return config;
}

i2s_pdm_rx_config_t XiaoEsp32S3Sense::microphone_config(uint32_t sample_rate_hz,
                                                        i2s_slot_mode_t slot_mode,
                                                        bool clock_inverted) const {
  return {
      .clk_cfg = microphone_clock_config(sample_rate_hz),
      .slot_cfg = microphone_slot_config(slot_mode),
      .gpio_cfg = microphone_gpio_config(clock_inverted),
  };
}
