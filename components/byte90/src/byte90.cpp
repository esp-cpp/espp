#include "byte90.hpp"

using namespace espp;

Byte90::Byte90()
    : BaseComponent("Byte90") {}

////////////////////////
//   SPI Functions    //
////////////////////////

bool Byte90::init_spi_bus() {
  if (spi_bus_initialized_) {
    return true;
  }

  spi_bus_config_t bus_cfg;
  memset(&bus_cfg, 0, sizeof(bus_cfg));
  bus_cfg.mosi_io_num = spi_mosi_io;
  bus_cfg.miso_io_num = -1;
  bus_cfg.sclk_io_num = spi_sclk_io;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = frame_buffer_size * sizeof(lv_color_t) + 100;
  auto ret = spi_bus_initialize(spi_num, &bus_cfg, SPI_DMA_CH_AUTO);
  if (ret != ESP_OK) {
    logger_.error("Failed to initialize bus.");
    return false;
  }

  logger_.info("SPI bus initialized");
  spi_bus_initialized_ = true;

  return true;
}
