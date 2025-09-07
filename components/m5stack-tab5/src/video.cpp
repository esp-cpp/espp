#include "m5stack-tab5.hpp"

#include <algorithm>

#include <esp_lcd_mipi_dsi.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_ldo_regulator.h>

#define ILI9881C_CMD_CNDBKxSEL (0xFF)
#define ILI9881C_CMD_BKxSEL_BYTE0 (0x98)
#define ILI9881C_CMD_BKxSEL_BYTE1 (0x81)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE0 (0x00)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE1 (0x01)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE2 (0x02)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE3 (0x03)
#define ILI9881C_CMD_BKxSEL_BYTE2_PAGE4 (0x04)

#define ILI9881C_PAD_CONTROL (0xB7)
#define ILI9881C_DSI_2_LANE (0x03)
#define ILI9881C_DSI_3_4_LANE (0x02)

#define ILI9881C_CMD_GS_BIT (1 << 0)
#define ILI9881C_CMD_SS_BIT (1 << 1)

typedef struct {
  int cmd;               /*<! The specific LCD command */
  const void *data;      /*<! Buffer that holds the command specific data */
  size_t data_bytes;     /*<! Size of `data` in memory, in bytes */
  unsigned int delay_ms; /*<! Delay in milliseconds after this command */
} ili9881c_lcd_init_cmd_t;

static const ili9881c_lcd_init_cmd_t vendor_specific_init_default[] = {
    // {cmd, { data }, data_size, delay_ms}
    /**** CMD_Page 3 ****/
    {ILI9881C_CMD_CNDBKxSEL,
     (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                 ILI9881C_CMD_BKxSEL_BYTE2_PAGE3},
     3, 0},
    {0x01, (uint8_t[]){0x00}, 1, 0},
    {0x02, (uint8_t[]){0x00}, 1, 0},
    {0x03, (uint8_t[]){0x53}, 1, 0},
    {0x04, (uint8_t[]){0x53}, 1, 0},
    {0x05, (uint8_t[]){0x13}, 1, 0},
    {0x06, (uint8_t[]){0x04}, 1, 0},
    {0x07, (uint8_t[]){0x02}, 1, 0},
    {0x08, (uint8_t[]){0x02}, 1, 0},
    {0x09, (uint8_t[]){0x00}, 1, 0},
    {0x0a, (uint8_t[]){0x00}, 1, 0},
    {0x0b, (uint8_t[]){0x00}, 1, 0},
    {0x0c, (uint8_t[]){0x00}, 1, 0},
    {0x0d, (uint8_t[]){0x00}, 1, 0},
    {0x0e, (uint8_t[]){0x00}, 1, 0},
    {0x0f, (uint8_t[]){0x00}, 1, 0},
    {0x10, (uint8_t[]){0x00}, 1, 0},
    {0x11, (uint8_t[]){0x00}, 1, 0},
    {0x12, (uint8_t[]){0x00}, 1, 0},
    {0x13, (uint8_t[]){0x00}, 1, 0},
    {0x14, (uint8_t[]){0x00}, 1, 0},
    {0x15, (uint8_t[]){0x00}, 1, 0},
    {0x16, (uint8_t[]){0x00}, 1, 0},
    {0x17, (uint8_t[]){0x00}, 1, 0},
    {0x18, (uint8_t[]){0x00}, 1, 0},
    {0x19, (uint8_t[]){0x00}, 1, 0},
    {0x1a, (uint8_t[]){0x00}, 1, 0},
    {0x1b, (uint8_t[]){0x00}, 1, 0},
    {0x1c, (uint8_t[]){0x00}, 1, 0},
    {0x1d, (uint8_t[]){0x00}, 1, 0},
    {0x1e, (uint8_t[]){0xc0}, 1, 0},
    {0x1f, (uint8_t[]){0x80}, 1, 0},
    {0x20, (uint8_t[]){0x02}, 1, 0},
    {0x21, (uint8_t[]){0x09}, 1, 0},
    {0x22, (uint8_t[]){0x00}, 1, 0},
    {0x23, (uint8_t[]){0x00}, 1, 0},
    {0x24, (uint8_t[]){0x00}, 1, 0},
    {0x25, (uint8_t[]){0x00}, 1, 0},
    {0x26, (uint8_t[]){0x00}, 1, 0},
    {0x27, (uint8_t[]){0x00}, 1, 0},
    {0x28, (uint8_t[]){0x55}, 1, 0},
    {0x29, (uint8_t[]){0x03}, 1, 0},
    {0x2a, (uint8_t[]){0x00}, 1, 0},
    {0x2b, (uint8_t[]){0x00}, 1, 0},
    {0x2c, (uint8_t[]){0x00}, 1, 0},
    {0x2d, (uint8_t[]){0x00}, 1, 0},
    {0x2e, (uint8_t[]){0x00}, 1, 0},
    {0x2f, (uint8_t[]){0x00}, 1, 0},
    {0x30, (uint8_t[]){0x00}, 1, 0},
    {0x31, (uint8_t[]){0x00}, 1, 0},
    {0x32, (uint8_t[]){0x00}, 1, 0},
    {0x33, (uint8_t[]){0x00}, 1, 0},
    {0x34, (uint8_t[]){0x00}, 1, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x36, (uint8_t[]){0x00}, 1, 0},
    {0x37, (uint8_t[]){0x00}, 1, 0},
    {0x38, (uint8_t[]){0x3C}, 1, 0},
    {0x39, (uint8_t[]){0x00}, 1, 0},
    {0x3a, (uint8_t[]){0x00}, 1, 0},
    {0x3b, (uint8_t[]){0x00}, 1, 0},
    {0x3c, (uint8_t[]){0x00}, 1, 0},
    {0x3d, (uint8_t[]){0x00}, 1, 0},
    {0x3e, (uint8_t[]){0x00}, 1, 0},
    {0x3f, (uint8_t[]){0x00}, 1, 0},
    {0x40, (uint8_t[]){0x00}, 1, 0},
    {0x41, (uint8_t[]){0x00}, 1, 0},
    {0x42, (uint8_t[]){0x00}, 1, 0},
    {0x43, (uint8_t[]){0x00}, 1, 0},
    {0x44, (uint8_t[]){0x00}, 1, 0},
    {0x50, (uint8_t[]){0x01}, 1, 0},
    {0x51, (uint8_t[]){0x23}, 1, 0},
    {0x52, (uint8_t[]){0x45}, 1, 0},
    {0x53, (uint8_t[]){0x67}, 1, 0},
    {0x54, (uint8_t[]){0x89}, 1, 0},
    {0x55, (uint8_t[]){0xab}, 1, 0},
    {0x56, (uint8_t[]){0x01}, 1, 0},
    {0x57, (uint8_t[]){0x23}, 1, 0},
    {0x58, (uint8_t[]){0x45}, 1, 0},
    {0x59, (uint8_t[]){0x67}, 1, 0},
    {0x5a, (uint8_t[]){0x89}, 1, 0},
    {0x5b, (uint8_t[]){0xab}, 1, 0},
    {0x5c, (uint8_t[]){0xcd}, 1, 0},
    {0x5d, (uint8_t[]){0xef}, 1, 0},
    {0x5e, (uint8_t[]){0x01}, 1, 0},
    {0x5f, (uint8_t[]){0x08}, 1, 0},
    {0x60, (uint8_t[]){0x02}, 1, 0},
    {0x61, (uint8_t[]){0x02}, 1, 0},
    {0x62, (uint8_t[]){0x0A}, 1, 0},
    {0x63, (uint8_t[]){0x15}, 1, 0},
    {0x64, (uint8_t[]){0x14}, 1, 0},
    {0x65, (uint8_t[]){0x02}, 1, 0},
    {0x66, (uint8_t[]){0x11}, 1, 0},
    {0x67, (uint8_t[]){0x10}, 1, 0},
    {0x68, (uint8_t[]){0x02}, 1, 0},
    {0x69, (uint8_t[]){0x0F}, 1, 0},
    {0x6a, (uint8_t[]){0x0E}, 1, 0},
    {0x6b, (uint8_t[]){0x02}, 1, 0},
    {0x6c, (uint8_t[]){0x0D}, 1, 0},
    {0x6d, (uint8_t[]){0x0C}, 1, 0},
    {0x6e, (uint8_t[]){0x06}, 1, 0},
    {0x6f, (uint8_t[]){0x02}, 1, 0},
    {0x70, (uint8_t[]){0x02}, 1, 0},
    {0x71, (uint8_t[]){0x02}, 1, 0},
    {0x72, (uint8_t[]){0x02}, 1, 0},
    {0x73, (uint8_t[]){0x02}, 1, 0},
    {0x74, (uint8_t[]){0x02}, 1, 0},
    {0x75, (uint8_t[]){0x06}, 1, 0},
    {0x76, (uint8_t[]){0x02}, 1, 0},
    {0x77, (uint8_t[]){0x02}, 1, 0},
    {0x78, (uint8_t[]){0x0A}, 1, 0},
    {0x79, (uint8_t[]){0x15}, 1, 0},
    {0x7a, (uint8_t[]){0x14}, 1, 0},
    {0x7b, (uint8_t[]){0x02}, 1, 0},
    {0x7c, (uint8_t[]){0x10}, 1, 0},
    {0x7d, (uint8_t[]){0x11}, 1, 0},
    {0x7e, (uint8_t[]){0x02}, 1, 0},
    {0x7f, (uint8_t[]){0x0C}, 1, 0},
    {0x80, (uint8_t[]){0x0D}, 1, 0},
    {0x81, (uint8_t[]){0x02}, 1, 0},
    {0x82, (uint8_t[]){0x0E}, 1, 0},
    {0x83, (uint8_t[]){0x0F}, 1, 0},
    {0x84, (uint8_t[]){0x08}, 1, 0},
    {0x85, (uint8_t[]){0x02}, 1, 0},
    {0x86, (uint8_t[]){0x02}, 1, 0},
    {0x87, (uint8_t[]){0x02}, 1, 0},
    {0x88, (uint8_t[]){0x02}, 1, 0},
    {0x89, (uint8_t[]){0x02}, 1, 0},
    {0x8A, (uint8_t[]){0x02}, 1, 0},
    {ILI9881C_CMD_CNDBKxSEL,
     (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                 ILI9881C_CMD_BKxSEL_BYTE2_PAGE4},
     3, 0},
    {0x6C, (uint8_t[]){0x15}, 1, 0},
    {0x6E, (uint8_t[]){0x30}, 1, 0},
    {0x6F, (uint8_t[]){0x33}, 1, 0},
    {0x8D, (uint8_t[]){0x1F}, 1, 0},
    {0x87, (uint8_t[]){0xBA}, 1, 0},
    {0x26, (uint8_t[]){0x76}, 1, 0},
    {0xB2, (uint8_t[]){0xD1}, 1, 0},
    {0x35, (uint8_t[]){0x1F}, 1, 0},
    {0x33, (uint8_t[]){0x14}, 1, 0},
    {0x3A, (uint8_t[]){0xA9}, 1, 0},
    {0x3B, (uint8_t[]){0x3D}, 1, 0},
    {0x38, (uint8_t[]){0x01}, 1, 0},
    {0x39, (uint8_t[]){0x00}, 1, 0},
    {ILI9881C_CMD_CNDBKxSEL,
     (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                 ILI9881C_CMD_BKxSEL_BYTE2_PAGE1},
     3, 0},
    {0x22, (uint8_t[]){0x09}, 1, 0},
    {0x31, (uint8_t[]){0x00}, 1, 0},
    {0x40, (uint8_t[]){0x53}, 1, 0},
    {0x50, (uint8_t[]){0xC0}, 1, 0},
    {0x51, (uint8_t[]){0xC0}, 1, 0},
    {0x53, (uint8_t[]){0x47}, 1, 0},
    {0x55, (uint8_t[]){0x46}, 1, 0},
    {0x60, (uint8_t[]){0x28}, 1, 0},
    {0x2E, (uint8_t[]){0xC8}, 1, 0},
    {0xA0, (uint8_t[]){0x01}, 1, 0},
    {0xA1, (uint8_t[]){0x10}, 1, 0},
    {0xA2, (uint8_t[]){0x1B}, 1, 0},
    {0xA3, (uint8_t[]){0x0C}, 1, 0},
    {0xA4, (uint8_t[]){0x14}, 1, 0},
    {0xA5, (uint8_t[]){0x25}, 1, 0},
    {0xA6, (uint8_t[]){0x1A}, 1, 0},
    {0xA7, (uint8_t[]){0x1D}, 1, 0},
    {0xA8, (uint8_t[]){0x68}, 1, 0},
    {0xA9, (uint8_t[]){0x1B}, 1, 0},
    {0xAA, (uint8_t[]){0x26}, 1, 0},
    {0xAB, (uint8_t[]){0x5B}, 1, 0},
    {0xAC, (uint8_t[]){0x1B}, 1, 0},
    {0xAD, (uint8_t[]){0x17}, 1, 0},
    {0xAE, (uint8_t[]){0x4F}, 1, 0},
    {0xAF, (uint8_t[]){0x24}, 1, 0},
    {0xB0, (uint8_t[]){0x2A}, 1, 0},
    {0xB1, (uint8_t[]){0x4E}, 1, 0},
    {0xB2, (uint8_t[]){0x5F}, 1, 0},
    {0xB3, (uint8_t[]){0x39}, 1, 0},
    {0xC0, (uint8_t[]){0x0F}, 1, 0},
    {0xC1, (uint8_t[]){0x1B}, 1, 0},
    {0xC2, (uint8_t[]){0x27}, 1, 0},
    {0xC3, (uint8_t[]){0x16}, 1, 0},
    {0xC4, (uint8_t[]){0x14}, 1, 0},
    {0xC5, (uint8_t[]){0x28}, 1, 0},
    {0xC6, (uint8_t[]){0x1D}, 1, 0},
    {0xC7, (uint8_t[]){0x21}, 1, 0},
    {0xC8, (uint8_t[]){0x6C}, 1, 0},
    {0xC9, (uint8_t[]){0x1B}, 1, 0},
    {0xCA, (uint8_t[]){0x26}, 1, 0},
    {0xCB, (uint8_t[]){0x5B}, 1, 0},
    {0xCC, (uint8_t[]){0x1B}, 1, 0},
    {0xCD, (uint8_t[]){0x1B}, 1, 0},
    {0xCE, (uint8_t[]){0x4F}, 1, 0},
    {0xCF, (uint8_t[]){0x24}, 1, 0},
    {0xD0, (uint8_t[]){0x2A}, 1, 0},
    {0xD1, (uint8_t[]){0x4E}, 1, 0},
    {0xD2, (uint8_t[]){0x5F}, 1, 0},
    {0xD3, (uint8_t[]){0x39}, 1, 0},
    {ILI9881C_CMD_CNDBKxSEL,
     (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                 ILI9881C_CMD_BKxSEL_BYTE2_PAGE0},
     3, 0},
    {0x35, (uint8_t[]){0x00}, 1, 0},
    {0x29, (uint8_t[]){0x00}, 0, 0},

    //============ Gamma END===========
};

namespace espp {

esp_err_t M5StackTab5::lcd_reset(esp_lcd_panel_t *panel) {
  if (panel == nullptr)
    return ESP_ERR_INVALID_ARG;
  M5StackTab5 *tab5 = static_cast<M5StackTab5 *>(panel->user_data);
  if (tab5 == nullptr)
    return ESP_ERR_INVALID_ARG;
  tab5->lcd_reset(true);
  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  tab5->lcd_reset(false);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  return ESP_OK;
}

esp_err_t M5StackTab5::lcd_disp_init(esp_lcd_panel_t *panel) {
  if (panel == nullptr)
    return ESP_ERR_INVALID_ARG;
  M5StackTab5 *tab5 = static_cast<M5StackTab5 *>(panel->user_data);
  if (tab5 == nullptr)
    return ESP_ERR_INVALID_ARG;

  auto io = tab5->lcd_handles_.io;

  esp_err_t err;

  // read out the ID of the display

  // The ID register is on the CMD_Page 1
  err = esp_lcd_panel_io_tx_param(io, ILI9881C_CMD_CNDBKxSEL,
                                  (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                                              ILI9881C_CMD_BKxSEL_BYTE2_PAGE1},
                                  3);
  if (err != ESP_OK) {
    tab5->logger_.error("Failed to set ILI9881C command page: {}", esp_err_to_name(err));
    return err;
  }

  uint8_t id[3] = {0};
  err = esp_lcd_panel_io_rx_param(io, 0x00, &id[0], 1);
  err = esp_lcd_panel_io_rx_param(io, 0x01, &id[1], 1);
  err = esp_lcd_panel_io_rx_param(io, 0x02, &id[2], 1);
  if (err != ESP_OK) {
    tab5->logger_.error("Failed to read LCD ID: {}", esp_err_to_name(err));
    return err;
  }
  tab5->logger_.info("LCD ID: {:02X} {:02X} {:02X}", id[0], id[1], id[2]);
  // id should be 0x98 0x81 0x5C for ILI9881C
  if (id[0] != 0x98 || id[1] != 0x81 || id[2] != 0x5C) {
    tab5->logger_.warn("Unexpected LCD ID, expected ILI9881C but got {:02X} {:02X} {:02X}", id[0],
                       id[1], id[2]);
  }

  // For modifying MIPI-DSI lane settings
  uint8_t lane_command = ILI9881C_DSI_2_LANE;
  err = esp_lcd_panel_io_tx_param(io, ILI9881C_PAD_CONTROL,
                                  (uint8_t[]){
                                      lane_command,
                                  },
                                  1);
  if (err != ESP_OK) {
    tab5->logger_.error("Failed to set ILI9881C PAD_CONTROL: {}", esp_err_to_name(err));
    return err;
  }

  // back to CMD_Page 0
  err = esp_lcd_panel_io_tx_param(io, ILI9881C_CMD_CNDBKxSEL,
                                  (uint8_t[]){ILI9881C_CMD_BKxSEL_BYTE0, ILI9881C_CMD_BKxSEL_BYTE1,
                                              ILI9881C_CMD_BKxSEL_BYTE2_PAGE0},
                                  3);
  if (err != ESP_OK) {
    tab5->logger_.error("Failed to set ILI9881C command page: {}", esp_err_to_name(err));
    return err;
  }

  // ILI9881C initialization sequence
  tab5->dsi_write_command(LCD_CMD_SLPOUT, {}, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));

  // go through all the vendor specific init commands
  for (size_t i = 0;
       i < sizeof(vendor_specific_init_default) / sizeof(vendor_specific_init_default[0]); i++) {
    const ili9881c_lcd_init_cmd_t *cmd = &vendor_specific_init_default[i];
    err = esp_lcd_panel_io_tx_param(io, cmd->cmd, static_cast<const uint8_t *>(cmd->data),
                                    cmd->data_bytes);
    if (err != ESP_OK) {
      tab5->logger_.error("Failed to send ILI9881C init command 0x{:02X}: {}", cmd->cmd,
                          esp_err_to_name(err));
      return err;
    }
    if (cmd->delay_ms > 0) {
      std::this_thread::sleep_for(std::chrono::milliseconds(cmd->delay_ms));
    }
  }

  uint8_t cmd_val = 0;

  // madctl: MX, MY, RGB
  cmd_val |= LCD_CMD_BGR_BIT; // BGR order
  tab5->dsi_write_command(LCD_CMD_MADCTL, std::span<const uint8_t>(&cmd_val, 1),
                          0); // adjust as needed

  // Pixel format: RGB565
  cmd_val = 0x55; // 16-bit/pixel
  tab5->dsi_write_command(LCD_CMD_COLMOD, std::span<const uint8_t>(&cmd_val, 1), 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(10));

  // Display ON
  tab5->dsi_write_command(LCD_CMD_DISPON, {}, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // call the original panel init
  err = tab5->original_panel_init_(panel);

  return err;
}

esp_err_t M5StackTab5::lcd_disp_invert_color(esp_lcd_panel_t *panel, bool invert_color) {
  if (panel == nullptr)
    return ESP_ERR_INVALID_ARG;
  M5StackTab5 *tab5 = static_cast<M5StackTab5 *>(panel->user_data);
  if (tab5 == nullptr)
    return ESP_ERR_INVALID_ARG;
  // ILI9881C command to invert colors
  const uint8_t cmd = invert_color ? LCD_CMD_INVON : LCD_CMD_INVOFF;
  tab5->dsi_write_command(cmd, {}, 0);
  return ESP_OK;
}

esp_err_t M5StackTab5::lcd_disp_on_off(esp_lcd_panel_t *panel, bool on) {
  if (panel == nullptr)
    return ESP_ERR_INVALID_ARG;
  M5StackTab5 *tab5 = static_cast<M5StackTab5 *>(panel->user_data);
  if (tab5 == nullptr)
    return ESP_ERR_INVALID_ARG;
  tab5->brightness(on ? 100.0f : 0.0f);
  const uint8_t cmd = on ? LCD_CMD_DISPON : LCD_CMD_DISPOFF;
  tab5->dsi_write_command(cmd, {}, 0);
  return ESP_OK;
}

esp_err_t M5StackTab5::lcd_disp_sleep(esp_lcd_panel_t *panel, bool sleep) {
  if (panel == nullptr)
    return ESP_ERR_INVALID_ARG;
  M5StackTab5 *tab5 = static_cast<M5StackTab5 *>(panel->user_data);
  if (tab5 == nullptr)
    return ESP_ERR_INVALID_ARG;
  tab5->brightness(sleep ? 0.0f : 100.0f);
  const uint8_t cmd = sleep ? LCD_CMD_SLPIN : LCD_CMD_SLPOUT;
  tab5->dsi_write_command(cmd, {}, 0);
  std::this_thread::sleep_for(std::chrono::milliseconds(120));
  return ESP_OK;
}

bool M5StackTab5::initialize_lcd() {
  logger_.info("Initializing M5Stack Tab5 LCD (MIPI-DSI, ILI9881C, {}x{})", display_width_,
               display_height_);

  if (!ioexp_0x43_) {
    if (!initialize_io_expanders()) {
      logger_.error("Failed to init IO expanders for LCD reset");
      return false;
    }
  }

  // enable DSI PHY power
  static esp_ldo_channel_handle_t phy_pwr_chan = nullptr;
  {
    logger_.info("Acquiring MIPI DSI PHY power LDO channel");
    esp_ldo_channel_config_t phy_pwr_cfg{};
    memset(&phy_pwr_cfg, 0, sizeof(phy_pwr_cfg));
    static constexpr int MIPI_DSI_PHY_PWR_LDO_CHANNEL = 3;
    static constexpr int MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV = 2500;
    phy_pwr_cfg.chan_id = MIPI_DSI_PHY_PWR_LDO_CHANNEL;
    phy_pwr_cfg.voltage_mv = MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV;
    esp_err_t err = esp_ldo_acquire_channel(&phy_pwr_cfg, &phy_pwr_chan);
    if (err != ESP_OK) {
      logger_.error("Failed to acquire MIPI DSI PHY power LDO channel: {}", esp_err_to_name(err));
      return false;
    }
  }

  // Ensure panel reset sequence via IO expander
  lcd_reset(true);
  using namespace std::chrono_literals;
  std::this_thread::sleep_for(20ms);
  lcd_reset(false);
  std::this_thread::sleep_for(120ms);

  // Configure backlight PWM like esp-box
  if (!backlight_) {
    backlight_channel_configs_.push_back({.gpio = static_cast<size_t>(lcd_backlight_io),
                                          .channel = LEDC_CHANNEL_0,
                                          .timer = LEDC_TIMER_0,
                                          .duty = 0.0f,
                                          .speed_mode = LEDC_LOW_SPEED_MODE,
                                          .output_invert = !backlight_value});
    backlight_ = std::make_shared<Led>(Led::Config{.timer = LEDC_TIMER_0,
                                                   .frequency_hz = 5000,
                                                   .channels = backlight_channel_configs_,
                                                   .duty_resolution = LEDC_TIMER_10_BIT});

    // // for now we're just going to set the gpio
    // gpio_set_direction(lcd_backlight_io, GPIO_MODE_OUTPUT);
  }

  brightness(100.0f);

  // Create MIPI-DSI bus and DBI panel-IO
  if (lcd_handles_.mipi_dsi_bus == nullptr) {
    esp_lcd_dsi_bus_config_t bus_cfg{};
    memset(&bus_cfg, 0, sizeof(bus_cfg));
    bus_cfg.bus_id = 0;
    bus_cfg.num_data_lanes = 2; // Tab5 uses 4 lanes
    bus_cfg.phy_clk_src = MIPI_DSI_PHY_CLK_SRC_DEFAULT;
    bus_cfg.lane_bit_rate_mbps = 1000; // 720*1280 RGB565 60Hz ~= 884 Mbps, use 1 Gbps for safety
    logger_.info("Creating DSI bus with {} data lanes at {} Mbps", bus_cfg.num_data_lanes,
                 bus_cfg.lane_bit_rate_mbps);
    esp_err_t err = esp_lcd_new_dsi_bus(&bus_cfg, &lcd_handles_.mipi_dsi_bus);
    if (err != ESP_OK) {
      logger_.error("Failed to create DSI bus: {}", esp_err_to_name(err));
      return false;
    }
  }

  if (lcd_handles_.io == nullptr) {
    esp_lcd_dbi_io_config_t io_cfg{};
    memset(&io_cfg, 0, sizeof(io_cfg));
    io_cfg.virtual_channel = 0;
    io_cfg.lcd_cmd_bits = 8;
    io_cfg.lcd_param_bits = 8;
    logger_.info("Creating DSI DBI panel IO with {} cmd bits and {} param bits",
                 io_cfg.lcd_cmd_bits, io_cfg.lcd_param_bits);
    esp_err_t err = esp_lcd_new_panel_io_dbi(lcd_handles_.mipi_dsi_bus, &io_cfg, &lcd_handles_.io);
    if (err != ESP_OK) {
      logger_.error("Failed to create DSI DBI panel IO: {}", esp_err_to_name(err));
      return false;
    }
  }

  if (lcd_handles_.panel == nullptr) {
    esp_lcd_dpi_panel_config_t dpi_cfg{};
    memset(&dpi_cfg, 0, sizeof(dpi_cfg));
    dpi_cfg.dpi_clk_src = MIPI_DSI_DPI_CLK_SRC_DEFAULT;
    dpi_cfg.dpi_clock_freq_mhz = 80;
    dpi_cfg.virtual_channel = 0;
    // dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB888;
    dpi_cfg.pixel_format = LCD_COLOR_PIXEL_FORMAT_RGB565;
    dpi_cfg.num_fbs = 1;
    dpi_cfg.video_timing.h_size = 800;
    dpi_cfg.video_timing.v_size = 1280;
    dpi_cfg.video_timing.hsync_back_porch = 140;
    dpi_cfg.video_timing.hsync_pulse_width = 40;
    dpi_cfg.video_timing.hsync_front_porch = 40;
    dpi_cfg.video_timing.vsync_back_porch = 16; // 20 for ili9881
    dpi_cfg.video_timing.vsync_pulse_width = 4;
    dpi_cfg.video_timing.vsync_front_porch = 16; // 20 for ili9881

    // TODO:
    dpi_cfg.flags.use_dma2d = true;

    logger_.info("Creating MIPI DSI DPI panel with resolution {}x{}", dpi_cfg.video_timing.h_size,
                 dpi_cfg.video_timing.v_size);
    esp_err_t err = esp_lcd_new_panel_dpi(lcd_handles_.mipi_dsi_bus, &dpi_cfg, &lcd_handles_.panel);
    if (err != ESP_OK) {
      logger_.error("Failed to create MIPI DSI DPI panel: {}", esp_err_to_name(err));
      return false;
    }
  }

  // save the original functions
  original_panel_del_ = lcd_handles_.panel->del;
  original_panel_init_ = lcd_handles_.panel->init;

  // overwrite the functions of the MIPI DPI panel
  lcd_handles_.panel->init = lcd_disp_init;
  lcd_handles_.panel->reset = lcd_reset;
  lcd_handles_.panel->mirror = nullptr; // lcd_disp_mirror;
  lcd_handles_.panel->invert_color = lcd_disp_invert_color;
  lcd_handles_.panel->disp_on_off = lcd_disp_on_off;
  lcd_handles_.panel->disp_sleep = lcd_disp_sleep;
  lcd_handles_.panel->user_data = this;

  ESP_ERROR_CHECK(esp_lcd_panel_reset(lcd_handles_.panel));
  ESP_ERROR_CHECK(esp_lcd_panel_init(lcd_handles_.panel));
  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(lcd_handles_.panel, true));

  logger_.info("Register DPI panel event callback for LVGL flush ready notification");
  esp_lcd_dpi_panel_event_callbacks_t cbs = {
      .on_color_trans_done = &M5StackTab5::notify_lvgl_flush_ready,
      // .on_refresh_done = &M5StackTab5::monitor_refresh_rate,
  };
  ESP_ERROR_CHECK(esp_lcd_dpi_panel_register_event_callbacks(lcd_handles_.panel, &cbs, this));

  logger_.info("DSI bus and panel IO created successfully, starting DisplayDriver initialization");
  using namespace std::placeholders;
  DisplayDriver::initialize(espp::display_drivers::Config{
      .write_command = std::bind_front(&M5StackTab5::dsi_write_command, this),
      .lcd_send_lines = nullptr,       // std::bind_front(&M5StackTab5::dsi_write_lcd_lines, this),
      .reset_pin = GPIO_NUM_NC,        // reset handled via IO expander
      .data_command_pin = GPIO_NUM_NC, // DSI has no DC pin
      .reset_value = false,
      .invert_colors = invert_colors,
      .swap_color_order = swap_color_order,
      .offset_x = 0,
      .offset_y = 0,
      .swap_xy = swap_xy,
      .mirror_x = mirror_x,
      .mirror_y = mirror_y,
      .mirror_portrait = false,
  });

  logger_.info("LCD driver initialized (callbacks bound)");
  return true;
}

bool M5StackTab5::initialize_display(size_t pixel_buffer_size) {
  uint16_t *fbs[1];
  ESP_ERROR_CHECK(esp_lcd_dpi_panel_get_frame_buffer(lcd_handles_.panel, 1, (void **)&fbs[0]));

  logger_.info("Initializing LVGL display with pixel buffer size: {} pixels", pixel_buffer_size);
  if (!display_) {
    display_ = std::make_shared<Display<Pixel>>(
        Display<Pixel>::LvglConfig{.width = display_width_,
                                   .height = display_height_,
                                   // .flush_callback = std::bind_front(&M5StackTab5::flush, this),
                                   .flush_callback = DisplayDriver::flush,
                                   .rotation_callback = DisplayDriver::rotate,
                                   .rotation = rotation},
        Display<Pixel>::OledConfig{
            .set_brightness_callback = [this](float b) { this->brightness(b * 100.0f); },
            .get_brightness_callback = [this]() { return this->brightness() / 100.0f; }},
        Display<Pixel>::DynamicMemoryConfig{
            .pixel_buffer_size = pixel_buffer_size,
            .double_buffered = true,
            .allocation_flags = MALLOC_CAP_8BIT | MALLOC_CAP_DMA,
        },
        // typename Display<Pixel>::StaticMemoryConfig{.pixel_buffer_size = pixel_buffer_size,
        //                                             .vram0 = (Pixel*)fbs[0],
        //                                             .vram1 = nullptr},
        Logger::Verbosity::WARN);
  }

  // // set color depth
  // lv_display_set_color_format(lv_display_get_default(),
  //                             LV_COLOR_FORMAT_RGB565);

  logger_.info("LVGL display initialized");
  return true;
}

void M5StackTab5::brightness(float brightness) {
  brightness = std::clamp(brightness, 0.0f, 100.0f);
  if (backlight_) {
    backlight_->set_duty(LEDC_CHANNEL_0, brightness);
  } else {
    gpio_set_level(lcd_backlight_io, brightness > 0 ? 1 : 0);
  }
}

float M5StackTab5::brightness() const {
  if (backlight_) {
    auto maybe_duty = backlight_->get_duty(LEDC_CHANNEL_0);
    if (maybe_duty.has_value())
      return maybe_duty.value();
  }
  return gpio_get_level(lcd_backlight_io) ? 100.0f : 0.0f;
}

// -----------------
// DSI write helpers
// -----------------

void M5StackTab5::flush(lv_display_t *disp, const lv_area_t *area, uint8_t *px_map) {
  if (lcd_handles_.panel == nullptr) {
    logger_.error("Flush failed: no panel handle");
    return;
  }
  logger_.debug("Flush called for area ({},{})->({},{})", area->x1, area->y1, area->x2, area->y2);
  int offsetx1 = area->x1;
  int offsetx2 = area->x2;
  int offsety1 = area->y1;
  int offsety2 = area->y2;
  // pass the draw buffer to the driver
  esp_lcd_panel_draw_bitmap(lcd_handles_.panel, offsetx1, offsety1, offsetx2 + 1, offsety2 + 1,
                            px_map);
}

bool M5StackTab5::notify_lvgl_flush_ready(esp_lcd_panel_handle_t panel,
                                          esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx) {
  espp::M5StackTab5 *tab5 = static_cast<espp::M5StackTab5 *>(user_ctx);
  if (tab5 == nullptr) {
    fmt::print("\t\t ERROR: notify_lvgl_flush_ready: invalid user_ctx\n");
    return false;
  }
  tab5->logger_.debug("Notifying LVGL that flush is ready");
  tab5->display_->notify_flush_ready();
  return false;
}

void M5StackTab5::dsi_write_command(uint8_t cmd, std::span<const uint8_t> params,
                                    uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI write_command 0x{:02X} failed: no panel IO", cmd);
    return;
  }
  esp_lcd_panel_io_handle_t io = lcd_handles_.io;
  const void *data_ptr = params.data();
  size_t data_size = params.size();
  logger_.debug("DSI tx_param 0x{:02X} with {} bytes", cmd, data_size);
  esp_err_t err = esp_lcd_panel_io_tx_param(io, (int)cmd, data_ptr, data_size);
  if (err != ESP_OK) {
    logger_.error("DSI tx_param 0x{:02X} failed: {}", cmd, esp_err_to_name(err));
  }
}

void M5StackTab5::dsi_write_lcd_lines(int sx, int sy, int ex, int ey, const uint8_t *color_data,
                                      uint32_t /*flags*/) {
  if (!lcd_handles_.io) {
    logger_.error("DSI tx_color for area ({},{})->({},{}) failed: no panel IO", sx, sy, ex, ey);
    return;
  }
  esp_lcd_panel_io_handle_t io = lcd_handles_.io;

  // Calculate total number of pixels in
  // the area
  const int width = (ex - sx + 1);
  const int height = (ey - sy + 1);
  const size_t num_pixels = static_cast<size_t>(width) * static_cast<size_t>(height);

  logger_.debug("DSI tx_color for area ({},{})->({},{}) size={} px", sx, sy, ex, ey,
                width * height);

  // Ensure drawing area is set, then stream pixel data via RAMWR using panel IO color API
  lv_area_t area{
      .x1 = (lv_coord_t)sx, .y1 = (lv_coord_t)sy, .x2 = (lv_coord_t)ex, .y2 = (lv_coord_t)ey};
  DisplayDriver::set_drawing_area(&area);

  // RAMWR expects RGB565 little-endian words; DisplayDriver::fill already byte-swapped when needed
  esp_err_t err =
      esp_lcd_panel_io_tx_color(io, (int)DisplayDriver::Command::ramwr, color_data, num_pixels);
  if (err != ESP_OK) {
    logger_.error("DSI tx_color failed for area ({},{})->({},{}) size={} px: {}", sx, sy, ex, ey,
                  num_pixels, esp_err_to_name(err));
  }
}

} // namespace espp
