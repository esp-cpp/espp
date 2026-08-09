#pragma once

// Remote UI abstraction for the Android TV Remote example.
//
// The example can present its status / read the pairing code / drive the remote
// either over the serial console (default) or on a LilyGo T-Deck, using its
// screen + keyboard. Exactly one UI is compiled in, selected by a compile
// definition set from CMake (-DATV_BOARD=...):
//
//   (none)        -> ConsoleUi   (serial monitor)
//   ATV_UI_TDECK  -> LilyGo T-Deck (screen + keyboard)
//
// Board selection cannot be a pure Kconfig choice because it changes the set of
// components built (and the flash/PSRAM config), which IDF resolves before
// Kconfig. Hence the CMake flag + per-board sdkconfig.defaults.

#include <atomic>
#include <cctype>
#include <charconv>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <system_error>
#include <thread>
#include <vector>

#include "android_tv_remote.hpp"
#include "logger.hpp"

using AtvKey = espp::AndroidTvRemote::Key;
using AtvAction = espp::AndroidTvRemote::Action;

#if defined(ATV_UI_TDECK)
#define ATV_UI_BOARD 1
#endif

// WiFi credentials collected/loaded by the UI.
struct WifiCreds {
  std::string ssid;
  std::string password;
};

// A normalized input event, produced from whatever native input the board has.
enum class UiKey {
  None,
  Char, // printable character in `ch`
  Enter,
  Backspace,
  Up,
  Down,
  Left,
  Right,
  Quit,
};
struct UiEvent {
  UiKey key{UiKey::None};
  char ch{0};
};

// Interface the example drives regardless of the selected UI.
class RemoteUi {
public:
  virtual ~RemoteUi() = default;
  // Bring up hardware / console. Returns false on failure.
  virtual bool begin(espp::Logger &logger) = 0;
  // Obtain WiFi credentials to connect with (from saved config / Kconfig, or by
  // prompting the user). Blocks until available.
  virtual WifiCreds get_wifi_credentials() = 0;
  // Show a short status line to the user.
  virtual void status(const std::string &line) = 0;
  // Block until the user enters the 6-character pairing code shown on the TV,
  // returning it. std::nullopt if the user aborts / input is unavailable.
  virtual std::optional<std::string> read_code() = 0;
  // Interactively drive the remote until the user quits.
  virtual void control_loop(espp::AndroidTvRemote &remote) = 0;
};

// Maps a normalized UI event to a remote command. Returns false when the user
// requested to quit the control loop. Shared by every board UI.
inline bool atv_handle_control_event(espp::AndroidTvRemote &remote, const UiEvent &event,
                                     RemoteUi &ui) {
  std::error_code ec;
  UiKey key = event.key;
  char c = event.ch;
  // WASD double as the D-pad on keyboards without arrow keys.
  if (key == UiKey::Char) {
    switch (c) {
    case 'w': case 'W': key = UiKey::Up; break;
    case 'a': case 'A': key = UiKey::Left; break;
    case 's': case 'S': key = UiKey::Down; break;
    case 'd': case 'D': key = UiKey::Right; break;
    default: break;
    }
  }
  switch (key) {
  case UiKey::Up: remote.send_key(AtvKey::DpadUp, AtvAction::Short, ec); ui.status("Up"); break;
  case UiKey::Down: remote.send_key(AtvKey::DpadDown, AtvAction::Short, ec); ui.status("Down"); break;
  case UiKey::Left: remote.send_key(AtvKey::DpadLeft, AtvAction::Short, ec); ui.status("Left"); break;
  case UiKey::Right: remote.send_key(AtvKey::DpadRight, AtvAction::Short, ec); ui.status("Right"); break;
  case UiKey::Enter: remote.send_key(AtvKey::Enter, AtvAction::Short, ec); ui.status("Select"); break;
  case UiKey::Quit: return false;
  case UiKey::Char:
    switch (c) {
    case 'h': case 'H': remote.home(ec); ui.status("Home"); break;
    case 'b': case 'B': remote.back(ec); ui.status("Back"); break;
    case ' ': remote.media_play_pause(ec); ui.status("Play/Pause"); break;
    case ',': remote.volume_down(ec); ui.status("Volume-"); break;
    case '.': remote.volume_up(ec); ui.status("Volume+"); break;
    case 'n': case 'N': remote.media_next(ec); ui.status("Next"); break;
    case 'p': case 'P': remote.media_previous(ec); ui.status("Previous"); break;
    case 'm': case 'M': remote.mute(ec); ui.status("Mute"); break;
    case 'q': case 'Q': return false;
    default: break;
    }
    break;
  default: break;
  }
  if (ec)
    ui.status(std::string("command failed: ") + ec.message());
  return true;
}

// ===========================================================================
//  Console UI (default) -- serial monitor
// ===========================================================================
#if !defined(ATV_UI_BOARD)

#include <cstdio>

#include "sdkconfig.h"

#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"

// Install the VFS driver for the active ESP console so that fgetc(stdin) blocks
// for and receives interactive keystrokes (the pairing code typed into
// `idf.py monitor`). Without this, stdin has no backing driver and reads return
// EOF forever. Mirrors espp::Cli::configure_stdin_stdout() but inlined so the
// example does not depend on the cli component just to read a line.
inline void atv_configure_console() {
  using namespace std::chrono_literals;
  setvbuf(stdin, nullptr, _IONBF, 0);
#if CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG
  usb_serial_jtag_driver_config_t cfg = USB_SERIAL_JTAG_DRIVER_CONFIG_DEFAULT();
  usb_serial_jtag_driver_install(&cfg);
  usb_serial_jtag_vfs_use_driver();
  usb_serial_jtag_vfs_set_rx_line_endings(ESP_LINE_ENDINGS_CR);
  usb_serial_jtag_vfs_set_tx_line_endings(ESP_LINE_ENDINGS_CRLF);
#elif CONFIG_ESP_CONSOLE_UART
  const uart_port_t port = (uart_port_t)CONFIG_ESP_CONSOLE_UART_NUM;
  uart_config_t uart_config;
  memset(&uart_config, 0, sizeof(uart_config));
  uart_config.baud_rate = CONFIG_ESP_CONSOLE_UART_BAUDRATE;
  uart_config.data_bits = UART_DATA_8_BITS;
  uart_config.parity = UART_PARITY_DISABLE;
  uart_config.stop_bits = UART_STOP_BITS_1;
  uart_config.source_clk = UART_SCLK_DEFAULT;
  ESP_ERROR_CHECK(uart_driver_install(port, 256, 0, 0, NULL, 0));
  ESP_ERROR_CHECK(uart_param_config(port, &uart_config));
  uart_vfs_dev_use_driver(port);
  uart_vfs_dev_port_set_rx_line_endings(port, ESP_LINE_ENDINGS_CR);
  uart_vfs_dev_port_set_tx_line_endings(port, ESP_LINE_ENDINGS_CRLF);
#endif
}

// Block reading a single non-empty line from the serial console (stdin).
inline std::string atv_read_line_from_console() {
  using namespace std::chrono_literals;
  std::string line;
  while (true) {
    int c = fgetc(stdin);
    if (c == EOF) {
      clearerr(stdin);
      std::this_thread::sleep_for(20ms);
      continue;
    }
    if (c == '\r' || c == '\n') {
      if (line.empty())
        continue; // ignore leading newlines
      break;
    }
    line.push_back(static_cast<char>(c));
  }
  return line;
}

// Prompt for and read an in-range device index from the serial console.
inline size_t atv_read_index_from_console(espp::Logger &logger, size_t count) {
  logger.info("Multiple devices found. Enter the index [0-{}] to pair/connect, then press Enter:",
              count - 1);
  while (true) {
    std::string line = atv_read_line_from_console();
    size_t start = line.find_first_not_of(" \t");
    if (start != std::string::npos) {
      size_t index = 0;
      auto res = std::from_chars(line.data() + start, line.data() + line.size(), index);
      if (res.ec == std::errc() && index < count)
        return index;
    }
    logger.warn("Invalid selection '{}'; enter a number between 0 and {}", line, count - 1);
  }
}

class ConsoleUi : public RemoteUi {
public:
  bool begin(espp::Logger &logger) override {
    logger_ = &logger;
    atv_configure_console();
    return true;
  }
  WifiCreds get_wifi_credentials() override {
    // Console build uses the compile-time credentials (empty SSID falls back to
    // the credentials saved in NVS by a previous run).
    return {CONFIG_ANDROID_TV_REMOTE_EXAMPLE_WIFI_SSID,
            CONFIG_ANDROID_TV_REMOTE_EXAMPLE_WIFI_PASSWORD};
  }
  void status(const std::string &line) override {
    if (logger_)
      logger_->info("{}", line);
  }
  std::optional<std::string> read_code() override {
    if (logger_)
      logger_->info("Enter the 6-character pairing code shown on the TV, then press Enter:");
    return atv_read_line_from_console();
  }
  void control_loop(espp::AndroidTvRemote &remote) override {
    using namespace std::chrono_literals;
    // The serial console does not translate arrow keys cleanly, so run a short
    // scripted demonstration instead of an interactive loop.
    std::error_code ec;
    status("Connected; sending a scripted demo sequence (HOME, DPAD_DOWN, ENTER, play/pause)");
    remote.home(ec);
    std::this_thread::sleep_for(250ms);
    remote.send_key(AtvKey::DpadDown, AtvAction::Short, ec);
    std::this_thread::sleep_for(250ms);
    remote.send_key(AtvKey::Enter, AtvAction::Short, ec);
    std::this_thread::sleep_for(250ms);
    remote.media_play_pause(ec);
    status("Demo sequence complete");
  }

private:
  espp::Logger *logger_{nullptr};
};

inline std::unique_ptr<RemoteUi> atv_make_remote_ui() { return std::make_unique<ConsoleUi>(); }

#else // ATV_UI_BOARD -------------------------------------------------------

#include "lvgl.h"

#include "nvs.hpp"
#include "t-deck.hpp"
#include "task.hpp"

// Full-screen tabview GUI for the T-Deck, styled after the T-Deck BSP example:
// a "Setup" tab (WiFi entry), a "Remote" tab (status + pairing-code display +
// an on-screen D-pad / media buttons), and a "Log" tab. All LVGL access is
// guarded by a recursive mutex; a task pumps lv_task_handler(). The physical
// keyboard is routed to the active phase (WiFi field / pairing code / control).
class TDeckUi : public RemoteUi {
public:
  bool begin(espp::Logger &logger) override {
    logger_ = &logger;
    auto &board = espp::TDeck::get();
    if (!board.initialize_lcd())
      return false;
    if (!board.initialize_display(board.lcd_width() * 50))
      return false;
    if (!board.initialize_touch())
      return false;
    load_saved_creds();
    build_ui();
    start_pump();
    board.initialize_keyboard(true, [this](uint8_t key) {
      if (key != 0)
        on_key(key);
    });
    return true;
  }

  WifiCreds get_wifi_credentials() override {
    set_active_tab(TAB_SETUP);
    set_focus(0);
    set_label(setup_status_label_, saved_creds_.ssid.empty()
                                       ? "Enter WiFi, then press Connect (or Enter)"
                                       : "Press Connect to use the saved network, or edit it");
    input_mode_ = InputMode::Wifi;
    {
      std::unique_lock<std::mutex> lock(signal_mutex_);
      wifi_submitted_ = false;
      signal_cv_.wait(lock, [this] { return wifi_submitted_; });
    }
    input_mode_ = InputMode::Idle;
    WifiCreds creds;
    {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
      creds.ssid = text_of(ssid_ta_);
      creds.password = text_of(pass_ta_);
    }
    save_creds(creds);
    set_active_tab(TAB_REMOTE);
    return creds;
  }

  void status(const std::string &line) override {
    set_label(setup_status_label_, line);
    set_label(remote_status_label_, line);
    append_log(line);
  }

  std::optional<std::string> read_code() override {
    set_active_tab(TAB_REMOTE);
    set_label(remote_status_label_, "Pairing: type the code shown on the TV, then Enter");
    {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
      code_.clear();
      show_code_locked();
      lv_obj_remove_flag(code_label_, LV_OBJ_FLAG_HIDDEN);
    }
    input_mode_ = InputMode::Code;
    {
      std::unique_lock<std::mutex> lock(signal_mutex_);
      code_ready_ = false;
      signal_cv_.wait(lock, [this] { return code_ready_; });
    }
    input_mode_ = InputMode::Idle;
    std::string code;
    {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
      code = code_;
      lv_obj_add_flag(code_label_, LV_OBJ_FLAG_HIDDEN);
    }
    return code;
  }

  void control_loop(espp::AndroidTvRemote &remote) override {
    set_active_tab(TAB_REMOTE);
    set_label(remote_status_label_, "Connected. Use the buttons, keyboard, or WASD.");
    set_label(last_cmd_label_, "");
    set_remote_controls_enabled(true);
    // discard any button/keyboard events queued before we started listening
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.clear();
    }
    input_mode_ = InputMode::Control;
    while (true) {
      UiEvent event = wait_event();
      if (!atv_handle_control_event(remote, event, *this))
        break;
    }
    input_mode_ = InputMode::Idle;
    set_remote_controls_enabled(false);
    set_label(remote_status_label_, "Disconnected.");
  }

private:
  enum class InputMode { Idle, Wifi, Code, Control };
  enum Tab { TAB_SETUP = 0, TAB_REMOTE = 1, TAB_LOG = 2 };
  // Command carried by an on-screen remote button, stored in its user-data.
  enum class BtnCmd : intptr_t {
    None = 0, Up, Down, Left, Right, Ok, Home, Back, Play, VolDown, VolUp
  };

  // ---- input event queue (native callbacks -> control_loop) ----
  void push_event(const UiEvent &event) {
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      queue_.push_back(event);
    }
    queue_cv_.notify_one();
  }
  UiEvent wait_event() {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    queue_cv_.wait(lock, [this] { return !queue_.empty(); });
    UiEvent event = queue_.front();
    queue_.pop_front();
    return event;
  }

  // ---- keyboard routing (runs in the T-Deck keyboard task) ----
  void on_key(uint8_t key) {
    switch (input_mode_.load()) {
    case InputMode::Wifi: on_key_wifi(key); break;
    case InputMode::Code: on_key_code(key); break;
    case InputMode::Control: on_key_control(key); break;
    case InputMode::Idle: break;
    }
  }

  void on_key_wifi(uint8_t key) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    lv_obj_t *ta = focused_field_ == 0 ? ssid_ta_ : pass_ta_;
    if (key == '\r' || key == '\n') {
      if (focused_field_ == 0)
        set_focus_locked(1); // advance SSID -> password
      else
        submit_wifi();
    } else if (key == 8 || key == 0x7f) {
      lv_textarea_delete_char(ta);
    } else if (key >= 0x20 && key < 0x7f) {
      lv_textarea_add_char(ta, key);
    }
  }

  void on_key_code(uint8_t key) {
    if (key == '\r' || key == '\n') {
      bool ready = false;
      {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
        ready = code_.size() == 6;
      }
      if (ready) {
        {
          std::lock_guard<std::mutex> lock(signal_mutex_);
          code_ready_ = true;
        }
        signal_cv_.notify_one();
      }
      return;
    }
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    if (key == 8 || key == 0x7f) {
      if (!code_.empty())
        code_.pop_back();
      show_code_locked();
    } else if (is_hex(key) && code_.size() < 6) {
      code_.push_back(static_cast<char>(std::toupper(key)));
      show_code_locked();
    }
  }

  void on_key_control(uint8_t key) {
    if (key == '\r' || key == '\n')
      push_event({UiKey::Enter, 0});
    else if (key >= 0x20 && key < 0x7f)
      push_event({UiKey::Char, static_cast<char>(key)});
  }

  static bool is_hex(uint8_t c) {
    return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f') || (c >= 'A' && c <= 'F');
  }

  // ---- WiFi credential persistence (NVS) ----
  static std::string trim_nul(std::string value) {
    value.erase(value.find_last_not_of('\0') + 1);
    return value;
  }
  void load_saved_creds() {
    std::error_code ec;
    espp::NvsHandle handle(kWifiNamespace, ec);
    if (ec)
      return;
    std::string ssid, pass;
    std::error_code read_ec;
    handle.get(std::string("ssid"), ssid, read_ec);
    handle.get(std::string("pass"), pass, read_ec);
    saved_creds_.ssid = trim_nul(ssid);
    saved_creds_.password = trim_nul(pass);
  }
  void save_creds(const WifiCreds &creds) {
    saved_creds_ = creds;
    std::error_code ec;
    espp::NvsHandle handle(kWifiNamespace, ec);
    if (ec)
      return;
    handle.set(std::string("ssid"), creds.ssid, ec);
    handle.set(std::string("pass"), creds.password, ec);
    handle.commit(ec);
  }

  // ---- UI construction ----
  void build_ui() {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    tabview_ = lv_tabview_create(lv_screen_active());
    lv_tabview_set_tab_bar_position(tabview_, LV_DIR_TOP);
    lv_tabview_set_tab_bar_size(tabview_, 36);
    lv_obj_set_size(tabview_, lv_display_get_horizontal_resolution(lv_display_get_default()),
                    lv_display_get_vertical_resolution(lv_display_get_default()));
    lv_obj_t *setup_tab = lv_tabview_add_tab(tabview_, LV_SYMBOL_WIFI " Setup");
    lv_obj_t *remote_tab = lv_tabview_add_tab(tabview_, LV_SYMBOL_KEYBOARD " Remote");
    lv_obj_t *log_tab = lv_tabview_add_tab(tabview_, LV_SYMBOL_LIST " Log");
    lv_obj_clear_flag(lv_tabview_get_content(tabview_), LV_OBJ_FLAG_SCROLLABLE);
    build_setup_tab(setup_tab);
    build_remote_tab(remote_tab);
    build_log_tab(log_tab);
  }

  void build_setup_tab(lv_obj_t *tab) {
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(tab, 8, 0);

    lv_obj_t *title = lv_label_create(tab);
    lv_label_set_text(title, "WiFi network");

    ssid_ta_ = lv_textarea_create(tab);
    lv_textarea_set_one_line(ssid_ta_, true);
    lv_textarea_set_placeholder_text(ssid_ta_, "SSID");
    lv_textarea_set_text(ssid_ta_, saved_creds_.ssid.c_str());
    lv_obj_set_width(ssid_ta_, lv_pct(100));
    lv_obj_add_event_cb(ssid_ta_, &TDeckUi::event_cb, LV_EVENT_CLICKED, this);

    pass_ta_ = lv_textarea_create(tab);
    lv_textarea_set_one_line(pass_ta_, true);
    lv_textarea_set_password_mode(pass_ta_, true);
    lv_textarea_set_placeholder_text(pass_ta_, "Password");
    lv_textarea_set_text(pass_ta_, saved_creds_.password.c_str());
    lv_obj_set_width(pass_ta_, lv_pct(100));
    lv_obj_add_event_cb(pass_ta_, &TDeckUi::event_cb, LV_EVENT_CLICKED, this);

    connect_button_ = lv_btn_create(tab);
    lv_obj_t *connect_label = lv_label_create(connect_button_);
    lv_label_set_text(connect_label, LV_SYMBOL_WIFI " Connect");
    lv_obj_center(connect_label);
    lv_obj_add_event_cb(connect_button_, &TDeckUi::event_cb, LV_EVENT_CLICKED, this);

    lv_obj_t *hint = lv_label_create(tab);
    lv_label_set_long_mode(hint, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(hint, lv_pct(100));
    lv_label_set_text(hint, "Tap a field or press Enter to move SSID -> password -> Connect.");

    setup_status_label_ = lv_label_create(tab);
    lv_label_set_long_mode(setup_status_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(setup_status_label_, lv_pct(100));
    lv_label_set_text(setup_status_label_, "");
    set_focus_locked(0);
  }

  lv_obj_t *make_remote_button(lv_obj_t *parent, const char *text, BtnCmd cmd, int w, int h) {
    lv_obj_t *button = lv_btn_create(parent);
    lv_obj_set_size(button, w, h);
    lv_obj_set_user_data(button, reinterpret_cast<void *>(static_cast<intptr_t>(cmd)));
    lv_obj_add_event_cb(button, &TDeckUi::event_cb, LV_EVENT_CLICKED, this);
    lv_obj_t *label = lv_label_create(button);
    lv_label_set_text(label, text);
    lv_obj_center(label);
    remote_buttons_.push_back(button);
    return button;
  }

  void build_remote_tab(lv_obj_t *tab) {
    lv_obj_set_flex_flow(tab, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(tab, 6, 0);

    remote_status_label_ = lv_label_create(tab);
    lv_label_set_long_mode(remote_status_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(remote_status_label_, lv_pct(100));
    lv_label_set_text(remote_status_label_, "Not connected.");

    code_label_ = lv_label_create(tab);
    lv_obj_set_style_text_color(code_label_, lv_palette_main(LV_PALETTE_AMBER), 0);
    lv_label_set_text(code_label_, "");
    lv_obj_add_flag(code_label_, LV_OBJ_FLAG_HIDDEN);

    // D-pad: a fixed-size container with the arrows around a center OK.
    lv_obj_t *dpad = lv_obj_create(tab);
    lv_obj_remove_style_all(dpad);
    lv_obj_set_size(dpad, 150, 116);
    const int bw = 46, bh = 34;
    make_remote_button(dpad, LV_SYMBOL_UP, BtnCmd::Up, bw, bh);
    lv_obj_align(remote_buttons_.back(), LV_ALIGN_TOP_MID, 0, 0);
    make_remote_button(dpad, LV_SYMBOL_LEFT, BtnCmd::Left, bw, bh);
    lv_obj_align(remote_buttons_.back(), LV_ALIGN_LEFT_MID, 0, 0);
    make_remote_button(dpad, LV_SYMBOL_OK, BtnCmd::Ok, bw, bh);
    lv_obj_align(remote_buttons_.back(), LV_ALIGN_CENTER, 0, 0);
    make_remote_button(dpad, LV_SYMBOL_RIGHT, BtnCmd::Right, bw, bh);
    lv_obj_align(remote_buttons_.back(), LV_ALIGN_RIGHT_MID, 0, 0);
    make_remote_button(dpad, LV_SYMBOL_DOWN, BtnCmd::Down, bw, bh);
    lv_obj_align(remote_buttons_.back(), LV_ALIGN_BOTTOM_MID, 0, 0);

    // Media / navigation row.
    lv_obj_t *media_row = lv_obj_create(tab);
    lv_obj_remove_style_all(media_row);
    lv_obj_set_size(media_row, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(media_row, LV_FLEX_FLOW_ROW_WRAP);
    lv_obj_set_style_pad_column(media_row, 6, 0);
    lv_obj_set_style_pad_row(media_row, 6, 0);
    make_remote_button(media_row, LV_SYMBOL_HOME, BtnCmd::Home, 50, 34);
    make_remote_button(media_row, LV_SYMBOL_LEFT " Back", BtnCmd::Back, 70, 34);
    make_remote_button(media_row, LV_SYMBOL_PLAY, BtnCmd::Play, 50, 34);
    make_remote_button(media_row, LV_SYMBOL_VOLUME_MID, BtnCmd::VolDown, 50, 34);
    make_remote_button(media_row, LV_SYMBOL_VOLUME_MAX, BtnCmd::VolUp, 50, 34);

    last_cmd_label_ = lv_label_create(tab);
    lv_obj_set_width(last_cmd_label_, lv_pct(100));
    lv_label_set_text(last_cmd_label_, "");
    set_remote_controls_enabled(false);
  }

  void build_log_tab(lv_obj_t *tab) {
    lv_obj_set_style_pad_all(tab, 4, 0);
    log_label_ = lv_label_create(tab);
    lv_label_set_long_mode(log_label_, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(log_label_, lv_pct(100));
    lv_label_set_text(log_label_, "");
  }

  // ---- LVGL event trampoline ----
  static void event_cb(lv_event_t *event) {
    auto *self = static_cast<TDeckUi *>(lv_event_get_user_data(event));
    if (self)
      self->on_event(event);
  }
  void on_event(lv_event_t *event) {
    if (lv_event_get_code(event) != LV_EVENT_CLICKED)
      return;
    auto *target = static_cast<lv_obj_t *>(lv_event_get_target(event));
    if (target == ssid_ta_ || target == pass_ta_) {
      set_focus(target == ssid_ta_ ? 0 : 1);
      return;
    }
    if (target == connect_button_) {
      std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
      submit_wifi();
      return;
    }
    // Otherwise it is a remote-control button; act only while in control mode.
    if (input_mode_.load() != InputMode::Control)
      return;
    auto cmd = static_cast<BtnCmd>(reinterpret_cast<intptr_t>(lv_obj_get_user_data(target)));
    switch (cmd) {
    case BtnCmd::Up: push_event({UiKey::Up, 0}); break;
    case BtnCmd::Down: push_event({UiKey::Down, 0}); break;
    case BtnCmd::Left: push_event({UiKey::Left, 0}); break;
    case BtnCmd::Right: push_event({UiKey::Right, 0}); break;
    case BtnCmd::Ok: push_event({UiKey::Enter, 0}); break;
    case BtnCmd::Home: push_event({UiKey::Char, 'h'}); break;
    case BtnCmd::Back: push_event({UiKey::Char, 'b'}); break;
    case BtnCmd::Play: push_event({UiKey::Char, ' '}); break;
    case BtnCmd::VolDown: push_event({UiKey::Char, ','}); break;
    case BtnCmd::VolUp: push_event({UiKey::Char, '.'}); break;
    case BtnCmd::None: break;
    }
  }

  // submit_wifi() must be called with lvgl_mutex_ held (it does not read the
  // text areas -- get_wifi_credentials() does that after being woken).
  void submit_wifi() {
    {
      std::lock_guard<std::mutex> lock(signal_mutex_);
      wifi_submitted_ = true;
    }
    signal_cv_.notify_one();
  }

  // ---- small LVGL helpers (all lock lvgl_mutex_ unless noted _locked) ----
  static std::string text_of(lv_obj_t *ta) {
    const char *t = lv_textarea_get_text(ta);
    return t ? t : "";
  }
  void set_label(lv_obj_t *label, const std::string &text) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    if (label)
      lv_label_set_text(label, text.c_str());
  }
  void set_active_tab(int tab) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    if (tabview_)
      lv_tabview_set_active(tabview_, tab, LV_ANIM_OFF);
  }
  void set_focus(int field) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    set_focus_locked(field);
  }
  void set_focus_locked(int field) {
    focused_field_ = field;
    auto style = [](lv_obj_t *ta, bool on) {
      lv_obj_set_style_border_width(ta, on ? 2 : 1, 0);
      lv_obj_set_style_border_color(
          ta, on ? lv_palette_main(LV_PALETTE_BLUE) : lv_palette_main(LV_PALETTE_GREY), 0);
    };
    if (ssid_ta_)
      style(ssid_ta_, field == 0);
    if (pass_ta_)
      style(pass_ta_, field == 1);
  }
  void show_code_locked() {
    std::string shown = "Code: ";
    for (size_t i = 0; i < 6; i++) {
      shown += i < code_.size() ? code_[i] : '_';
      shown += ' ';
    }
    lv_label_set_text(code_label_, shown.c_str());
  }
  void set_remote_controls_enabled(bool enabled) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    for (lv_obj_t *button : remote_buttons_) {
      if (enabled)
        lv_obj_clear_state(button, LV_STATE_DISABLED);
      else
        lv_obj_add_state(button, LV_STATE_DISABLED);
    }
  }
  void append_log(const std::string &line) {
    std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
    log_lines_.emplace_front(line);
    while (log_lines_.size() > kMaxLogLines)
      log_lines_.pop_back();
    if (!log_label_)
      return;
    std::string text;
    for (const auto &l : log_lines_) {
      if (!text.empty())
        text += "\n";
      text += l;
    }
    lv_label_set_text(log_label_, text.c_str());
  }

  void start_pump() {
    using namespace std::chrono_literals;
    espp::Task::callback_m_cv_fn callback = [this](std::mutex &m,
                                                   std::condition_variable &cv) -> bool {
      {
        std::lock_guard<std::recursive_mutex> lock(lvgl_mutex_);
        lv_task_handler();
      }
      std::unique_lock<std::mutex> lock(m);
      cv.wait_for(lock, 16ms);
      return false; // keep running
    };
    // The tabview (nested containers + flex) needs a deeper stack than a flat
    // UI. Keep the priority low (default) so it never starves the TLS handshake
    // running on the main task.
    pump_task_ = espp::Task::make_unique(espp::Task::Config{
        .callback = callback,
        .task_config = {.name = "lvgl", .stack_size_bytes = 12 * 1024},
    });
    pump_task_->start();
  }

  static constexpr const char *kWifiNamespace = "atv_wifi";
  static constexpr size_t kMaxLogLines = 12;

  espp::Logger *logger_{nullptr};
  std::recursive_mutex lvgl_mutex_;
  std::unique_ptr<espp::Task> pump_task_;

  // LVGL objects
  lv_obj_t *tabview_{nullptr};
  lv_obj_t *ssid_ta_{nullptr};
  lv_obj_t *pass_ta_{nullptr};
  lv_obj_t *connect_button_{nullptr};
  lv_obj_t *setup_status_label_{nullptr};
  lv_obj_t *remote_status_label_{nullptr};
  lv_obj_t *code_label_{nullptr};
  lv_obj_t *last_cmd_label_{nullptr};
  lv_obj_t *log_label_{nullptr};
  std::vector<lv_obj_t *> remote_buttons_;

  int focused_field_{0};
  std::string code_;            // guarded by lvgl_mutex_
  std::deque<std::string> log_lines_; // guarded by lvgl_mutex_
  WifiCreds saved_creds_;

  std::atomic<InputMode> input_mode_{InputMode::Idle};

  // control-loop event queue
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::deque<UiEvent> queue_;

  // wifi-submit / code-ready signalling
  std::mutex signal_mutex_;
  std::condition_variable signal_cv_;
  bool wifi_submitted_{false};
  bool code_ready_{false};
};

inline std::unique_ptr<RemoteUi> atv_make_remote_ui() { return std::make_unique<TDeckUi>(); }

#endif // ATV_UI_BOARD
