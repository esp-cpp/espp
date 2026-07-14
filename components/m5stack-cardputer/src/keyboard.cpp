#include "m5stack-cardputer.hpp"

#include <esp_rom_sys.h>

using namespace espp;

//////////////////////////////
// Variant / internal I2C   //
//////////////////////////////

M5StackCardputer::Variant M5StackCardputer::variant() {
  if (!variant_detected_) {
    detect_variant();
  }
  return variant_;
}

I2c *M5StackCardputer::internal_i2c() {
  if (!variant_detected_) {
    detect_variant();
  }
  return internal_i2c_.get();
}

void M5StackCardputer::detect_variant() {
  if (variant_detected_) {
    return;
  }
  // The ADV has an internal I2C bus on GPIO 8/9 hosting its TCA8418 keyboard
  // controller; the original repurposes those pins as 74HC138 address lines.
  // Probe for the TCA8418 to tell them apart.
  internal_i2c_ = std::make_unique<I2c>(I2c::Config{
      .port = internal_i2c_port,
      .sda_io_num = internal_i2c_sda,
      .scl_io_num = internal_i2c_scl,
      .sda_pullup_en = GPIO_PULLUP_ENABLE,
      .scl_pullup_en = GPIO_PULLUP_ENABLE,
      .clk_speed = internal_i2c_clock_speed,
      .log_level = get_log_level(),
  });
  if (internal_i2c_->probe_device(tca8418_address)) {
    variant_ = Variant::ADV;
    logger_.info("Detected Cardputer ADV (TCA8418 keyboard controller found)");
  } else {
    variant_ = Variant::ORIGINAL;
    logger_.info("Detected original Cardputer (no TCA8418 on GPIO 8/9)");
    // release the bus and return GPIO 8/9 to plain GPIO so the keyboard
    // matrix scanner can drive them as 74HC138 address lines
    internal_i2c_.reset();
    gpio_reset_pin(internal_i2c_sda);
    gpio_reset_pin(internal_i2c_scl);
  }
  variant_detected_ = true;
}

////////////////////////
// Keyboard Functions //
////////////////////////

bool M5StackCardputer::initialize_keyboard(const keypress_callback_t &callback,
                                           std::chrono::milliseconds poll_interval,
                                           const espp::Task::BaseConfig &task_config) {
  logger_.info("Initializing keyboard");
  if (keyboard_initialized_) {
    logger_.warn("Keyboard already initialized, not initializing again!");
    return false;
  }

  keypress_callback_ = callback;
  keyboard_poll_interval_ = poll_interval;

  bool ok =
      (variant() == Variant::ADV) ? initialize_keyboard_tca8418() : initialize_keyboard_matrix();
  if (!ok) {
    return false;
  }

  using namespace std::placeholders;
  keyboard_task_ = espp::Task::make_unique({
      .callback = std::bind(&M5StackCardputer::keyboard_task_callback, this, _1, _2, _3),
      .task_config = task_config,
  });

  keyboard_initialized_ = true;

  return keyboard_task_->start();
}

bool M5StackCardputer::initialize_keyboard_matrix() {
  // Reset the pins first to detach them from any peripheral signals and
  // select the plain-GPIO IOMUX function, then configure them with
  // gpio_config(). The demultiplexer address lines are outputs; the sense
  // lines are inputs with pullups (keys read active low).
  uint64_t output_mask = 0;
  for (auto gpio : {keyboard_a0_io, keyboard_a1_io, keyboard_a2_io}) {
    gpio_reset_pin(gpio);
    output_mask |= (1ULL << gpio);
  }
  gpio_config_t output_config{
      .pin_bit_mask = output_mask,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&output_config);
  for (auto gpio : {keyboard_a0_io, keyboard_a1_io, keyboard_a2_io}) {
    gpio_set_level(gpio, 0);
  }

  uint64_t input_mask = 0;
  for (auto gpio : keyboard_input_ios) {
    gpio_reset_pin(gpio);
    input_mask |= (1ULL << gpio);
  }
  gpio_config_t input_config{
      .pin_bit_mask = input_mask,
      .mode = GPIO_MODE_INPUT,
      .pull_up_en = GPIO_PULLUP_ENABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  gpio_config(&input_config);
  return true;
}

bool M5StackCardputer::initialize_keyboard_tca8418() {
  if (!internal_i2c_) {
    logger_.error("Internal I2C bus not available, cannot initialize TCA8418");
    return false;
  }
  auto write_reg = [this](uint8_t reg, uint8_t value) {
    uint8_t buffer[2] = {reg, value};
    return internal_i2c_->write(tca8418_address, buffer, 2);
  };
  // configure the keypad matrix: rows 0-6 and columns 0-7 (7x8 = 56 keys)
  if (!write_reg(TCA8418_REG_KP_GPIO_1, 0x7F) || !write_reg(TCA8418_REG_KP_GPIO_2, 0xFF) ||
      !write_reg(TCA8418_REG_KP_GPIO_3, 0x00)) {
    logger_.error("Failed to configure the TCA8418 keypad matrix");
    return false;
  }
  // drain any stale events from the FIFO (it holds at most 10), then clear
  // all pending interrupt flags
  uint8_t event = 0;
  int guard = 0;
  do {
    if (!internal_i2c_->read_at_register(tca8418_address, TCA8418_REG_KEY_EVENT_A, &event, 1)) {
      break;
    }
  } while (event != 0 && ++guard < 16);
  if (!write_reg(TCA8418_REG_INT_STAT, 0xFF)) {
    logger_.error("Failed to clear the TCA8418 interrupt status");
    return false;
  }
  // enable key event processing / interrupt generation
  if (!write_reg(TCA8418_REG_CFG, 0x01)) {
    logger_.error("Failed to enable TCA8418 key event processing");
    return false;
  }
  return true;
}

bool M5StackCardputer::keyboard_task_callback(std::mutex &m, std::condition_variable &cv,
                                              bool &task_notified) {
  if (variant_ == Variant::ADV) {
    process_tca8418_events();
  } else {
    scan_keyboard_matrix();
  }

  // sleep for the poll interval, unless we're notified to stop
  {
    std::unique_lock<std::mutex> lock(m);
    cv.wait_for(lock, keyboard_poll_interval_, [&task_notified] { return task_notified; });
  }
  return false; // don't stop the task
}

void M5StackCardputer::scan_keyboard_matrix() {
  // Scan the matrix: for each of the 8 demultiplexer states, one (active low)
  // 74HC138 output selects a scan line and each of the 7 sense inputs reads
  // one key on it. Demux state i and input index j map to matrix coordinates:
  //   col = 2*j     when i > 3, else 2*j + 1
  //   row = 3 - (i > 3 ? i - 4 : i)
  std::array<uint16_t, KEYBOARD_ROWS> new_state{};
  for (int i = 0; i < 8; i++) {
    gpio_set_level(keyboard_a0_io, (i >> 0) & 1);
    gpio_set_level(keyboard_a1_io, (i >> 1) & 1);
    gpio_set_level(keyboard_a2_io, (i >> 2) & 1);
    // let the demultiplexer output and the (pullup-only) sense lines settle
    esp_rom_delay_us(20);
    int row = 3 - ((i > 3) ? (i - 4) : i);
    for (size_t j = 0; j < keyboard_input_ios.size(); j++) {
      bool pressed = gpio_get_level(keyboard_input_ios[j]) == 0;
      if (pressed) {
        int col = (i > 3) ? (2 * j) : (2 * j + 1);
        new_state[row] |= (1 << col);
      }
    }
  }

  std::array<uint16_t, KEYBOARD_ROWS> old_state;
  {
    std::lock_guard<std::mutex> lock(keyboard_state_mutex_);
    old_state = keyboard_state_;
    keyboard_state_ = new_state;
  }

  if (new_state != old_state) {
    logger_.debug("keyboard state: {:#06x} {:#06x} {:#06x} {:#06x}", new_state[0], new_state[1],
                  new_state[2], new_state[3]);
  }

  if (!keypress_callback_) {
    return;
  }
  // resolve the modifier state from the new scan, then report each key that
  // changed state
  Modifiers mods{.fn = (new_state[fn_row] & (1 << fn_col)) != 0,
                 .shift = (new_state[shift_row] & (1 << shift_col)) != 0,
                 .ctrl = (new_state[ctrl_row] & (1 << ctrl_col)) != 0,
                 .opt = (new_state[opt_row] & (1 << opt_col)) != 0,
                 .alt = (new_state[alt_row] & (1 << alt_col)) != 0};
  for (uint8_t row = 0; row < KEYBOARD_ROWS; row++) {
    uint16_t changed = new_state[row] ^ old_state[row];
    if (!changed) {
      continue;
    }
    for (uint8_t col = 0; col < KEYBOARD_COLS; col++) {
      if (!(changed & (1 << col))) {
        continue;
      }
      bool pressed = (new_state[row] & (1 << col)) != 0;
      emit_key_event(row, col, pressed, mods);
    }
  }
}

void M5StackCardputer::process_tca8418_events() {
  // Drain the TCA8418 key event FIFO. Each event byte: bit 7 = 1 for press /
  // 0 for release; bits 6:0 = key number k. The controller numbers keys by
  // its own 7x8 (row Tr, col Tc) matrix as k = 10*Tr + Tc + 1, which maps to
  // the logical 4x14 grid (same layout as the original) as:
  //   col = Tr * 2 + (Tc > 3 ? 1 : 0);  row = Tc % 4
  bool any_events = false;
  while (true) {
    uint8_t event = 0;
    if (!internal_i2c_->read_at_register(tca8418_address, TCA8418_REG_KEY_EVENT_A, &event, 1)) {
      logger_.error("Failed to read the TCA8418 key event FIFO");
      break;
    }
    if (event == 0) {
      break;
    }
    any_events = true;
    bool pressed = (event & 0x80) != 0;
    uint8_t key_number = (event & 0x7F);
    if (key_number == 0) {
      logger_.warn("Ignoring TCA8418 event with invalid key number 0");
      continue;
    }
    uint8_t n = key_number - 1;
    uint8_t tr = n / 10;
    uint8_t tc = n % 10;
    uint8_t col = tr * 2 + ((tc > 3) ? 1 : 0);
    uint8_t row = tc % 4;
    if (row >= KEYBOARD_ROWS || col >= KEYBOARD_COLS) {
      logger_.warn("Ignoring TCA8418 event with out-of-range key number {}", key_number);
      continue;
    }
    Modifiers mods;
    {
      std::lock_guard<std::mutex> lock(keyboard_state_mutex_);
      if (pressed) {
        keyboard_state_[row] |= (1 << col);
      } else {
        keyboard_state_[row] &= ~(1 << col);
      }
      mods = {.fn = (keyboard_state_[fn_row] & (1 << fn_col)) != 0,
              .shift = (keyboard_state_[shift_row] & (1 << shift_col)) != 0,
              .ctrl = (keyboard_state_[ctrl_row] & (1 << ctrl_col)) != 0,
              .opt = (keyboard_state_[opt_row] & (1 << opt_col)) != 0,
              .alt = (keyboard_state_[alt_row] & (1 << alt_col)) != 0};
    }
    emit_key_event(row, col, pressed, mods);
  }
  if (any_events) {
    // clear the key-event interrupt flag so the INT line releases
    uint8_t buffer[2] = {TCA8418_REG_INT_STAT, 0x01};
    internal_i2c_->write(tca8418_address, buffer, 2);
  }
}

void M5StackCardputer::emit_key_event(uint8_t row, uint8_t col, bool pressed,
                                      const Modifiers &modifiers) {
  logger_.debug("key ({}, {}) {}", row, col, pressed ? "pressed" : "released");
  if (!keypress_callback_) {
    return;
  }
  KeyEvent event{.row = row,
                 .col = col,
                 .pressed = pressed,
                 .value = key_value(row, col, modifiers),
                 .special = modifiers.fn ? special_key(row, col) : SpecialKey::NONE,
                 .modifiers = modifiers};
  keypress_callback_(event);
}

std::array<uint16_t, M5StackCardputer::KEYBOARD_ROWS> M5StackCardputer::keyboard_state() const {
  std::lock_guard<std::mutex> lock(keyboard_state_mutex_);
  return keyboard_state_;
}

bool M5StackCardputer::is_key_pressed(uint8_t row, uint8_t col) const {
  if (row >= KEYBOARD_ROWS || col >= KEYBOARD_COLS) {
    return false;
  }
  std::lock_guard<std::mutex> lock(keyboard_state_mutex_);
  return (keyboard_state_[row] & (1 << col)) != 0;
}

M5StackCardputer::Modifiers M5StackCardputer::modifiers() const {
  std::lock_guard<std::mutex> lock(keyboard_state_mutex_);
  return {.fn = (keyboard_state_[fn_row] & (1 << fn_col)) != 0,
          .shift = (keyboard_state_[shift_row] & (1 << shift_col)) != 0,
          .ctrl = (keyboard_state_[ctrl_row] & (1 << ctrl_col)) != 0,
          .opt = (keyboard_state_[opt_row] & (1 << opt_col)) != 0,
          .alt = (keyboard_state_[alt_row] & (1 << alt_col)) != 0};
}

char M5StackCardputer::key_value(uint8_t row, uint8_t col, const Modifiers &modifiers) {
  if (row >= KEYBOARD_ROWS || col >= KEYBOARD_COLS) {
    return 0;
  }
  const auto &entry = key_map_[row][col];
  return modifiers.shift ? entry.shifted : entry.value;
}

M5StackCardputer::SpecialKey M5StackCardputer::special_key(uint8_t row, uint8_t col) {
  if (row >= KEYBOARD_ROWS || col >= KEYBOARD_COLS) {
    return SpecialKey::NONE;
  }
  return key_map_[row][col].special;
}

const char *M5StackCardputer::special_key_name(SpecialKey key) {
  switch (key) {
  case SpecialKey::NONE:
    return "None";
  case SpecialKey::ESC:
    return "Esc";
  case SpecialKey::F1:
    return "F1";
  case SpecialKey::F2:
    return "F2";
  case SpecialKey::F3:
    return "F3";
  case SpecialKey::F4:
    return "F4";
  case SpecialKey::F5:
    return "F5";
  case SpecialKey::F6:
    return "F6";
  case SpecialKey::F7:
    return "F7";
  case SpecialKey::F8:
    return "F8";
  case SpecialKey::F9:
    return "F9";
  case SpecialKey::F10:
    return "F10";
  case SpecialKey::F11:
    return "F11";
  case SpecialKey::F12:
    return "F12";
  case SpecialKey::DELETE:
    return "Delete";
  case SpecialKey::UP:
    return "Up";
  case SpecialKey::DOWN:
    return "Down";
  case SpecialKey::LEFT:
    return "Left";
  case SpecialKey::RIGHT:
    return "Right";
  default:
    return "Unknown";
  }
}
