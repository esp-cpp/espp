#pragma once

#include <algorithm>
#include <array>
#include <atomic>

#include "driver/gpio.h"
#include "driver/mcpwm_prelude.h"

#include "base_component.hpp"

namespace espp {
/**
 * @brief Interface for controlling the 6 PWMs (high/low sides for phases
 *        A,B,C) of a brushless dc motor (BLDC motor). Wraps around the ESP32
 *        https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/mcpwm.html
 *        peripheral.
 */
class BldcDriver : public BaseComponent {
public:
  static constexpr size_t TIMER_RESOLUTION_HZ = 80 * 1000 * 1000;
  static constexpr size_t FREQUENCY_HZ = 20 * 1000;
  static constexpr size_t TICKS_PER_PERIOD = (TIMER_RESOLUTION_HZ / FREQUENCY_HZ);

  /**
   *  @brief Configure the driver for 6-channel control of a BLDC.
   */
  struct Config {
    int gpio_a_h;               /**< Phase A high side gpio. */
    int gpio_a_l;               /**< Phase A low side gpio. */
    int gpio_b_h;               /**< Phase B high side gpio. */
    int gpio_b_l;               /**< Phase B low side gpio. */
    int gpio_c_h;               /**< Phase C high side gpio. */
    int gpio_c_l;               /**< Phase C low side gpio. */
    int gpio_enable{-1};        /**< Enable pin for the BLDC driver (if any). */
    int gpio_fault{-1};         /**< Fault pin for the BLDC driver (if any). */
    float power_supply_voltage; /**< Voltage of the power supply. */
    float limit_voltage{-1}; /**< What voltage the motor should be limited to. Less than 0 means no
                                limit. Will be clamped to power supply voltage. */
    uint64_t dead_zone_ns{
        100}; /**< Dead zone in nanoseconds. Will be applied to both sides of the waveform. */
    espp::Logger::Verbosity log_level{
        espp::Logger::Verbosity::WARN}; /**< Verbosity for the bldc driver. */
  };

  /**
   * @brief Initialize the bldc driver.
   * @note Enables the driver.
   * @param config Config used to initialize the driver.
   */
  explicit BldcDriver(const Config &config)
      : BaseComponent("BldcDriver", config.log_level)
      , gpio_ah_((gpio_num_t)config.gpio_a_h)
      , gpio_al_((gpio_num_t)config.gpio_a_l)
      , gpio_bh_((gpio_num_t)config.gpio_b_h)
      , gpio_bl_((gpio_num_t)config.gpio_b_l)
      , gpio_ch_((gpio_num_t)config.gpio_c_h)
      , gpio_cl_((gpio_num_t)config.gpio_c_l)
      , gpio_en_(config.gpio_enable)
      , gpio_fault_(config.gpio_fault)
      , dead_zone_ns_(config.dead_zone_ns) {
    configure_power(config.power_supply_voltage, config.limit_voltage);
    init(config);
  }

  /**
   * @brief Disable the BldcDriver and destroy it.
   */
  ~BldcDriver() {
    disable();
    for (auto &gen : generators_) {
      mcpwm_del_generator(gen);
    }
    for (auto &comp : comparators_) {
      mcpwm_del_comparator(comp);
    }
    mcpwm_del_fault(fault_handle_);
    for (auto &oper : operators_) {
      mcpwm_del_operator(oper);
    }
    mcpwm_del_timer(timer_);
  }

  /**
   * @brief Enable the driver, set the enable pin high (if we were provided an
   *        enable pin), and remove any force level on the pins if there was a
   *        force level.
   */
  void enable() {
    if (enabled_) {
      return;
    }
    logger_.info("Enabling");
    auto err = mcpwm_timer_enable(timer_);
    if (err != ESP_OK) {
      logger_.error("Failed to enable timer: {}", esp_err_to_name(err));
      return;
    }
    err = mcpwm_timer_start_stop(timer_, MCPWM_TIMER_START_NO_STOP);
    if (err != ESP_OK) {
      logger_.error("Failed to start timer: {}", esp_err_to_name(err));
      return;
    }
    enabled_ = true;
    if (gpio_en_ >= 0) {
      gpio_set_level((gpio_num_t)gpio_en_, 1);
    }
  }

  /**
   * @brief Disable the driver, set the enable pin low (if we were provided an
   *        enable pin), add a force level to each pin to force the gates all
   *        low.
   */
  void disable() {
    if (!enabled_) {
      return;
    }
    logger_.info("Disabling");
    mcpwm_timer_start_stop(timer_, MCPWM_TIMER_STOP_FULL);
    mcpwm_timer_disable(timer_);
    enabled_ = false;
    if (gpio_en_ >= 0) {
      gpio_set_level((gpio_num_t)gpio_en_, 0);
    }
  }

  /**
   * @brief Check if the driver is enabled.
   * @return True if the driver is enabled, false otherwise.
   */
  bool is_enabled() const { return enabled_; }

  /**
   * @brief Check if the driver is faulted.
   * @note If no fault pin was provided, this will always return false.
   * @return True if the driver is faulted, false otherwise.
   */
  bool is_faulted() {
    if (gpio_fault_ < 0) {
      return false;
    }
    return gpio_get_level((gpio_num_t)gpio_fault_) == 1;
  }

  /**
   * @brief This function does nothing, merely exists for later if we choose
   *        to implement it as part of the FOC control algorithm.
   * @note The integers provided are -1 (low), 0 (high impedance), and 1
   *       (active) for the 3 possible states.
   * @param state_a Desired state for phase A.
   * @param state_b Desired state for phase B.
   * @param state_c Desired state for phase C.
   */
  void set_phase_state(int state_a, int state_b, int state_c) {
    // TODO: we don't do anything for this...
  }

  /**
   * @brief Set the phase voltages (in volts) to the driver.
   * @note Uses the previously configured power supply voltage and limit
   *       voltage (if any was provided) to convert to the appropriate pwm.
   * @param ua Desired voltage [0.0f, power_supply_voltage] for phase A.
   * @param ub Desired voltage [0.0f, power_supply_voltage] for phase B.
   * @param uc Desired voltage [0.0f, power_supply_voltage] for phase C.
   */
  void set_voltage(float ua, float ub, float uc) {
    if (!enabled_) {
      logger_.warn("Cannot set voltage, not enabled!");
      return;
    }
    float limit = limit_voltage_;
    ua = std::clamp(ua, 0.0f, limit);
    ub = std::clamp(ub, 0.0f, limit);
    uc = std::clamp(uc, 0.0f, limit);
    float full_power = power_supply_voltage_;
    float dc_a = std::clamp(ua / full_power, 0.0f, 1.0f);
    float dc_b = std::clamp(ub / full_power, 0.0f, 1.0f);
    float dc_c = std::clamp(uc / full_power, 0.0f, 1.0f);
    set_pwm(dc_a, dc_b, dc_c);
  }

  /**
   * @brief Directly set the duty cycle in range [0.0, 1.0] for each phase.
   * @param duty_a Duty cycle [0.0, 1.0] for phase A.
   * @param duty_b Duty cycle [0.0, 1.0] for phase B.
   * @param duty_c Duty cycle [0.0, 1.0] for phase C.
   */
  void set_pwm(float duty_a, float duty_b, float duty_c) {
    if (!enabled_) {
      logger_.warn("Cannot set pwm, not enabled!");
      return;
    }

    duty_a = std::clamp(duty_a, 0.0f, 1.0f);
    duty_b = std::clamp(duty_b, 0.0f, 1.0f);
    duty_c = std::clamp(duty_c, 0.0f, 1.0f);

    // NOTE: You'll get errors if you multiply by the configured
    // TICKS_PER_PERIOD, but no issues if you divide by 2 and it seems to work
    // :)
    uint32_t a_ticks = (uint32_t)(duty_a * (float)(TICKS_PER_PERIOD / 2));
    uint32_t b_ticks = (uint32_t)(duty_b * (float)(TICKS_PER_PERIOD / 2));
    uint32_t c_ticks = (uint32_t)(duty_c * (float)(TICKS_PER_PERIOD / 2));

    mcpwm_comparator_set_compare_value(comparators_[0], a_ticks);
    mcpwm_comparator_set_compare_value(comparators_[1], b_ticks);
    mcpwm_comparator_set_compare_value(comparators_[2], c_ticks);
  }

  /**
   * @brief Update the power supply configuration and optional motor voltage
   *        limit.
   * @param power_supply_voltage New max voltage (volts) from the power
   *        supply.
   * @param voltage_limit New limit voltage (volts) to allow to the motor.
   */
  void configure_power(float power_supply_voltage, float voltage_limit = -1.0f) {
    power_supply_voltage_ = power_supply_voltage;
    if (voltage_limit > 0) {
      limit_voltage_ = std::min(voltage_limit, power_supply_voltage_.load());
    } else {
      limit_voltage_ = power_supply_voltage_.load();
    }
  }

  /**
   * @brief Get the current motor voltage limit (if any).
   * @return Voltage limit the driver will allow to the motor.
   */
  float get_voltage_limit() const { return limit_voltage_.load(); }

  /**
   * @brief Get the configured power supply voltage.
   * @return Maximum voltage the driver can supply to the motor.
   */
  float get_power_supply_limit() const { return power_supply_voltage_.load(); }

  /**
   * @brief Set the power supply voltage.
   * @param voltage New power supply voltage.
   * @note Will update the voltage limit to the minimum of the current limit and
   *       the new power supply voltage.
   */
  void set_power_supply_voltage(float voltage) {
    power_supply_voltage_ = voltage;
    limit_voltage_ = std::min(limit_voltage_.load(), voltage);
  }

  /**
   * @brief Set the voltage limit.
   * @param voltage New voltage limit.
   * @note Will be clamped to the power supply voltage.
   */
  void set_voltage_limit(float voltage) {
    limit_voltage_ = std::min(voltage, power_supply_voltage_.load());
  }

protected:
  static int GROUP_ID;

  void init(const Config &config) {
    if (GROUP_ID >= SOC_MCPWM_GROUPS) {
      GROUP_ID = 0;
      logger_.error("Exceeded max number of MCPWM groups ({}), resetting to 0", SOC_MCPWM_GROUPS);
    }
    configure_enable_gpio();
    configure_timer();
    configure_operators();
    configure_fault();
    configure_comparators();
    configure_generators();
    GROUP_ID++;
    enable();
  }

  void configure_enable_gpio() {
    if (gpio_en_ < 0) {
      return;
    }
    logger_.info("Configure enable pin");
    gpio_config_t drv_en_config;
    memset(&drv_en_config, 0, sizeof(drv_en_config));
    drv_en_config.pin_bit_mask = 1ULL << gpio_en_;
    drv_en_config.mode = GPIO_MODE_OUTPUT;
    ESP_ERROR_CHECK(gpio_config(&drv_en_config));
    gpio_set_level((gpio_num_t)gpio_en_, 0);
  }

  void configure_timer() {
    logger_.info("Create MCPWM timer, group: {}", GROUP_ID);
    mcpwm_timer_config_t timer_config;
    memset(&timer_config, 0, sizeof(timer_config));
    timer_config.group_id = GROUP_ID;
    timer_config.clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT;
    timer_config.resolution_hz = TIMER_RESOLUTION_HZ;
    timer_config.count_mode = MCPWM_TIMER_COUNT_MODE_UP_DOWN;
    timer_config.period_ticks = TICKS_PER_PERIOD;
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer_));
  }

  void configure_operators() {
    logger_.info("Create MCPWM operator, group: {}", GROUP_ID);
    mcpwm_operator_config_t operator_config;
    memset(&operator_config, 0, sizeof(operator_config));
    operator_config.group_id = GROUP_ID;
    for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &operators_[i]));
    }

    logger_.info("Connect operators to the same timer");
    for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_operator_connect_timer(operators_[i], timer_));
    }
  }

  void configure_fault() {
    if (gpio_fault_ < 0) {
      return;
    }
    logger_.info("Create fault detector, group: {}", GROUP_ID);
    mcpwm_gpio_fault_config_t gpio_fault_config{};
    memset(&gpio_fault_config, 0, sizeof(gpio_fault_config));
    gpio_fault_config.gpio_num = (gpio_num_t)gpio_fault_;
    gpio_fault_config.group_id = GROUP_ID;
    gpio_fault_config.flags.active_level = 1; // high level means fault, refer to TMC6300 datasheet
    gpio_fault_config.flags.pull_down = true; // internally pull down
    gpio_fault_config.flags.io_loop_back = true; // enable loop back to GPIO input
    ESP_ERROR_CHECK(mcpwm_new_gpio_fault(&gpio_fault_config, &fault_handle_));

    logger_.info("Set brake mode on the fault event");
    mcpwm_brake_config_t brake_config{};
    memset(&brake_config, 0, sizeof(brake_config));
    brake_config.brake_mode = MCPWM_OPER_BRAKE_MODE_CBC;
    brake_config.fault = fault_handle_;
    brake_config.flags.cbc_recover_on_tez = true;
    for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_operator_set_brake_on_fault(operators_[i], &brake_config));
    }

    logger_.info("Configure fault pin");
    gpio_config_t drv_fault_config;
    memset(&drv_fault_config, 0, sizeof(drv_fault_config));
    drv_fault_config.pin_bit_mask = 1ULL << gpio_fault_;
    drv_fault_config.mode = GPIO_MODE_INPUT;
    drv_fault_config.pull_up_en = GPIO_PULLUP_DISABLE;
    drv_fault_config.pull_down_en = GPIO_PULLDOWN_ENABLE;
    ESP_ERROR_CHECK(gpio_config(&drv_fault_config));
  }

  void configure_comparators() {
    logger_.info("Create comparators");
    mcpwm_comparator_config_t compare_config;
    memset(&compare_config, 0, sizeof(compare_config));
    compare_config.flags.update_cmp_on_tez = true;
    for (int i = 0; i < 3; i++) {
      ESP_ERROR_CHECK(mcpwm_new_comparator(operators_[i], &compare_config, &comparators_[i]));
      // set compare value to 0, we will adjust the speed in a period timer callback
      ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparators_[i], 0));
    }
  }

  void configure_generators() {
    logger_.info("Create generators");
    mcpwm_generator_config_t gen_config = {};
    const gpio_num_t gen_gpios[6] = {gpio_ah_, gpio_al_, gpio_bh_, gpio_bl_, gpio_ch_, gpio_cl_};
    for (int i = 0; i < 6; i++) {
      gen_config.gen_gpio_num = gen_gpios[i];
      ESP_ERROR_CHECK(mcpwm_new_generator(operators_[i / 2], &gen_config, &generators_[i]));
    }

    // set high and low generators to output the same waveform using the
    // comparator. we will use the dead time module to add edge delay, also make
    // gen_high and gen_low complementary

    // A high / low
    configure_generator_action(generators_[0], generators_[1], comparators_[0]);
    configure_generator_deadtime(generators_[0], generators_[1]);
    // B high / low
    configure_generator_action(generators_[2], generators_[3], comparators_[1]);
    configure_generator_deadtime(generators_[2], generators_[3]);
    // C high / low
    configure_generator_action(generators_[4], generators_[5], comparators_[2]);
    configure_generator_deadtime(generators_[4], generators_[5]);
  }

  void configure_generator_action(mcpwm_gen_handle_t &gen_high, mcpwm_gen_handle_t &gen_low,
                                  mcpwm_cmpr_handle_t comp) {
    logger_.info("Setup generator action");

    // set high/low generators to output low when the timer is counting up, and
    // high when the timer is counting down.
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_high,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_high,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comp, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_low,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comp, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_low,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_DOWN, comp, MCPWM_GEN_ACTION_HIGH)));

    // set the brake event to stop the motor (set low) for both the high and low generators
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
        gen_high, MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC,
                                               MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_brake_event(
        gen_low, MCPWM_GEN_BRAKE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_OPER_BRAKE_MODE_CBC,
                                              MCPWM_GEN_ACTION_LOW)));
  }

  void configure_generator_deadtime(mcpwm_gen_handle_t &gen_high, mcpwm_gen_handle_t &gen_low) {
    logger_.info("Setup generator deadtime");
    // TODO: determine the right number of ticks here....
    mcpwm_dead_time_config_t dt_config;

    uint32_t dead_zone_ticks = (uint32_t)((dead_zone_ns_ * TIMER_RESOLUTION_HZ) / 1000000000UL);

    memset(&dt_config, 0, sizeof(dt_config));
    dt_config.posedge_delay_ticks = dead_zone_ticks;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_high, gen_high, &dt_config));

    memset(&dt_config, 0, sizeof(dt_config));
    dt_config.negedge_delay_ticks = dead_zone_ticks;
    dt_config.flags.invert_output = true;
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_high, gen_low, &dt_config));
  }

  gpio_num_t gpio_ah_;
  gpio_num_t gpio_al_;
  gpio_num_t gpio_bh_;
  gpio_num_t gpio_bl_;
  gpio_num_t gpio_ch_;
  gpio_num_t gpio_cl_;
  int gpio_en_;
  int gpio_fault_;
  mcpwm_fault_handle_t fault_handle_;
  std::atomic<float> power_supply_voltage_;
  std::atomic<float> limit_voltage_;
  uint64_t dead_zone_ns_;
  std::atomic<bool> enabled_{false};
  mcpwm_timer_handle_t timer_;
  std::array<mcpwm_oper_handle_t, 3> operators_;
  std::array<mcpwm_cmpr_handle_t, 3> comparators_;
  std::array<mcpwm_gen_handle_t, 6> generators_;
};
} // namespace espp
