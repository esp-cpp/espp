#include <bitset>
#include <chrono>
#include <thread>
#include <vector>

#include "format.hpp"
#include "nvs.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  std::this_thread::sleep_for(200ms);
  fmt::print("Starting NVS example!\n");

  //! [nvs example]
  std::error_code ec;
  uint8_t counter = 0;
  espp::Nvs nvs;
  nvs.init(ec);
  ec.clear();
  // note that the namespace and key strings must be <= 15 characters
  nvs.get_or_set_var("system", "reset_counter", counter, counter, ec);
  ec.clear();
  fmt::print("Reset Counter = {}\n", counter);

  bool flag = false;
  nvs.get_or_set_var("other-namespace", "flag", flag, flag, ec);
  ec.clear();
  fmt::print("Got / Set Flag = {}\n", flag);
  // now toggle the flag
  flag = !flag;
  nvs.set_var("other-namespace", "flag", flag, ec);
  ec.clear();
  fmt::print("Toggled Flag = {}\n", flag);

  // test protection against really long namespace names (length > 15)
  std::string long_ns(16, 'a');
  nvs.get_or_set_var(long_ns, "flag", flag, flag, ec);
  if (ec) {
    fmt::print("Expected error: {}\n", ec.message());
  } else {
    fmt::print("Unexpected success\n");
  }
  ec.clear();

  // test getting a non-existent key
  nvs.get_var("system", "not-here", counter, ec);
  if (ec) {
    fmt::print("Expected error: {}\n", ec.message());
  } else {
    fmt::print("Unexpected success\n");
  }
  ec.clear();

  // test getting a string value
  std::string str;
  nvs.get_or_set_var("system", "string", str, std::string("default"), ec);
  if (ec) {
    fmt::print("Error: {}\n", ec.message());
  } else {
    fmt::print("String = {}\n", str);
  }
  ec.clear();

  // test setting a string value
  str = "hello";
  nvs.set_var("system", "string", str, ec);
  if (ec) {
    fmt::print("Error: {}\n", ec.message());
  } else {
    fmt::print("String set to '{}'\n", str);
  }
  ec.clear();

  counter++;

  if (counter > 10) {
    nvs.erase(ec);
    nvs.init(ec);
    counter = 0;
    fmt::print("NVS erased, Reset Counter set to 0\n");
    ec.clear();
  }

  nvs.set_var("system", "reset_counter", counter, ec);
  fmt::print("Next Reset Counter will be = {}\n", counter);
  fmt::print("NVS example complete!\n");
  //! [nvs example]

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
