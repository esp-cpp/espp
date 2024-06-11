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

  {
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
  }

  {
    //! [nvshandle example]
    // Init NVS
    std::error_code ec;
    espp::Nvs nvs;
    nvs.init(ec);
    ec.clear();
    
    // Open
    fmt::print("\nOpening Non-Volatile Storage (NVS) handle... ");
    // Handle will automatically close when going out of scope or when it's reset.
    espp::NVSHandle storage("storage", ec);
    fmt::print("Done\n");
    ec.clear();
    
    // Read
    fmt::print("Reading restart counter from NVS ... ");
    int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
    storage.get("restart_counter", restart_counter, ec);
    if (ec) {
      if (ec.value() == static_cast<int>(NvsErrc::Key_Not_Found)) {
        fmt::print("The value is not initialized yet!\n");
      } else if (ec.value() == static_cast<int>(NvsErrc::Read_NVS_Failed)) {
        fmt::print("Failed to read from NVS: %s\n", ec.message().c_str());
      } else {
        fmt::print("An error occurred: %s\n", ec.message().c_str());
      }
    } else {
      fmt::print("Done\n");
      fmt::print("Restart counter = %" PRIu32 "\n", restart_counter);
    }
    ec.clear();

    // Write
    fmt::print("Updating restart counter in NVS ... ");
    restart_counter++;
    storage.set("restart_counter", restart_counter, ec);
    fmt::print("Done\n");
    ec.clear();

    // Commit written value.
    // After setting any values, nvs_commit() must be called to ensure changes are written
    // to flash storage. Implementations may write to storage at other times,
    // but this is not guaranteed.
    fmt::print("Committing updates in NVS ... ");
    storage.commit(ec);
    fmt::print("Done\n");
    ec.clear();

    fmt::print("\n");

    // Restart module
    for (int i = 10; i >= 0; i--) {
        fmt::print("Restarting in %d seconds...\n", i);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
    fmt::print("Restarting now.\n");
    fflush(stdout);
    esp_restart();
    //! [nvshandle example]
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
