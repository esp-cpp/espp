#include <bitset>
#include <chrono>
#include <thread>
#include <vector>

#include "format.hpp"
#include "nvs.hpp"
#include "nvs_handle_espp.hpp"

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

    // test getting a handle to one of the NVS namespaces
    espp::NvsHandle system_handle = nvs.get_handle("system", ec);
    if (ec) {
      fmt::print("Error getting handle: {}\n", ec.message());
    } else {
      fmt::print("Got handle to 'system' namespace\n");
    }
    ec.clear();
    // test getting a handle to a new (non-existent) namespace
    espp::NvsHandle new_handle = nvs.get_handle("new-namespace", ec);
    if (ec) {
      fmt::print("Error getting handle to new namespace: {}\n", ec.message());
    } else {
      fmt::print("Got handle to 'new-namespace'\n");
    }
    ec.clear();

    // test delayed initialization of nvs handle
    espp::NvsHandle test_handle;
    test_handle.init("system", ec);
    // now test getting a variable from the copied handle
    test_handle.get("reset_counter", counter, ec);
    if (ec) {
      fmt::print("Error getting variable from delay-inited handle: {}\n", ec.message());
    } else {
      fmt::print("Got reset_counter from delay-inited handle: {}\n", counter);
    }

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
      fmt::print("String = '{}'\n", str);
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

    // test getting the string value again
    nvs.get_var("system", "string", str, ec);
    if (ec) {
      fmt::print("Error: {}\n", ec.message());
    } else {
      fmt::print("String = '{}'\n", str);
    }
    ec.clear();

    // test erasing the string value
    nvs.erase("system", "string", ec);
    if (ec) {
      fmt::print("Error: {}\n", ec.message());
    } else {
      fmt::print("String erased\n");
    }
    ec.clear();

    // now test getting it again to ensure it was erased
    nvs.get_var("system", "string", str, ec);
    if (ec) {
      fmt::print("Sucess, got expected error when reading erased value: {}\n", ec.message());
    } else {
      fmt::print("Failure, got unexpected success when reading erased value\n");
    }
    ec.clear();

    counter++;

    if (counter > 10) {
      // test erasing the whole namespace
      nvs.erase("system", ec);
      if (ec) {
        fmt::print("Error: {}\n", ec.message());
      } else {
        fmt::print("Namespace erased\n");
      }
      ec.clear();

      // now erase the wole nvs partition
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
    espp::NvsHandle storage("storage", ec);
    fmt::print("Done\n");
    ec.clear();

    // Read
    fmt::print("Reading restart counter from NVS ... ");
    int32_t restart_counter = 0;
    storage.get("restart_counter", restart_counter, ec);
    if (ec) {
      if (ec.value() == static_cast<int>(NvsErrc::Key_Not_Found)) {
        fmt::print("The value is not initialized yet!\n");
      } else if (ec.value() == static_cast<int>(NvsErrc::Read_NVS_Failed)) {
        fmt::print("Failed to read from NVS: {}\n", ec.message().c_str());
      } else {
        fmt::print("An error occurred: {}\n", ec.message().c_str());
      }
    } else {
      fmt::print("Done\n");
      fmt::print("Restart counter = {}\n", restart_counter);
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
    fflush(stdout);
    //! [nvshandle example]
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
