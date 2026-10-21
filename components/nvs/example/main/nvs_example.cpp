#include <bitset>
#include <chrono>
#include <cstring>
#include <string>
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
      fmt::print("String = '{}' ({} bytes)\n", str, str.size());
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
      fmt::print("String = '{}' ({} bytes)\n", str, str.size());
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

  {
    //! [nvs validation]
    // Validate the NVS operations above with explicit pass / fail checks and
    // print a summary at the end. The checks run in their own namespace, which
    // is erased before and after, so every boot runs the same sequence.
    std::error_code ec;
    espp::Nvs nvs;
    nvs.init(ec);
    ec.clear();

    int passed = 0;
    int failed = 0;
    std::vector<std::string> failures;
    auto check = [&](const char *name, bool ok) {
      fmt::print("  [{}] {}\n", ok ? "PASS" : "FAIL", name);
      if (ok) {
        passed++;
      } else {
        failed++;
        failures.emplace_back(name);
      }
    };
    // a key is "missing" when reading it fails with Key_Not_Found
    auto key_missing = [&](const char *key) {
      std::error_code read_ec;
      int32_t dummy = 0;
      nvs.get_var("validate", key, dummy, read_ec);
      return read_ec && read_ec.value() == static_cast<int>(NvsErrc::Key_Not_Found);
    };
    auto has_nul = [](const std::string &s) { return s.find('\0') != std::string::npos; };

    fmt::print("\nValidating NVS operations...\n");
    // start from a clean namespace (erasing a namespace that does not exist is
    // fine - the first boot after a flash erase gets here with nothing stored)
    nvs.erase("validate", ec);
    ec.clear();

    // integers: get_or_set_var on a missing key stores and returns the default
    uint8_t u8 = 0;
    nvs.get_or_set_var("validate", "u8", u8, uint8_t{42}, ec);
    check("uint8 get_or_set_var on a missing key returns the default", !ec && u8 == 42);
    ec.clear();
    u8 = 0;
    nvs.get_var("validate", "u8", u8, ec);
    check("uint8 default was stored", !ec && u8 == 42);
    ec.clear();

    int32_t i32 = 0;
    nvs.set_var("validate", "i32", int32_t{-123456}, ec);
    nvs.get_var("validate", "i32", i32, ec);
    check("int32 set_var / get_var round trip", !ec && i32 == -123456);
    ec.clear();

    // bool: set_var / get_var round trip in both directions
    bool b = false;
    nvs.set_var("validate", "bool", true, ec);
    nvs.get_var("validate", "bool", b, ec);
    bool bool_true_ok = !ec && b;
    nvs.set_var("validate", "bool", false, ec);
    nvs.get_var("validate", "bool", b, ec);
    check("bool set_var / get_var round trip", bool_true_ok && !ec && !b);
    ec.clear();

    // float: stored bit-exact
    float f = 0.0f;
    nvs.set_var("validate", "float", 3.14159f, ec);
    nvs.get_var("validate", "float", f, ec);
    check("float set_var / get_var round trip", !ec && f == 3.14159f);
    ec.clear();

    // strings: get_or_set_var on a missing key stores and returns the default,
    // and size() is the string's length (no zero terminator in the value)
    std::string s;
    nvs.get_or_set_var("validate", "str", s, std::string("default"), ec);
    check("string get_or_set_var on a missing key returns the default",
          !ec && s == "default" && s.size() == 7 && !has_nul(s));
    ec.clear();

    // get_or_set_var on an existing key returns the stored value, not the default
    s.clear();
    nvs.get_or_set_var("validate", "str", s, std::string("other"), ec);
    check("string get_or_set_var on an existing key returns the stored value",
          !ec && s == "default" && s.size() == 7 && !has_nul(s));
    ec.clear();

    // set_var / get_var round trip: equal, exact size, last character intact
    nvs.set_var("validate", "str", std::string("hello world"), ec);
    s.clear();
    nvs.get_var("validate", "str", s, ec);
    check("string set_var / get_var round trip",
          !ec && s == "hello world" && s.size() == 11 && s.back() == 'd' && !has_nul(s));
    ec.clear();

    // overwriting with a shorter string: the read is the new string only,
    // even when the destination already holds something longer
    nvs.set_var("validate", "str", std::string("hi"), ec);
    s = std::string(32, 'x');
    nvs.get_var("validate", "str", s, ec);
    check("shorter string overwrite reads back exactly", !ec && s == "hi" && s.size() == 2);
    ec.clear();

    // an empty string is stored and read back as empty
    nvs.set_var("validate", "str", std::string(""), ec);
    s = "not empty";
    nvs.get_var("validate", "str", s, ec);
    check("empty string set_var / get_var round trip", !ec && s.empty());
    ec.clear();

    // the longest value a string key holds comfortably (NVS strings are limited
    // by the page size; 64 characters is well inside that)
    std::string long_str(64, 'a');
    nvs.set_var("validate", "str", long_str, ec);
    s.clear();
    nvs.get_var("validate", "str", s, ec);
    check("64 character string round trip", !ec && s == long_str && s.size() == 64);
    ec.clear();

    // c-string comparison: the stored string ends where the value ends
    check("string c_str() has no embedded terminator", strlen(s.c_str()) == s.size());

    // erasing a key: the key is gone, the others survive
    nvs.erase("validate", "str", ec);
    check("erase key succeeds", !ec);
    ec.clear();
    check("erased key is not found", key_missing("str"));
    nvs.get_var("validate", "i32", i32, ec);
    check("other keys survive erasing one key", !ec && i32 == -123456);
    ec.clear();

    // reading a missing key reports Key_Not_Found and leaves the value alone
    s = "untouched";
    nvs.get_var("validate", "missing", s, ec);
    check("reading a missing string key fails and leaves the value alone", ec && s == "untouched");
    ec.clear();

    // erasing the namespace: everything is gone
    nvs.erase("validate", ec);
    check("erase namespace succeeds", !ec);
    ec.clear();
    check("keys are not found after erasing the namespace",
          key_missing("u8") && key_missing("i32") && key_missing("bool"));

    fmt::print("NVS validation: {}/{} checks passed - {}\n", passed, passed + failed,
               failed == 0 ? "PASS" : "FAIL");
    for (const auto &name : failures) {
      fmt::print("  failed: {}\n", name);
    }
    fflush(stdout);
    //! [nvs validation]
  }

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
