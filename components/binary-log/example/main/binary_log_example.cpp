#include <chrono>
#include <functional>
#include <thread>

#include <esp_core_dump.h>

#include "binary-log.hpp"
#include "file_system.hpp"
#include "format.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  fmt::print("Starting binary log example\n");
  // initialize the file system
  std::error_code ec;
  auto &fs = espp::FileSystem::get();
  auto root = fs.get_root_path();
  const std::filesystem::path logfile = root / std::filesystem::path{"log.out"};

  // first see if the file exists
  if (std::filesystem::exists(logfile, ec)) {
    fmt::print("Removing existing log file\n");
    fs.remove(logfile, ec);
  }

  {
    //! [Binary Log example]
    float num_seconds_to_run = 3.0f;
    // create logger
    fmt::print("Creating binary log file: {}\n", logfile.c_str());
    binary_log::binary_log log(logfile.c_str());

    fmt::print("Starting binary log\n");
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;

      BINARY_LOG(log, "elapsed: {:.3f}s", elapsed);
      BINARY_LOG(log, "remaining: {:.3f}s", remaining);

      if (remaining < 0) {
        BINARY_LOG(log, "You overstayed your welcome by {:.03}s!", -remaining);
      }
      std::this_thread::sleep_for(500ms);
    }
    //! [Logger example]
  }

  // now print out the logger files
  fmt::print("Printing log file\n");

  size_t file_size = std::filesystem::file_size(logfile, ec);
  std::ifstream ifs(logfile, std::ios::in | std::ios::binary);
  // read bytes
  std::vector<char> file_bytes(file_size);
  ifs.read(file_bytes.data(), file_size);
  ifs.close();
  fmt::print("Read {} bytes from log file\n", file_size);
  fmt::print("File contents:\n");
  fmt::print("\t{::02x}\n", file_bytes);

  // print the contents of the file system
  fmt::print("{}\n", fs.list_directory(root, {}));

  // now erase the file system
  fmt::print("Erase file system\n");
  fs.remove_contents(root, ec);

  fmt::print("Finished binary log example\n");

  // now loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
