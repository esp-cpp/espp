#include <chrono>
#include <functional>
#include <thread>

#include <esp_core_dump.h>

#include "binary-log.hpp"
#include "file_system.hpp"
#include "logger.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  espp::Logger logger({.tag = "blog_example", .level = espp::Logger::Verbosity::INFO});

  logger.info("Starting binary log example");
  // initialize the file system
  std::error_code ec;
  auto &fs = espp::FileSystem::get();
  auto root = fs.get_root_path();
  const std::filesystem::path logfile = root / std::filesystem::path{"log.out"};

  // first see if the file exists
  if (std::filesystem::exists(logfile, ec)) {
    logger.info("Removing existing log file");
    fs.remove(logfile, ec);
  }

  {
    //! [Binary Log example]
    float num_seconds_to_run = 3.0f;
    // create logger
    logger.info("Creating binary log file: {}", logfile.c_str());
    static constexpr size_t buffer_size = 1024;
    static constexpr size_t index_buffer_size = 1024;
    binary_log::binary_log<buffer_size, index_buffer_size> log(logfile.c_str());

    logger.info("Starting binary logging for {}s", num_seconds_to_run);
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;

      BINARY_LOG(log, "elapsed: {}s", elapsed);
      BINARY_LOG(log, "remaining: {}s", remaining);

      if (remaining < 0) {
        BINARY_LOG(log, "You overstayed your welcome by {}s!", -remaining);
      }
      std::this_thread::sleep_for(10ms);
    }
    //! [Logger example]
  }

  // now print out the logger files
  logger.info("Printing log file");

  size_t file_size = std::filesystem::file_size(logfile, ec);
  std::ifstream ifs(logfile, std::ios::in | std::ios::binary);
  // read bytes
  std::vector<char> file_bytes(file_size);
  ifs.read(file_bytes.data(), file_size);
  ifs.close();
  logger.info("Read {} bytes from log file", file_size);
  logger.info("File contents:\n{::#02x}", file_bytes);

  // now print the contents of the other two associated files
  logger.info("Printing index file");
  std::filesystem::path indexfile = logfile;
  indexfile.replace_extension(logfile.extension().string() + ".index");
  file_size = std::filesystem::file_size(indexfile, ec);
  ifs.open(indexfile, std::ios::in | std::ios::binary);
  // read bytes
  file_bytes.resize(file_size);
  ifs.read(file_bytes.data(), file_size);
  ifs.close();
  logger.info("Read {} bytes from index file", file_size);
  logger.info("File contents:\n{::#02x}", file_bytes);

  // print the contents of the runlength file
  logger.info("Printing runlength file");
  std::filesystem::path runlengthfile = logfile;
  runlengthfile.replace_extension(logfile.extension().string() + ".runlength");
  file_size = std::filesystem::file_size(runlengthfile, ec);
  ifs.open(runlengthfile, std::ios::in | std::ios::binary);
  // read bytes
  file_bytes.resize(file_size);
  ifs.read(file_bytes.data(), file_size);
  ifs.close();
  logger.info("Read {} bytes from runlength file", file_size);
  logger.info("File contents:\n{::#02x}", file_bytes);

  // print the contents of the file system
  logger.info("Directory contents:\n{}", fs.list_directory(root, {}));

  // now erase the file system
  logger.info("Erase file system");
  fs.remove_contents(root, ec);

  logger.info("Finished binary log example");

  // now loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
