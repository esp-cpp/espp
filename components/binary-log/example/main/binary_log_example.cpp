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

  auto print_files = [&fs, &logger, &root, &logfile]() {
    logger.info("Printing log file");
    std::error_code ec;

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
  };

  // first see if the file exists
  if (std::filesystem::exists(logfile, ec)) {
    logger.info("Removing existing log file");
    fs.remove(logfile, ec);
  }

  {
    //! [Binary Log example]
    float num_seconds_to_run = 3.0f;
    // create standard logger. This will buffer the logs in memory until the buffer is
    // full or the logger is destroyed
    logger.info("Creating binary log file: {}", logfile.c_str());
    static constexpr size_t log_buffer_size = 1024;
    static constexpr size_t index_buffer_size = 1024;
    static constexpr size_t runlength_buffer_size = 1024;
    using Packer = binary_log::packer<log_buffer_size, index_buffer_size, runlength_buffer_size>;
    binary_log::binary_log<Packer> log(logfile);

    logger.info("Starting binary logging for {}s", num_seconds_to_run);
    auto start = std::chrono::high_resolution_clock::now();
    auto now = std::chrono::high_resolution_clock::now();
    float elapsed = std::chrono::duration<float>(now - start).count();
    while (elapsed < num_seconds_to_run) {
      now = std::chrono::high_resolution_clock::now();
      elapsed = std::chrono::duration<float>(now - start).count();
      auto remaining = num_seconds_to_run - elapsed;

      // NOTE: use of a single log within the majority of this loop allows for
      // run-length encoding of the log messages to be used to reduce the size
      BINARY_LOG(log, "elapsed: {}s\nremaining: {}s", elapsed, remaining);

      // print a log each second. to track it, just get the remainder
      // milliseconds and see if they're less than 15
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() % 1000 < 15) {
        BINARY_LOG(log, "This is a log message at {}s", elapsed)
      }

      if (remaining < 0) {
        BINARY_LOG(log, "You overstayed your welcome by {}s!", -remaining);
      }
      std::this_thread::sleep_for(10ms);
    }
    //! [Logger example]
  }

  // now print out the logger files
  print_files();

  // now erase the file system
  logger.info("Erase file system");
  fs.remove_contents(root, ec);

  {
    //! [Binary Log example]
    float num_seconds_to_run = 3.0f;
    // create ringbuffer logger. This will not write to disk, but instead will
    // only store log data in memory. The log buffer will be a ring buffer,
    // discarding the oldest data, while the index and runlength buffers will be
    // simple arrays which will abort if they run out of space.
    logger.info("Creating ringbuffer binary log");
    static constexpr size_t log_buffer_size = 1024;
    static constexpr size_t index_buffer_size = 1024;
    static constexpr size_t runlength_buffer_size = 1024;
    using Packer =
        binary_log::ringbuffer_packer<log_buffer_size, index_buffer_size, runlength_buffer_size>;
    binary_log::binary_log<Packer> log(logfile);

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

      // print a log each second. to track it, just get the remainder
      // milliseconds and see if they're less than 15
      if (std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count() % 1000 < 15) {
        BINARY_LOG(log, "This is a log message at {}s", elapsed)
      }

      if (remaining < 0) {
        BINARY_LOG(log, "You overstayed your welcome by {}s!", -remaining);
      }
      std::this_thread::sleep_for(10ms);
    }
    // Since this is a memory-based logger, we must flush and read out the data
    // manually
    log.flush();

    const Packer &packer = log.get_packer();
    auto log_buffer = packer.get_log_buffer();
    auto index_buffer_sv = packer.get_index_buffer();
    // convert the string view into a vector for easy printing with libfmt
    std::vector<uint8_t> index_buffer(index_buffer_sv.begin(), index_buffer_sv.end());
    auto runlength_buffer_sv = packer.get_runlength_buffer();
    // convert the string view into a vector for easy printing with libfmt
    std::vector<uint8_t> runlength_buffer(runlength_buffer_sv.begin(), runlength_buffer_sv.end());

    logger.info("Log file size: {} bytes", log_buffer.size());
    logger.info("Index file size: {} bytes", index_buffer.size());
    logger.info("Runlength file size: {} bytes", runlength_buffer.size());
    logger.info("--------------------------------");
    logger.info("Total file size: {} bytes",
                log_buffer.size() + index_buffer.size() + runlength_buffer.size());

    logger.info("Log data:\n{::#02x}", log_buffer);
    logger.info("Index data:\n{::#02x}", index_buffer);
    logger.info("Runlength data:\n{::#02x}", runlength_buffer);
    //! [Logger example]
  }

  logger.info("Finished binary log example");

  // now loop forever
  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
