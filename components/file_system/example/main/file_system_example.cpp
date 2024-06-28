#include <chrono>
#include <fstream>
#include <thread>
#include <vector>

#include "logger.hpp"

#include "file_system.hpp"

using namespace std::chrono_literals;
extern "C" void app_main(void) {
  static espp::Logger logger(
      {.tag = "file system example", .level = espp::Logger::Verbosity::INFO});
  // This example shows using the file system to read/write files
  logger.info("Running file system example!");

  {
    //! [file_system info example]
    auto &fs = espp::FileSystem::get();
    // NOTE: partition label is configured by menuconfig and should match the
    //       partition label in the partition table (partitions.csv).
    // returns a const char*
    auto partition_label = fs.get_partition_label();
    // returns a std::string
    auto mount_point = fs.get_mount_point();
    // returns a std::filesystem::path
    auto root_path = fs.get_root_path();
    logger.info("Partition label: {}", partition_label);
    logger.info("Mount point:     {}", mount_point);
    logger.info("Root path:       {}", root_path.string());
    // human_readable returns a string with the size and unit, e.g. 1.2 MB
    auto total_space = fs.human_readable(fs.get_total_space());
    auto free_space = fs.human_readable(fs.get_free_space());
    auto used_space = fs.human_readable(fs.get_used_space());
    logger.info("Total space: {}", total_space);
    logger.info("Free space:  {}", free_space);
    logger.info("Used space:  {}", used_space);
    //! [file_system info example]
  }

  const std::string_view test_dir = "sandbox";
  const std::string_view test_file = "test.csv";
  const std::string_view file_contents = "Hello World!";
  // use posix api
  {
    logger.info("Testing filesystem using POSIX APIs...");

    //! [file_system posix example]
    auto mount_point = espp::FileSystem::get().get_mount_point();
    const std::string sandbox = std::string(mount_point) + "/" + std::string(test_dir);
    struct stat st;
    // check that it exists - IT SHOULDN'T
    logger.info("Directory {} exists: {}", sandbox,
                stat(sandbox.c_str(), &st) == 0 && S_ISDIR(st.st_mode));

    // make a directory
    mkdir(sandbox.c_str(), 0755);
    logger.info("Created directory {}", sandbox);

    // check that it exists - IT SHOULD
    logger.info("Directory {} exists: {}", sandbox,
                stat(sandbox.c_str(), &st) == 0 && S_ISDIR(st.st_mode));

    // write to a file
    std::string file = sandbox + "/" + std::string(test_file);
    FILE *f = fopen(file.c_str(), "w");
    if (f == nullptr) {
      logger.error("Couldn't open {} for writing!", file);
    } else {
      fwrite(file_contents.data(), 1, file_contents.size(), f);
      fclose(f);
      logger.info("Wrote '{}' to {}", file_contents, file);
    }

    // get the file size
    stat(file.c_str(), &st);
    size_t file_size = st.st_size;
    logger.info("File '{}' is {}", file, espp::FileSystem::get().human_readable(file_size));

    // read from a file
    f = fopen(file.c_str(), "r"); // NOTE: could use rb for binary
    if (f == nullptr) {
      logger.error("Couldn't open {} for reading!", file);
    } else {
      // // alternative way to get the file size if you've already opened it
      // fseek(f, 0, SEEK_END); // go to end
      // size_t file_size = ftell(f); // another way to get the file size
      // fseek(f, 0, SEEK_SET); // go back to beginning

      std::vector<char> bytes;
      bytes.resize(file_size);
      fread(bytes.data(), 1, file_size, f);
      fclose(f);
      logger.info("Read bytes from file {}", bytes);
    }

    // rename the file
    std::string file2 = sandbox + "/test2.csv";
    // Check if destination file exists before renaming
    if (stat(file2.c_str(), &st) == 0) {
      // Delete it if it exists
      unlink(file2.c_str());
    }
    // Rename original file
    if (rename(file.c_str(), file2.c_str()) != 0) {
      logger.error("Could not rename {} to {}", file, file2);
    } else {
      logger.info("Renamed '{}' to '{}'", file, file2);
    }

    // list files in a directory
    auto &fs = espp::FileSystem::get();
    espp::FileSystem::ListConfig config;
    std::string directory_listing = fs.list_directory(sandbox, config);
    logger.info("Directory listing for {}:\n{}", sandbox, directory_listing);

    // list all files recursively
    config.recursive = true;
    auto root = fs.get_root_path();
    std::string root_listing = fs.list_directory(root, config);
    logger.info("Recursive directory listing for {}:\n{}", root.string(), root_listing);

    // get a lsit of files in a directory
    auto files = fs.get_files_in_path(sandbox);
    logger.info("Files in {}: ", sandbox);
    for (const auto &f : files) {
      logger.info("\t{}", f);
    }

    files = fs.get_files_in_path(root, true); // include directories
    logger.info("Files in {} (including directories): ", root.string());
    for (const auto &f : files) {
      logger.info("\t{}", f);
    }

    files = fs.get_files_in_path(root, true, true); // include directories, recursive
    logger.info("Files in {} (including directories, recursive): ", root.string());
    for (const auto &f : files) {
      logger.info("\t{}", f);
    }

    files = fs.get_files_in_path(root, false, true); // do not include directories, recursive
    logger.info("Files in {} (not including directories, recursive): ", root.string());
    for (const auto &f : files) {
      logger.info("\t{}", f);
    }

    // cleanup
    auto items = {file, file2, sandbox};
    for (auto &item : items) {
      // use stat to figure out if it exists
      auto code = stat(item.c_str(), &st);
      bool exists = code == 0;
      if (!exists) {
        logger.warn("Not removing '{}', it doesn't exist!", item);
        continue;
      }
      // and if it is a directory
      bool is_dir = S_ISDIR(st.st_mode);
      int ec = is_dir ? rmdir(item.c_str()) : unlink(item.c_str());
      if (ec) {
        logger.error("Could not remove {}, error code: {}", item, ec);
      } else {
        logger.info("Cleaned up {}", item);
      }
    }
    //! [file_system posix example]
  }

  // use std::filesystem api
  {
    logger.info("Testing filesystem using std::filesystem APIs...");

    //! [file_system std filesystem example]
    // NOTE: use the overloads that take ec as parameter, else it will throw
    //       exception on error.
    std::error_code ec;
    namespace fs = std::filesystem;
    // NOTE: cannot use chdir on littlefs , so we need to always use absolute
    //       paths.
    const fs::path sandbox = espp::FileSystem::get().get_root_path() / fs::path{test_dir};

    // check that it exists - IT SHOULDN'T
    logger.info("Directory {} exists: {}", sandbox.string(), fs::exists(sandbox));

    // make a directory
    fs::create_directory(sandbox, ec);
    if (ec) {
      logger.error("Could not create directory {} - {}", sandbox.string(), ec.message());
    } else {
      logger.info("Created directory {}: {}", sandbox.string(), fs::exists(sandbox));
    }

    // check that it exists - IT SHOULD
    logger.info("Directory {} exists: {}", sandbox.string(), fs::exists(sandbox));

    fs::path file = sandbox / fs::path{test_file};

    // write to a file
    std::ofstream ofs(file);
    ofs << file_contents;
    ofs.close();
    ofs.flush();
    logger.info("Wrote '{}' to {}", file_contents, file.string());

    // Get file size
    size_t file_size = fs::file_size(file, ec);
    if (ec) {
      logger.error("Could not get file size of '{}' - {}", file.string(), ec.message());
    } else {
      logger.info("File '{}' has a file size of {}", file.string(),
                  espp::FileSystem::get().human_readable(file_size));
    }

    // read from a file
    std::ifstream ifs(file, std::ios::in | std::ios::binary | std::ios::ate); // at the end
    // ifstream::pos_type file_size = ifs.tellg(); // an alternate way to get size
    ifs.seekg(0, std::ios::beg);
    // read bytes
    std::vector<char> file_bytes(file_size);
    ifs.read(file_bytes.data(), file_size);
    // convert bytes to string_view
    std::string_view file_string(file_bytes.data(), file_size);
    logger.info("Read bytes from file: {}", file_bytes);
    logger.info("Read string from file: {}", file_string);
    ifs.close();

    // rename the file
    fs::path file2 = sandbox / "test2.csv";
    fs::rename(file, file2, ec);
    if (ec) {
      logger.error("Could not rename {} to {} - {}", file.string(), file2.string(), ec.message());
    } else {
      logger.info("Renamed {} to {}", file.string(), file2.string());
    }

    logger.info("Directory iterator:");
    logger.warn(
        "NOTE: directory_iterator is not implemented in esp-idf right now :( (as of v5.2.2)");
    // NOTE: directory_iterator is not implemented in esp-idf right now :(
    // directory_iterator can be iterated using a range-for loop
    for (auto const &dir_entry : fs::recursive_directory_iterator{sandbox, ec}) {
      logger.info("\t{}", dir_entry.path().string());
    }
    if (ec) {
      logger.error("Could not iterate over directory '{}': {}", sandbox.string(), ec.message());
      logger.info("\tThis is expected since directory_iterator is not implemented in esp-idf.");
    }

    // cleanup
    auto items = {file, file2, sandbox};
    for (const auto &item : items) {
      if (!fs::exists(item)) {
        logger.warn("Not removing '{}', it doesn't exist!", item.string());
        continue;
      }
      // and if it is a directory
      bool is_dir = fs::is_directory(item);
      auto err = is_dir ? rmdir(item.string().c_str()) : unlink(item.string().c_str());
      if (err) {
        logger.error("Could not remove {}, error code: {}", item.string(), err);
      } else {
        logger.info("Cleaned up {}", item.string());
      }
      // fs::remove(item, ec); // NOTE: cannot use fs::remove since it seems POSIX remove()
      // doesn't work
    }
    //! [file_system std filesystem example]
  }

  const std::string sandbox =
      std::string(espp::FileSystem::get().get_mount_point()) + "/" + std::string(test_dir);
  rmdir(sandbox.c_str());

  logger.info("file_system example complete!");

  while (true) {
    std::this_thread::sleep_for(1s);
  }
}
