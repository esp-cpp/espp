#include "espp.hpp"

using namespace std;

int main() {
  espp::Logger logger({.tag = "espp::FileSystem", .level = espp::Logger::Verbosity::DEBUG});

  logger.info("File system test");

  // test the file system is able to list the directory
  auto &fs = espp::FileSystem::get();

  std::filesystem::path mount_point = espp::FileSystem::get_mount_point();
  auto sandbox = mount_point;

  logger.info("Mount point: {}", mount_point.string());

  std::error_code ec;

  // list files in a directory
  logger.info("Directory iterator:");
  // NOTE: directory_iterator is not implemented in esp-idf right now :(
  // directory_iterator can be iterated using a range-for loop
  for (auto const &dir_entry : std::filesystem::recursive_directory_iterator{sandbox, ec}) {
    logger.info("\t{}", dir_entry.path().string());
  }
  if (ec) {
    logger.error("Could not iterate over directory '{}': {}", sandbox.string(), ec.message());
    logger.info("\tThis is expected since directory_iterator is not implemented in esp-idf.");
  }

  logger.info("Recursive directory listing:");
  auto &espp_fs = espp::FileSystem::get();
  auto files = espp_fs.get_files_in_path(sandbox, true, true);
  for (const auto &f : files) {
    logger.info("\t{}", f);
  }

  espp::FileSystem::ListConfig config;
  std::string directory_listing = fs.list_directory(sandbox, config);
  logger.info("Directory listing for {}:\n{}", sandbox, directory_listing);

  // list all files recursively
  config.recursive = true;
  auto root = fs.get_root_path();
  std::string root_listing = fs.list_directory(root, config);
  logger.info("Recursive directory listing for {}:\n{}", root.string(), root_listing);

  return 0;
}
