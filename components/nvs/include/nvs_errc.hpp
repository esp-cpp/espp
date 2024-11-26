#pragma once

#include <string>
#include <system_error>

namespace {
/**
 * @brief Custom C++ error codes used for NVS operations.
 */
enum class NvsErrc {
  // no 0
  Namespace_Length_Too_Long = 10,
  Key_Length_Too_Long,
  Open_NVS_Handle_Failed,
  Write_NVS_Failed,
  Commit_NVS_Failed,
  Read_NVS_Failed,
  Key_Not_Found,
  Init_NVS_Failed,
  Erase_NVS_Failed,
  Handle_Uninitialized,
  Erase_NVS_Key_Failed,
  Erase_NVS_Namespace_Failed,
};

struct NvsErrCategory : std::error_category {
  const char *name() const noexcept override;
  std::string message(int ev) const override;
};

const char *NvsErrCategory::name() const noexcept { return "nvs"; }

std::string NvsErrCategory::message(int ev) const {
  switch (static_cast<NvsErrc>(ev)) {
  case NvsErrc::Namespace_Length_Too_Long:
    return "Namespace too long, must be <= 15 characters";

  case NvsErrc::Key_Length_Too_Long:
    return "Key too long, must be <= 15 characters";

  case NvsErrc::Open_NVS_Handle_Failed:
    return "Failed to open NVS handle";

  case NvsErrc::Write_NVS_Failed:
    return "Failed to write to NVS";

  case NvsErrc::Commit_NVS_Failed:
    return "Failed to commit to NVS";

  case NvsErrc::Read_NVS_Failed:
    return "Failed to read from NVS";

  case NvsErrc::Key_Not_Found:
    return "Key not found in NVS";

  case NvsErrc::Init_NVS_Failed:
    return "Failed to initialize NVS";

  case NvsErrc::Erase_NVS_Failed:
    return "Failed to erase NVS";

  case NvsErrc::Handle_Uninitialized:
    return "Handle not initialized";

  case NvsErrc::Erase_NVS_Key_Failed:
    return "Failed to erase NVS key";

  case NvsErrc::Erase_NVS_Namespace_Failed:
    return "Failed to erase NVS namespace";

  default:
    return "(unrecognized error)";
  }
}

const NvsErrCategory theNvsErrCategory{};

/**
 * @brief overload of std::make_error_code used by custom error codes.
 */
[[maybe_unused]] std::error_code make_nvs_error_code(NvsErrc e) {
  return {static_cast<int>(e), theNvsErrCategory};
}

} // namespace
