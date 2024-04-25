set(CLI_INCLUDES "include")

if (CONFIG_COMPILER_CXX_EXCEPTIONS)
  list(APPEND CLI_INCLUDES "../../external/cli/include")
else()
  message(WARNING "C++ exceptions are disabled, CLI is disabled")
endif()

idf_component_register(
  INCLUDE_DIRS ${CLI_INCLUDES}
  SRC_DIRS "src"
  REQUIRES driver vfs logger)
