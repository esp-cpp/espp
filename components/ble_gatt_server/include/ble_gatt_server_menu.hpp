#pragma once

#include <sdkconfig.h>

#if CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
#if CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)

#include <functional>
#include <memory>
#include <string>

#include "NimBLEDevice.h"

#include "ble_gatt_server.hpp"
#include "cli.hpp"

namespace espp {
std::unique_ptr<cli::Menu> make_ble_gatt_server_menu(std::reference_wrapper<BleGattServer> server);
} // namespace espp
#endif // CONFIG_BT_NIMBLE_ENABLED || defined(_DOXYGEN_)
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS || defined(_DOXYGEN_)
