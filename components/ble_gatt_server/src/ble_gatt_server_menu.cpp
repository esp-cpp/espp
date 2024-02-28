#include "ble_gatt_server_menu.hpp"

#if CONFIG_COMPILER_CXX_EXCEPTIONS
#if CONFIG_BT_NIMBLE_ENABLED

std::unique_ptr<cli::Menu>
espp::make_ble_gatt_server_menu(std::reference_wrapper<espp::BleGattServer> server) {
  // add a menu to allow disconnect / unpairing
  auto ble_menu = std::make_unique<cli::Menu>("ble", "View and control BLE devices");
  ble_menu->Insert(
      "battery",
      [server](std::ostream &out) -> void {
        int battery_level = server.get().battery_service().get_battery_level();
        out << fmt::format("Battery level: {}%\n", battery_level);
      },
      "Print the battery level.");
  ble_menu->Insert(
      "battery",
      [server](std::ostream &out, int new_level) -> void {
        if (new_level < 0 || new_level > 100) {
          out << "Battery level must be between 0 and 100, inclusive.\n";
          return;
        }
        server.get().battery_service().set_battery_level(new_level);
        out << fmt::format("Battery set to {}%\n", new_level);
      },
      "Set the battery level.");
  ble_menu->Insert(
      "paired",
      [server](std::ostream &out) -> void {
        std::string output = "";
        auto num_bonds = NimBLEDevice::getNumBonds();
        output += fmt::format("Paired devices: {}\n", num_bonds);
        for (int i = 0; i < num_bonds; i++) {
          auto bond_addr = NimBLEDevice::getBondedAddress(i);
          output += fmt::format("Paired device: {}\n", bond_addr.toString());
        }
        out << output;
      },
      "Print the addresses of all paired BLE devices, if any.");
  ble_menu->Insert(
      "connected",
      [server](std::ostream &out) -> void {
        std::string output = "";
        auto nimble_server = server.get().server();
        auto peer_ids = nimble_server->getPeerDevices();
        output += fmt::format("Connected devices: {}\n", peer_ids.size());
        for (auto &peer_id : peer_ids) {
          auto peer = nimble_server->getPeerIDInfo(peer_id);
          output += fmt::format("Connected to: {}\n", peer.getAddress().toString());
        }
        out << output;
      },
      "Print the address of the connected BLE device, if any.");
  ble_menu->Insert(
      "disconnect",
      [server](std::ostream &out) -> void {
        std::string output = "";
        auto nimble_server = server.get().server();
        auto peer_ids = nimble_server->getPeerDevices();
        for (auto &peer_id : peer_ids) {
          auto peer = nimble_server->getPeerIDInfo(peer_id);
          output += fmt::format("Disconnecting from: {}\n", peer.getAddress().toString());
          nimble_server->disconnect(peer_id);
        }
        out << output;
      },
      "disconnect from the current BLE device");
  ble_menu->Insert(
      "unpair",
      [server](std::ostream &out) -> void {
        NimBLEDevice::deleteAllBonds();
        out << "Unpaired all devices\n";
      },
      "unpair from the current BLE device");
  return ble_menu;
}

#endif // CONFIG_BT_NIMBLE_ENABLED
#endif // CONFIG_COMPILER_CXX_EXCEPTIONS
