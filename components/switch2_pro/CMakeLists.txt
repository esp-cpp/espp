idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component ble_gatt_server esp-nimble-cpp nvs task timer
  # esp_hw_support (esp_mac.h) is only included by the sources, so it is private
  PRIV_REQUIRES mbedtls esp_hw_support)

# ESP32-S3 / C3: reconnect and wake-from-sleep need the console's sub-spec (5 ms)
# connection interval to be accepted by the closed BTDM controller. The official,
# default-on way is CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE, which requires
# ESP-IDF >= v6.1 (or the v6.0.2 / v5.5.x / v5.4.x / v5.3.x backports;
# espressif/esp-idf#18467). Warn at configure time if it is not enabled:
# sustained input, reconnect and wake will otherwise silently fail (fresh
# pairing and the first ~1.5 s of input still work).
if(IDF_TARGET STREQUAL "esp32s3" OR IDF_TARGET STREQUAL "esp32c3")
  if(NOT CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE)
    idf_build_get_property(_switch2_idf_ver IDF_VERSION)
    message(WARNING
      "[switch2_pro] ${IDF_TARGET}: CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE (official "
      "ESP-IDF sub-spec-interval support) is not enabled. The console drives the link at 5 ms for SUSTAINED input (it drops even the "
      "fresh session to 5 ms ~1.5 s after subscription) as well as reconnect/wake, so only "
      "the initial pairing handshake (~15 ms) will work here — sustained input, reconnect, "
      "and wake will fail. Update to ESP-IDF >= v6.1 (you have ${_switch2_idf_ver}) for the "
      "default-on option. See espressif/esp-idf#18467.")
  endif()
endif()
