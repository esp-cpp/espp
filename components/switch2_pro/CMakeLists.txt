idf_component_register(
  INCLUDE_DIRS "include"
  SRC_DIRS "src"
  REQUIRES base_component ble_gatt_server esp-nimble-cpp timer
  PRIV_REQUIRES mbedtls)

# Opt-in: patch the prebuilt BLE controller library to accept the console's
# sub-spec 5 ms connection interval. Off by default. Covers the RISC-V NimBLE
# controller (C6/C61/C2/H2, libble_app.a) and the BTDM/RivieraWaves controller
# (S3/C3, libbtdm_app.a) — the patcher picks the right object + byte pattern per
# target. Mutates the global $IDF_PATH install, so it is deliberately explicit
# and never silent.
if(CONFIG_SWITCH2_PRO_PATCH_NIMBLE_5MS)
  if(IDF_TARGET STREQUAL "esp32c6" OR IDF_TARGET STREQUAL "esp32c61"
     OR IDF_TARGET STREQUAL "esp32c2" OR IDF_TARGET STREQUAL "esp32h2"
     OR IDF_TARGET STREQUAL "esp32s3" OR IDF_TARGET STREQUAL "esp32c3")
    message(WARNING
      "[switch2_pro] SWITCH2_PRO_PATCH_NIMBLE_5MS is ON: patching the prebuilt "
      "BLE controller library in $ENV{IDF_PATH} for a 5 ms connection interval "
      "(${IDF_TARGET}). This modifies your global ESP-IDF install; run "
      "tools/patch_nimble_5ms.py --target ${IDF_TARGET} --restore to undo.")
    find_package(Python3 COMPONENTS Interpreter REQUIRED)
    execute_process(
      COMMAND ${Python3_EXECUTABLE}
              ${CMAKE_CURRENT_LIST_DIR}/tools/patch_nimble_5ms.py
              --idf-path $ENV{IDF_PATH} --target ${IDF_TARGET}
      RESULT_VARIABLE _switch2_patch_result)
    if(NOT _switch2_patch_result EQUAL 0)
      message(FATAL_ERROR "[switch2_pro] 5 ms controller patch failed (${_switch2_patch_result})")
    endif()
  else()
    message(WARNING
      "[switch2_pro] SWITCH2_PRO_PATCH_NIMBLE_5MS has no effect on ${IDF_TARGET}: "
      "no known controller patch for this target (supported: C6/C61/C2/H2 NimBLE, "
      "S3/C3 BTDM).")
  endif()
endif()

# ESP32-S3 / C3: reconnect and wake-from-sleep need the console's sub-spec (5 ms)
# connection interval to be accepted by the closed BTDM controller. The official,
# default-on way is CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE, which requires
# ESP-IDF >= v6.1 (or the v6.0.2 / v5.5.x / v5.4.x / v5.3.x backports;
# espressif/esp-idf#18467). Warn at configure time if neither that option nor the
# legacy binary patch is enabled — reconnect/wake will otherwise silently fail
# (fresh pairing and first-session input still work).
if(IDF_TARGET STREQUAL "esp32s3" OR IDF_TARGET STREQUAL "esp32c3")
  if(NOT CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE
     AND NOT CONFIG_SWITCH2_PRO_PATCH_NIMBLE_5MS)
    idf_build_get_property(_switch2_idf_ver IDF_VERSION)
    message(WARNING
      "[switch2_pro] ${IDF_TARGET}: neither CONFIG_BT_CTRL_BLE_MIN_CONN_INTERVAL_ENABLE "
      "(official ESP-IDF sub-spec-interval support) nor SWITCH2_PRO_PATCH_NIMBLE_5MS is "
      "enabled, so the console's 5 ms reconnect/wake will not work (fresh pairing + input "
      "still do). Update to ESP-IDF >= v6.1 (you have ${_switch2_idf_ver}) for the "
      "default-on option, or enable the legacy patch. See espressif/esp-idf#18467.")
  endif()
endif()
