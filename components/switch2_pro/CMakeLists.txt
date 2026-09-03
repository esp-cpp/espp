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
