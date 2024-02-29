#include "gfps.hpp"

// static handle to nvs storage for the embedded namespace
static constexpr char *nvs_namespace_embedded = "embedded";
static constexpr char *nvs_stored_key_names[] = {"KeyList", "Name"};
static nvs_handle_t nvs_handle_embedded;

// Loads stored key
//
// key    - Type of key to fetch.
// output - Buffer to contain retrieved key.
// length - On input, contains the size of the output buffer.
//          On output, contains the Length of key.
nearby_platform_status nearby_platform_LoadValue(nearby_fp_StoredKey key, uint8_t *output,
                                                 size_t *length) {
  esp_err_t err = nvs_get_blob(nvs_handle_embedded, nvs_stored_key_names[key], output, length);
  if (err != ESP_OK) {
    return kNearbyStatusError;
  }
  return kNearbyStatusOK;
}

// Saves stored key
//
// key    - Type of key to store.
// output - Buffer containing key to store.
// length - Length of key.
nearby_platform_status nearby_platform_SaveValue(nearby_fp_StoredKey key, const uint8_t *input,
                                                 size_t length) {
  esp_err_t err = nvs_set_blob(nvs_handle_embedded, nvs_stored_key_names[key], input, length);
  if (err != ESP_OK) {
    return kNearbyStatusError;
  }
  return kNearbyStatusOK;
}

// Initializes persistence module
nearby_platform_status nearby_platform_PersistenceInit() {
  // open the NVS "embedded" namespace and store the handle
  esp_err_t err = nvs_open(nvs_namespace_embedded, NVS_READWRITE, &nvs_handle_embedded);
  if (err != ESP_OK) {
    return kNearbyStatusError;
  }
  return kNearbyStatusOK;
}
