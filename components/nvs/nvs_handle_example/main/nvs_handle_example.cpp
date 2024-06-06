#include <bitset>
#include <chrono>
#include <thread>
#include <vector>

#include "format.hpp"
#include "nvs.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
  // Init NVS
  std::error_code ec;
  espp::Nvs nvs;
  nvs.init(ec);
  ec.clear();
  
  // Open
  printf("\n");
  printf("Opening Non-Volatile Storage (NVS) handle... ");
  // Handle will automatically close when going out of scope or when it's reset.
  espp::NVSHandle storage("storage", ec);
  printf("Done\n");
  ec.clear();
  
  // Read
  printf("Reading restart counter from NVS ... ");
  int32_t restart_counter = 0; // value will default to 0, if not set yet in NVS
  storage.get("restart_counter", restart_counter, ec);
  if (ec) {
    printf("The value is not initialized yet!\n");
  } else {
    printf("Done\n");
    printf("Restart counter = %" PRIu32 "\n", restart_counter);
  }
  ec.clear();

  // Write
  printf("Updating restart counter in NVS ... ");
  restart_counter++;
  storage.set("restart_counter", restart_counter, ec);
  printf("Done\n");
  ec.clear();

  // Commit written value.
  // After setting any values, nvs_commit() must be called to ensure changes are written
  // to flash storage. Implementations may write to storage at other times,
  // but this is not guaranteed.
  printf("Committing updates in NVS ... ");
  storage.commit(ec);
  printf("Done\n");
  ec.clear();

  printf("\n");

  // Restart module
  for (int i = 10; i >= 0; i--) {
      printf("Restarting in %d seconds...\n", i);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();
}
