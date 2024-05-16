#include <bitset>
#include <chrono>
#include <thread>
#include <vector>

#include "format.hpp"
#include "nvs.hpp"

using namespace std::chrono_literals;

extern "C" void app_main(void) {
    std::this_thread::sleep_for(200ms);
    fmt::print("Starting NVS example!\n");
    //! [NVS example]
    std::error_code ec;
    uint8_t counter = 0;
    espp::Nvs nvs;
    nvs.init(ec);
    ec.clear();
    //note that the namespace and key strings must be <= 15 characters
    nvs.get_or_set_var("system", "reset_counter", counter, counter, ec); 
    ec.clear();
    fmt::print("Reset Counter = {}\n", counter);

    counter ++;

    if(counter > 10) {
        nvs.erase_and_refresh(ec);
        counter = 0;
        fmt::print("NVS erased, Reset Counter set to 0\n");
        ec.clear();
    }

    nvs.set_var("system", "reset_counter", counter, ec);
    fmt::print("Next Reset Counter will be = {}\n", counter);
    fmt::print("NVS example complete!\n");
    //! [NVS example]

    while (true) {
        std::this_thread::sleep_for(1s);
    }
}
