#include <rtps/utils/Diagnostics.hpp>

namespace rtps {
namespace Diagnostics {

namespace ThreadPool {
std::atomic<uint32_t> dropped_incoming_packets_usertraffic{0};
std::atomic<uint32_t> dropped_incoming_packets_metatraffic{0};

std::atomic<uint32_t> dropped_outgoing_packets_usertraffic{0};
std::atomic<uint32_t> dropped_outgoing_packets_metatraffic{0};

std::atomic<uint32_t> processed_incoming_metatraffic{0};
std::atomic<uint32_t> processed_outgoing_metatraffic{0};
std::atomic<uint32_t> processed_incoming_usertraffic{0};
std::atomic<uint32_t> processed_outgoing_usertraffic{0};

std::atomic<uint32_t> max_ever_elements_outgoing_usertraffic_queue{0};
std::atomic<uint32_t> max_ever_elements_incoming_usertraffic_queue{0};

std::atomic<uint32_t> max_ever_elements_outgoing_metatraffic_queue{0};
std::atomic<uint32_t> max_ever_elements_incoming_metatraffic_queue{0};

} // namespace ThreadPool

namespace StatefulReader {
std::atomic<uint32_t> sfr_unexpected_sn{0};
std::atomic<uint32_t> sfr_retransmit_requests{0};
} // namespace StatefulReader

namespace Writer {
std::atomic<uint32_t> history_overwrite_drops{0};
} // namespace Writer

namespace Network {
std::atomic<uint32_t> lwip_allocation_failures{0};
}

namespace OS {
// Declared in the header since the component consolidation (#712) but never
// defined - any ODR-use was an undefined symbol at link time.
std::atomic<uint32_t> current_free_heap_size{0};
} // namespace OS

namespace SEDP {
std::atomic<uint32_t> max_ever_remote_participants{0};
std::atomic<uint32_t> current_remote_participants{0};

std::atomic<uint32_t> max_ever_matched_reader_proxies{0};
std::atomic<uint32_t> current_max_matched_reader_proxies{0};

std::atomic<uint32_t> max_ever_matched_writer_proxies{0};
std::atomic<uint32_t> current_max_matched_writer_proxies{0};

std::atomic<uint32_t> max_ever_unmatched_reader_proxies{0};
std::atomic<uint32_t> current_max_unmatched_reader_proxies{0};

std::atomic<uint32_t> max_ever_unmatched_writer_proxies{0};
std::atomic<uint32_t> current_max_unmatched_writer_proxies{0};
} // namespace SEDP

} // namespace Diagnostics
} // namespace rtps
