#include "heap_monitor.hpp"

using namespace espp;

std::string HeapMonitor::get_region_name(int heap_flags) {
  std::string name = "Heap( ";
  if (heap_flags & MALLOC_CAP_DEFAULT) {
    name += "DEFAULT ";
  }
  if (heap_flags & MALLOC_CAP_INTERNAL) {
    name += "INTERNAL ";
  }
  if (heap_flags & MALLOC_CAP_SPIRAM) {
    name += "SPIRAM ";
  }
  if (heap_flags & MALLOC_CAP_DMA) {
    name += "DMA ";
  }
  if (heap_flags & MALLOC_CAP_8BIT) {
    name += "8BIT ";
  }
  if (heap_flags & MALLOC_CAP_32BIT) {
    name += "32BIT ";
  }
  if (heap_flags & MALLOC_CAP_RTCRAM) {
    name += "RTCRAM ";
  }
  if (heap_flags & MALLOC_CAP_TCM) {
    name += "TCM ";
  }
  if (heap_flags & MALLOC_CAP_DMA_DESC_AHB) {
    name += "DMA AHB ";
  }
  if (heap_flags & MALLOC_CAP_DMA_DESC_AXI) {
    name += "DMA AXI ";
  }
  if (heap_flags & MALLOC_CAP_CACHE_ALIGNED) {
    name += "CACHE_ALIGNED ";
  }
  if (heap_flags & MALLOC_CAP_SIMD) {
    name += "SIMD ";
  }
  name += ")";
  return name;
}

HeapMonitor::HeapInfo HeapMonitor::get_info(int heap_flags) {
  multi_heap_info_t info;
  heap_caps_get_info(&info, heap_flags);
  HeapInfo heap_info;
  heap_info.heap_flags = heap_flags;
  heap_info.free_bytes = info.total_free_bytes;
  heap_info.min_free_bytes = info.minimum_free_bytes;
  heap_info.largest_free_block = info.largest_free_block;
  heap_info.allocated_bytes = info.total_allocated_bytes;
  heap_info.total_size = heap_caps_get_total_size(heap_flags);
  return heap_info;
}

std::vector<HeapMonitor::HeapInfo> HeapMonitor::get_info(const std::vector<int> &heap_flags) {
  std::vector<HeapInfo> heap_info_list;
  for (const auto &flags : heap_flags) {
    heap_info_list.push_back(get_info(flags));
  }
  return heap_info_list;
}

std::string HeapMonitor::get_table(const std::vector<int> &heap_flags) {
  std::string table;
  table += get_table_header() + "\n";
  auto info_list = get_info(heap_flags);
  for (const auto &info : info_list) {
    table += fmt::format("{:t}\n", info);
  }
  return table;
}

std::string HeapMonitor::get_csv(const std::vector<int> &heap_flags) {
  std::string table;
  table += get_csv_header() + "\n";
  auto info_list = get_info(heap_flags);
  for (const auto &info : info_list) {
    table += fmt::format("{:c}\n", info);
  }
  return table;
}
