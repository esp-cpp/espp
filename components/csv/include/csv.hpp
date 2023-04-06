#pragma once

#define __gnu_linux__

#include "csv2/reader.hpp"
#include "csv2/writer.hpp"

namespace espp {

/**
 * @brief Comma Separated Value (CSV) reader/writer convenience wrapper around
 *        <a href="https://github.com/p-ranav/csv2">p-ranav/csv2</a> which
 *        exposes csv2::Reader and csv2::Writer classes for managing efficient
 *        (lazy-loaded) parsing and serizaliztion of human-readable
 *        CSV-formatted data.
 *
 * @note This class does not really exist or do anything, but it's the only
 *       way I could figure out how to get this documentation built into the
 *       system :(
 *
 * \section csv_ex1 CSV Reader Example
 * \snippet csv_example.cpp csv reader example
 * \section csv_ex2 Complex CSV Writer Example
 * \snippet csv_example.cpp csv writer example
 */
class __csv_documentation__ {};
} // namespace espp
