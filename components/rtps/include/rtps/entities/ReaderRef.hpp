/*
The MIT License
Copyright (c) 2026 ATDev
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE
*/

#ifndef RTPS_ENTITIES_READER_REF_HPP
#define RTPS_ENTITIES_READER_REF_HPP

#include "rtps/config.hpp"
#include <array>
#include <cstdint>

namespace rtps {
class Reader;

/// A local reader resolved for a received submessage, with the slot
/// generation captured under the participant lock (see
/// Participant::getReader(id, generation_out)): the *IfCurrent dispatch
/// wrappers reject the delivery if the endpoint was deleted, and its slot
/// possibly reused, after the lookup.
struct ReaderRef {
  Reader *reader{nullptr};
  uint32_t generation{0};
};

/// Every reader a submessage can be for: a participant holds at most
/// NUM_READERS_PER_PARTICIPANT.
using ReaderRefs = std::array<ReaderRef, Config::NUM_READERS_PER_PARTICIPANT>;
} // namespace rtps

#endif // RTPS_ENTITIES_READER_REF_HPP
