/*
The MIT License
Copyright (c) 2019 Lehrstuhl Informatik 11 - RWTH Aachen University
Modifications Copyright (c) 2026 ATDev
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

This file is part of embeddedRTPS.

Author: i11 - Embedded Software, RWTH Aachen University
*/

#ifndef RTPS_CONFIG_H
#define RTPS_CONFIG_H

#ifdef RTPS_CONFIG_HEADER
#include RTPS_CONFIG_HEADER
#else
#if defined(ESP_PLATFORM)
#include "rtps/config_esp32.hpp"
#else
#include "rtps/config_desktop.hpp"
#endif
#endif

// Storage POLICY, orthogonal to the limits profile selected above. Dynamic
// (heap-backed, grow-on-full std::deque) storage is the default on host/PC
// builds; on ESP it is an explicit opt-in (Kconfig RTPS_STORAGE_DYNAMIC ->
// -DRTPS_STORAGE_DYNAMIC), so the MCU keeps zero-heap, deterministic history by
// default no matter which limits profile is chosen. The limits headers set
// capacity caps only and never enable dynamic storage by themselves. Define
// RTPS_STORAGE_STATIC to force static storage on a host build. Neither policy
// changes any bytes on the wire.
#if !defined(RTPS_STORAGE_DYNAMIC) && !defined(RTPS_STORAGE_STATIC) && !defined(ESP_PLATFORM)
#define RTPS_STORAGE_DYNAMIC
#endif

// A participant's two SEDP writers keep one change per live user endpoint
// (the 3 builtin writers and 3 builtin readers of a participant are not
// announced). With static storage the ring cannot grow, so it must hold every
// endpoint the participant can have: an overwritten announcement would
// un-discover that endpoint for every peer joining later. The writers refuse
// to overwrite (StatefulWriter::newChange) and this makes the two caps agree
// at compile time; raise HISTORY_SIZE_STATEFUL along with the per-participant
// endpoint caps.
#if !defined(RTPS_STORAGE_DYNAMIC)
static_assert(rtps::Config::HISTORY_SIZE_STATEFUL + 3 >= rtps::Config::NUM_WRITERS_PER_PARTICIPANT,
              "HISTORY_SIZE_STATEFUL must cover the user writers of a participant "
              "(NUM_WRITERS_PER_PARTICIPANT - 3 builtin) so SEDP never drops a live announcement");
static_assert(rtps::Config::HISTORY_SIZE_STATEFUL + 3 >= rtps::Config::NUM_READERS_PER_PARTICIPANT,
              "HISTORY_SIZE_STATEFUL must cover the user readers of a participant "
              "(NUM_READERS_PER_PARTICIPANT - 3 builtin) so SEDP never drops a live announcement");
#endif

#endif // RTPS_CONFIG_H
