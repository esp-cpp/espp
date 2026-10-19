/*
 * The data in this file is the STMicroelectronics VL53L5CX firmware and default
 * configuration shipped with the VL53L5CX Ultra Lite Driver (ULD, API revision
 * VL53L5CX_1.1.2; taken from https://github.com/stm32duino/VL53L5CX v1.2.3,
 * src/vl53l5cx_buffers.h). It is redistributed under its original license:
 *
 * Copyright (c) 2021 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#pragma once

#include <cstddef>
#include <cstdint>

/// Private to the vl53l component: the VL53L5CX firmware image and the default
/// configuration / crosstalk buffers the driver uploads at initialization.
namespace espp::vl53l5cx_detail {
inline constexpr size_t kFirmwareSize = 86016;
inline constexpr size_t kDefaultConfigurationSize = 972;
inline constexpr size_t kDefaultXtalkSize = 776;
inline constexpr size_t kGetNvmCommandSize = 40;
extern const uint8_t kFirmware[kFirmwareSize];         ///< VL53L5CX (63 deg FoV)
extern const uint8_t kFirmwareVl53l7cx[kFirmwareSize]; ///< VL53L7CX (90 deg FoV), same size
extern const uint8_t kDefaultConfiguration[kDefaultConfigurationSize];
extern const uint8_t kDefaultXtalk[kDefaultXtalkSize];
extern const uint8_t kGetNvmCommand[kGetNvmCommandSize];
} // namespace espp::vl53l5cx_detail
