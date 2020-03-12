/* Copyright 2018 Canaan Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// LoBo: added CRC functions

#ifndef _DRIVER_UTILITY_H
#define _DRIVER_UTILITY_H

#include <stddef.h>
#include <stdint.h>
#ifdef __cplusplus
#include <type_traits>
#endif

#ifdef __cplusplus
extern "C"
{
#endif

#define readb(addr) (*(volatile uint8_t *)(addr))
#define readw(addr) (*(volatile uint16_t *)(addr))
#define readl(addr) (*(volatile uint32_t *)(addr))
#define readq(addr) (*(volatile uint64_t *)(addr))

#define writeb(v, addr)                      \
    {                                        \
        (*(volatile uint8_t *)(addr)) = (v); \
    }
#define writew(v, addr)                       \
    {                                         \
        (*(volatile uint16_t *)(addr)) = (v); \
    }
#define writel(v, addr)                       \
    {                                         \
        (*(volatile uint32_t *)(addr)) = (v); \
    }
#define writeq(v, addr)                       \
    {                                         \
        (*(volatile uint64_t *)(addr)) = (v); \
    }

    extern const uint32_t Crc32LookupTable[256];
    extern const uint16_t Crc16LookupTable[256];
    extern const uint8_t Crc8LookupTable[256];

    uint32_t get_bit_mask(volatile uint32_t *bits, uint32_t mask);
    void set_bit_mask(volatile uint32_t *bits, uint32_t mask, uint32_t value);
    uint32_t get_bit_idx(volatile uint32_t *bits, uint32_t idx);
    void set_bit_idx(volatile uint32_t *bits, uint32_t idx, uint32_t value);
    void busy_wait(uint64_t millionseconds);
    uint8_t hal_crc8(const void* data, size_t length, uint8_t previousCrc8);
    uint16_t hal_crc16(const void* data, size_t length, uint16_t previousCrc16);
    uint32_t hal_crc32(const void* data, size_t length, uint32_t previousCrc32);

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

namespace details
{
template <size_t Size>
struct aligned_storage;

template <>
struct aligned_storage<4>
{
    using type = uint32_t;
};

template <>
struct aligned_storage<8>
{
    using type = uint64_t;
};

template <size_t Size>
using aligned_storage_t = typename aligned_storage<Size>::type;
}

template <class T>
T read_pod(volatile T &src)
{
    union {
        details::aligned_storage_t<sizeof(T)> storage;
        T value;
    } data;
    
    data.storage = reinterpret_cast<volatile details::aligned_storage_t<sizeof(T)> &>(src);
    return data.value;
}

template <class T>
void write_pod(volatile T &dest, const T &value)
{
    auto &storage = reinterpret_cast<volatile details::aligned_storage_t<sizeof(T)> &>(dest);
    storage = reinterpret_cast<const details::aligned_storage_t<sizeof(T)> &>(value);
}

#endif

#endif /* _DRIVER_UTILITY_H */
