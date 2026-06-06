//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPL Spatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2026 Jaroslav Pevno, JPL Spatial is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#pragma once

#include <JPLSpatial/Core.h>

#include <limits>

#if defined(JPL_COMPILER_MSVC) and (JPL_CPU_ADDRESS_BITS == 64)
#include <intrin.h> // _umul128
#endif

namespace JPL
{
    struct WyRand
    {
        using result_type = uint64;

        uint64 State = 0;

        explicit WyRand(uint64 seed) noexcept
            : State(seed)
        {}

        static constexpr result_type min() { return 0; }
        static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }

        result_type operator()() noexcept
        {
            State += 0xa0761d6478bd642full;
            return wymum(State, State ^ 0xe7037ed1a0b428dbull);
        }

    private:
        static uint64 wymum(uint64 a, uint64 b) noexcept
        {
#if defined(JPL_COMPILER_MSVC) and (JPL_CPU_ADDRESS_BITS == 64)

#if defined(JPL_CPU_X86)
            
            uint64 high = 0;
            const uint64 low = _umul128(a, b, &high);
            return high ^ low;

#elif defined(JPL_CPU_ARM)
            
            const uint64 high = __umulh(a, b);
            const uint64 low = a * b;
            return high ^ low;

#else
#Error "Unsupported CPU architecture."
#endif

#elif defined(__SIZEOF_INT128__)
            const __uint128_t p = static_cast<__uint128_t>(a) * b;
            return static_cast<uint64>(p >> 64) ^ static_cast<uint64>(p);

#else
            // Portable fallback: 64x64 -> 128 multiply using 32-bit limbs.
            const uint64 a0 = static_cast<uint32>(a);
            const uint64 a1 = a >> 32;
            const uint64 b0 = static_cast<uint32>(b);
            const uint64 b1 = b >> 32;

            const uint64 p00 = a0 * b0;
            const uint64 p01 = a0 * b1;
            const uint64 p10 = a1 * b0;
            const uint64 p11 = a1 * b1;

            const uint64 middle = (p00 >> 32) + static_cast<uint32>(p01) + static_cast<uint32>(p10);

            const uint64 low =
                (p00 & 0xffffffffull) | (middle << 32);

            const uint64 high =
                p11 + (p01 >> 32) + (p10 >> 32) + (middle >> 32);

            return high ^ low;
#endif
        }
    };

    struct SplitMix64
    {
        using result_type = uint64;

        uint64 State;

        explicit SplitMix64(uint64 seed) : State(seed) {}

        static constexpr result_type min() { return 0; }
        static constexpr result_type max() { return std::numeric_limits<result_type>::max(); }

        result_type operator()() noexcept
        {
            uint64 z = (State += 0x9e3779b97f4a7c15ull);
            z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ull;
            z = (z ^ (z >> 27)) * 0x94d049bb133111ebull;
            return z ^ (z >> 31);
        }
    };

    using Rand = WyRand;
} // namespace JPL
