//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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
#include <JPLSpatial/Math/Math.h>
#include <JPLSpatial/Math/SIMD.h>

#include <span>
#include <cstdint>

namespace JPL
{
    //==========================================================================
    /// Variance estimation utility
    template<class T>
    class Variance
    {
    public:
        // Welford’s online variance algorithm
        struct Online
        {
            uint32_t Count = 0;
            T Mean = T(0.0);
            T M2 = T(0.0);

            inline constexpr void Add(T x) noexcept
            {
                ++Count;
                const T delta = x - Mean;
                Mean += delta / Count;
                const T delta2 = x - Mean;
                M2 += delta * delta2;
            }

            inline constexpr T GetVariance() const noexcept
            {
                return (Count > 1) ? (M2 / (Count - 1)) : T(0);
            }
            
            inline constexpr T GetSNR() const noexcept
            {
                if (Count <= 1)
                    return 0.0f; // Not enough data

                const T variance = (M2 / (Count - 1));
                return variance == T(0) ? T(1) : Mean / Math::Sqrt(variance);
            }

            inline constexpr T GetSNR_r0() const noexcept
            {
                if (Count <= 1)
                    return T(0); // Not enough data

                const T variance = (M2 / (Count - 1));
                return variance == T(0) ? T(0) : Mean / Math::Sqrt(variance);
            }
        };

        static inline constexpr T ComputeFor(std::span<const T> values) noexcept
        {
            const auto count = static_cast<uint32_t>(values.size());
            if (count <= 1)
                return T(0); // Not enough samples to compute variance

            T sum = T(0);
            for (T v : values)
                sum += v;

            const T mean = sum / count;

            T sumSq = T(0);
            for (T v : values)
                sumSq += (v - mean) * (v - mean);

            return sumSq / (count - 1);
        }

        static inline constexpr T ComputeSNRFor(std::span<const T> values) noexcept
        {
            const auto count = static_cast<uint32_t>(values.size());
            if (count <= 1)
                return T(0);

            T sum = T(0);
            for (T v : values)
                sum += v;

            const T mean = sum / count;

            T sumSq = T(0);
            for (T v : values)
                sumSq += (v - mean) * (v - mean);

            const T variance = sumSq / (count - 1);

            return variance == T(0) ? T(1) : mean / Math::Sqrt(variance);
        }
    };

    using OnlineVariance = typename Variance<float>::Online;
    using OnlineVarianceSIMD = typename Variance<simd>::Online;
} // namespace JPL