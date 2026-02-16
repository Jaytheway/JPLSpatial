//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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

#include "JPLSpatial/Core.h"
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Auralization/CrossoverFilter.h"

#include <gtest/gtest.h>

#include <vector>
#include <cmath>
#include <random>

namespace JPL
{
    // Helpers
    static inline float rms(const std::vector<float>& x)
    {
        long double acc = 0.0;
        for (float v : x)
            acc += (long double)v * v;
        return std::sqrt((double)(acc / std::max<size_t>(1, x.size())));
    }
    static inline simd rms(const std::vector<simd>& x)
    {
        simd acc = 0.0;
        for (simd v : x)
            acc += v * v;
        return Math::Sqrt(acc / simd(float(std::max<size_t>(1, x.size()))));
    }
    static inline float MaxAbsDiff(const std::vector<float>& a, const std::vector<float>& b)
    {
        float m = 0.0f;
        const size_t n = std::min(a.size(), b.size());
        for (size_t i = 0; i < n; ++i)
            m = std::max(m, std::abs(a[i] - b[i]));
        return m;
    }
    static inline void MmakeImpulse(std::vector<float>& x)
    {
        std::fill(x.begin(), x.end(), 0.0f);
        if (!x.empty())
            x[0] = 1.0f;
    }
    static inline void MakeSine(std::vector<float>& x, float sampleRate, float f)
    {
        const float w = JPL_TWO_PI * f / sampleRate;
        float p = 0.0f;
        for (size_t i = 0; i < x.size(); ++i)
        {
            x[i] = std::sin(p);
            p += w;
        }
    }

    // TODO: more/better tests

    TEST(FourBandLR4, RecombinationImpulseRMSNoChange)
    {
        FourBandCrossover split;
        static constexpr float sampleRate = 48000.0f;
        split.Prepare(sampleRate);
        static constexpr int N = 4096;
        std::vector<float> in(N), out(N);
        MmakeImpulse(in);

        // Unity gains, fused path
        split.ProcessBlock(in, simd(1.0f), out);

        const float diff = std::abs(rms(in) - rms(out));
        EXPECT_LT(diff, 1e-6f);
    }

    TEST(FourBandLR4, EnergyExtractionVsRecombine_Noise)
    {
        FourBandCrossover split;
        static constexpr float sampleRate = 48000.0f;
        split.Prepare(sampleRate);

        static constexpr int N = 8192;
        std::vector<float> in(N), out(N);
        std::vector<simd> b(N);

        // white noise
        std::mt19937 rng(12345);
        std::uniform_real_distribution<float> U(-1.0f, 1.0f);
        for (int i = 0; i < N; ++i)
            in[i] = U(rng);

        // SoA pass
        split.ProcessBlock(in, b);

        // Recombine from SoA with unity
        for (int i = 0; i < N; ++i)
            out[i] = b[i].reduce();

        // Compare to fused unity recombination (reference)
        std::vector<float> outRef(N);
        split.Reset();
        split.ProcessBlock(in, simd(1.0f), outRef);

        const float err = rms(out) - rms(outRef); // magnitudes should match closely
        EXPECT_NEAR(err, 0.0f, 1e-4f);

        const float maxErr = MaxAbsDiff(out, outRef);
        EXPECT_LT(maxErr, 1e-4f);
    }
} // namespace JPL