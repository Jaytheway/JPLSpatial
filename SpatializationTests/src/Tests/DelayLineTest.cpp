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

#include "JPLSpatial/Auralization/DelayLine.h"

#include <gtest/gtest.h>

#include <span>
#include <array>
#include <format>

namespace JPL
{
	class DelayLineTest : public testing::Test
	{
	protected:
		DelayLineTest() = default;
        
        inline float saw(uint32_t n) { return static_cast<float>(n); }

        // MaxDelay < WindowSize  would be undefined for an interpolated delay line,
        // so we start at WindowSize+1
        // small  (Ring = 8)
        // medium (Ring = 16)
        // large  (Ring = 512)
        static constexpr std::array<uint32_t, 3> DelayLengths{ 7, 14, 500 }; // test delays

        template<class Function>
        static void ForEachDelay(Function&& func)
        {
            for (uint32_t D : DelayLengths)
            {
                SCOPED_TRACE(std::format("Testing delay: {}", D));
                func(D);
            }
        }
    };

    // Test 0 : check that the internal buffer of delay line is filled correctly
    TEST_F(DelayLineTest, DelayLineBufferFill)
    {
        ForEachDelay([this](uint32_t maxDelaySamples)
        {
            DelayLine<> dl(maxDelaySamples);

            for (uint32_t n = 0; n < maxDelaySamples; ++n)
            {
                const float sample = saw(n);
                dl.Push(sample);
            }

            for (uint32_t n = 0; n < maxDelaySamples; ++n)
            {
                const float sample = saw(n);
                const float sampleD = dl.GetReadWindow<0>(maxDelaySamples - 1 - n);
                EXPECT_FLOAT_EQ(sampleD, sample);
            }
        });
    }

    // Test 1 : basic push/read with zero delay
    TEST_F(DelayLineTest, ZeroDelayReturnsLatestSample)
    {
        ForEachDelay([this](uint32_t maxDelaySamples)
        {
            DelayLine<> dl(maxDelaySamples);
            // pre-fill at least WindowSize samples so the saw() data is present
            for (uint32_t n = 0; n < DelayLine<>::WindowSize; ++n)
                dl.Push(saw(n));

            for (uint32_t n = DelayLine<>::WindowSize; n < 200; ++n)
            {
                const float sample = saw(n);
                dl.Push(sample);
                std::span<const float> p = dl.GetReadWindow<DelayLine<>::WindowSize>(0);
                EXPECT_FLOAT_EQ(p[0], sample);
            }
        });
    }

    // Test 2 : fixed integer delay, no wrap
    TEST_F(DelayLineTest, FixedDelayNoWrap)
    {
        ForEachDelay([this](uint32_t maxDelaySamples)
        {
            DelayLine<> dl(maxDelaySamples);
            const uint32_t D = dl.GetSize() / 2; // ( < MaxDelay )

            const uint32_t preFillSize = D + DelayLine<>::WindowSize;

            // pre-fill
            uint32_t n = 0;
            for (n = 0; n < preFillSize; ++n)
            {
                const float sample = saw(n);
                dl.Push(sample);
            }
           
            for (uint32_t nb = 0; nb < preFillSize; ++nb)
            {
                float sampleD = dl.GetReadWindow<0>(preFillSize - 1 - nb);
                EXPECT_FLOAT_EQ(sampleD, saw(nb));
            }

            for (; n < dl.GetSize() - 1; ++n)
            {
                dl.Push(saw(n));
                std::span<const float> p = dl.GetReadWindow<DelayLine<>::WindowSize>(n - 1);
                EXPECT_FLOAT_EQ(p[0], saw(DelayLine<>::WindowSize - 1));
                // last sample must equal to first sample pushed
                EXPECT_FLOAT_EQ(p[DelayLine<>::WindowSize - 1], saw(0));
            }
        });
    }

    // Test 3 : wrap-around behaviour (delay near MaxDelay)
    TEST_F(DelayLineTest, WrapStillContiguous)
    {
        ForEachDelay([this](uint32_t maxDelaySamples)
        {
            SCOPED_TRACE(std::format("Testing wrap-around for delay: {}", maxDelaySamples));

            constexpr uint32_t K = DelayLine<>::WindowSize;
            ASSERT_TRUE(maxDelaySamples >= K)
                << "maxDelaySamples must be at least WindowSize (" << K << ")";

            DelayLine<> dl(maxDelaySamples);
            const uint32_t Ring = dl.GetSize();      // 2^k >= maxDelay
            const uint32_t Mask = Ring - 1;

            // Choose three delays: just >=K, middle, and almost Ring
            const uint32_t Dmin = K;
            const uint32_t Dmid = Ring / 2;
            const uint32_t Dmax = Ring - 3;
            const std::array<uint32_t, 3> delays{ Dmin, Dmid, Dmax };

            // Pre-fill so every delay has valid history
            const uint32_t prime = Dmax + 2 * K;
            for (uint32_t n = 0; n < prime; ++n)
                dl.Push(saw(n));

            // ───── Wrap the write pointer a few times
            const uint32_t total = prime + 4 * Ring;
            for (uint32_t n = prime; n < total; ++n)
            {
                dl.Push(saw(n));
                const uint32_t wr = dl.GetWriteIndex();

                for (uint32_t D : delays)
                {
                    // Reference ring-start index "inside the ring zone" (0…Ring-1)
                    // Note: if we're decrementing index on write, we need to +offset
                    const uint32_t offset = D + K;
                    const uint32_t start = (wr + Ring + offset + 1) & Mask;

                    std::span<const float> p = dl.GetReadWindow<DelayLine<>::WindowSize>(D + K);

                    // 1) Pointer must address inside the whole allocation
                    uintptr_t pAddr = reinterpret_cast<uintptr_t>(p.data());
                    uintptr_t bufBase = reinterpret_cast<uintptr_t>(dl.raw());
                    uintptr_t bufEnd = bufBase + sizeof(float) * (Ring + K);
                    EXPECT_GE(pAddr, bufBase);
                    EXPECT_LT(pAddr + sizeof(float) * K, bufEnd);

                    // 2) Next K samples must equal the buffer’s own data
                    for (uint32_t i = 0; i < K; ++i)
                        EXPECT_FLOAT_EQ(p[i], dl.raw()[start + i])
                        << "n=" << n
                        << "  D=" << D
                        << "  i=" << i
                        << "  Ring=" << Ring;
                }
            }
        });
	}

    TEST_F(DelayLineTest, Tap_LinearInterpolation)
    {
        DelayLine<20> dl(14);
        static constexpr float frac = 0.25f; // quarter of a sample

        auto li = dl.CreateTap<LinearInterpolator>(0.0f);
        li.SetDelay(frac);

        // push impulse
        dl.Push(0.0f);
        // push one more frame so wr == 2
        dl.Push(1.0f);

        // window now spans slots [0,1]  => contains 1.0 & 0.0
        std::span<const float> p = dl.GetReadWindow<DelayLine<>::WindowSize>(0);
        EXPECT_FLOAT_EQ(p[0], 1.0f);   // 1.0 (newest)
        EXPECT_FLOAT_EQ(p[1], 0.0f);   // 0.0 (oldest)

        // get value 0.25 samples back in time
        auto y = li.Process(dl);
        EXPECT_FLOAT_EQ(y, 0.75f);
    }

    // TODO: tests for other kinds of interpolators


} // namespace JPL