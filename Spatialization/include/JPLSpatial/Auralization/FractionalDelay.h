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
#include "JPLSpatial/ErrorReporting.h"

#include <span>

namespace JPL
{
    //==========================================================================
    /// Nearest fractional interpolator
    class NearestInterpolator
    {
    public:
        NearestInterpolator() = default;

        static constexpr uint32 InputLength = 1;

        /// @param fraction must be in (0,1).
        inline void SetFraction(float /*fraction*/)
        {
            //JPL_ASSERT(fraction >= 0.0f && fraction <= 1.0f);
            //mFraction = static_cast<uint32_t>(fraction + 0.5f);
        }

        inline float GetFraction() const { return 0.0f; }

        /// Process one taken from the integer tap of the delay line.
        /// @param data: must point to contiguous buffer of one sample.
        inline float Process(std::span<const float, InputLength> data) const noexcept
        {
            return data[0];
        }

        inline float Process(float data) const noexcept
        {
            return data;
        }

    private:
        //uint32_t mFraction = 0;
    };

    //==========================================================================
    /// Linear fractional interpolator
    class LinearInterpolator
    {
    public:
        LinearInterpolator() = default;

        inline static constexpr uint32 InputLength = 2;

        /// @param fraction must be in (0, 1).
        inline void SetFraction(float fraction)
        {
            JPL_ASSERT(fraction >= 0.0f && fraction <= 1.0f);
            mFraction = fraction;
        }

        inline float GetFraction() const { return mFraction; }

        /// Process two samples taken from the integer tap of the delay line.
        /// @param data: must point to contiguous buffer of two last samples
        /// from newest to oldest.
        inline float Process(std::span<const float, InputLength> data) const noexcept
        {
            const float a = data[0];
            const float b = data[1];
            return a + mFraction * (b - a);
        }

    private:
        float mFraction = 0.0f;
    };

    //==========================================================================
    /// First-order Thiran fractional–delay all-pass interpolator
    class Thiran1stInterpolator
    {
    public:
        Thiran1stInterpolator() = default;

        static constexpr uint32 InputLength = 2;

        /// @param fraction must be in [0,1).
        inline void SetFraction(float fraction)
        {
            JPL_ASSERT(fraction >= 0.0f && fraction < 1.0f);
            mA = (1.0f - fraction) / (1.0f + fraction);
            mFraction = fraction;
        }

        inline float GetFraction() const { return mFraction; }

        /// Process two samples taken from the integer tap of the delay line.
        /// @param data: must point to contiguous buffer of two last samples
        /// from oldest to newest.
        inline float Process(std::span<const float, InputLength> data) noexcept
        {
            const float x = data[0];              // current input x[n]
            const float x1 = data[1];             // previous input x[n-1]
            const float y = mA * (x - mY1) + x1;  // all-pass difference eq.
            mY1 = y;                              // y[n-1] <- y[n]
            return y;
        }

        /// Zero the state.
        inline void Reset(float v = 0.0f) noexcept
        {
            mY1 = v;
        }

    private:
        float mA = 0.0f;    // Thiran coefficient
        float mY1 = 0.0f;   // previous output y[n-1]
        float mFraction = 0.0f;
    };


} // namespace JPL
