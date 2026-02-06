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

#include "JPLSpatial/Math/Math.h"
//#include "JPLSpatial/Math/MathMD.h"
#include <gtest/gtest.h>

#include <random>
#include <vector>

namespace JPL
{
    //======================================================================
    using JPL::Math::Sqrt;
    using JPL::Math::InvSqrt;

    //======================================================================
    /// Helpers

    template <class T>
    static T rel_err(T a, T b)
    {
        if (std::isnan(a) && std::isnan(b)) return T(0);
        if (a == b) return T(0);
        T denom = std::max<T>(Math::Abs(a), Math::Abs(b));
        if (denom == T(0)) return Math::Abs(a - b); // both ~0
        return Math::Abs(a - b) / denom;
    }

    template <class T>
    static std::vector<T> make_random_vec(size_t n, T lo, T hi, uint32_t seed = 0xC0FFEEu)
    {
        std::mt19937 gen(seed);
        std::uniform_real_distribution<T> dis(lo, hi);
        std::vector<T> v(n);
        for (size_t i = 0; i < n; ++i)
        {
#if JPL_SQRT_ASSUME_POSITIVE_FINITE
            v[i] = Math::Abs(dis(gen));
#else
            v[i] = dis(gen);
#endif
        }
        return v;
    }

    //======================================================================
    /// Sqrt

    TEST(Sqrt, Constexpr_Evaluates)
    {
        // constexpr path (also checks double-seed fix)
        static constexpr float  f0 = Sqrt<float, 3>(0.0f);
        static constexpr float  f4 = Sqrt<float, 2>(4.0f);
        static constexpr double d4 = Sqrt<double, 4>(4.0);

        static_assert(f0 == 0.0f, "sqrt(0) should be 0");
        static_assert(f4 > 2.0f - 1e-6f && f4 < 2.0f + 1e-6f, "sqrt(4) ~ 2");
        static_assert(d4 > 2.0 - 1e-12 && d4 < 2.0 + 1e-12, "sqrt(4) ~ 2");
    }

    //======================================================================
    /// InvSqrt

    TEST(InvSqrt, Constexpr_Evaluates)
    {
        // Match tidy values (uses Sqrt's constexpr polish for float)
        {
            static constexpr float a = InvSqrt<float, 2>(4.0f);
            static constexpr float b = 1.0f / Sqrt<float, 2>(4.0f);
            static_assert(a > 0.5f - 1e-6f && a < 0.5f + 1e-6f, "1/sqrt(4) ~ 0.5");
            static_assert(Math::Abs(a - b) < 1e-6f, "InvSqrt matches 1/Sqrt at constexpr");
        }
        {
            static constexpr double a = InvSqrt<double, 3>(9.0);
            static constexpr double b = 1.0 / Sqrt<double, 3>(9.0);
            static_assert(a > (1.0 / 3.0) - 1e-11 && a < (1.0 / 3.0) + 1e-11, "1/sqrt(9) ~ 1/3");
            static_assert(Math::Abs(a - b) < 1e-12, "InvSqrt matches 1/Sqrt at constexpr (double)");
        }
    }

    //======================================================================
    /// SIMD array vs scalar
#if 0 //? not used for now
    // SSE
#if defined(JPL_USE_SSE)
    TEST(InvSqrtArray, SSE_NR1_MatchesScalar)
    {
        const size_t N = (1 << 14) + 3; // force a non-multiple-of-4 tail
        auto xs = make_random_vec<float>(N, 1e-8f, 1e8f);
        std::vector<float> out_vec(N), out_sca(N);

        Math::InvSqrtArray<1>(xs.data(), out_vec.data(), N);
        for (size_t i = 0; i < N; ++i)
            out_sca[i] = InvSqrt<float, 1>(xs[i]);

        double max_rel = 0.0;
        for (size_t i = 0; i < N; ++i)
            max_rel = std::max(max_rel, double(rel_err(out_vec[i], out_sca[i])));
        EXPECT_LT(max_rel, 3e-6) << "SSE InvSqrtArray<1> deviates from scalar";
    }

    TEST(InvSqrtArray, SSE_NR2_MatchesScalar_Tighter)
    {
        const size_t N = (1 << 14) + 1;
        auto xs = make_random_vec<float>(N, 1e-8f, 1e8f);
        std::vector<float> out_vec(N), out_sca(N);

        Math::InvSqrtArray<2>(xs.data(), out_vec.data(), N);
        for (size_t i = 0; i < N; ++i)
            out_sca[i] = InvSqrt<float, 2>(xs[i]);

        double max_rel = 0.0;
        for (size_t i = 0; i < N; ++i)
            max_rel = std::max(max_rel, double(rel_err(out_vec[i], out_sca[i])));
        EXPECT_LT(max_rel, 1e-6) << "SSE InvSqrtArray<2> deviates from scalar";
    }
#else
    TEST(InvSqrtArray, SSE_Skipped) { SUCCEED() << "SSE not enabled"; }
#endif

    // NEON
#if defined(JPL_USE_NEON)
    TEST(InvSqrtArray, NEON_NR1_MatchesScalar)
    {
        const size_t N = (1 << 14) + 5;
        auto xs = make_random_vec<float>(N, 1e-8f, 1e8f);
        std::vector<float> out_vec(N), out_sca(N);

        Math::InvSqrtArray<1>(xs.data(), out_vec.data(), N);
        for (size_t i = 0; i < N; ++i)
            out_sca[i] = InvSqrt<float, 1>(xs[i]);

        double max_rel = 0.0;
        for (size_t i = 0; i < N; ++i)
            max_rel = std::max(max_rel, double(rel_err(out_vec[i], out_sca[i])));
        EXPECT_LT(max_rel, 3e-6) << "NEON InvSqrtArray<1> deviates from scalar";
    }

    TEST(InvSqrtArray, NEON_NR2_MatchesScalar_Tighter)
    {
        const size_t N = (1 << 14) + 2;
        auto xs = make_random_vec<float>(N, 1e-8f, 1e8f);
        std::vector<float> out_vec(N), out_sca(N);

        Math::InvSqrtArray<2>(xs.data(), out_vec.data(), N);
        for (size_t i = 0; i < N; ++i)
            out_sca[i] = InvSqrt<float, 2>(xs[i]);

        double max_rel = 0.0;
        for (size_t i = 0; i < N; ++i)
            max_rel = std::max(max_rel, double(rel_err(out_vec[i], out_sca[i])));
        EXPECT_LT(max_rel, 1e-6) << "NEON InvSqrtArray<2> deviates from scalar";
    }
#else
    TEST(InvSqrtArray, NEON_Skipped) { SUCCEED() << "NEON not enabled"; }
#endif
#endif
} // namespace JPL