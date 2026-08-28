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
#if JPL_HAS_PATH_TRACING
#include "JPLSpatial/Core.h"
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/PathTracing/SphericalHarmonics.h"
#include "JPLSpatial/Utilities/TypeUtilities.h"
#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Utilities/Variance.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <vector>
#include <span>

namespace JPL
{
    struct NullPanner {};

    template<class T>
    concept CPannerType = Type::CIsSpecializationOf<T, VBAPannerBase> || std::same_as<T, NullPanner>;

    template<CPannerType PannerType, class EnergyType = float>
    class EnergyResponseChannels;

    template<CPannerType PannerType> using EnergyResponseChannelsFloat = EnergyResponseChannels<PannerType, float>;
    template<CPannerType PannerType> using EnergyResponseChannelsSIMD = EnergyResponseChannels<PannerType, simd>;
    
    using EnergyResponseHistogramFloat = EnergyResponseChannels<NullPanner, float>;
    using EnergyResponseHistogramSIMD = EnergyResponseChannels<NullPanner, simd>;

    class EnergyResponseBase
    {
    protected:
        EnergyResponseBase(float deltaTime, float maxTime)
            : mTimeStep(deltaTime)
            , mTimeStepInv((deltaTime > 0.0f) ? (1.0f / deltaTime) : 0.0f)
            , mNumBins(std::size_t(std::ceil(maxTime * mTimeStepInv)) + 1)
        {
        }

    public:
        float GetTimeStep() const { return mTimeStep; }
        std::size_t GetNumBins() const { return mNumBins; }
        // Get the time (center) of bin i
        JPL_INLINE float TimeAtBin(int i) const { return (i + 0.5f) * mTimeStep; }
        JPL_INLINE int GetBinFor(float time) const { return static_cast<int>(time * mTimeStepInv); }
        JPL_INLINE float GetTotalTimeCapacity() const { return mTimeStep * mNumBins; }

    protected:
        const float mTimeStep;
        const float mTimeStepInv;
        const std::size_t mNumBins;
    };

    /// Channel based energy response
    template<CPannerType PannerType, class EnergyType>
    class EnergyResponseChannels : public EnergyResponseBase
    {
        static const inline NullPanner sNullPanner{};
    public:
        static_assert(std::same_as<EnergyType, float> || std::same_as<EnergyType, simd>);

        EnergyResponseChannels(float deltaTime, float maxTime, const PannerType& panner) requires (not std::same_as<PannerType, NullPanner>)
            : EnergyResponseBase(deltaTime, maxTime)
            , mPanner(panner)
            , mBins(panner.GetNumChannels() * mNumBins, 0.0f, GetDefaultMemoryResource())
        {
            JPL_ASSERT(mPanner.IsInitialized(), "Panner must be initialized.");
        }

        EnergyResponseChannels(float deltaTime, float maxTime) requires (std::same_as<PannerType, NullPanner>)
            : EnergyResponseBase(deltaTime, maxTime)
            , mPanner(sNullPanner)
            , mBins(mNumBins, 0.0f, GetDefaultMemoryResource())
        {
        }

        ~EnergyResponseChannels() = default;

        // Add energy contribution at a specific time and direction
        int AddSample(float time, const EnergyType& energy, const auto& dir)
        {
            const auto bin = GetBinFor(time);
            if (bin < 0 || bin >= mNumBins)
            {
                return bin;
            }

            if constexpr (std::same_as<PannerType, NullPanner>)
            {
                mBins[bin] += energy;
            }
            else
            {
                float buffer[256]; // 256 channels should be enough
                JPL_ASSERT(mPanner.GetNumChannels() <= 256);

                const uint32 numChannels = mPanner.GetNumChannels();
                std::span gains(buffer, numChannels);

                // Get panning gains
                mPanner.GetSpeakerGains(dir, gains);

                // Accumulate panned energy
                for (uint32 i = 0; i < numChannels; ++i)
                {
                    mBins[bin * numChannels + i] += energy * gains[i];
                }
            }

            return bin;
        }

        // Retrieve the raw bin array for post‐processing
        JPL_INLINE std::span<const EnergyType> GetBins() const { return mBins; }

        void Clear()
        {
            std::fill(mBins.begin(), mBins.end(), 0.0f);
        }

        // Bin stride is just the number of channels the historgram records for
        uint32 GetBinStride() const
        {
            if constexpr (std::same_as<PannerType, NullPanner>)
            {
                return 1u;
            }
            else
            {
                return mPanner.GetNumChannels();
            }
        }

    private:
        const PannerType& mPanner;
        std::pmr::vector<EnergyType> mBins;
    };

    /// Spherical harmonic energy response
    template<int SphericalHarmonicOrder = 3>
    class EnergyResponse : public EnergyResponseBase
    {
    public:
		static_assert(SphericalHarmonicOrder >= 1 && SphericalHarmonicOrder <= SphericalHarmonics::DegreeLimit,
					  "SphericalHarmonicOrder must be between 1 and 4 inclusive.");

        // Third‐order (L=3) -> 16 coeffs
        static constexpr int SH_ORDER = std::min(std::max(SphericalHarmonicOrder, 1), 4);   // clamped to [1, 4]
        static constexpr int SH_COEFFS = OrderToNumSH(SH_ORDER);                            // 16 for 4×4 SH

    public:
        EnergyResponse(float deltaTime, float maxTime)
            : EnergyResponseBase(deltaTime, maxTime)
        {
            mBins.fill(std::vector<float>(mNumBins, 0.0f));
        }

        ~EnergyResponse() = default;

        // dir must be unit‐length
        template<class Vec3Type>
        static void EvaluateSH(const Vec3Type& dir, std::span<float, SH_COEFFS> outCoeffs)
        {
            int idx = 0;
            for (int l = 0; l <= SH_ORDER; ++l)
            {
                for (int m = -l; m <= l; ++m)
                {
                    outCoeffs[idx++] = SphericalHarmonics::Evaluate(l, m, &dir.X);
                }
            }
        }

        // Add energy contribution at a specific time and direction
        template<class Vec3Type>
        int AddSample(float time, float energy, const Vec3Type& dir)
        {
            const auto bin = GetBinFor(time);
            if (bin < 0 || bin >= mNumBins)
            {
                return bin;
            }

            // Evaluate each basis function Y_c(dir)
            float sh[SH_COEFFS];
            EvaluateSH(dir, sh);

            for (int c = 0; c < SH_COEFFS; ++c)
            {
                mBins[c][bin] += energy * sh[c];
            }

            return bin;
        }

        // Retrieve the raw bin array for post‐processing
        const std::array<std::vector<float>, SH_COEFFS>& GetBins() const { return mBins; }

        void Clear()
        {
			for (std::vector<float>& bin : mBins)
			{
				std::fill(bin.begin(), bin.end(), 0.0f);
			}
        }

    private:
        // bins[c][b] = coefficient c at time‐bin b
        std::array<std::vector<float>, SH_COEFFS> mBins;
    };


    class EnergyVarianceHistogram : public EnergyResponseBase
    {
    public:
        EnergyVarianceHistogram(float deltaTime, float maxTime)
            : EnergyResponseBase(deltaTime, maxTime)
            , mBins(mNumBins, 0.0f)
            , mVariance(mNumBins)
        {
        }

        ~EnergyVarianceHistogram() = default;

        // Add energy contribution at a specific time
        template<class Vec3Type>
        int AddSample(float time, float energy, const Vec3Type&)
        {
            const auto bin = GetBinFor(time);
            if (bin >= 0 && bin < mNumBins)
            {
                mBins[bin] += energy;
                mVariance[bin].Add(energy);
            }

            return bin;
        }

        // Retrieve the raw bin array for post‐processing
        std::span<const float> GetBins() const { return mBins; }

        void Clear()
        {
            std::fill(mBins.begin(), mBins.end(), 0.0f);
            std::fill(mVariance.begin(), mVariance.end(), OnlineVariance{});
        }

        const std::vector<OnlineVariance>& GetVariance() const { return mVariance; }

    private:
        std::vector<float> mBins;
        std::vector<OnlineVariance> mVariance;
    };
} // namespace JPL
#endif