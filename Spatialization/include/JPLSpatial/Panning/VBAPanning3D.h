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

#include "JPLSpatial/Core.h"

#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Panning/VBAPLUT3D.h"

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/DirectionEncoding.h"
#include "JPLSpatial/Memory/Bits.h"
#include "JPLSpatial/Memory/Memory.h"

#include <vector>
#include <ranges>
#include <utility>
#include <limits>

namespace JPL
{
    /// Forward declaration
    namespace VBAP
    {
        template<class Traits = VBAPStandartTraits, auto cLUTType = VBAP::ELUTSize::KB_983>
        class Panning3D;
    } // namespace VBAP

    //======================================================================
    /// Alias for VBAP panner for panning to a 3D speaker layout
    /// (i.e. layout including top/bottom speakers)
    template<class Traits = VBAPStandartTraits, auto cLUTType = VBAP::ELUTSize::KB_983>
    using VBAPanner3D = VBAPannerBase<VBAP::Panning3D<Traits, cLUTType>, Traits>;

    namespace VBAP
    {
        //==================================================================
        /// VBAP panning traits for panning to a 3D speaker layout
        /// (i.e. layout including top/bottom speakers)
        template<class Traits, auto cLUTType>
        class Panning3D
        {
            static_assert(VBAP::CLUTType<decltype(cLUTType)> && "cLUTType must be enum class VBAP::ELUTSize");

            using Base = VBAPannerBase<Panning3D<Traits, cLUTType>, Traits>;
        public:
            using Vec3Type = typename Base::Vec3Type;

            //======================================================================
            using LUTCodec = Octahedron16Bit;
            static constexpr size_t cLUTSize = LUTCodec::cAxisRange * LUTCodec::cAxisRange;

            /// Packed look-up table of speaker triplets and corresponding gains.
            /// Index into the table is an octahedron-encoded direction vector.
            using LUTType = VBAP::LUT<cLUTType, cLUTSize, Vec3Type>;

            using LUTInterface =
                VBAP::LUTInterface<
                &Traits::GetChannelVector,
                LUTCodec,
                LUTType
                >;

            //======================================================================
            /// Implementation of the source layout for 2D panning
            struct SourceLayout : Base::VBAPLayoutBase
            {
                using LayoutBase = typename Base::VBAPLayoutBase;

                // Note:
                //  Initializing SourceLayout dimensions with these defaults results in
                //  'cMaxNumVirtualSources` directions to process when runtime data changes.
                //  See `Dimensions` comment
                static constexpr size_t cMaxNumRings = 8;                                       // Default max number of rings
                static constexpr size_t cMaxNumSamples = 32;                                    // Default max number of samples per ring
                static constexpr size_t cMaxNumVirtualSources = cMaxNumRings * cMaxNumSamples;  // Max number samples that can be generated for given VBAPdData
                static constexpr size_t cMaxNumVirtualSourcesSqrt = 16;                         // Max number samples/rings if computed dimentions are samples=rings
                static_assert(cMaxNumVirtualSourcesSqrt * cMaxNumVirtualSourcesSqrt == cMaxNumVirtualSources);

                /// Represents dimentions of the virtual source group cap on a 3D sphere.
                /// This structure serves to distribute "just enough" virtual sources
                /// per source channel to produce gap-less image for given target
                /// speaker/channel map.
                struct Dimensions
                {
                    /// Number of rings to generate for a spread cap
                    uint32 NumRings;

                    /// Number samples (virtual soruces) to generate for each ring of a spread cap
                    uint32 NumSamplesPerRing;

                    /// Get total number of virtual sources for these dimentions
                    [[nodiscard]] JPL_INLINE size_t GetSize() const noexcept { return static_cast<size_t>(NumRings) * NumSamplesPerRing; }

                    /// Calculate dimensions for `numChannels` with `maxGeoDistanceDot` between points
                    [[nodiscard]] static Dimensions For(uint32 numChannels, float maxGeoDistanceDot = std::numeric_limits<float>::max());
                };

            public:
                [[nodiscard]] JPL_INLINE Dimensions GetDimentions() const noexcept { return mDimensions; }
                [[nodiscard]] JPL_INLINE size_t GetNumVirtualSources() const noexcept;
                [[nodiscard]] JPL_INLINE static constexpr size_t GetMaxNumVirtualSources() noexcept;

            private:
                friend Base;
                /// Initialize VBAP data for standard channel map panning.
                /// (called by the panner that has the value for `shortestEdgeApertureDot`)
                bool Initialize(ChannelMap channelMap, ChannelMap targetMap, float shortestEdgeApertureDot = std::numeric_limits<float>::max());

                /// Generate canonical spread cap for given `spreadNormalized` value.
                /// Spread of the cap from the perspective of our MDAP implementation is the Focus parameter
                /// (called by paner in ProcessVBAPData)
                void GenerateSpreadCap(std::span<Vec3Type> outBuffer, float spreadNormalized) const;

            private:
                float mNominalChannelCapSpread;                 // Cached nominal per cap spread for given channel count
                Dimensions mDimensions;                         // Dimensions for virtual source distribution for a spread cap
                float mPhiTerm;                                 // Cached term to generate spread cap

                // Cached sin/cos to generate spread cap
                std::pmr::vector<std::pair<float, float>> mSamplesSinCos{ GetDefaultMemoryResource() };
            };

            //======================================================================
            /// @returns error message if `channelMap` is not a valid channel map
            /// for a VBAP panning target; nullopt otherwise
            [[nodiscard]] static inline std::optional<const char*> IsValidTargetChannelMap(ChannelMap channelMap)
            {
                if (!JPL_ENSURE(channelMap.HasTopChannels()))
                    return "Failed to initialize Panning3D, provided invalid target channel map. Panning3D requires channel map with top channels.";

                if (channelMap.GetNumChannels() < 3)
                    return "Trying to initialize with < 3 channels, 3D panning is not possible for such layout.";

                return std::nullopt;
            }
        };
    } // namespace VBAP
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

// It may or may not be desireable to add channel center
// direcection virtual source to the channel cap
// (in the center of the inner most ring)
#define JPL_ADD_CENTER_VS 0

namespace JPL::VBAP
{
    //==========================================================================
    namespace Internal
    {
        // Compute dot between neighbors for `numPoints` on a cap rim of `angularDiameter` of a cap.
        [[nodiscard]] static JPL_INLINE float ComputeNeigbourSampleDotFromN(float angularDiameter, uint32 numPoints)
        {
            if (numPoints <= 1)
                return 1.0f; // only one point: zero separation
            const float alpha = 0.5f * angularDiameter;
            const auto [s, c] = Math::SinCos(alpha);
            const float s2 = s * s;
            const float c2 = c * c;
            const float dphi = JPL_TWO_PI / static_cast<float>(numPoints);
            const float dot = std::clamp(s2 * std::cos(dphi) + c2, -1.0f, 1.0f);
            return dot;
        }

        // Geodesic angle between neighbors for N points.
        // Compute geodesic angle between neighbors for `numPoints` and `angularDiameter` of a cap.
        [[nodiscard]] static JPL_INLINE float ComputeNeighbourAngleFromN(float angularDiameter, uint32 numPoints)
        {
            return std::acos(ComputeNeigbourSampleDotFromN(angularDiameter, numPoints));
        }


        // Minimum number of points so that geodesic gap < m.
        // Calculate minimum number of points for cap of `angularDiameter`
        // so that geodesic gap between them is < m.
        [[nodiscard]] static JPL_INLINE uint32 ComputeMinNumPointsOnCapRing(float angularDiameter, float dotM, uint32 maxNumSamplesFallback)
        {
            const float alpha = 0.5f * angularDiameter;
            const auto [s, c] = Math::SinCos(alpha);
            const float s2 = s * s;
            const float c2 = c * c;

            if (!JPL_ENSURE(s2 != 0.0))
                return 1u; // degenerate cap: a point

            // target cos
            const float rhs = std::clamp((dotM - c2) / s2, -1.0f, 1.0f);
            const float dphi_max = std::acos(rhs);

            if (!JPL_ENSURE(dphi_max > 0.0f))
                return maxNumSamplesFallback; // m too small to place neighboring samples

            return static_cast<uint32>(std::ceil(JPL_TWO_PI / dphi_max));
        };
    }

    //==========================================================================
    template<class Traits, auto cLUTType>
    inline bool Panning3D<Traits, cLUTType>::SourceLayout::Initialize(ChannelMap channelMap, ChannelMap targetMap, float shortestEdgeApertureDot)
    {
        if (auto error = IsValidSourceChannelMap(channelMap))
        {
            JPL_ERROR_TAG("VBAPanner2D", error.value());
            return false;
        }

        // Sanitize input parameters, we don't use LFE for panning
        const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

        // Create sample distribution for the number of channels
        // and smallest geodesic distance between output channel vectors
        mDimensions = Dimensions::For(numChannels, shortestEdgeApertureDot);

        JPL_ASSERT(mDimensions.GetSize() <= cMaxNumVirtualSources);

        mNominalChannelCapSpread = [](uint32 numChannels, uint32 numRings)
        {
            // This compensation ensures the gap between neigbouring
            // channel zones equals to distance between rings
            const uint32 channelSpreadComp = numChannels / 2;
            return static_cast<float>(numRings) / (numRings * numChannels - channelSpreadComp);
        }(numChannels, mDimensions.NumRings);

        // Precompute phi term to generte rings
        mPhiTerm = JPL_PI / (mDimensions.NumRings + 1);
        // +1 avoids last rim at spread 1.0 to fall onto a single point
        // and to distribute positions more evenly, since each position
        // is "centre" rather than edge of the contribution to the field

        // Precompute azimuth of each ring
        {
            const uint32 quadrantSamples = mDimensions.NumSamplesPerRing >> 2;
            mSamplesSinCos.resize(quadrantSamples);

            const float thetaTerm = JPL_TWO_PI / mDimensions.NumSamplesPerRing;

            float theta = thetaTerm * 0.5f; // start with an offset to make mirroring work
            for (auto& sampleCS : mSamplesSinCos)
            {
                sampleCS = Math::SinCos(theta);
                theta += thetaTerm;
            }
        }

        const auto numVirtualSorucesPerChannel = static_cast<uint32>(mDimensions.GetSize());
        return LayoutBase::InitializeBase(channelMap, targetMap, numVirtualSorucesPerChannel);
    }

    template<class Traits, auto cLUTType>
    inline void Panning3D<Traits, cLUTType>::SourceLayout::GenerateSpreadCap(std::span<Vec3Type> outBuffer, float spreadNormalized) const
    {
        /*
            TOTAL COST (8/32 ring/samples):
                - 8 sin/cos
                - 130 mul
                - 10 add
                    - 2 bit shifts
                    - 192 flipping sing of a float (x = -x)

            At 4.5 GHz roughly 1274 cycles, ~0.28 us or 0.00028 ms
        */

        JPL_ASSERT(outBuffer.size() >= mDimensions.GetSize() + JPL_ADD_CENTER_VS);

        // Small static buffer should be enough
        static constexpr size_t BUF_SIZE = SourceLayout::cMaxNumSamples;
        std::pair<float, float> ringSCsBuffer[BUF_SIZE];

        JPL_ASSERT(Math::IsPositiveAndBelow(mDimensions.NumRings, BUF_SIZE + 1));

        auto ringCSs = ringSCsBuffer | std::views::take(mDimensions.NumRings);

        // Precompute centre of each ring
        {
            const float capSpread = mNominalChannelCapSpread * (1.0f - spreadNormalized);
            const float phiTerm = capSpread * mPhiTerm;

            float phi = phiTerm;
            for (auto& ringCS : ringCSs)
            {
                ringCS = Math::SinCos(phi);
                phi += phiTerm;
            }
        }

        Vec3Type* destination = outBuffer.data();

        // Generate canon sphere quadrant
        for (const auto& [sinPhi, cosPhi] : ringCSs)
        {
            for (const auto& [sinTheta, cosTheta] : mSamplesSinCos)
            {
                *destination++ = Vec3Type{
                    sinPhi * cosTheta,
                    sinPhi * sinTheta,
                    cosPhi
                };
            }
        }

        // Generate the rest of sphere by mirroring vectors
        {
            const uint32 quadrantSamples = mDimensions.NumSamplesPerRing >> 2;
            const uint32 quadrantSize = mDimensions.NumRings * quadrantSamples;
            const uint32 halfSphereSize = quadrantSize << 1;

            // Duplicate first quadrant
            std::copy(outBuffer.begin(), outBuffer.begin() + quadrantSize, destination);

            // Mirror duplicated first quadrant to make a half sphere
            for (Vec3Type& mirror : std::span(destination, quadrantSize))
                mirror.X = -mirror.X;

            // Advance destination buffer pointer
            destination += quadrantSize;

            // Duplicate half sphere
            std::copy(outBuffer.begin(), outBuffer.begin() + halfSphereSize, destination);

            // Mirror half sphere to make a whole sphere
            for (Vec3Type& mirror : std::span(destination, halfSphereSize))
                mirror.Y = -mirror.Y;
        }

        /* TODO: BETTER DISTRIBUTION.We could use a better distribution of samples, to keep the density more or less equal, or just different.
                                    - right now 8 samples at the 1st ring is much more dense than 8 samples at the ring around poles
                                    - this may or may not be desireable for the resulting gains
        */

#if JPL_ADD_CENTER_VS
        // Add center point
        outBuffer[i] = Vec3(0.0f, 0.0f, 1.0f);
#endif
    }

    template<class Traits, auto cLUTType>
    JPL_INLINE size_t Panning3D<Traits, cLUTType>::SourceLayout::GetNumVirtualSources() const noexcept
    {
        return mDimensions.GetSize() + JPL_ADD_CENTER_VS;
    }

    template<class Traits, auto cLUTType>
    JPL_INLINE constexpr size_t Panning3D<Traits, cLUTType>::SourceLayout::GetMaxNumVirtualSources() noexcept
    {
        return cMaxNumVirtualSources + JPL_ADD_CENTER_VS;
    }

    //==========================================================================
    template<class Traits, auto cLUTType>
    inline Panning3D<Traits, cLUTType>::SourceLayout::Dimensions Panning3D<Traits, cLUTType>::SourceLayout::Dimensions::For(uint32 numChannels, float maxGeoDistanceDot)
    {
        // Note: Initializing SourceLayout dimensions with default constant dimensions (8, 32)
        // results in 256 directions to process when runtime data changes, which can be a lot.
        // 
        // So we look for the minimal viable number of rings and samples per ring,
        // while avoiding situation where we have no sources between any 2 speakers.
        // So we find the smallest angular distance between too consecutive speakers
        // and make sure the largest possible geodesic distance between 2 samples is smaller.
        // 
        // (this would not be needed for user-defined VS maps, only for the default ones that use spread/focus)

        uint32 totalRings = static_cast<uint32>(cMaxNumRings);
        uint32 samplesPerRing = static_cast<uint32>(cMaxNumSamples);

        if (maxGeoDistanceDot < std::numeric_limits<float>::max())
        {
            JPL_ASSERT(maxGeoDistanceDot >= -1.000001f && maxGeoDistanceDot <= 1.000001f);

            // 1. Estimate number of rings required to avoid inactive speaker edges

            // Since we distribute our channels along the equator,
            // we can just divide the circumference by max allowed aperture
            const float m = std::acos(maxGeoDistanceDot); // geodesic threshold (radians)
            totalRings = static_cast<uint32>(std::ceil(JPL_TWO_PI / std::max(m, 1e-6f)));

            // circumference of a cap rim given angular diameter d
            // static auto getCapCircumference = [](float d) { return JPL_TWO_PI * std::sin(0.5f * d); };

            // 2. Estimate number of samples per ring required to avoid inactive speaker edges

            if (numChannels <= 2)
            {
                // For 1-2 channels we can reuse the same distribution as totalRings
                // since each channel's cap can reach a circumference between poles
                // at spread 1.0, focus 0.0.
                samplesPerRing = totalRings;

                // For high channel count target (i.e. small min speaker aperture)
                // we can overflow max virtual sources, so we need to clamp it
                if (RoundUpBy4(samplesPerRing) * RoundUpBy4(totalRings) > cMaxNumVirtualSources)
                {
                    samplesPerRing = totalRings = cMaxNumVirtualSourcesSqrt;
                }
            }
            else
            {

                // ..while for > 2 channels we need to compute geodesic distance
                // to estimate minimum number of samples per ring

                const float angularDiameter = JPL_TWO_PI / numChannels;
                samplesPerRing = Internal::ComputeMinNumPointsOnCapRing(angularDiameter, maxGeoDistanceDot, cMaxNumSamples);

#if defined(JPL_DEBUG) || defined(JPL_TEST)
                {
                    const float dotActual = Internal::ComputeNeigbourSampleDotFromN(angularDiameter, samplesPerRing);
                    // because of ceil quantization, actual neighbor dot should always be >= requested
                    JPL_ASSERT(dotActual + 1e-6f >= maxGeoDistanceDot);
                }
#endif // JPL_DEBUG || JPL_TEST
            }
        }

        // TODO: totalRings can be smaller than numChannels,
        // if we're willing to sacrifice vectorization,
        // we can move max out:
        //      std::max(RoundUpBy4(totalRings / numChannels), 2u))

        // Create sample distribution for the number of channels.
        return Dimensions{
            .NumRings = RoundUpBy4(std::max(totalRings / numChannels, 1u)),
            .NumSamplesPerRing = RoundUpBy4(samplesPerRing)
        };
    }

} // namespace JPL::VBAP