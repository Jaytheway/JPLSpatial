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
#include "JPLSpatial/ErrorReporting.h"

#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Panning/VBAPLUT2D.h"

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"
#include "JPLSpatial/Math/Vec3Buffer.h"
#include "JPLSpatial/Memory/Bits.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <span>

namespace JPL
{
    /// Forward declaration
    namespace VBAP
    {
        template<class Traits = VBAPStandartTraits>
        class Panning2D;
    } // namespace VBAP


    //======================================================================
    /// Alias for VBAP panner for panning to a plane speaker layout
    /// (i.e. no top/bottom channels, only floor plane)
    template<class Traits = VBAPStandartTraits>
    using VBAPanner2D = VBAPannerBase<VBAP::Panning2D<Traits>, Traits>;

    namespace VBAP
    {
        //==================================================================
        /// VBAP panning traits for panning to a plane speaker layout
        /// (i.e. no top/bottom channels, only floor plane)
        template<class Traits>
        class Panning2D
        {
            using Base = VBAPannerBase<Panning2D<Traits>, Traits>;
        public:
            using Vec3Type = typename Base::Vec3Type;

            //==================================================================
            using LUTType = VBAP::LUT2D;

            using LUTInterface =
                VBAP::LUTInterface2D<
                &Traits::GetChannelVector,
                &Traits::GetChannelAngle
                >;

            //=================================================================
            /// Implementation of the source layout for 2D panning
            struct SourceLayout : Base::VBAPLayoutBase
            {
                using LayoutBase = typename Base::VBAPLayoutBase;

                static constexpr size_t cMaxNumVirtualSources = 32;
            public:

                [[nodiscard]] JPL_INLINE size_t GetNumVirtualSources() const noexcept { return mNumVirtualSourcesPerChannel; }
                [[nodiscard]] static JPL_INLINE constexpr size_t GetMaxNumVirtualSources() noexcept { return cMaxNumVirtualSources; }

                /// Rerturns minimal distance between two virtual source samples.
                /// This is mostly needed for debugging and verification.
                /// Must be called only after the Source Layout has been initialized by a panner.
                [[nodiscard]] JPL_INLINE float GetMinDistanceBetweenSamples() const noexcept { return mPhiTerm; }

            private:
                friend Base;
                /// Initialize VBAP data for standard channel map panning.
                /// (called by the panner that has the value for `shortestEdgeApertureDot`)
                bool Initialize(ChannelMap channelMap, ChannelMap targetMap, float shortestEdgeApertureDot = std::numeric_limits<float>::max());

                /// Generate canonical spread cap for given `spreadNormalized` value.
                /// Spread of the cap from the perspective of our MDAP implementation is the Focus parameter
                /// (called by paner in ProcessVBAPData)
                void GenerateSpreadCap(Vec3SIMDBufferView& outBuffer, float spreadNormalized) const;
#if 0           // scalar version for a reference
                void GenerateSpreadCap(std::span<Vec3Type> outBuffer, float spreadNormalized) const;
#endif

            private:
                uint32 mNumVirtualSourcesPerChannel;
                
                // Here phi is the distance between two samples on a ring (2 * PI / num samples)
                float mPhiTerm;     // Cached term to generate spread cap
                float mOffsetTerm;
            };

            //==================================================================
            /// @returns error message if `channelMap` is not a valid channel map
            /// for a VBAP panning target; nullopt otherwise
            [[nodiscard]] static inline std::optional<const char*> IsValidTargetChannelMap(ChannelMap channelMap)
            {
                if (!channelMap.IsValid())
                    return "Channel map is invalid.";

                // TODO: do we want to use the same class for both, 2D and 3D cases and just swap internal handling?
                if (!JPL_ENSURE(!channelMap.HasTopChannels()))
                    return "Channel map has top channels, cannob be handles by 2D panner.";

                if (channelMap.GetNumChannels() == 1)
                    return "Trying to initialize with single target channel, panning is not possible.";

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
namespace JPL::VBAP
{
    template<class Traits>
    inline bool Panning2D<Traits>::SourceLayout::Initialize(ChannelMap channelMap, ChannelMap targetMap, float shortestEdgeApertureAngle)
    {
        if (auto error = IsValidSourceChannelMap(channelMap))
        {
            JPL_ERROR_TAG("Panning2D", error.value());
            return false;
        }

        // Sanitize input parameters, we don't use LFE for panning and VS per channel should be at least 2
        const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

        mNumVirtualSourcesPerChannel = [](uint32 numChannels, float shortestEdgeApertureAngle)
        {
            // 'shortestEdgeApertureAngle' is geodesic threshold in radians
            uint32 totalRings = 4;
            if (shortestEdgeApertureAngle < std::numeric_limits<float>::max())
            {
                JPL_ASSERT(shortestEdgeApertureAngle >= -1e-6f && shortestEdgeApertureAngle <= JPL_TWO_PI);

                // Estimate number of virtual source required to avoid inactive speakers

                // Since we distribute our channels along the equator,
                // we can just divide the circumference by max allowed aperture
                totalRings = static_cast<uint32>(std::ceil(JPL_TWO_PI / std::max(shortestEdgeApertureAngle, 1e-6f)));
            }

            // TODO: totalRings can be smaller than numChannels,
            // if we're willing to sacrifice vectorization,
            // we can move max out:
            //      std::max(RoundUpBy4(totalRings / numChannels), 2u))
            return RoundUpBy4(std::max(totalRings / numChannels, 1u));
        }(numChannels, shortestEdgeApertureAngle);

        JPL_ASSERT(mNumVirtualSourcesPerChannel * numChannels <= cMaxNumVirtualSources);

        // Cache for arranging virtual sources later
        mPhiTerm = JPL_TWO_PI / (mNumVirtualSourcesPerChannel * numChannels);

        // As we increase the focus, we need to offset the start of the cap to contract towards channel center angle.
        // Note: this doesn't apply for mono source, since we mirror half samples around the channel center angle.
        mOffsetTerm = numChannels > 1 ? mPhiTerm * mNumVirtualSourcesPerChannel * 0.5f : 0.0f;

        return LayoutBase::InitializeBase(channelMap, targetMap, mNumVirtualSourcesPerChannel);
    }

    template<class Traits>
    inline void Panning2D<Traits>::SourceLayout::GenerateSpreadCap(Vec3SIMDBufferView& outBuffer, float spreadNormalized) const
    {
        JPL_ASSERT(outBuffer.size() == GetNumSIMDOps(mNumVirtualSourcesPerChannel));

        simd* destX = outBuffer.X;
        simd* destY = outBuffer.Y;
        simd* destZ = outBuffer.Z;

        // We don't use Y coordinate in 2D panning
        std::fill(destY, destY + outBuffer.size(), simd::zero());

        const uint32 toMirror = static_cast<uint32>(FloorToDiv2(outBuffer.size()));
        const uint32 tail = static_cast<uint32>(GetDiv2Tail(outBuffer.size()));

        // How much we can potentially "mirror"
        const uint32 halfSamples = toMirror >> 1;

        // Generate half that we can mirror + tail,
        // mirror only that half
        const uint32 halfAndTail = halfSamples + tail;

        // Generate canon half ring
        {
            const float spread = (1.0f - spreadNormalized);
            const float phiTerm = spread * mPhiTerm; // apply spread
            const simd offset(-spread * mOffsetTerm);
            const simd rampFirst = simd(0.5f, 1.5f, 2.5f, 3.5f); // start with an offset to make mirroring work

            simd phi = FMA(phiTerm, rampFirst, offset);
            const simd delta(4.0f * phiTerm);

            for (uint32 i = 0; i < halfAndTail; ++i)
            {
                Math::SinCos(phi, (*destX++), (*destZ++));
                phi += delta;
            }
        }

        // Generate the rest of ring by mirroring vectors
        if (halfSamples)
        {
            // Copy first half ring's only Z component 
            std::memcpy(destZ, outBuffer.Z, halfSamples * sizeof(simd));

            // Copy and flip first half ring's X component to make a full ring
            for (uint32 i = 0; i < halfSamples; ++i)
            {
                (*destX++) = -outBuffer.X[i];
            }
        }
    }

#if 0
    template<class Traits>
    inline void Panning2D<Traits>::SourceLayout::GenerateSpreadCap(std::span<Vec3Type> outBuffer, float spreadNormalized) const
    {
        /*
            TOTAL COST (for 16 samples):
                - 8 sin/cos
                - 2 mul
                - 11 add
                - 1 bit shift
                - 8 flipping sing of a float (x = -x)

            At 4.5 GHz roughly 452 cycles, ~0.1 us or 0.0001004 ms
        */

        JPL_ASSERT(outBuffer.size() >= mNumVirtualSourcesPerChannel);

        const uint32 halfSamples = mNumVirtualSourcesPerChannel >> 1;
        Vec3Type* destination = outBuffer.data();

        // Generate canon half ring
        {
            const float phiTerm = (1.0f - spreadNormalized) * mPhiTerm; // apply spread
            float phi = phiTerm * 0.5f; // start with an offset to make mirroring work

            for (uint32 i = 0; i < halfSamples; ++i)
            {
                const auto& [sinPhi, cosPhi] = Math::SinCos(phi);

                destination[i] = Vec3Type{
                    sinPhi,
                    0.0f,
                    cosPhi
                };

                phi += phiTerm;
            }

            // Advance destination buffer pointer
            destination += halfSamples;
        }

        // Generate the rest of ring by mirroring vectors
        {
            // Duplicate first half ring
            std::copy(outBuffer.begin(), outBuffer.begin() + halfSamples, destination);

            // Mirror duplicated first half ring to make a full ring
            for (Vec3Type& mirror : std::span(destination, halfSamples))
                SetX(mirror, -GetX(mirror));
        }
    }
#endif
} // namespace JPL::VBAP
