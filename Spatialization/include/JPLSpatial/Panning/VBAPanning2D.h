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
#include "JPLSpatial/Memory/Bits.h"

#include <span>
#include <algorithm>

#include <cmath>
#include <limits>
#include <optional>

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
            using LUTType = VBAP::LUT2D<typename Traits::template Array>;

            using LUTInterface =
                VBAP::LUTInterface2D<
                &Traits::GetChannelVector,
                &Traits::GetChannelAngle,
                typename Traits::template Array
                >;

            //=================================================================
            /// Implementation of the source layout for 2D panning
            struct SourceLayout : Base::VBAPLayoutBase
            {
                using LayoutBase = Base::VBAPLayoutBase;

                static constexpr size_t cMaxNumVirtualSources = 32;
            public:

                [[nodiscard]] JPL_INLINE size_t GetNumVirtualSources() const noexcept { return mNumVirtualSourcesPerChannel; }
                [[nodiscard]] static JPL_INLINE constexpr size_t GetMaxNumVirtualSources() noexcept { return cMaxNumVirtualSources; }

            private:
                friend class Base;
                /// Initialize VBAP data for standard channel map panning.
                /// (called by the panner that has the value for `shortestEdgeApertureDot`)
                bool Initialize(ChannelMap channelMap, ChannelMap targetMap, float shortestEdgeApertureDot = std::numeric_limits<float>::max());

                /// Generate canonical spread cap for given `spreadNormalized` value.
                /// Spread of the cap from the perspective of our MDAP implementation is the Focus parameter
                /// (called by paner in ProcessVBAPData)
                void GenerateSpreadCap(std::span<Vec3Type> outBuffer, float spreadNormalized) const;

            private:
                uint32 mNumVirtualSourcesPerChannel;
                float mPhiTerm;                     // Cached term to generate spread cap
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

        JPL_ASSERT(mNumVirtualSourcesPerChannel <= cMaxNumVirtualSources);

        // Cache for arranging virtual sources later
        mPhiTerm = JPL_TWO_PI / (mNumVirtualSourcesPerChannel * numChannels);

        return LayoutBase::InitializeBase(channelMap, targetMap, mNumVirtualSourcesPerChannel);
    }

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
                mirror.X = -mirror.X;
        }
    }
} // namespace JPL::VBAP
