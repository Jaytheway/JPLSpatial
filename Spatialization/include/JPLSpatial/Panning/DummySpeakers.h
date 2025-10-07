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
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Memory/Memory.h"

#include <type_traits>
#include <vector>
#include <algorithm>

namespace JPL::VBAP
{
    //==========================================================================
   /// Utility class to encapsulate dummy speaker handling while building a LUT
    template<auto GetChannelVectorFunction>
    class DummySpeakers
    {
    public:
        using Vec3Type = std::remove_cvref_t<decltype(GetChannelVectorFunction(EChannel{}))>;

        JPL_INLINE constexpr DummySpeakers(ChannelMap channelMap, std::pmr::vector<Vec3Type>& speakerVectors) noexcept
            : mMap(channelMap)
            , mVectors(speakerVectors)
            , mDummyIndices(GetDefaultMemoryResource())
        {
        }

        JPL_INLINE constexpr void AddDummy(const Vec3Type& speakerVector)
        {
            mVectors.push_back(speakerVector);
            mDummyIndices.push_back(static_cast<uint32>(mVectors.size() - 1));
        }

        JPL_INLINE constexpr void AddIfChannelNotPresent(EChannel channel)
        {
            if (!mMap.Has(channel))
                AddDummy(GetChannelVectorFunction(channel));
        }

        [[nodiscard]] JPL_INLINE constexpr bool Contains(uint32 index) const
        {
            return std::find(mDummyIndices.begin(), mDummyIndices.end(), index) != mDummyIndices.end();
        }

        [[nodiscard]] JPL_INLINE constexpr bool HasDummyAt(const Vec3Type& direction, float tolerance) const noexcept
        {
            for (uint32 idx : mDummyIndices)
            {
                if (Math::IsNearlyEqual(direction, mVectors[idx], tolerance))
                    return true;
            }

            return false;
        }

        [[nodiscard]] JPL_INLINE constexpr uint32 GetNumDummies() const noexcept { return static_cast<uint32>(mDummyIndices.size()); }
        [[nodiscard]] JPL_INLINE constexpr ChannelMap GetChannelMap() const noexcept { return mMap; }

    private:
        ChannelMap mMap;
        std::pmr::vector<Vec3Type>& mVectors;
        std::pmr::vector<uint32> mDummyIndices;
    };
} // namespace JPL::VBAP