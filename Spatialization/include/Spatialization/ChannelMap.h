//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatialization **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatialization
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatialization is offered under the terms of the ISC license:
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

#include "Core.h"
#include "ErrorReporting.h"

#include <bit>

namespace JPL
{
    //==========================================================================
    /// Default speaker/channel masks
    /// This should be in the order the channels are laid out in the user's
    /// audio processing block.
    /// So far this more-or-less conforms to the Microsoft's standard.
    enum EChannel : uint32
    {
        FrontLeft           = JPL_BIT(0),
        FrontRight          = JPL_BIT(1),
        FrontCenter         = JPL_BIT(2),
        LFE                 = JPL_BIT(3),
        BackLeft            = JPL_BIT(4),
        BackRight           = JPL_BIT(5),
        FrontLeftCenter     = JPL_BIT(6),
        FrontRightCenter    = JPL_BIT(7),
        BackCenter          = JPL_BIT(8),
        SideLeft            = JPL_BIT(9),
        SideRight           = JPL_BIT(10),
        TopCenter           = JPL_BIT(11),
        TopFrontLeft        = JPL_BIT(12),
        TopFrontCenter      = JPL_BIT(13),
        TopFrontRight       = JPL_BIT(14),
        TopBackLeft         = JPL_BIT(15),
        TopBackCenter       = JPL_BIT(16),
        TopBackRight        = JPL_BIT(17),
        
        // ..other speakers can be added here


        NUM_GroundChannels = std::bit_width((uint32)SideRight), // Number of ground channels
        TOP_Channels = TopCenter,                                // Top channels have value >= than TOP_Channels

        Invalid = 0
    };

    //==========================================================================
    /// Common channel masks
    namespace ChannelMask
    {
        static constexpr uint32 Invalid = uint32(0);

        static constexpr uint32 Mono = FrontCenter;
        static constexpr uint32 Stereo = FrontLeft | FrontRight;
        static constexpr uint32 LCR = FrontLeft | FrontRight | FrontCenter;
        static constexpr uint32 Quad = FrontLeft | FrontRight | BackLeft | BackRight;
        static constexpr uint32 Surround_4_1 = FrontLeft | FrontRight | LFE | BackLeft | BackRight;
        static constexpr uint32 Surround_5_1 = FrontLeft | FrontRight | FrontCenter | LFE | BackLeft | BackRight;
        static constexpr uint32 Surround_6_1 = FrontLeft | FrontRight | FrontCenter | LFE | BackCenter | BackLeft | BackRight;
        static constexpr uint32 Surround_7_1 = FrontLeft | FrontRight | FrontCenter | LFE | BackCenter | BackLeft | BackRight | SideLeft | SideRight;
    }

    //==========================================================================
    /// ChannelMap is just a interface for strongly-typed access to some sort of
    /// channel mask
    class ChannelMap
    {
        constexpr ChannelMap(uint32 value) : mChannelMask(value) {}
    public:
        constexpr ChannelMap() = default;

        static constexpr uint32 InvalidChannelIndex = ~uint32(0);
     
        constexpr bool Has(EChannel channel) const { return (mChannelMask & static_cast<uint32_t>(channel)) == channel; }
        constexpr bool HasLFE() const { return (mChannelMask & static_cast<uint32_t>(EChannel::LFE)) == EChannel::LFE; }
        constexpr bool IsValid() const { return mChannelMask != ChannelMask::Invalid; }

        constexpr uint32 GetNumChannels() const { return std::popcount(mChannelMask); }
        constexpr uint32 GetChannelIndex(EChannel channel) const
        {
            if (!Has(channel))
                return InvalidChannelIndex;
            
            uint32 channelIndex = 0;
            int value = static_cast<int>(mChannelMask);
            while (value != 0)
            {
                if (channel == static_cast<EChannel>(value & -value))
                    return channelIndex;
                
                value &= (value - 1);
                ++channelIndex;
            }
            return InvalidChannelIndex;
        }

        constexpr EChannel GetChannelAtIndex(uint32 index) const
        {
            EChannel foundChannel = EChannel::Invalid;

            ForEachChannel([index, &foundChannel](EChannel channel, uint32 channelIndex)
            {
                if (channelIndex == index)
                    foundChannel = channel;
            });

            return foundChannel;
        }

        static constexpr uint32 MaxSupportedChannels() { return 8u; }

        static constexpr ChannelMap FromChannelMask(uint32 channelMask) { return channelMask; }
        static constexpr ChannelMap FromNumChannels(uint32 numChannels)
        {
            switch (numChannels)
            {
            case 1: return ChannelMask::Mono;
            case 2: return ChannelMask::Stereo;
            case 3: return ChannelMask::LCR;
            case 4: return ChannelMask::Quad;
            case 5: return ChannelMask::Surround_4_1;
            case 6: return ChannelMask::Surround_5_1;
            case 7: return ChannelMask::Surround_6_1;
            case 8: return ChannelMask::Surround_7_1;
            default:
                JPL_ASSERT("Currently only up to 8 channels are supported.");
                return ChannelMask::Invalid;
            }
        }

        template<class Predicate>
        constexpr void ForEachChannel(Predicate predicate) const
        {
            uint32 channelIndex = 0;
            int value = static_cast<int>(mChannelMask);
            while (value != 0)
            {
                const auto channel = static_cast<EChannel>(value & -value);
                if constexpr (std::invocable<Predicate, EChannel, uint32>)
                    predicate(channel, channelIndex);
                else if constexpr (std::invocable<Predicate, EChannel>)
                    predicate(channel);
                else
                    static_assert(false, "Invalid predicate signature.");

                // Clear the lowest set bit
                value &= (value - 1);
                ++channelIndex;
            }
        }

    private:
        uint32 mChannelMask = ChannelMask::Invalid;
    };
} // namespace JPL