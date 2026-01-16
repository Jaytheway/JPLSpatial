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

#include "Core.h"
#include "ErrorReporting.h"

#include <bit>
#include <string_view>

namespace JPL
{
    //==========================================================================
    /// Default speaker/channel masks
    /// This should be in the order the channels are laid out in the user's
    /// audio processing block.
    //  This no longer conforms to WAVEFORMATEXTENSIBLE, but supposidely conforms
    //  to common speaker arrangements.
    //! The main difference between wave channel formats and this, is in that the
    //! side channels in wave come after back channels
    enum EChannel : uint32
    {
        FrontLeft           = JPL_BIT(0),
        FrontRight          = JPL_BIT(1),
        FrontCenter         = JPL_BIT(2),
        LFE                 = JPL_BIT(3),

        SideLeft            = JPL_BIT(4),   // also "left surround side", "left centre"
        SideRight           = JPL_BIT(5),   // also "right surround side", "right centre"
       
        FrontLeftCenter     = JPL_BIT(6),
        FrontRightCenter    = JPL_BIT(7),
        
        BackLeft            = JPL_BIT(8),   // also "left surround", "left surround read"
        BackRight           = JPL_BIT(9),   // also "right surround", "right surround rear"
        BackCenter          = JPL_BIT(10),  // also "surround" or "centre surround"
        
        WideLeft            = JPL_BIT(11),
        WideRight           = JPL_BIT(12),

        TopCenter           = JPL_BIT(13),
        TopFrontLeft        = JPL_BIT(14),
        TopFrontCenter      = JPL_BIT(15),
        TopFrontRight       = JPL_BIT(16),

        // These used by Dolby Atmos 7.0.2 and 7.1.2
        TopSideLeft         = JPL_BIT(17),
        TopSideRight        = JPL_BIT(18),

        TopBackLeft         = JPL_BIT(19),  // also "top rear left"
        TopBackCenter       = JPL_BIT(20),
        TopBackRight        = JPL_BIT(21),  // also "top rear right"

        // ..other speakers can be added here

        NUM_GroundChannels = std::bit_width((uint32)WideRight), // Number of ground channels
        
        TOP_Channels = TopCenter,                               // Top channels have value >= than TOP_Channels
        NUM_TopChannels = std::bit_width((uint32)TopBackRight) - NUM_GroundChannels, // Number of ground channels

        Invalid = 0
    };

    //==========================================================================
    /// Common channel masks
    namespace ChannelMask
    {
        // TODO: channel order matters and some formats differ only by it, so our simple bit mask may not be functional for everything
        
        // TODO: Extended setups WIP. Integrate, test, etc...

        static constexpr uint32 Invalid = uint32(0);
        
        //======================================================================
        static constexpr uint32 Mono                = FrontCenter;
        static constexpr uint32 Stereo              = FrontLeft | FrontRight;
        static constexpr uint32 LCR                 = FrontLeft | FrontRight | FrontCenter;
	    static constexpr uint32 LRS                 = FrontLeft | FrontRight | BackCenter;
	    static constexpr uint32 LCRS                = FrontLeft | FrontRight | FrontCenter | BackCenter;
        static constexpr uint32 Quad                = FrontLeft | FrontRight | BackLeft | BackRight;

        // TODO: this clashes with Surround 5.0, we should just remove it, or refactor ChannelMask to be a more than just an integer bitmask
	    static constexpr uint32 Pentagonal          = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight;
        
        //? Couldn't arrange in a way to preserve channel order across different setups
	    //?static constexpr uint32 Hexagonal           = FrontLeft | FrontRight | FrontCenter | BackCenter | BackLeft | BackRight;
	    
        static constexpr uint32 Octagonal           = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight | BackCenter | WideLeft | WideRight;

        //======================================================================
        static constexpr uint32 Surround_4_1        = FrontLeft | FrontRight | LFE | BackLeft | BackRight;
	    static constexpr uint32 Surround_5_0        = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight;
        static constexpr uint32 Surround_5_1        = FrontLeft | FrontRight | FrontCenter | LFE | BackLeft | BackRight;
	    static constexpr uint32 Surround_6_0        = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight | BackCenter;
        static constexpr uint32 Surround_6_1        = FrontLeft | FrontRight | FrontCenter | LFE | BackLeft | BackRight | BackCenter;
        
        //======================================================================
        // DTS surround setups
	    static constexpr uint32 Surround_7_0        = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight;
	    static constexpr uint32 Surround_7_1        = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight;

        //======================================================================
        static constexpr uint32 Surround_5_0_2      = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight | TopSideLeft | TopSideRight;
	    static constexpr uint32 Surround_5_1_2      = FrontLeft | FrontRight | FrontCenter | LFE | BackLeft | BackRight | TopSideLeft | TopSideRight;
	    static constexpr uint32 Surround_5_0_4      = FrontLeft | FrontRight | FrontCenter | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_5_1_4      = FrontLeft | FrontRight | FrontCenter | LFE | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;
	    
        //======================================================================
        // Dolby Atmos surround setups
        static constexpr uint32 Surround_7_0_2      = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight | TopSideLeft | TopSideRight;
        static constexpr uint32 Surround_7_1_2      = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight | TopSideLeft | TopSideRight;
        static constexpr uint32 Surround_7_0_4      = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_7_1_4      = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;

        static constexpr uint32 Surround_7_0_6      = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopSideLeft | TopSideRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_7_1_6      = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight | TopFrontLeft | TopFrontRight | TopSideLeft | TopSideRight | TopBackLeft | TopBackRight;
	    
        //======================================================================
        // Atmos surround setups
        static constexpr uint32 Surround_9_0_4      = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight | WideLeft | WideRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_9_1_4      = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight | WideLeft | WideRight | TopFrontLeft | TopFrontRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_9_0_6      = FrontLeft | FrontRight | FrontCenter | SideLeft | SideRight | BackLeft | BackRight | WideLeft | WideRight | TopFrontLeft | TopFrontRight | TopSideLeft | TopSideRight | TopBackLeft | TopBackRight;
	    static constexpr uint32 Surround_9_1_6      = FrontLeft | FrontRight | FrontCenter | LFE | SideLeft | SideRight | BackLeft | BackRight | WideLeft | WideRight | TopFrontLeft | TopFrontRight | TopSideLeft | TopSideRight | TopBackLeft | TopBackRight;

        JPL_INLINE std::string_view ToString(uint32 channelMask);
    }


    //==========================================================================
    /// ChannelMap is just a interface for strongly-typed access to some sort of
    /// channel mask
    class [[nodiscard]] ChannelMap
    {
        constexpr explicit ChannelMap(uint32 value) : mChannelMask(value) {}
    public:
        constexpr ChannelMap() = default;

        static constexpr uint32 InvalidChannelIndex = ~uint32(0);
     
        [[nodiscard]] constexpr bool Has(EChannel channel) const noexcept { return (mChannelMask & static_cast<uint32_t>(channel)) == channel; }
        [[nodiscard]] constexpr bool HasLFE() const noexcept { return (mChannelMask & static_cast<uint32_t>(EChannel::LFE)) == EChannel::LFE; }
        [[nodiscard]] constexpr bool HasTopChannels() const noexcept { return mChannelMask >= EChannel::TOP_Channels; }
        [[nodiscard]] constexpr bool IsValid() const noexcept { return mChannelMask != ChannelMask::Invalid; }

        [[nodiscard]] constexpr uint32 GetNumChannels() const noexcept { return std::popcount(mChannelMask); }
        [[nodiscard]] constexpr uint32 GetChannelIndex(EChannel channel) const
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

        [[nodiscard]] constexpr EChannel GetChannelAtIndex(uint32 index) const
        {
            EChannel foundChannel = EChannel::Invalid;

            ForEachChannel([index, &foundChannel](EChannel channel, uint32 channelIndex)
            {
                if (channelIndex == index)
                    foundChannel = channel;
            });

            return foundChannel;
        }

        [[nodiscard]] constexpr uint32 GetChannelMask() const { return mChannelMask; }
        
        //[[nodiscard]] static constexpr uint32 MaxSupportedChannels() noexcept { return 8u; }
        
        [[nodiscard]] static constexpr ChannelMap FromChannelMask(uint32 channelMask) { return ChannelMap(channelMask); }
        [[nodiscard]] static constexpr ChannelMap FromNumChannels(uint32 numChannels)
        {
            switch (numChannels)
            {
            case 1: return ChannelMap(ChannelMask::Mono);
            case 2: return ChannelMap(ChannelMask::Stereo);
            case 3: return ChannelMap(ChannelMask::LCR);
            case 4: return ChannelMap(ChannelMask::Quad);
            case 5: return ChannelMap(ChannelMask::Surround_4_1);
            case 6: return ChannelMap(ChannelMask::Surround_5_1);
            case 7: return ChannelMap(ChannelMask::Surround_6_1);
            case 8: return ChannelMap(ChannelMask::Surround_7_1);
            default:
                JPL_ASSERT("ChannelMap can be created from channel count up to 8 channels.");
                return ChannelMap(ChannelMask::Invalid);
            }
        }

        template<class Predicate>
        constexpr void ForEachChannel(Predicate predicate) const
        {
            uint32 channelIndex = 0;
            int value = static_cast<int>(mChannelMask);
            while (value != 0)
            {
                static_assert(std::invocable<Predicate, EChannel, uint32> ||
                              std::invocable<Predicate, EChannel>, "Invalid predicate signature.");

                const auto channel = static_cast<EChannel>(value & -value);
                if constexpr (std::invocable<Predicate, EChannel, uint32>)
                    predicate(channel, channelIndex);
                else
                    predicate(channel);

                // Clear the lowest set bit
                value &= (value - 1);
                ++channelIndex;
            }
        }
        
        [[nodiscard]] constexpr bool operator==(const ChannelMap& other) const noexcept { return mChannelMask == other.mChannelMask; }
        [[nodiscard]] constexpr bool operator!=(const ChannelMap& other) const noexcept { return mChannelMask != other.mChannelMask; }

    private:
        friend struct std::hash<JPL::ChannelMap>;
        uint32 mChannelMask = ChannelMask::Invalid;
    };
    struct NamedChannelMask
} // namespace JPL

//==============================================================================
namespace std
{
    template <>
    struct [[nodiscard]] hash<JPL::ChannelMap>
    {
        constexpr std::size_t operator()(const JPL::ChannelMap& channelMap) const
        {
            return channelMap.mChannelMask;
        }
    };
}

namespace JPL
{
    JPL_INLINE std::string_view ChannelMask::ToString(uint32 channelMask)
    {
        switch (channelMask)
        {
        case ChannelMask::Invalid:          return "INVALID";
        case ChannelMask::Mono:             return "Mono";
        case ChannelMask::Stereo:           return "Stereo";
        case ChannelMask::LCR:              return "LCR";
        case ChannelMask::LRS:              return "LRS";
        case ChannelMask::LCRS:             return "LCRS";
        case ChannelMask::Quad:             return "Quad";
        //case ChannelMask::Pentagonal: return "Pentagonal"; //? clashes with Surround 5.0
        case ChannelMask::Octagonal:        return "Octagonal";
        case ChannelMask::Surround_4_1:     return "Surround 4.1";
        case ChannelMask::Surround_5_0:     return "Surround 5.0";
        case ChannelMask::Surround_5_1:     return "Surround 5.1";
        case ChannelMask::Surround_6_0:     return "Surround 6.0";
        case ChannelMask::Surround_6_1:     return "Surround 6.1";
        case ChannelMask::Surround_7_0:     return "Surround 7.0";
        case ChannelMask::Surround_7_1:     return "Surround 7.1";
        case ChannelMask::Surround_5_0_2:   return "Surround 5.0.2";
        case ChannelMask::Surround_5_1_2:   return "Surround 5.1.2";
        case ChannelMask::Surround_5_0_4:   return "Surround 5.0.4";
        case ChannelMask::Surround_5_1_4:   return "Surround 5.1.4";
        case ChannelMask::Surround_7_0_2:   return "Surround 7.0.2";
        case ChannelMask::Surround_7_1_2:   return "Surround 7.1.2";
        case ChannelMask::Surround_7_0_4:   return "Surround 7.0.4";
        case ChannelMask::Surround_7_1_4:   return "Surround 7.1.4";
        case ChannelMask::Surround_7_0_6:   return "Surround 7.0.6";
        case ChannelMask::Surround_7_1_6:   return "Surround 7.1.6";
        case ChannelMask::Surround_9_0_4:   return "Surround 9.0.4";
        case ChannelMask::Surround_9_1_4:   return "Surround 9.1.4";
        case ChannelMask::Surround_9_0_6:   return "Surround 9.0.6";
        case ChannelMask::Surround_9_1_6:   return "Surround 9.1.6";
        default:
            JPL_ASSERT(false, "Unknown channel mask.");
            return "< UKNOWN >";
        }
    }
} // namespace JPL