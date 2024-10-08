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
#include "ChannelMap.h"

#include <vector>
#include <numbers>
#include <span>
#include <algorithm>
#include <numeric>
#include <ranges>
#include <functional>

#include <concepts>
#include <array>
#include <map>
#include <cmath>

namespace JPL
{
    //======================================================================
    /// Customization. You can either inherit from this and shadow some types
    /// and functions, or use a completely separate traits type with VBAP API.
    struct VBAPStandartTraits
    {
        /// If you override GainType, make sure to also override ChannelGains
        using GainType = float;
        using AngleType = float;

        /// Can be overriden by the user, e.g. to use custom allocator
        template<class T> 
        using Array = std::vector<T>;

        /// Can be useful to override to save a tiny bit of memory if max number
        /// of channels ever used is known at compile time. Or to handle larger
        /// channel maps.
        /// (though currently ChannelMap interface doesn't support 64-bit masks
        /// and is not very customizable, e.g. stroing in an int mask vs array)
        static constexpr auto MAX_CHANNELS = 32;

        /// If you override ChannelGains, make sure to also override GainType
        using ChannelGains = typename std::array<typename GainType, MAX_CHANNELS>;

        /// Just hiding convenient aliases in traits, no need to ever override these
        static constexpr auto two_pi = std::numbers::pi_v<AngleType> * AngleType(2.0);
        static constexpr auto pi = std::numbers::pi_v<AngleType>;
        static constexpr auto half_pi = std::numbers::pi_v<AngleType> * AngleType(0.5);
        
        // TODO: customizable EChannel type and ChannelMap type as well?
        static AngleType GetChannelAngle(EChannel channel)
        {
            /// Standard channel vectors converted to angles
            // TODO: for now we discard elevation
            static const std::map<EChannel, AngleType> gChannelAngles
            {
                { FrontLeft,       -0.785398f },     // FrontLeft
                { FrontRight,      0.785398f },      // FrontRight
                { FrontCenter,     0.0f },           // FrontCenter
                { LFE,             0.0f },           // LFE
                { BackLeft,        -2.35619f },      // BackLeft
                { BackRight,       2.35619f },       // BackRight
                { FrontLeftCenter, -0.321719f },     // FrontLeftCenter
                { FrontRightCenter,0.321719f },      // FrontRightCenter
                { BackCenter,      3.14159f },       // BackCenter
                { SideLeft,        -1.5708f },       // SideLeft
                { SideRight,       1.5708f },        // SideRight
                { TopCenter,       3.14159f },       // TopCenter
                { TopFrontLeft,    -0.785398f },     // TopFrontLeft
                { TopFrontCenter,  0.0f },           // TopFrontCenter
                { TopFrontRight,   0.785398f },      // TopFrontRight
                { TopBackLeft,     -2.35619f },      // TopBackLeft
                { TopBackCenter,   3.14159f },       // TopBackCenter
                { TopBackRight,    2.35619f }        // TopBackRight
            };

            return gChannelAngles.at(channel);
        }
    };

    //======================================================================
    namespace Internal
    {
        /// Standard `abs` is not constexpr
        template<class T, std::enable_if_t<std::is_arithmetic_v<T>>...>
        constexpr auto abs(const T& x) noexcept { return x < 0 ? -x : x; }
    }

    //======================================================================
    /// Simple structure to help sort channel angles
    template<class Traits = VBAPStandartTraits>
    struct ChannelAngle
    {
        Traits::AngleType Angle;
        uint32 ChannelId; // Channel index or identifier

        // Operators necessary for sorting
        constexpr std::strong_ordering operator<=>(const ChannelAngle& other) const
        {
            const Traits::AngleType a1 = Angle < Traits::AngleType(0.0) ? Angle + Traits::two_pi : Angle;
            const Traits::AngleType a2 = other.Angle < Traits::AngleType(0.0) ? other.Angle + Traits::two_pi : other.Angle;
            if (a1 < a2) return std::strong_ordering::less;
            if (a1 > a2) return std::strong_ordering::greater;
            return std::strong_ordering::equal;
        }
        constexpr bool operator==(const ChannelAngle& other) const { return Internal::abs(Angle - other.Angle) < static_cast<Traits::AngleType>(1e-6); }

        /// Get channel angles from ChannelMap, normalize to [0, Pi] and sort in assending order
        static void GetSortedChannelAngles(
            ChannelMap channelMap,
            Traits::template Array<ChannelAngle<Traits>>& sortedChannelAngles,
            bool skipLFO = true);
    };

    //======================================================================
    template<class Traits = VBAPStandartTraits>
    struct PanUpdateData
    {
        Traits::AngleType PanAngle; // in radians
        float Spread;               // [0, 1]
        float Focus;                // [0, 1]

        // TODO: handle elevetion
        //Traits::AngleType Elevation;
    };

    template<class Traits = VBAPStandartTraits>
    struct VirtualSource
    {
        /// Absolute angle of this virtual source in radians.
        Traits::AngleType Angle;

        // TODO: consider storing position as well, and or elvetion, optional per-source distance attenuation / weight?
        //Traits::AngleType Elevation;
        //float Weight;
    };

    //======================================================================
    // TODO: we can probably create nominal channel groups for basic layouts, like stereo, quad, etc., and reuse them
    template<class Traits = VBAPStandartTraits>
    struct ChannelGroup
    {
        // TODO: for now elevetion is not handled, 
        //float Elevation;

        /// Angle of the channel this group is associated to. In radians.
        Traits::AngleType Angle;

        /// Index of the channel this group is associated to.
        uint32_t Channel;
        
        /// VBAP sources distributed in 360 degrees and assigned to this source channel
        Traits::template Array<VirtualSource<Traits>> VirtualSources;

        // TODO: consider not storing this and only using local array when actually processing contributions?
        /// Accumulated and normalized gains of virtual sources associated to this channel group.
        Traits::ChannelGains Gains;
    };

    //======================================================================
    /// Data required to calculate and apply VBAP gains, that represents
    /// groups of Virtual Sources per source channel
    template<class Traits = VBAPStandartTraits>
    struct VBAPData
    {
        /// Groups of virtual sources associated with source channels
        Traits::template Array<ChannelGroup<Traits>> ChannelGroups;

        /// Initialize VBAP data for standard channel map panning.
        /// Virtual Sources are laid out relative to nominal direction
        /// (i.e. forward facing, "no panning applied")
        bool Initialize(ChannelMap channelMap, uint32 virtualSourcesPerChannel);
    };

    //======================================================================
    /// Structs for stroner-typed function parameters
    template<class Traits = VBAPStandartTraits>
    struct PanSource
    {
        Traits::AngleType SourceAngle;
        Traits::AngleType GroupAngle;
    };

    struct PanParameters
    {
        float Focus;
        float Spread;
    };

    //======================================================================
    /// Vector Based Amplitude Panner that can handle panning from any
    /// source channel map to any target/output channel map.
    /// 
    /// Technically it uses MDAP (Multiple Directions Amplitude Panning)
    /// with virtual sources.
    /// 
    /// VBAPData and ProcessVBAPData function can be used for a more conventional
    /// channel map -> channel map panning, based on parameters like spread and
    /// focus, which can be driven by, for example, distance from the soruce
    /// to the listener (on the client side).
    /// 
    /// While VirtualSource(s) can be used directly with ProcessVirtualSources
    /// function for a more customizable spatialization, e.g. for volumetric
    /// sound sources, or any other circumstances where position and spatial
    /// extend of each channel needed to be manually cotrolled.
    template<class Traits = VBAPStandartTraits>
    class VBAPanner
    {
    public:
        /// Aliases to avoid typing wordy templates
        using GainType = Traits::GainType;
        using AngleType = Traits::AngleType;

        static_assert(std::same_as<typename Traits::GainType, typename Traits::ChannelGains::value_type>,
                      "Make sure if overriding either GainType or ChannelGains of VBAPanner traits, override both of them with the same type."
                      "E.g. both floats, or both doubles.");

        static_assert(std::floating_point<GainType>, "GainType should be floating point.");
        static_assert(std::floating_point<AngleType>, "AngleType should be floating point.");

        using ChannelAngle = typename ChannelAngle<Traits>;
        using PanUpdateData = typename PanUpdateData<Traits>;
        using VirtualSource = typename VirtualSource<Traits>;
        using ChannelGroup = typename ChannelGroup<Traits>;
        using VBAPData = typename VBAPData<Traits>;
        using PanSource = typename PanSource<Traits>;

        template<class T>
        using Array = typename Traits::template Array<T>;
        using ChannelAngleArray = typename Array<ChannelAngle>;

        /// Consts and defaults
        static constexpr uint8 sDeafultQuadrantResolution = 128;
        static constexpr uint8 sMinQuadrantResolution = 90;

        /// Initialize LUT for a specific `channelMap` and resolution.
        /// The resolution affects the precision of the preprocessed channel gains. 
        /// @param channelMap - channel map to create LUT for
        /// @param quadrantResolution - the overall resolution of LUT = quadrantResolution * 4,
        /// it is not recommended to use values < sDeafultQuadrantResolution
        bool InitializeLUT(ChannelMap channelMap, uint8 quadrantResolution = sDeafultQuadrantResolution);

        /// Get number of channels the LUT is initialized for.
        /// Effectively this is the channel count of the channel map the panner
        /// is initialized for, including LFE (though LFE gain is always 0.0).
        JPL_INLINE uint32 GetNumChannels() const { return mNumChannels; }
       
        /// Get the channel map the panner is initialized to.
        JPL_INLINE ChannelMap GetChannelMap() const { return mChannelMap; }
       
        /// Get gain value at LUT position. Channel stride should be handled by the caller.
        /// 
        /// E.g. to get gain value for channel 3, knowing LUT position (e.g. retrieved by
        /// calling AngleToLUTPosition), `positionInLUT` parameter should be:
        /// AngleToLUTPosition * NumberOfChannels + 3.
        /// 
        /// Channel gain values for each angle are laid out continuously in the LUT.
        /// So to get all the target channel gains for a specific angle, simply get the beginning
        /// of the gain data, e.g.: GetLUTValue(AngleToLUTPosition * NumberOfChannels)
        /// ..and copy the contiguous gains from that position (LFE is inlude if present in
        /// the channel map, though the gain for LFE is always 0.0).
        JPL_INLINE GainType GetLUTValue(uint32 positionInLUT) const { return mLUT[positionInLUT]; }
      
        /// Size of the LUT = Quadrant Resolution * 4 * Number of Channels
        JPL_INLINE size_t GetLUTSize() const { return mLUT.size(); }

        /// Resolution of the LUT = Quadrant Resolution * 4
        JPL_INLINE size_t GetLUTResolution() const { return mLUTResolution; }

        /// Get LUT position from direction angle, where 0.0 is forward
        /// @param angleNormalized angle in radians in [0, 2Pi]
        JPL_INLINE int AngleNormalizedToLUTPosition(AngleType angleNormalised) const;

        /// Get LUT position from direction angle, where 0.0 is forward
        /// @param angleInRadians in radians in [-Pi, Pi]
        JPL_INLINE int AngleToLUTPosition(AngleType angleInRadians) const;

        /// Get LUT position from direction vector
        JPL_INLINE int CartesianToLUTPosition(float x, float z) const;

        /// Convert LUT position to angle (within LUT resolution, irrespective
        /// of number of channels of the channel map)
        JPL_INLINE AngleType LUTPositionToAngle(int pos) const;

        /// Calculate output channel gains based on angle `theta` and output `speakerAnalges`.
        /// This is more precise than other functions using LUT, but less performant,
        /// especially for large number of speakers.
        /// @param speakerAngles - sorted angles of the output speaker layout
        /// @param outGains - gains corresponding to the sorted speakers angles
        JPL_INLINE static void ProcessAngle(AngleType theta, std::span<const AngleType> speakerAnalges, std::span<GainType> outGains);

        /// Get preprocessed speaker gains at specific LUT posotion
        JPL_INLINE void GetSpeakerGains(int lutPosition, std::span<GainType> outGains) const;

        /// Get preprocessed speaker gains from LUT based on `angle` in [-Pi, Pi] in radians
        JPL_INLINE void GetSpeakerGains(AngleType angle, std::span<GainType> outGains) const;

        /// Get preprocessed speaker gains from LUT based on direction vector [x, z] where -z is forward.
        JPL_INLINE void GetSpeakerGains(float x, float z, std::span<GainType> outGains) const;

        /// Get and accumulate speaker gains for all virtual soruces into @outGains
        /// @projection is a function that can be used to preprocess virtual source angles
        /// before the channel gains are retrieved from the LUT. It takes single AngleType (float) parameter and must return AngleType (float).
        template<class AngleProjection = std::identity>
        void ProcessVirtualSources(
            std::span<const VirtualSource> virtualSources,
            std::span<GainType> outGains,
            const AngleProjection& projection = {}) const requires std::is_invocable_r_v<AngleType, AngleProjection, AngleType>;

        /// This function can be called whenever pan data changes to update 
        /// `Gains` of each ChannelGroup of the VBAPData based on PanUpdateData.
        /// VBAPData must be initialized beforehand.
        /// E.g. this would be called for each source that has unique channel layout or panning state
        void ProcessVBAPData(VBAPData& vbap, const PanUpdateData& updateData) const;

        /// Utility function to apply focus and spread to source angle, which must be in [-Pi, Pi] range.
        /// @returns resulting angle
        JPL_INLINE static AngleType ApplyFocusAndSpread(const PanSource& panSoruce, const PanParameters& parameters);

    private:
        JPL_INLINE static void AccumulateChannelGains(std::span<const GainType> contributingGains, std::span<GainType> outputGains);

        /// This should be called after all sources contributing to `gains` have been accumulated,
        /// which can be done by calling AccumulateChannelGains function
        JPL_INLINE static void NormalzieAccumulatedGains(std::span<GainType> gains, std::integral auto numVirtualSources);

    private:
        /// Total number of discreet values for 360-degrees field.
        /// The actual size of the LUT data is mLUTResolution * number of output channels.
        uint16 mLUTResolution = sDeafultQuadrantResolution * 4;

        Array<GainType> mLUT;
        uint32_t mNumChannels = 0;
        ChannelMap mChannelMap;
    };

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
    template<class Traits>
    inline void ChannelAngle<Traits>::GetSortedChannelAngles(
        ChannelMap channelMap,
        Traits::template Array<ChannelAngle<Traits>>& sortedChannelAngles,
        bool skipLFO /*= true*/)
    {
        sortedChannelAngles.reserve(channelMap.GetNumChannels() - skipLFO * channelMap.HasLFE());

        channelMap.ForEachChannel([&sortedChannelAngles, skipLFO](EChannel channel, uint32 channelIndex)
        {
            // We don't use LFE for panning
            if (skipLFO && channel == EChannel::LFE)
                return;

            const Traits::AngleType channelAngle = Traits::GetChannelAngle(channel);

            sortedChannelAngles.emplace_back(channelAngle < Traits::AngleType(0.0) ? channelAngle + Traits::two_pi : channelAngle, channelIndex);
        });

        std::ranges::sort(sortedChannelAngles);
    }

    //==========================================================================
    template<class Traits>
    inline bool VBAPData<Traits>::Initialize(ChannelMap channelMap, uint32 virtualSourcesPerChannel)
    {
        // TODO: handle top channels, maybe we can just ignore them for now, instead of failing to initialize?
        if (!JPL_ENSURE(channelMap.IsValid(), std::format("Failed to initialize VBAPData, provided invalid channel map.").c_str()))
            return false;

        // Sanitize input parameters, we don't use LFE for panning and VS per channel should be at least 2
        const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();
        virtualSourcesPerChannel = std::max(virtualSourcesPerChannel, 2u);

        // Reserve memory upfroant
        ChannelGroups.clear();
        ChannelGroups.resize(numChannels);
        std::ranges::for_each(ChannelGroups, [virtualSourcesPerChannel](ChannelGroup<Traits>& channelGroup)
        {
            channelGroup.VirtualSources.reserve(virtualSourcesPerChannel);
        });

        // Source channel map is unsorted, we need to sort it
        // to make our lives easier distributing virtual sourses
        using ChannelAngleArray = typename Traits::template Array<ChannelAngle<Traits>>;
        ChannelAngleArray sourceChannelsSorted;
        ChannelAngle<Traits>::GetSortedChannelAngles(channelMap, sourceChannelsSorted);

        // Width of a singe source channel in radians
        const Traits::AngleType channelWidth = Traits::two_pi / static_cast<Traits::AngleType>(numChannels);
        const Traits::AngleType channelHalfWidth = channelWidth * 0.5f;

        // If we don't have center channel, we need to offset channel groups
        const Traits::AngleType channelAngleOffset = !channelMap.Has(EChannel::FrontCenter)
                                        ? 0.5f * channelWidth
                                        : 0.0f;

        // Width of a single VS in radians
        const Traits::AngleType vsBaseWidth = Traits::two_pi / static_cast<Traits::AngleType>(virtualSourcesPerChannel * numChannels);
        const Traits::AngleType vsHalfWidth = vsBaseWidth * 0.5f;


        // 1. Find equal source channel positions for 100% spread
        // 2. Lay out virtual sources for source channel group evenly withing its equal section
        for (uint32 i = 0; i < sourceChannelsSorted.size(); ++i)
        {
            // Assign a centre angle of the next equal section of the source plane
            const Traits::AngleType channelAngle = channelAngleOffset + channelWidth * i;

            // Create channel group for each source channel
            ChannelGroup<Traits>& channelGroup = ChannelGroups[i];
            channelGroup.Angle = channelAngle;
            channelGroup.Channel = sourceChannelsSorted[i].ChannelId;

            /*
                To handle LFE:
                - channelGroup.Channel - is the index of the source channel
                - in the audio process block, instead of iterating actual inputs of thes block, iterate VBAP channel groups
                - if LFE pressent, simply copy input to output, applying other parameters, like distance attenuation, but skipping VBAP channel group

                There's no need to access channel group by ID when sending audio to the output, since we coppy each group to each output channel
            */

            // Find the position of the first virtual source for this channel
            const Traits::AngleType channelArchStart = channelAngle - channelHalfWidth;

            const Traits::AngleType channelVSStart = channelArchStart + vsHalfWidth;
            JPL_ASSERT(channelVSStart <= Traits::two_pi);

            for (uint32 vi = 0; vi < virtualSourcesPerChannel; ++vi)
            {
                const Traits::AngleType sourceAngle = channelVSStart + vsBaseWidth * vi;
                channelGroup.VirtualSources.emplace_back(VirtualSource<Traits>{ .Angle = sourceAngle });
            }
        }

        // Convert to [-Pi, Pi] for simpler math when processing panning
        std::ranges::for_each(ChannelGroups, [](ChannelGroup<Traits>& channelGroup)
        {
            if (channelGroup.Angle > Traits::pi)
                channelGroup.Angle -= Traits::two_pi;

            std::ranges::for_each(channelGroup.VirtualSources, [](VirtualSource<Traits>& virtualSource)
            {
                if (virtualSource.Angle > Traits::pi)
                    virtualSource.Angle -= Traits::two_pi;
            });
        });

        return true;
    }

    //==========================================================================
    template<class Traits>
    JPL_INLINE int VBAPanner<Traits>::AngleNormalizedToLUTPosition(AngleType angleNormalised) const
    {
        const int pos = static_cast<int>((angleNormalised / Traits::two_pi) * mLUTResolution + Traits::AngleType(0.5));
        return pos % mLUTResolution;
    }

    template<class Traits>
    JPL_INLINE int VBAPanner<Traits>::AngleToLUTPosition(AngleType angleInRadians) const
    {
        // Normalize to [0, 2Pi]
        if (angleInRadians < Traits::AngleType(0.0))
            angleInRadians += Traits::two_pi;
        return AngleNormalizedToLUTPosition(angleInRadians);
    }

    template<class Traits>
    JPL_INLINE int VBAPanner<Traits>::CartesianToLUTPosition(float x, float z) const
    {
        // Angle in [-Pi, Pi]
        const Traits::AngleType angle = std::atan2(x, z);
        return AngleToLUTPosition(angle);
    }

    template<class Traits>
    JPL_INLINE Traits::AngleType VBAPanner<Traits>::LUTPositionToAngle(int pos) const
    {
        return (static_cast<Traits::AngleType>(pos) / static_cast<Traits::AngleType>(mLUTResolution)) * Traits::two_pi;
    }

    template<class Traits>
    bool VBAPanner<Traits>::InitializeLUT(ChannelMap channelMap, uint8 quadrantResolution /*= sDeafultQuadrantResolution*/)
    {
        if (!channelMap.IsValid())
            return false;

        mChannelMap = channelMap;

        if (quadrantResolution < sMinQuadrantResolution)
        {
            JPL_TRACE_TAG("VBAPanner", "Trying to initialize with resolution < 90 degrees per quadrant, 90 degrees is going to be used instead.");
            quadrantResolution = sMinQuadrantResolution;
        }

        mLUTResolution = quadrantResolution * 4;

        const uint32 numChannels = channelMap.GetNumChannels();
        ChannelAngleArray speakerAngles;
        static constexpr bool skipLFE = false;
        ChannelAngle::GetSortedChannelAngles(channelMap, speakerAngles, skipLFE);

        JPL_ASSERT(speakerAngles.size() == numChannels);
        JPL_ASSERT(numChannels <= Traits::MAX_CHANNELS);

        const uint32 lfeIndex = channelMap.GetChannelIndex(EChannel::LFE);

        mNumChannels = numChannels;
        mLUT.clear();
        mLUT.resize(mNumChannels * mLUTResolution, GainType(0.0));

        for (int pos = 0; pos < mLUTResolution; ++pos)
        {
            // Position of the next angle value in the LUT
            // (number of channels stride)
            const uint32_t offset = mNumChannels * pos;

            if (mNumChannels == 1)
            {
                mLUT[offset + speakerAngles[0].ChannelId] = GainType(1.0);
                continue;
            }

            const AngleType theta = LUTPositionToAngle(pos);

            // Assign speaker contribution values
            uint32 s;
            for (s = 0; s < speakerAngles.size() - 1; s++)
            {
                // LFE should be at index 0 or 1 in sorted `speakerAngles`
                if (speakerAngles[s].ChannelId == lfeIndex)
                {
                    // Mute LFE channel, user can handle it elsewhere
                    mLUT[offset + speakerAngles[s].ChannelId] = GainType(0.0);
                    continue;
                }

                const uint32 s1 = s;
                const uint32 s2 = speakerAngles[s1 + 1].ChannelId == lfeIndex ? s1 + 2 : s1 + 1;

                const ChannelAngle& ch1 = speakerAngles[s1];
                const ChannelAngle& ch2 = speakerAngles[s2];

                AngleType angle1 = ch1.Angle;
                AngleType angle2 = ch2.Angle;

                // Handle wrap-around between last and first speaker
                if (s1 == mNumChannels - 1 && angle2 < angle1)
                    angle2 += Traits::two_pi;

                // Adjust theta if necessary
                AngleType thetaAdjusted = theta;
                if (theta < angle1 && angle2 - angle1 > Traits::pi)
                    thetaAdjusted += Traits::two_pi;

                if (thetaAdjusted >= angle1 && thetaAdjusted < angle2)
                {
                    const AngleType alpha = (thetaAdjusted - angle1) / (angle2 - angle1) * Traits::half_pi;
                    mLUT[offset + ch1.ChannelId] = std::cos(alpha);
                    mLUT[offset + ch2.ChannelId] = std::sin(alpha);
                    break;
                }
            }

            // This should not be possible
            JPL_ASSERT(speakerAngles[s].ChannelId != lfeIndex);

            // Handle wrap-around between last and first speaker
            if (s == speakerAngles.size() - 1)
            {
                const ChannelAngle& ch1 = speakerAngles[s];
                const ChannelAngle& ch2 = speakerAngles[0];

                const AngleType angle1 = ch1.Angle;
                const AngleType angle2 = ch2.Angle + Traits::two_pi;

                AngleType thetaAdjusted = theta;
                if (theta < angle1)
                    thetaAdjusted += Traits::two_pi;

                const AngleType alpha = (thetaAdjusted - angle1) / (angle2 - angle1) * Traits::half_pi;
                mLUT[offset + ch1.ChannelId] = std::cos(alpha);
                mLUT[offset + ch2.ChannelId] = std::sin(alpha);
            }
        }

        return true;
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::ProcessAngle(AngleType theta, std::span<const AngleType> speakerAnalges, std::span<GainType> outGains)
    {
        JPL_ASSERT(speakerAnalges.size() == outGains.size());

        int channel1 = 0;
        int channel2 = 0;
        const int lastChannel = static_cast<int>(speakerAnalges.size() - 1);
        AngleType alpha = 0.0f;

        for (; channel1 < lastChannel; ++channel1)
        {
            const AngleType angle1 = speakerAnalges[channel1];
            const AngleType angle2 = speakerAnalges[channel1 + 1];

            if (theta >= angle1 && theta < angle2)
            {
                alpha = Traits::half_pi * (theta - angle1) / (angle2 - angle1);
                break;
            }
        }

        if (channel1 == lastChannel)
        {
            const AngleType angle1 = speakerAnalges[lastChannel];
            const AngleType angle2 = speakerAnalges[0];

            if (theta < angle2)
                theta += Traits::two_pi;

            alpha = Traits::half_pi * (theta - angle1) / (Traits::two_pi + angle2 - angle1);
        }
        else
        {
            channel2 = channel1 + 1;
        }

        outGains[channel1] = std::cos(alpha);
        outGains[channel2] = std::sin(alpha);
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::GetSpeakerGains(int lutPosition, std::span<GainType> outGains) const
    {
        JPL_ASSERT(outGains.size() <= mNumChannels);

        // Sanity check
        static_assert(std::is_same_v<decltype(mLUT)::value_type, typename Traits::GainType>);

        const GainType* speakerGain = &mLUT[mNumChannels * lutPosition];
        std::memcpy(outGains.data(), speakerGain, sizeof(GainType) * outGains.size());
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::GetSpeakerGains(AngleType angle, std::span<GainType> outGains) const
    {
        GetSpeakerGains(AngleToLUTPosition(angle), outGains);
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::GetSpeakerGains(float x, float z, std::span<GainType> outGains) const
    {
        GetSpeakerGains(CartesianToLUTPosition(x, z), outGains);
    }

    template<class Traits>
    template<class AngleProjection>
    JPL_INLINE void VBAPanner<Traits>::ProcessVirtualSources(
        std::span<const VirtualSource> virtualSources,
        std::span<GainType> outGains,
        const AngleProjection& projection) const requires std::is_invocable_r_v<AngleType, AngleProjection, AngleType>
    {
        JPL_ASSERT(outGains.size() <= mNumChannels);

        std::ranges::for_each(virtualSources, [this, &outGains, &projection](const VirtualSource& virtualSource)
        {
            GainType buffer[Traits::MAX_CHANNELS]{ GainType(0.0) };
            std::span<GainType> mixBuffer(buffer, outGains.size());

            GetSpeakerGains(projection(virtualSource.Angle), mixBuffer);

            // Accumulate output channel gains of all the VSs
            AccumulateChannelGains(mixBuffer, outGains);
        });

        NormalzieAccumulatedGains(outGains, virtualSources.size());
    }

    template<class Traits>
    inline void VBAPanner<Traits>::ProcessVBAPData(VBAPData& vbap, const PanUpdateData& updateData) const
    {
        JPL_ASSERT(!vbap.ChannelGroups.empty());

        // Clear all channel group gains
        std::ranges::for_each(vbap.ChannelGroups, [](ChannelGroup& channelGroup)
        {
            std::memset(channelGroup.Gains.data(), 0, channelGroup.Gains.size() * sizeof(Traits::ChannelGains::value_type));
        });

        // Calculate and accumulate channel panning gains
        std::ranges::for_each(vbap.ChannelGroups, [this, &updateData](ChannelGroup& channelGroup)
        {
            auto applyPanParameters = [&channelGroup, &updateData](AngleType angle)
            {
                // Calculate VBAP channel gains based on source direction + virtual source offset angle.
                // We simply rotate, or offset each VS by the sound source direction panning angle.
                return updateData.PanAngle +
                    ApplyFocusAndSpread({
                        .SourceAngle = angle,
                        .GroupAngle = channelGroup.Angle,
                                        }, {
                                            .Focus = updateData.Focus,
                                            .Spread = updateData.Spread
                                        });
            };

            ProcessVirtualSources(channelGroup.VirtualSources, channelGroup.Gains | std::views::take(mNumChannels), applyPanParameters);

            // TODO: handle Source Orientation somehow
            //       (probably should add a user options to chose spatialization base on just position, or position + orientation)
        });
    }

    template<class Traits>
    JPL_INLINE Traits::AngleType VBAPanner<Traits>::ApplyFocusAndSpread(const PanSource& panSoruce, const PanParameters& parameters)
    {
        // Calculate VS angle based on Focus and Spread values.
        // Focus is essentially how much energy between all VSs of the group
        // is concentrated at the vector of that group, i.e. how close each VS to the group centre.
        // This means we can simply lerp the angle/position of each VS to its corresponding group angle,
        // and multiplying that value by the spread effectively scales all the angles
        // towards the 0 (forward) angle, effectively reducing the maximum arch
        // of all the sources, consequently narrowing the energy distribution.
        // 
        // Because we apply gain compensation at the end, we can have any number of VSs at any position,
        // and the overal energy should remain constant/consistent.
        auto lerp = [](auto a, auto b, auto t) { return a + t * (b - a); };
        return lerp(panSoruce.SourceAngle, panSoruce.GroupAngle, parameters.Focus) * parameters.Spread;
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::AccumulateChannelGains(std::span<const GainType> contributingGains, std::span<GainType> outputGains)
    {
        std::ranges::transform(contributingGains, outputGains, outputGains.begin(), [](GainType sourceGain, GainType groupGains)
        {
            return groupGains + sourceGain * sourceGain;
        });
    }

    template<class Traits>
    JPL_INLINE void VBAPanner<Traits>::NormalzieAccumulatedGains(std::span<GainType> gains, std::integral auto numVirtualSources)
    {
        const GainType inverseNumSources = GainType(1.0) / numVirtualSources;
        std::ranges::transform(gains, gains.begin(), [inverseNumSources](GainType g)
        {
            return std::sqrt(g * inverseNumSources);
        });
    }

} // namespace JPL
