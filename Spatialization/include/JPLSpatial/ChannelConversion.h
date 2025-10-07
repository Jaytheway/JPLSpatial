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
#include "JPLSpatial/Memory/Memory.h"

#include <array>
#include <vector>
#include <utility>

namespace JPL
{
    /// Implementation adapted from David Reid's https://github.com/mackron/miniaudio

    /// Rectangular conversion ratios
    // Plane LEFT      0
    // Plane RIGHT     1
    // Plane FRONT     2
    // Plane BACK      3
    // Plane BOTTOM    4
    // Plane TOP       5
    constexpr std::array<std::array<float, 6>, 22> cChannelPlaneRatios =
    {
        std::to_array({ 0.5f,  0.0f,  0.5f,  0.0f,  0.0f,  0.0f}),     // FrontLeft
        std::to_array({ 0.0f,  0.5f,  0.5f,  0.0f,  0.0f,  0.0f}),     // FrontRight
        std::to_array({ 0.0f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f}),     // FrontCentre
        std::to_array({ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}),     // LFE

        std::to_array({ 1.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}),     // SideLeft
        std::to_array({ 0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}),     // SideRight

        std::to_array({ 0.25f, 0.0f,  0.75f, 0.0f,  0.0f,  0.0f}),     // FrontLeftCentre
        std::to_array({ 0.0f,  0.25f, 0.75f, 0.0f,  0.0f,  0.0f}),     // FrontRightCenter

        std::to_array({ 0.5f,  0.0f,  0.0f,  0.5f,  0.0f,  0.0f}),     // BackLeft
        std::to_array({ 0.0f,  0.5f,  0.0f,  0.5f,  0.0f,  0.0f}),     // BackRight
        std::to_array({ 0.0f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f}),     // BackCentre

        // We put our wide channels to side planes
        std::to_array({ 1.0f,  0.0f,  0.0f,  0.0f,  0.0f,  0.0f}),     // WideLeft
        std::to_array({ 0.0f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f}),     // WideRigh

        std::to_array({ 0.0f,  0.0f,  0.0f,  0.0f,  0.0f,  1.0f}),     // TopCentre
        std::to_array({ 0.33f, 0.0f,  0.33f, 0.0f,  0.0f,  0.34f}),    // TopFrontLeft
        std::to_array({ 0.0f,  0.0f,  0.5f,  0.0f,  0.0f,  0.5f}),     // TopFrontCentre
        std::to_array({ 0.0f,  0.33f, 0.33f, 0.0f,  0.0f,  0.34f}),    // TopFrontRight

        std::to_array({ 0.5f,  0.0f,  0.0f,  0.0f,  0.0f,  0.5f}),     // TopSideLeft
        std::to_array({ 0.0f,  0.5f,  0.0f,  0.0f,  0.0f,  0.5f}),     // TopSideRight

        std::to_array({ 0.33f, 0.0f,  0.0f,  0.33f, 0.0f,  0.34f}),    // TopBackLeft
        std::to_array({ 0.0f,  0.0f,  0.0f,  0.5f,  0.0f,  0.5f}),     // TopBackCentre
        std::to_array({ 0.0f,  0.33f, 0.0f,  0.33f, 0.0f,  0.34f})     // TopBakcRight
    };

    // Paramerers have to be valid indices of cChannelPlaneRatios
    static JPL_INLINE float CalculateChannelPositionRectangularWeight(uint32 channelPositionA, uint32 channelPositionB)
    {
        const float contribution =
            cChannelPlaneRatios[channelPositionA][0] * cChannelPlaneRatios[channelPositionB][0] +
            cChannelPlaneRatios[channelPositionA][1] * cChannelPlaneRatios[channelPositionB][1] +
            cChannelPlaneRatios[channelPositionA][2] * cChannelPlaneRatios[channelPositionB][2] +
            cChannelPlaneRatios[channelPositionA][3] * cChannelPlaneRatios[channelPositionB][3] +
            cChannelPlaneRatios[channelPositionA][4] * cChannelPlaneRatios[channelPositionB][4] +
            cChannelPlaneRatios[channelPositionA][5] * cChannelPlaneRatios[channelPositionB][5];

        return contribution;
    }

    /// Utility helper to access 2D array kind of weights
    class ChannelConversionWeights
    {
    public:
        ChannelConversionWeights() = default;
        ChannelConversionWeights(uint32 numOutputs, uint32 numInputs)
        {
            Resize(numOutputs, numInputs);
        }

        ChannelConversionWeights(const ChannelConversionWeights&) = default;
        ChannelConversionWeights& operator=(const ChannelConversionWeights&) = default;
        ChannelConversionWeights(ChannelConversionWeights&&) = default;
        ChannelConversionWeights& operator=(ChannelConversionWeights&&) = default;

        JPL_INLINE void Resize(uint32 numOutputs, uint32 numInputs)
        {
            mWeights.resize(numOutputs, std::pmr::vector<float>(numInputs, 0.0f, GetDefaultMemoryResource()));
        }

        JPL_INLINE void Clear() noexcept { mWeights.clear(); }

        JPL_INLINE bool IsEmpty() const noexcept { return mWeights.empty(); }

        JPL_INLINE size_t size() const noexcept
        {
            size_t size = 0;
            for (const auto& outWeights : mWeights)
                size += outWeights.size();
            return size;
        }

        JPL_INLINE std::pair<uint32, uint32> GetDimensions() const
        {
            if (IsEmpty())
                return { 0u, 0u };

            return {
                static_cast<uint32>(mWeights.size()),
                static_cast<uint32>(mWeights[0].size())
            };
        }

        JPL_INLINE std::pmr::vector<float>& operator[](int i) { return mWeights[i]; }
        JPL_INLINE const std::pmr::vector<float>& operator[](int i) const { return mWeights[i]; }

    private:
        std::pmr::vector<std::pmr::vector<float>> mWeights{ GetDefaultMemoryResource() };
    };


    /// @param outWeights : must be at least size of `num channels in` * `num channels out`
    //template<class ChannelConversionWeightsType>
    static void ComputeChannelConversionRectangularWeights(
        ChannelMap channelMapIn,
        ChannelMap channelMapOut,
        ChannelConversionWeights& outWeights)
    {
        // Unmapped input channels.
        const uint32 numChannelsIn = channelMapIn.GetNumChannels();
        const uint32 numChannelsOut = channelMapOut.GetNumChannels();

        JPL_ASSERT(outWeights.size() >= numChannelsOut * numChannelsIn);

        //std::fill(outWeights.begin(), outWeights.end(), 0.0f);

        static auto isSpatialChannel = [](EChannel channel)
        {
            return channel != EChannel::LFE && channel != EChannel::FrontCenter && channel != EChannel::Invalid;
        };

        static auto toChannelPos = [](EChannel channel) { return std::bit_width(static_cast<uint32>(channel) - 1); };

        // We need to make sure all channels that are present in both channel maps have a 1:1 mapping.
        channelMapIn.ForEachChannel([channelMapOut, channelMapIn, &outWeights](EChannel channelIn, uint32 iChannelIn)
        {
            channelMapOut.ForEachChannel([channelIn, iChannelIn, &outWeights](EChannel channelOut, uint32 iChannelOut)
            {
                if (channelIn == channelOut)
                    outWeights[iChannelOut][iChannelIn] = 1.0f;
            });
        });


        channelMapIn.ForEachChannel([channelMapOut, numChannelsIn, &outWeights](EChannel channelIn, uint32 iChannelIn)
        {
            if (!isSpatialChannel(channelIn))
                return;

            if (channelMapOut.Has(channelIn))
                return;

            channelMapOut.ForEachChannel([channelIn, iChannelIn, numChannelsIn, &outWeights](EChannel channelOut, uint32 iChannelOut)
            {
                if (!isSpatialChannel(channelOut))
                    return;

                const float weight = CalculateChannelPositionRectangularWeight(toChannelPos(channelIn), toChannelPos(channelOut));
                //const uint32 offset = iChannelOut * numChannelsIn + iChannelIn;

                if (outWeights[iChannelOut][iChannelIn] == 0.0f)
                    outWeights[iChannelOut][iChannelIn] = weight;

                // Only apply the weight if we haven't already got some contribution from the respective channels.
                /*if (outWeights[offset] == 0.0f)
                    outWeights[offset] = weight;*/
            });
        });

        // Unmapped output channels.
        channelMapOut.ForEachChannel([channelMapIn, numChannelsIn, &outWeights](EChannel channelOut, uint32 iChannelOut)
        {
            if (!isSpatialChannel(channelOut))
                return;

            if (channelMapIn.Has(channelOut))
                return;

            channelMapIn.ForEachChannel([iChannelOut, channelOut, numChannelsIn, &outWeights](EChannel channelIn, uint32 iChannelIn)
            {
                if (!isSpatialChannel(channelIn))
                    return;

                const float weight = CalculateChannelPositionRectangularWeight(toChannelPos(channelOut), toChannelPos(channelIn));
                //const uint32 offset = iChannelOut * numChannelsIn + iChannelIn;

                if (outWeights[iChannelOut][iChannelIn] == 0.0f)
                    outWeights[iChannelOut][iChannelIn] = weight;

                // Only apply the weight if we haven't already got some contribution from the respective channels.
                /*if (outWeights[offset] == 0)
                    outWeights[offset] = weight;*/
            });
        });

        // TODO: handle LFE
    }


#if 0
    inline void ApplyChannelConversion(const ChannelConversionWeights<Array>& weights, std::span<const float> inValues, std::span<float> outValues)
    {
        //JPL_ASSERT(weights.size() >= inValues.size() + outValues.size());

        for (uint32 iChannelOut = 0; iChannelOut < outValues.size(); ++iChannelOut)
        {
            float accumulation = 0;

            for (uint32 iChannelIn = 0; iChannelIn < inValues.size(); ++iChannelIn)
                accumulation += inValues[iChannelIn] * weights[iChannelOut][iChannelIn];

            outValues[iChannelOut] = accumulation;
        }
    }
#endif

#if 0
    // Hypothetical audio block on mixing thread
    //! We can probably just reuse weight-based channel conversion interface
    void AudioMixingCallback(const float** inSamples, float** outSamples, uint32 numInChannels, uint32 numOutChannels, uint32 numSamples)
    {
        for (uint32 iChannelIn = 0; iChannelIn < numInChannels; ++iChannelIn)
        {
            const float* channelInData = inSamples[iChannelIn];

            for (uint32 iChannelOut = 0; iChannelOut < numOutChannels; ++iChannelOut)
            {
                float* channelOutData = outSamples[iChannelOut];

                // channelOutData += channelInData * VBAP
                const float g = VBAP[iChannelIn][iChannelOut];

                for (uint32 sample = 0; samples < numSamples; ++samples)
                    channelOutData[sample] += channelInData[sample] * g;
            }
        }
    }
#endif
} // namespace JPL