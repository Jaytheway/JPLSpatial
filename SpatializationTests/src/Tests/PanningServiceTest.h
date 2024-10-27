//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatialTests **
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

#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Services/PanningService.h"

#include <gtest/gtest.h>

#include <vector>
#include <memory>

namespace JPL
{
    class PanningServiceTest : public ::testing::Test
    {
    protected:
        JPL::PanningService panningService;

        // Helper methods to create valid/invalid channel maps and handles
        JPL::ChannelMap CreateValidChannelMap(uint32 channelMask) const;
        JPL::ChannelMap CreateInvalidChannelMap() const;
        JPL::PanEffectHandle CreateInvalidPanEffectHandle() const;

        // Helper method to create a list of valid target channel maps
        std::vector<JPL::ChannelMap> CreateValidTargetChannelMaps(std::initializer_list<uint32> channelMasks) const;
    };

    JPL::ChannelMap PanningServiceTest::CreateValidChannelMap(uint32 channelMask) const
    {
        // Assuming ChannelMap can be constructed with a valid layout
        return JPL::ChannelMap::FromChannelMask(channelMask);
    }

    JPL::ChannelMap PanningServiceTest::CreateInvalidChannelMap() const
    {
        // Return a default-constructed or explicitly invalid ChannelMap
        return JPL::ChannelMap(); // Assuming default is invalid
    }

    JPL::PanEffectHandle PanningServiceTest::CreateInvalidPanEffectHandle() const
    {
        // Return a default-constructed or explicitly invalid handle
        return JPL::PanEffectHandle(); // Assuming default is invalid
    }

    std::vector<JPL::ChannelMap> PanningServiceTest::CreateValidTargetChannelMaps(std::initializer_list<uint32> channelMasks) const
    {
        std::vector<JPL::ChannelMap> targetChannelMaps;
        for (uint32 channelMask : channelMasks)
        {
            targetChannelMaps.push_back(JPL::ChannelMap::FromChannelMask(channelMask));
        }
        return targetChannelMaps;
    }
    

    TEST_F(PanningServiceTest, CreatePannerFor_ValidChannelMap_ReturnsPanner)
    {
        JPL::ChannelMap validChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        const JPL::StandartPanner* panner = panningService.CreatePannerFor(validChannelMap);

        ASSERT_NE(panner, nullptr);
        EXPECT_TRUE(panner->IsInitialized());

        // Verify that the panner is stored in mPanners
        const JPL::StandartPanner* storedPanner = panningService.GetPannerFor(validChannelMap);
        EXPECT_EQ(panner, storedPanner);
    }

    TEST_F(PanningServiceTest, CreatePannerFor_InvalidChannelMap_ReturnsNullptr)
    {
        JPL::ChannelMap invalidChannelMap = CreateInvalidChannelMap();
        const JPL::StandartPanner* panner = panningService.CreatePannerFor(invalidChannelMap);

        EXPECT_EQ(panner, nullptr);
    }

    TEST_F(PanningServiceTest, CreatePannerFor_DuplicateCreation_ReturnsSamePanner)
    {
        JPL::ChannelMap validChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        const JPL::StandartPanner* panner1 = panningService.CreatePannerFor(validChannelMap);
        const JPL::StandartPanner* panner2 = panningService.CreatePannerFor(validChannelMap);

        ASSERT_NE(panner1, nullptr);
        ASSERT_NE(panner2, nullptr);
        EXPECT_EQ(panner1, panner2);
    }

    TEST_F(PanningServiceTest, GetPannerFor_ExistingPanner_ReturnsPanner)
    {
        JPL::ChannelMap validChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        const JPL::StandartPanner* pannerCreated = panningService.CreatePannerFor(validChannelMap);
        ASSERT_NE(pannerCreated, nullptr);

        const JPL::StandartPanner* pannerRetrieved = panningService.GetPannerFor(validChannelMap);
        EXPECT_EQ(pannerCreated, pannerRetrieved);
    }

    TEST_F(PanningServiceTest, GetPannerFor_NonExistentPanner_ReturnsNullptr)
    {
        JPL::ChannelMap validChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        const JPL::StandartPanner* panner = panningService.GetPannerFor(validChannelMap);

        EXPECT_EQ(panner, nullptr);
    }

    TEST_F(PanningServiceTest, InitializePanningEffect_ValidSourceAndTargets_ReturnsValidHandle)
    {
        JPL::ChannelMap sourceChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        std::vector<JPL::ChannelMap> targetChannelMaps = CreateValidTargetChannelMaps({ JPL::ChannelMask::Stereo, JPL::ChannelMask::Quad });

        JPL::PanEffectHandle handle = panningService.InitializePanningEffect(
            {
                .SourceChannelMap = sourceChannelMap,
                .TargetChannelMaps = targetChannelMaps
            });

        ASSERT_TRUE(handle.IsValid());

        // Verify that VBAP data is initialized
        auto vbapData = panningService.GetPanningDataFor(sourceChannelMap);
        EXPECT_NE(vbapData, nullptr);

        // Verify that channel gains are initialized for the targets
        for (const auto& targetChannelMap : targetChannelMaps)
        {
            auto channelGains = panningService.GetChannelGainsFor(handle, targetChannelMap);
            EXPECT_NE(channelGains, nullptr);
        }
    }

    TEST_F(PanningServiceTest, InitializePanningEffect_ValidSourceNoTargets_ReturnsValidHandle)
    {
        JPL::ChannelMap sourceChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);

        JPL::PanEffectHandle handle = panningService.InitializePanningEffect({ .SourceChannelMap = sourceChannelMap });

        ASSERT_TRUE(handle.IsValid());

        // Verify that VBAP data is initialized
        auto vbapData = panningService.GetPanningDataFor(sourceChannelMap);
        EXPECT_NE(vbapData, nullptr);

        // No targets, so no channel gains should be initialized
        auto channelGains = panningService.GetChannelGainsFor(handle, sourceChannelMap);
        EXPECT_EQ(channelGains, nullptr);
    }

    TEST_F(PanningServiceTest, InitializePanningEffect_InvalidSource_ReturnsInvalidHandle)
    {
        JPL::ChannelMap invalidSourceChannelMap = CreateInvalidChannelMap();
        std::vector<JPL::ChannelMap> targetChannelMaps = CreateValidTargetChannelMaps({ JPL::ChannelMask::Stereo, JPL::ChannelMask::Quad });

        JPL::PanEffectHandle handle = panningService.InitializePanningEffect(
            {
                .SourceChannelMap = invalidSourceChannelMap,
                .TargetChannelMaps = targetChannelMaps
            });

        EXPECT_FALSE(handle.IsValid());
    }

    TEST_F(PanningServiceTest, ReleasePanningEffect_ValidHandle_CleansUpData)
    {
        JPL::ChannelMap sourceChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        std::vector<JPL::ChannelMap> targetChannelMaps = CreateValidTargetChannelMaps({ JPL::ChannelMask::Stereo, JPL::ChannelMask::Quad });
        JPL::PanEffectHandle handle = panningService.InitializePanningEffect(
            {
                .SourceChannelMap = sourceChannelMap,
                .TargetChannelMaps = targetChannelMaps
            });

        ASSERT_TRUE(handle.IsValid());

        bool released = panningService.ReleasePanningEffect(handle);

        EXPECT_TRUE(released);

        // Verify that VBAP data is removed from mSourceVBAPs
        auto vbapDataIt = panningService.GetVBAPDataFor(handle);
        EXPECT_EQ(vbapDataIt, nullptr);

        // Verify that channel gains are removed
        for (const auto& targetChannelMap : targetChannelMaps)
        {
            auto channelGains = panningService.GetChannelGainsFor(handle, targetChannelMap);
            EXPECT_EQ(channelGains, nullptr);
        }
    }

    TEST_F(PanningServiceTest, ReleasePanningEffect_InvalidHandle_ReturnsFalse)
    {
        JPL::PanEffectHandle invalidHandle = CreateInvalidPanEffectHandle();

        bool released = panningService.ReleasePanningEffect(invalidHandle);

        EXPECT_FALSE(released);
    }

    TEST_F(PanningServiceTest, CreatePanningDataFor_ValidSourceChannelMap_ReturnsVBAPData)
    {
        JPL::ChannelMap sourceChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);

        auto vbapData = panningService.CreatePanningDataFor(sourceChannelMap);

        ASSERT_NE(vbapData, nullptr);
        EXPECT_TRUE(vbapData->IsInitialized());

        // Verify that VBAP data is stored
        auto storedVbapData = panningService.GetPanningDataFor(sourceChannelMap);
        EXPECT_EQ(vbapData, storedVbapData);
    }

    TEST_F(PanningServiceTest, CreatePanningDataFor_InvalidSourceChannelMap_ReturnsNullptr)
    {
        JPL::ChannelMap invalidSourceChannelMap = CreateInvalidChannelMap();

        auto vbapData = panningService.CreatePanningDataFor(invalidSourceChannelMap);

        EXPECT_EQ(vbapData, nullptr);
    }

    TEST_F(PanningServiceTest, MultipleSourcesAndTargets_IndependentData)
    {
        // Initialize first source
        JPL::ChannelMap sourceChannelMap1 = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        JPL::PanEffectHandle handle1 = panningService.InitializePanningEffect({ .SourceChannelMap = sourceChannelMap1 });
        ASSERT_TRUE(handle1.IsValid());

        // Create target channel maps for source 1 starting from Stereo layout
        std::vector<JPL::ChannelMap> targetChannelMaps1 = CreateValidTargetChannelMaps({ JPL::ChannelMask::LCR, JPL::ChannelMask::Quad });
        panningService.AddSourceTargets(handle1, targetChannelMaps1);

        // Initialize second source
        JPL::ChannelMap sourceChannelMap2 = CreateValidChannelMap(JPL::ChannelMask::Quad);
        JPL::PanEffectHandle handle2 = panningService.InitializePanningEffect({ .SourceChannelMap = sourceChannelMap2 });
        ASSERT_TRUE(handle2.IsValid());

        // Create target channel maps for source 2 starting from Surround5_1 layout
        std::vector<JPL::ChannelMap> targetChannelMaps2 = CreateValidTargetChannelMaps({ JPL::ChannelMask::Stereo, JPL::ChannelMask::Surround_5_1 });
        panningService.AddSourceTargets(handle2, targetChannelMaps2);

        // Verify that data for each source is independent
        // Check gains for source 1's targets
        for (const auto& targetChannelMap : targetChannelMaps1)
        {
            auto channelGains1 = panningService.GetChannelGainsFor(handle1, targetChannelMap);
            EXPECT_NE(channelGains1, nullptr);

            auto channelGains2 = panningService.GetChannelGainsFor(handle2, targetChannelMap);
            EXPECT_EQ(channelGains2, nullptr);
        }

        // Check gains for source 2's targets
        for (const auto& targetChannelMap : targetChannelMaps2)
        {
            auto channelGains1 = panningService.GetChannelGainsFor(handle1, targetChannelMap);
            EXPECT_EQ(channelGains1, nullptr);

            auto channelGains2 = panningService.GetChannelGainsFor(handle2, targetChannelMap);
            EXPECT_NE(channelGains2, nullptr);
        }
    }

    TEST_F(PanningServiceTest, SetPanningEffectParameters_InvalidHandle_ReturnsFalse)
    {
        JPL::PanEffectHandle invalidHandle = CreateInvalidPanEffectHandle();
        
        bool parametersSet = panningService.SetPanningEffectParameters(invalidHandle, {});

        EXPECT_FALSE(parametersSet);
    }

    TEST_F(PanningServiceTest, SetPanningEffectParameters_ValidHandle_ReturnsTrue)
    {
        JPL::ChannelMap sourceChannelMap = CreateValidChannelMap(JPL::ChannelMask::Stereo);
        std::vector<JPL::ChannelMap> targetChannelMaps = CreateValidTargetChannelMaps({ JPL::ChannelMask::Stereo, JPL::ChannelMask::Quad });

        JPL::PanEffectHandle handle = panningService.InitializePanningEffect(
            {
                .SourceChannelMap = sourceChannelMap,
                .TargetChannelMaps = targetChannelMaps
            });

        ASSERT_TRUE(handle.IsValid());

        bool parametersSet = panningService.SetPanningEffectParameters(handle, { .Focus = 0.5f, .Spread = 1.0f });

        EXPECT_TRUE(parametersSet);
    }

} // namespace JPL