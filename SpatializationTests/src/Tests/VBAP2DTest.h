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

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/MinimalQuat.h"

#include "JPLSpatial/Panning/VBAPanning2D.h"
#include "JPLSpatial/Panning/VBAPanning3D.h"
#include "JPLSpatial/Panning/VBAPEx.h"
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Algo/Algorithm.h"


#include "../Utility/TestUtils.h"

#include <gtest/gtest.h>
#include <numbers>
#include <algorithm>
#include <format>
#include <set>
#include <concepts>
#include <memory>
#include <ranges>

#include "../Utility/Vec3Types.h"

namespace JPL
{
	class VBAPTest : public testing::Test
	{
	protected:
		using Vec3 = MinimalVec3;

	protected:
		VBAPTest() = default;

		void SetUp() override
		{
		}

		void TearDown() override
		{
		}

		static constexpr auto toRad(std::floating_point auto degree) { return static_cast<decltype(degree)>(degree * (std::numbers::pi / 180.0)); };
		static constexpr auto toDegrees(std::floating_point auto rads) { return static_cast<decltype(rads)>(rads * (180.0 / std::numbers::pi)); };

	protected:
		struct ChannelMaskTest
		{
			std::string Label;
			uint32 Mask;
		};

		const std::vector<ChannelMaskTest> mChannelMasks
		{
			{ "IVNALID",		ChannelMask::Invalid },
			{ "Mono",			ChannelMask::Mono },
			{ "Stereo",			ChannelMask::Stereo },
			{ "Quad",			ChannelMask::Quad },
			{ "Surround 4.1",	ChannelMask::Surround_4_1 },
			{ "Surround 5.1",	ChannelMask::Surround_5_1 },
			{ "Surround 6.1",	ChannelMask::Surround_6_1 },
			{ "Surround 7.1",	ChannelMask::Surround_7_1 },
		};

		struct NamedChannelLayout
		{
			std::string Name;
			ChannelMap Layout;
		};

		using ChannelPoints = std::vector<GroupedPoint<Vec3>>;

		struct SourcePanningVisualization
		{
			static ChannelPoints AssignGroup(std::span<const Vec3> vectors, int group)
			{
				ChannelPoints points;
				points.reserve(vectors.size());
				for (const Vec3& v : vectors)
					points.emplace_back(v, group);
				return points;
			}

			void operator()(std::span<const Vec3> channelVSs, uint32 channelID)
			{
				auto p = AssignGroup(channelVSs, channelID);
				Points.insert(Points.end(), p.begin(), p.end());
			}

			void Reset()
			{
				Points.clear();
			}

			ChannelPoints Points;
		};

		struct SpeakerVisualization
		{
			using ChannelGains = typename VBAPStandartTraits::ChannelGains;

			void SetLayout(ChannelMap targetLayout)
			{
				Reset();

				targetLayout.ForEachChannel([this](EChannel channel/*, uint32 index*/)
				{
					if (channel != EChannel::LFE)
						Points.emplace_back(VBAPStandartTraits::GetChannelVector(channel), 0.0f);
				});
			}

			// Accumulate single source channel gains per output channel
			void AddChannelGains(const ChannelGains sourceGains)
			{
				for (uint32 ch = 0; ch < Points.size(); ++ch)
					Points[ch].Intensity += sourceGains[ch];
			}

			// Accumulate multiple source channel gains per output channel
			void AddChannelGains(const std::vector<ChannelGains>& sourceChannelGains)
			{
				for (const ChannelGains& sourceGains : sourceChannelGains)
					AddChannelGains(sourceGains);
			}

			void Reset()
			{
				Points.clear();
			}

			std::vector<IntencityPoint<Vec3>> Points;
		};
	};

	TEST_F(VBAPTest, LUTPositionToAngle)
	{
		VBAPanner2D<> panner;
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		std::array<float, 4> lutAngles;
		std::array<float, 4> lutAnglesExpected
		{
			0.0f,
			90.0f,
			180.0f,
			270.0f,
		};

		static constexpr std::array<int, 4> positionsToTest
		{
			0,
			128,
			256,
			384
		};

		for (int i = 0; i < positionsToTest.size(); ++i)
		{
			lutAngles[i] = toDegrees(panner.GetLUT()->LUTPositionToAngle(positionsToTest[i]));
		}

		for (auto i = 0; i < positionsToTest.size(); ++i)
		{
			SCOPED_TRACE(std::format("LUT position {}/{}", positionsToTest[i], panner.GetLUT()->GetLUTResolution()));
			EXPECT_FLOAT_EQ(lutAngles[i], lutAnglesExpected[i]);
		}
	}

	TEST_F(VBAPTest, CartesianToLUTPosition)
	{
		VBAPanner2D<> panner;
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		std::array<int, 4> positions;
		std::array<float, 4> anglesToTest
		{
			0.0f,
			90.0f,
			180.0f,
			270.0f,
		};

		static constexpr std::array<int, 4> expectedPositions
		{
			0,
			128,
			256,
			384
		};

		for (int i = 0; i < anglesToTest.size(); ++i)
		{
			positions[i] = panner.GetLUT()->CartesianToLUTPosition(std::sin(toRad(anglesToTest[i])), std::cos(toRad(anglesToTest[i])));
		}

		for (auto i = 0; i < anglesToTest.size(); ++i)
		{
			SCOPED_TRACE(std::format("Angle {}", anglesToTest[i]));
			EXPECT_EQ(positions[i], expectedPositions[i]);
		}
	}

	TEST_F(VBAPTest, InitializeLUT_PrecomputedGains)
	{
		// Initialize VBAP panner
		VBAPanner2D<> panner;

		struct VBAPTestCase
		{
			std::string description;
			JPL::ChannelMap channelMap;

			struct AngleTest
			{
				float testAngleDegrees;
				std::vector<float> expectedGains;
			};
			std::vector<AngleTest> angleTests;
		};

		const std::vector<VBAPTestCase> testCases = {
		{
			"Stereo Configuration",
			ChannelMap::FromChannelMask(ChannelMask::Stereo),
			{
				{ 315.0f,	{ 1.0f, 0.0f} },
				{ 0.0f,		{ 0.7071f, 0.7071f} },
				{ 45.0f,	{ 0.0f, 1.0f} },
				//{ 135.0f,	{ 0.5000f, 0.8660f} },
				{ 135.0f,	{ 0.0f, 1.0f} }, //! mixing from quad to stereo (using ranctangular weights), then normalizing
			}
		},
		{
			"Quadraphonic Configuration",
			ChannelMap::FromChannelMask(ChannelMask::Quad),
			{
				{ 315.0f,	{ 1.0f, 0.0f, 0.0f, 0.0f} },
				{ 45.0f,	{ 0.0f, 1.0f, 0.0f, 0.0f} },
				{ 225.0f,	{ 0.0f, 0.0f, 1.0f, 0.0f} },
				{ 135.0f,	{ 0.0f, 0.0f, 0.0f, 1.0f} },
				{ 0.0f,		{ 0.7071f, 0.7071f, 0.0f, 0.0f} },
				{ 90.0f,	{ 0.0f, 0.7071f, 0.0f, 0.7071f} },
				{ 180.0f,	{ 0.0f, 0.0f, 0.7071f, 0.7071f} },
				{ 270.0f,	{ 0.7071f, 0.0f, 0.7071f, 0.0f } },
			}
		},
		{
			"Surround 5.1 Configuration",
			ChannelMap::FromChannelMask(ChannelMask::Surround_5_1),
			{
				// Source	FL			FR			C			LFE			BL			BR
				{ 315.0f,	{ 1.0f,		0.0f,		0.0f,		0.0f,		0.0f,		0.0f} },
				{ 225.0f,	{ 0.0f,		0.0f,		0.0f,		0.0f,		1.0f,		0.0f} },
				{ 135.0f,	{ 0.0f,		0.0f,		0.0f,		0.0f,		0.0f,		1.0f} },
				{ 45.0f,	{ 0.0f,		1.0f,		0.0f,		0.0f,		0.0f,		0.0f} },
				{ 22.5f,	{ 0.0f,		0.7071f,	0.7071f,	0.0f,		0.0f,		0.0f} },
				{ 0.0f,		{ 0.0f,		0.0f,		1.0f,		0.0f,		0.0f,		0.0f} },
				{ 337.5f,	{ 0.7071f,	0.0f,		0.7071f,	0.0f,		0.0f,		0.0f} },
				{ 90.0f,	{ 0.0f,		0.7071f,	0.0f,		0.0f,		0.0f,		0.7071f} },
				{ 180.0f,	{ 0.0f,		0.0f,		0.0f,		0.0f,		0.7071f,	0.7071f} },
				{ 270.0f,	{ 0.7071f,	0.0f,		0.0f,		0.0f,		0.7071f,	0.0f } },
			}
		}
			// Add more configurations...
		};

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.description);

			if (!testCase.channelMap.IsValid())
			{
				EXPECT_FALSE(panner.InitializeLUT(testCase.channelMap));
				continue;
			}
			else
			{
				EXPECT_TRUE(panner.InitializeLUT(testCase.channelMap));
			}

			for (size_t i = 0; i < testCase.angleTests.size(); ++i)
			{
				const float testAngleDeg = testCase.angleTests[i].testAngleDegrees;
				const float testAngleRad = toRad(testAngleDeg);
				const int pos = panner.GetLUT()->AngleToLUTPosition(testAngleRad);

				// Retrieve gains from LUT
				const uint32_t offset = panner.GetNumChannels() * pos;
				std::vector<float> lutGains(panner.GetNumChannels());
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					lutGains[ch] = panner.GetLUT()->GetLUTValue(offset + ch);
				}

				const std::vector<float>& expectedGains = testCase.angleTests[i].expectedGains;

				SCOPED_TRACE(std::format("Angle {} degrees, Position {}", testAngleDeg, pos));
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					SCOPED_TRACE(std::format("Channel {}", ch));

					EXPECT_NEAR(lutGains[ch], expectedGains[ch], 0.005f);
				}
			}
		}
	}

	TEST_F(VBAPTest, EachLUTEntryIsNormalized)
	{
		VBAPanner2D<> panner;

		for (const ChannelMaskTest& test : mChannelMasks)
		{
			SCOPED_TRACE(std::format("Channel Map: {}", test.Label));

			const ChannelMap channelMap = ChannelMap::FromChannelMask(test.Mask);
			if (!channelMap.IsValid() || channelMap.GetNumChannels() == 1)
			{
				EXPECT_FALSE(panner.InitializeLUT(channelMap));
				continue;
			}
			else
			{
				ASSERT_TRUE(panner.InitializeLUT(channelMap));
			}

			for (int pos = 0; pos < panner.GetLUT()->GetLUTResolution(); ++pos)
			{

				const uint32_t offset = panner.GetNumChannels() * pos;
				float gainSum = 0.0f;
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					gainSum += panner.GetLUT()->GetLUTValue(offset + ch) * panner.GetLUT()->GetLUTValue(offset + ch);
				}
				gainSum = Math::Sqrt(gainSum);

				const float angle = toDegrees(panner.GetLUT()->LUTPositionToAngle(pos));

				SCOPED_TRACE(std::format("Position {}, angle {}", pos, angle));
				EXPECT_NEAR(gainSum, 1.0f, 0.01f);
			}
		}
	}

	TEST_F(VBAPTest, VBAPData_InitializesCorrectly)
	{
		struct VBAPDataTest
		{
			const ChannelMaskTest ChannelMap;
			uint32 NumVirtualSources;

			struct ChannelGroupExpectedData
			{
				float Angle;
				std::vector<float> vsAngles;
			};

			const std::vector<ChannelGroupExpectedData> ExpectedChannelGroups;
		};

		// First virtual source of a channel is positioned according to the following formula:
		// ChannelAngle - HalfChannelWidth + HalfVirtualSourceWidth
		// (channel groups and virtual sources are sorted by angle)
		const std::vector<VBAPDataTest> VBAPDataTestCases
		{
			// 2 Virtual Sources
			{
				.ChannelMap = { "IVNALID", ChannelMask::Invalid },
				.NumVirtualSources = 2,
				.ExpectedChannelGroups = {}
			},
			{
				.ChannelMap = { "Mono",	ChannelMask::Mono },
				.NumVirtualSources = 2,
				.ExpectedChannelGroups = {
					{
						.Angle = 0.0f,
						.vsAngles = { -90.0f, 90.0f }
					}
				}
			},
			{
				.ChannelMap = { "Stereo",	ChannelMask::Stereo },
				.NumVirtualSources = 2,
				.ExpectedChannelGroups = {
					{
						.Angle = 90.0f,
						.vsAngles = { 45.0f, 135.0f }
					},
					{
						.Angle = -90.0f,
						.vsAngles = { -135.0f, -45.0f }
					}
				}
			},
			{
				// 90 degree per channel
				// 45 degrees per virtual source
				// -22.5 degrees virtual sources offset
				.ChannelMap = { "Quad",	ChannelMask::Quad },
				.NumVirtualSources = 2,
				.ExpectedChannelGroups = {
					{
						.Angle = 45.0f,
						.vsAngles = { 22.5f, 67.5f }
					},
					{
						.Angle = 135.0f,
						.vsAngles = { 112.5f, 157.5f }
					},
					{
						.Angle = -135.0f,
						.vsAngles = { -157.5f, -112.5f }
					},
					{
						.Angle = -45.0f,
						.vsAngles = { -67.5f, -22.5f }
					},
				}
			},
			{
				// 72 degree per channel
				// 36 degrees per virtual source
				// -18 degrees virtual sources offset
				.ChannelMap = { "Surround 5.1",	ChannelMask::Surround_5_1 },
				.NumVirtualSources = 2,
				.ExpectedChannelGroups = {
					{
						.Angle = 0.0f,
						.vsAngles = { -18.0f, 18.0f }
					},
					{
						.Angle = 72.0f,
						.vsAngles = { 54.0f, 90.0f }
					},
					{
						.Angle = 144.0f,
						.vsAngles = { 126.0f, 162.0f }
					},
					{
						.Angle = -144.0f,
						.vsAngles = { -162.0f, -126.0f }
					},
					{
						.Angle = -72.0f,
						.vsAngles = { -90.0f, -54.0f }
					}
				}
			},

			// 3 Virtual Sources
			{
				// 360 degree per channel
				// 120 degrees per virtual source
				// -120 degrees virtual sources offset
				.ChannelMap = { "Mono",	ChannelMask::Mono },
				.NumVirtualSources = 3,
				.ExpectedChannelGroups = {
					{
						.Angle = 0.0f,
						.vsAngles = { -120.0f, 0.0f, 120.0f }
					}
				}
			},
			{
				// 180 degree per channel
				// 60 degrees per virtual source
				// -60 degrees virtual sources offset
				.ChannelMap = { "Stereo",	ChannelMask::Stereo },
				.NumVirtualSources = 3,
				.ExpectedChannelGroups = {
					{
						.Angle = 90.0f,
						.vsAngles = { 30.0f, 90.0f, 150.0f }
					},
					{
						.Angle = -90.0f,
						.vsAngles = { -150.0f, -90.0f, -30.0f }
					}
				}
			},
			{
				// 90 degree per channel
				// 30 degrees per virtual source
				// -30 degrees virtual sources offset
				.ChannelMap = { "Quad",	ChannelMask::Quad },
				.NumVirtualSources = 3,
				.ExpectedChannelGroups = {
					{
						.Angle = 45.0f,
						.vsAngles = { 15.0f, 45.0f, 75.0f }
					},
					{
						.Angle = 135.0f,
						.vsAngles = { 105.0f, 135.0f, 165.0f }
					},
					{
						.Angle = -135.0f,
						.vsAngles = { -165.0f, -135.0f, -105.0f }
					},
					{
						.Angle = -45.0f,
						.vsAngles = { -75.0f, -45.0f, -15.0f }
					},
				}
			},
			{
				// 72 degree per channel
				// 24 degrees per virtual source
				// -24 degrees virtual sources offset
				.ChannelMap = { "Surround 5.1",	ChannelMask::Surround_5_1 },
				.NumVirtualSources = 3,
				.ExpectedChannelGroups = {
					{
						.Angle = 0.0f,
						.vsAngles = { -24.0f, 0.0f, 24.0f }
					},
					{
						.Angle = 72.0f,
						.vsAngles = { 48.0f, 72.0f, 96.0f }
					},
					{
						.Angle = 144.0f,
						.vsAngles = { 120.0f, 144.0f, 168.0f }
					},
					{
						.Angle = -144.0f,
						.vsAngles = { -168.0f, -144.0f, -120.0f }
					},
					{
						.Angle = -72.0f,
						.vsAngles = { -96.0f, -72.0f, -48.0f }
					}
				}
			},
		};

		VBAPanner2D<> panner;
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		VBAPanner2D<>::SourceLayoutType data;

		// Just test the fact that SourceLayout internally initializes minimum of 2 virtual sources per channel,
		// the rest of data we can test fot the actual 2 virtual sources test case
		{
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromChannelMask(ChannelMask::Stereo), data));
			EXPECT_GE(data.GetNumVirtualSources(), 4);
		}

		for (const auto& test : VBAPDataTestCases)
		{
			const uint32 virtualSourcesPerChannel = test.NumVirtualSources;

			SCOPED_TRACE(std::format("Channel Map: {} | VS/channel: {}", test.ChannelMap.Label, virtualSourcesPerChannel));


			const ChannelMap channelMap = ChannelMap::FromChannelMask(test.ChannelMap.Mask);
			if (!channelMap.IsValid())
			{
				EXPECT_FALSE(panner.InitializeSourceLayout(channelMap, data));
				continue;
			}
			else
			{
				EXPECT_TRUE(panner.InitializeSourceLayout(channelMap, data));
			}

			const uint32 expectedNumChannelGroups = channelMap.GetNumChannels() - channelMap.HasLFE();
			EXPECT_EQ(data.ChannelGroups.size(), expectedNumChannelGroups);
			ASSERT_EQ(data.ChannelGroups.size(), test.ExpectedChannelGroups.size());

			std::set<EChannel> foundChannels;

			for (uint32 i = 0; i < data.ChannelGroups.size(); ++i)
			{
				const VBAPanner2D<>::ChannelGroup& channelGroup = data.ChannelGroups[i];
				const auto& excpectedChannelGroup = test.ExpectedChannelGroups[i];

				SCOPED_TRACE(std::format("Channel: {}", channelGroup.Channel));

				// invert axis direction for rotation right-nandedness
				const float channelGroupRotationAngle = channelGroup.Rotation.GetRotationAngle(Vec3(0.0f, -1.0f, 0.0f));
				EXPECT_NEAR(toDegrees(channelGroupRotationAngle), excpectedChannelGroup.Angle, 1e-4f);

				const EChannel channel = channelMap.GetChannelAtIndex(channelGroup.Channel);
				EXPECT_TRUE(channel != EChannel::Invalid);

				// Check if we have multiple channel groups created for the same channel
				EXPECT_FALSE(foundChannels.contains(channel));
				foundChannels.emplace(channel);
			}
		}
	}

	TEST_F(VBAPTest, ProcessVirtualSources)
	{

		struct VirtualSourcesTestCase
		{
			std::string Description;
			std::vector<float> VirtualSourceAnglesDegrees;
			std::vector<float> ExpectedGains;
		};

		// Testing for Quadraphonic output channel map
		const std::vector<VirtualSourcesTestCase> testCases =
		{
			{
				"Single virtual source at 0 degrees",
				{ 0.0f },
				{ 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				"Multiple virtual sources at 0 degrees",
				{ 0.0f, 0.0f, 0.0f },
				{ 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				"Virtual sources at speaker angles",
				{ -45.0f, 45.0f, -135.0f, 135.0f },
				{ 0.5f, 0.5f, 0.5f, 0.5f }
			}
		};

		VBAPanner2D<> panner;

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(static_cast<uint32>(testCase.ExpectedGains.size()))));

			// Prepare virtual sources
			std::vector<typename VBAPanner2D<>::VirtualSource> virtualSources;
			const float weight = 1.0f / testCase.VirtualSourceAnglesDegrees.size();
			for (float angleDeg : testCase.VirtualSourceAnglesDegrees)
			{
				const float rad = toRad(angleDeg);
				virtualSources.push_back({ Vec3{ std::sin(rad), 0.0f, -std::cos(rad)}, weight });
			}

			std::vector<float> outGains(panner.GetNumChannels(), 0.0f);
			panner.ProcessVirtualSources(virtualSources, outGains);

			ASSERT_EQ(outGains.size(), testCase.ExpectedGains.size());
			for (size_t i = 0; i < outGains.size(); ++i)
			{
				EXPECT_NEAR(outGains[i], testCase.ExpectedGains[i], 1e-5f) << "at index " << i;
			}
		}
	}

	TEST_F(VBAPTest, ProcessVBAPData)
	{
		// TODO: At Focus 1, spread 1 (or 0.5?), channel mapping should be direct from source to output, or do we still want to spread evenly?

		using PannerType = VBAPanner2D<>;
		PannerType panner;

		// Quadraphonic channel layout
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		struct ProcessVBAPDataTestCase
		{
			std::string Description;
			float PanAngleDegrees;
			float Spread;
			float Focus;
			std::vector<float> ExpectedGains; // Expected gains for the ChannelGroup's gains
		};

		const std::vector<ProcessVBAPDataTestCase> testCases = {
		{
			.Description = "PanAngle 0 degrees",
			.PanAngleDegrees = 0.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.7071f, 0.7071f, 0.0f, 0.0f }
		},
		{
			.Description = "PanAngle 90 degrees",
			.PanAngleDegrees = 90.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 0.7071f, 0.0f, 0.7071f }
		},
		{
			.Description = "PanAngle -90 degrees",
			.PanAngleDegrees = -90.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.7071f, 0.0f, 0.7071f, 0.0f}
		},
		{
			.Description = "PanAngle 180 degrees",
			.PanAngleDegrees = 180.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 0.0f, 0.7071f, 0.7071f }
		},
		{
			.Description = "PanAngle 45 degrees",
			.PanAngleDegrees = 45.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 1.0f, 0.0f, 0.0f }
		}
			// Add more tests if needed...
		};

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			typename PannerType::SourceLayoutType data;
			// 1 channel group with 2 virtual sources at positions { -90, 90 } in radians
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromNumChannels(1), data));

			const float panRad = toRad(testCase.PanAngleDegrees);
			typename PannerType::PanUpdateData positionData
			{
				.SourceDirection = Vec3{ std::sin(panRad), 0.0f, -std::cos(panRad) },
				.Focus = testCase.Focus,
				.Spread = testCase.Spread
			};

			typename VBAPStandartTraits::ChannelGains gains;

			panner.ProcessVBAPData(data, positionData, [&gains](uint32 channel) -> auto& { return gains; });

			ASSERT_TRUE(testCase.ExpectedGains.size() <= gains.size());
			for (size_t i = 0; i < testCase.ExpectedGains.size(); ++i)
			{
				EXPECT_NEAR(gains[i], testCase.ExpectedGains[i], 1e-4f) << "at index " << i;
			}
		}
	}

	TEST_F(VBAPTest, VBAPPannedGainsL2Normalzied)
	{
		VBAPanner2D<> panner;

		using ParametersType = typename VBAPanner2D<>::PanUpdateData;

		const std::vector<NamedChannelLayout> testTargetsChannelMaps
		{
			{ "Stereo",			ChannelMap::FromChannelMask(ChannelMask::Stereo) },
			{ "LCR",			ChannelMap::FromChannelMask(ChannelMask::LCR) },
			{ "Quad",			ChannelMap::FromChannelMask(ChannelMask::Quad) },
			{ "Surround 4.1",	ChannelMap::FromChannelMask(ChannelMask::Surround_4_1) },
			{ "Surround 5.0",	ChannelMap::FromChannelMask(ChannelMask::Surround_5_0) },
			{ "Surround 5.1",	ChannelMap::FromChannelMask(ChannelMask::Surround_5_1) },
			{ "Surround 6.0",	ChannelMap::FromChannelMask(ChannelMask::Surround_6_0) },
			{ "Surround 6.1",	ChannelMap::FromChannelMask(ChannelMask::Surround_6_1) },
			{ "Surround 7.0",	ChannelMap::FromChannelMask(ChannelMask::Surround_7_0) },
			{ "Surround 7.1",	ChannelMap::FromChannelMask(ChannelMask::Surround_7_1) },
			{ "Octogonal",		ChannelMap::FromChannelMask(ChannelMask::Octagonal) },
		};

		const std::vector<ChannelMap> testSourceChannelMaps
		{
			ChannelMap::FromChannelMask(ChannelMask::Mono),
			ChannelMap::FromChannelMask(ChannelMask::Stereo),
			ChannelMap::FromChannelMask(ChannelMask::LCR),
			ChannelMap::FromChannelMask(ChannelMask::Quad),
			ChannelMap::FromChannelMask(ChannelMask::Surround_5_0),
			ChannelMap::FromChannelMask(ChannelMask::Surround_6_0),
			ChannelMap::FromChannelMask(ChannelMask::Surround_7_0),
			ChannelMap::FromChannelMask(ChannelMask::Octagonal),

			// layouts with LFE
			ChannelMap::FromChannelMask(ChannelMask::Surround_4_1),
			ChannelMap::FromChannelMask(ChannelMask::Surround_5_1),
			ChannelMap::FromChannelMask(ChannelMask::Surround_6_1),
			ChannelMap::FromChannelMask(ChannelMask::Surround_7_1),
		};

		const std::vector<Vec3> testDirection
		{
			Vec3(1, 0, 0),
			Vec3(-1, 0, 0),
			Vec3(0, 1, 0),
			Vec3(0, -1, 0),
			Vec3(0, 0, 1),
			Vec3(0, 0, -1),
		};

		const std::vector<float> testFocus
		{
			0.0f,
			0.5f,
			1.0f
		};

		const std::vector<float> testSpread
		{
			0.0f,
			0.25f,
			0.5f,
			0.75f,
			1.0f
		};

		auto forEachTestParam = [&](auto testBody)
		{
			for (const ChannelMap& channelMap : testSourceChannelMaps)
			{
				for (const Vec3& direction : testDirection)
				{
					for (float focus : testFocus)
					{
						for (float spread : testSpread)
						{
							testBody(channelMap,
									 ParametersType{
										 .SourceDirection = direction,
										 .Focus = focus,
										 .Spread = spread
									 });
						}
					}
				}
			}
		};

		static auto paramsToString = [](const ParametersType& params)
		{
			return std::format("SourceDireciton = {}, Focus = {}, Spread = {}",
							   (std::stringstream() << params.SourceDirection).str(),
							   params.Focus,
							   params.Spread);
		};

		for (const NamedChannelLayout& test : testTargetsChannelMaps)
		{
			SCOPED_TRACE(std::format("Target Speaker Layout: {}", test.Name));

			if (!test.Layout.IsValid() || test.Layout.GetNumChannels() < 2)
				continue;

			ASSERT_TRUE(panner.InitializeLUT(test.Layout));

			auto testResultingGainSum = [&panner](
				const ChannelMap& sourceChannels,
				const ParametersType& params)
			{
				typename VBAPanner2D<>::SourceLayoutType data;
				ASSERT_TRUE(panner.InitializeSourceLayout(sourceChannels, data));

				const uint32 numTargetChannels = panner.GetNumChannels();
				const uint32 numChannels = sourceChannels.GetNumChannels();

				std::vector<typename VBAPStandartTraits::ChannelGains> sourceChannelGains(numChannels, { 0.0f });
				for (auto& chg : sourceChannelGains)
					chg.fill(0.0f);

				auto getSourcChannelGroupGains = [&sourceChannelGains](uint32 sourceChannelIndex) -> typename VBAPanner2D<>::ChannelGainsRef
				{
					return sourceChannelGains[sourceChannelIndex];
				};

				panner.ProcessVBAPData(data, params, getSourcChannelGroupGains);

				SCOPED_TRACE(std::format("Source number of channels: {} | Parameters: {}",
										 numChannels, paramsToString(params)));

				typename VBAPStandartTraits::ChannelGains accumulatedGains;
				accumulatedGains.fill(0.0f);

				// Accumulate per output channel gains
				for (const auto& channelGains : sourceChannelGains)
				{
					for (uint32 outChannelI = 0; outChannelI < channelGains.size(); ++outChannelI)
						accumulatedGains[outChannelI] += channelGains[outChannelI];
				}

				// Overall loudness of the output channels has to be near 1
				const float gainSumNorm = Accumulate(accumulatedGains, 0.0f, Algo::AccPow2<float>{});
				EXPECT_NEAR(gainSumNorm, 1.0f, 2e-6f);
			};

			forEachTestParam(testResultingGainSum);
		}
	}

#if 0
	TEST_F(VBAPTest, VBAPVisualization)
	{
		SpeakerVisualization speakerVis;
		SourcePanningVisualization sourceVis;

		auto generateLayout = [&speakerVis, &sourceVis]<class Params>(
			ChannelMap targetLayout,
			ChannelMap sourceLayout,
			const Params& params)
		{
			VBAPanner2D<> panner;
			if (!panner.InitializeLUT(targetLayout))
				return;

			const int numChannels = sourceLayout.GetNumChannels() - sourceLayout.HasLFE();

			std::vector<typename VBAPStandartTraits::ChannelGains> sourceChannelGains(numChannels, { 0.0f });

			auto getSourcChannelGroupGains = [&sourceChannelGains](uint32 sourceChannelIndex) -> typename VBAPanner2D<>::ChannelGainsRef
			{
				return sourceChannelGains[sourceChannelIndex];
			};

			VBAPanner2D<>::SourceLayoutType data;
			panner.InitializeSourceLayout(sourceLayout, data);

			speakerVis.Reset();

			panner.ProcessVBAPData(data, params, getSourcChannelGroupGains, sourceVis);

			// Accumulate speaker energy visualization for target layout
			speakerVis.SetLayout(targetLayout);
			speakerVis.AddChannelGains(sourceChannelGains);
		};

		//? this is no longer valid
		auto generateSingleDirectionVis = [&speakerVis, &sourceVis](ChannelMap targetLayout, const Vec3& direction)
		{
			VBAPanner2D<> panner;
			if (!panner.InitializeLUT(targetLayout))
				return;

			VBAPanner2D<>::SourceLayoutType data;
			panner.InitializeSourceLayout(ChannelMap::FromChannelMask(ChannelMask::Mono), data);

			typename VBAPStandartTraits::ChannelGains sourceGains{ 0.0f };

			//! test specific direction
			std::array<float, VBAPStandartTraits::MAX_CHANNELS> gains{ 0.0f };
			panner.GetSpeakerGains(direction, gains);
			for (uint32 i = 0; i < sourceGains.size(); ++i)
				sourceGains[i] = gains[i] * gains[i] * 0.5f;

			// Normalize
			for (float& g : sourceGains)
				g = Math::Sqrt(g);
			
			sourceVis.Reset();
			sourceVis(std::span<const Vec3>{ &direction, 1 }, 0);

			// Accumulate speaker energy visualization for target layout
			speakerVis.SetLayout(targetLayout);
			speakerVis.AddChannelGains(sourceGains);
		};

		const auto layoutSurround7 = ChannelMap::FromChannelMask(ChannelMask::Surround_7_0);
		const auto layoutSurround6 = ChannelMap::FromChannelMask(ChannelMask::Surround_6_0);
		const auto layoutSurround = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0);
		const auto layoutQuad = ChannelMap::FromChannelMask(ChannelMask::Quad);
		const auto layoutMono = ChannelMap::FromChannelMask(ChannelMask::Mono);
		const auto layoutStereo = ChannelMap::FromChannelMask(ChannelMask::Stereo);
		const auto layoutLCR = ChannelMap::FromChannelMask(ChannelMask::LCR);

		//? keep in mind, this rotation does not include aligning with -Z foward, so for listener facing -Z, it's going to be backwards
		//const auto sourceRotation = Basis<Vec3>::Rotation(Vec3(0.0f, 0.0f, 1.0f), -JPL_HALF_PI * 0.5f);

		const auto sourceRotation = Basis<Vec3>::FromUpAndForward(Vec3(0.0f, 1.0f, 0.0f), Vec3(-1.0f, 0.0f, 0.0f));
		
#if 0
		const typename VBAPanner2D<>::PanUpdateDataWithOrientation params
		{
			.Pan = {
				//.SourceDirection = Vec3(0.0f, -0.5f, -0.866f).Normalize(), // 30 degrees down
				.SourceDirection = Vec3(1.0f, 0.0f, -1.0f).Normalize(),
				.Focus = 0.0f,
				.Spread = 1.0f,
			},
			.Orientation = {
				// Roll right
				//.Up = Vec3(1.0f, 1.0f, 0.0f).Normalize(),
				//.Forward = Vec3(0.0f, 0.0f, -1.0f).Normalize()
				
				// Lean back
				//.Up = Vec3(0.0f, 1.0f, 1.0f).Normalize(),
				//.Forward = Vec3(0.0f, 1.0f, -1.0f).Normalize()

				/*.Up = Vec3(0.0f, 1.0f, 0.0f).Normalize(),
				.Forward = Vec3(-1.0f, 0.0f, 0.0f).Normalize()*/

				.Up = sourceRotation.Y,
				.Forward = sourceRotation.Z,
			}
		};
#endif

		const typename VBAPanner2D<>::PanUpdateData params
		{
			.SourceDirection = Vec3(1.0f, 1.0f, -1.0f).Normalize(), // checking there's no roll
			.Focus = 0.0f,
			.Spread = 0.5f,
		};
		generateLayout(layoutSurround, layoutQuad, params);
		//ChannelPoints points = generateSingleDirectionVis(layout54, Vec3(-0.5f, -1.0f, 0.3f).Normalize());

		/*std::cout << "Per channel gains: " << '\n';
		for (uint32 ch = 0; ch < channelVis.size(); ++ch)
			std::cout << std::format("Channel [{}]: {}", ch, channelVis[ch].Intensity) << '\n';*/

		// for the visualization with plot_vbap.py
		SaveToJSON(sourceVis.Points, speakerVis.Points, "visdata.json");
	}
#endif


	TEST_F(VBAPTest, WorksWithDifferentVec3Types)
	{
		const auto sourceChannelMap = ChannelMap::FromChannelMask(ChannelMask::Quad);
		const auto targetChannelMap = ChannelMap::FromChannelMask(ChannelMask::Quad);

		struct VirtualSourcesTestCase
		{
			std::string Description;
			std::vector<float> VirtualSourceAnglesDegrees;
			std::vector<float> ExpectedGains;
		};

		// Testing for Quadraphonic output channel map
		const std::vector<VirtualSourcesTestCase> testCases =
		{
			{
				"Single virtual source at 0 degrees",
				{ 0.0f },
				{ 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				"Multiple virtual sources at 0 degrees",
				{ 0.0f, 0.0f, 0.0f },
				{ 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				"Virtual sources at speaker angles",
				{ -45.0f, 45.0f, -135.0f, 135.0f },
				{ 0.5f, 0.5f, 0.5f, 0.5f }
			}
		};

		auto testVec3Type = [&]<class Vec3Type>()
		{
			using PannerType = VBAPanner2D<VBAPBaseTraits<Vec3Type>>;
			PannerType panner;

			ASSERT_TRUE(panner.InitializeLUT(targetChannelMap));

			typename PannerType::SourceLayoutType sourceLayout;
			ASSERT_TRUE(panner.InitializeSourceLayout(sourceChannelMap, sourceLayout));

			for (const auto& testCase : testCases)
			{
				SCOPED_TRACE(testCase.Description);

				// Prepare virtual sources
				std::vector<typename PannerType::VirtualSource> virtualSources;
				const float weight = 1.0f / testCase.VirtualSourceAnglesDegrees.size();
				for (float angleDeg : testCase.VirtualSourceAnglesDegrees)
				{
					const float rad = toRad(angleDeg);
					virtualSources.push_back({ Vec3Type{ std::sin(rad), 0.0f, -std::cos(rad)}, weight });
				}

				std::vector<float> outGains(panner.GetNumChannels(), 0.0f);
				panner.ProcessVirtualSources(virtualSources, outGains);

				ASSERT_EQ(outGains.size(), testCase.ExpectedGains.size());
				for (size_t i = 0; i < outGains.size(); ++i)
				{
					EXPECT_NEAR(outGains[i], testCase.ExpectedGains[i], 1e-5f) << "at index " << i;
				}
			}
		};
#if defined(JPL_TEST_WITH_JOLT)
		testVec3Type.operator()<JPH::Vec3>();
#endif
		testVec3Type.operator()<glm::vec3>();
		testVec3Type.operator()<MinimalVec3>();
	}

} // namespace JPL