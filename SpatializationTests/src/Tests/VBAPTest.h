//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatialTests
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatialTests is offered under the terms of the ISC license:
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

#include "JPLSpatial/VBAP.h"
#include "JPLSpatial/ChannelMap.h"

#include <gtest/gtest.h>
#include <numbers>
#include <algorithm>
#include <format>
#include <set>
#include <concepts>

namespace JPL
{
	static constexpr auto toRad(std::floating_point auto degree) { return static_cast<decltype(degree)>(degree * (std::numbers::pi / 180.0)); };
	static constexpr auto toDegrees(std::floating_point auto rads) { return static_cast<decltype(rads)>(rads * (180.0 / std::numbers::pi)); };


	class VBAPTest : public testing::Test
	{
	protected:
		VBAPTest() = default;

		void SetUp() override
		{
		}

		void TearDown() override
		{
		}

	protected:
		static constexpr float CalculateAlpha(float theta, float angle1, float angle2)
		{
			float deltaAngle = angle2 - angle1;
			if (deltaAngle <= 0.0f)
				deltaAngle += 2.0f * std::numbers::pi_v<float>;
			float deltaTheta = theta - angle1;
			if (deltaTheta < 0.0f)
				deltaTheta += 2.0f * std::numbers::pi_v<float>;
			const float alpha = (0.5f * std::numbers::pi_v<float>) * deltaTheta / deltaAngle;
			return alpha;
		}

		void TestProcessAngle(float thetaDegrees,
			const std::vector<float>& speakerAnglesDegrees,
			const std::vector<float>& expectedGains,
			float tolerance = 1e-5f)
		{
			// Convert angles to radians
			const float theta = toRad(thetaDegrees);
			std::vector<float> speakerAngles;
			for (float angleDeg : speakerAnglesDegrees)
				speakerAngles.push_back(toRad(angleDeg));

			// Prepare the output gains array
			std::vector<float> outGains(speakerAngles.size(), 0.0f);

			VBAPanner<>::ProcessAngle(theta, speakerAngles, outGains);

			ASSERT_EQ(outGains.size(), expectedGains.size());
			for (size_t i = 0; i < outGains.size(); ++i)
			{
				EXPECT_NEAR(outGains[i], expectedGains[i], tolerance) << "at index " << i;
			}
		}

	protected:
		struct ChannelMaskTest
		{
			std::string Label;
			uint32 Mask;
		};

		const std::vector<ChannelMaskTest> channelMasks
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
	};

	TEST_F(VBAPTest, LUTPositionToAngle)
	{
		VBAPanner panner;

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
			lutAngles[i] = toDegrees(panner.LUTPositionToAngle(positionsToTest[i]));
		}

		for (auto i = 0; i < positionsToTest.size(); ++i)
		{
			SCOPED_TRACE(std::format("LUT position {}/{}", positionsToTest[i], panner.GetLUTResolution()));
			EXPECT_FLOAT_EQ(lutAngles[i], lutAnglesExpected[i]);
		}
	}

	TEST_F(VBAPTest, CartesianToLUTPosition)
	{
		VBAPanner panner;

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
			positions[i] = panner.CartesianToLUTPosition(std::sin(toRad(anglesToTest[i])), std::cos(toRad(anglesToTest[i])));
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
		VBAPanner panner;

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
				{ 135.0f,	{ 0.5000f, 0.8660f} },
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
				const int pos = static_cast<int>((testAngleRad / (2.0f * std::numbers::pi_v<float>)) * panner.GetLUTResolution() + 0.5f) % panner.GetLUTResolution();

				// Retrieve gains from LUT
				const uint32_t offset = panner.GetNumChannels() * pos;
				std::vector<float> lutGains(panner.GetNumChannels());
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					lutGains[ch] = panner.GetLUTValue(offset + ch);
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

	TEST_F(VBAPTest, InitializeLUT_GainSum)
	{
		VBAPanner panner;

		for (const ChannelMaskTest& test : channelMasks)
		{
			SCOPED_TRACE(std::format("Channel Map: {}", test.Label));

			const ChannelMap channelMap = ChannelMap::FromChannelMask(test.Mask);
			if (!channelMap.IsValid())
			{
				EXPECT_FALSE(panner.InitializeLUT(channelMap));
				continue;
			}
			else
			{
				EXPECT_TRUE(panner.InitializeLUT(channelMap));
			}

			for (int pos = 0; pos < panner.GetLUTResolution(); ++pos)
			{
				const uint32_t offset = panner.GetNumChannels() * pos;
				float gainSum = 0.0f;
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					gainSum += panner.GetLUTValue(offset + ch) * panner.GetLUTValue(offset + ch);
				}
				gainSum = std::sqrt(gainSum);

				SCOPED_TRACE(std::format("Position {}", pos));
				EXPECT_NEAR(gainSum, 1.0f, 0.01f);
			}
		}
	}

	TEST_F(VBAPTest, VBAPData_Initialzie)
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

		VBAPData data;

		// Just test the fact that VBAPData internally initializes minimum of 2 virtual sources per channel,
		// the rest of data we can test fot the actual 2 virtual sources test case
		{
			const uint32 zeroVirtualSources = 0;
			ASSERT_TRUE(data.Initialize(ChannelMap::FromChannelMask(ChannelMask::Stereo), zeroVirtualSources));
			EXPECT_EQ(data.ChannelGroups[0].VirtualSources.size(), 2);

			// TODO: maybe allow a single virtual source for the basic panning, without Focus, but with Spread functionality?

			const uint32 oneVirtualSource = 1;
			ASSERT_TRUE(data.Initialize(ChannelMap::FromChannelMask(ChannelMask::Stereo), oneVirtualSource));
			EXPECT_EQ(data.ChannelGroups[0].VirtualSources.size(), 2);
		}

		for (const auto& test : VBAPDataTestCases)
		{
			const uint32 virtualSourcesPerChannel = test.NumVirtualSources;

			SCOPED_TRACE(std::format("Channel Map: {} | VS/channel: {}", test.ChannelMap.Label, virtualSourcesPerChannel));


			const ChannelMap channelMap = ChannelMap::FromChannelMask(test.ChannelMap.Mask);
			if (!channelMap.IsValid())
			{
				EXPECT_FALSE(data.Initialize(channelMap, virtualSourcesPerChannel));
				continue;
			}
			else
			{
				EXPECT_TRUE(data.Initialize(channelMap, virtualSourcesPerChannel));
			}

			const uint32 expectedNumChannelGroups = channelMap.GetNumChannels() - channelMap.HasLFE();
			EXPECT_EQ(data.ChannelGroups.size(), expectedNumChannelGroups);
			ASSERT_EQ(data.ChannelGroups.size(), test.ExpectedChannelGroups.size());

			std::set<EChannel> foundChannels;

			for (uint32 i = 0; i < data.ChannelGroups.size(); ++i)
			{
				const ChannelGroup<>& channelGroup = data.ChannelGroups[i];
				const auto& excpectedChannelGroup = test.ExpectedChannelGroups[i];

				SCOPED_TRACE(std::format("Channel: {}", channelGroup.Channel));
				
				EXPECT_NEAR(toDegrees(channelGroup.Angle), excpectedChannelGroup.Angle, 1e-4f);

				const EChannel channel = channelMap.GetChannelAtIndex(channelGroup.Channel);
				EXPECT_TRUE(channel != EChannel::Invalid);
				
				// Check if we have multiple channel groups created for the same channel
				EXPECT_FALSE(foundChannels.contains(channel));
				foundChannels.emplace(channel);

				EXPECT_EQ(channelGroup.VirtualSources.size(), virtualSourcesPerChannel);
				ASSERT_EQ(channelGroup.VirtualSources.size(), excpectedChannelGroup.vsAngles.size());

				for (uint32 i = 0; i < channelGroup.VirtualSources.size(); ++i)
				{
					SCOPED_TRACE(std::format("VS: {}", i));

					const VirtualSource<>& virtualSource = channelGroup.VirtualSources[i];
					const float expectedVSAngle = excpectedChannelGroup.vsAngles[i];

					EXPECT_NEAR(toDegrees(virtualSource.Angle), expectedVSAngle, 1e-4f);
				}
			}
		}
	}

	TEST_F(VBAPTest, ProcessAngle)
	{
		struct ProcessAngleTest
		{
			std::string Description;
			float TestAngle;
			std::vector<float> SpeakerAnglesDegrees;
			std::vector<float> ExpectedGains;
		};

		const std::vector<ProcessAngleTest> testCases
		{
			{
				"Stereo configuration, source at 45 degrees",
				45.0f,
				{0.0f, 90.0f},
				{0.7071f, 0.7071f}
			},
			{
				"Stereo configuration, source at 0 degrees",
				0.0f,
				{0.0f, 90.0f},
				{1.0f, 0.0f}
			},
			{
				"Stereo configuration, source at 90 degrees",
				90.0f,
				{0.0f, 90.0f},
				{0.0f, 1.0f}
			},
			{
				"Stereo configuration, source at 180 degrees (wrap-around)",
				180.0f,
				{0.0f, 90.0f},
				{0.5000f, 0.8660254f}
			},
			{
				"Quadraphonic configuration, source at 45 degrees",
				45.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.7071f, 0.7071f, 0.0f, 0.0f}
			},
			{
				"Quadraphonic configuration, source at 135 degrees",
				135.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.0f, 0.7071f, 0.7071f, 0.0f}
			},
			{
				"Quadraphonic configuration, source at 225 degrees",
				225.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.0f, 0.0f, 0.7071f, 0.7071f}
			},
			{
				"Quadraphonic configuration, source at 315 degrees (wrap-around)",
				315.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.7071f, 0.0f, 0.0f, 0.7071f}
			},
			// Edge cases
			{
				"Quadraphonic configuration, source at 0 degrees",
				0.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{1.0f, 0.0f, 0.0f, 0.0f}
			},
			{
				"Quadraphonic configuration, source at 90 degrees",
				90.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.0f, 1.0f, 0.0f, 0.0f}
			},
			{
				"Quadraphonic configuration, source at 180 degrees",
				180.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.0f, 0.0f, 1.0f, 0.0f}
			},
			{
				"Quadraphonic configuration, source at 270 degrees",
				270.0f,
				{0.0f, 90.0f, 180.0f, 270.0f},
				{0.0f, 0.0f, 0.0f, 1.0f}
			},\
		};


		// Run the test cases
		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);
			TestProcessAngle(testCase.TestAngle, testCase.SpeakerAnglesDegrees, testCase.ExpectedGains);
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

		VBAPanner panner;

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);
			
			ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(static_cast<uint32>(testCase.ExpectedGains.size()))));

			// Prepare virtual sources
			std::vector<VirtualSource<>> virtualSources;
			for (float angleDeg : testCase.VirtualSourceAnglesDegrees)
				virtualSources.push_back({ toRad(angleDeg) });

			std::vector<float> outGains(panner.GetNumChannels(), 0.0f);
			panner.ProcessVirtualSources(virtualSources, outGains);

			ASSERT_EQ(outGains.size(), testCase.ExpectedGains.size());
			for (size_t i = 0; i < outGains.size(); ++i)
			{
				EXPECT_NEAR(outGains[i], testCase.ExpectedGains[i], 1e-5f) << "at index " << i;
			}
		}
	}


	TEST_F(VBAPTest, ApplySpreadAndFocus)
	{
		struct FocusAndSpreadTestCase
		{
			std::string Description;
			PanSource<> PanSource;
			PanParameters Parameters;
			float ExpectedResultDegrees;
		};

		const std::vector<FocusAndSpreadTestCase> testCases = {
		{
			.Description = "Spread is 0, result should be 0",
			.PanSource = {
				.SourceAngle = toRad(30.0f),
				.GroupAngle = toRad(-45.0f)
			},
			.Parameters = {
				.Focus = 0.5f,
				.Spread = 0.0f
			},
			.ExpectedResultDegrees = 0.0f
		},
		{
			.Description = "Focus is 1, result should be GroupAngle * Spread",
			.PanSource = {
				.SourceAngle = toRad(30.0f),
				.GroupAngle = toRad(-45.0f)
			},
			.Parameters = {
				.Focus = 1.0f,
				.Spread = 0.8f
			},
			.ExpectedResultDegrees = -36.0f
		},
		{
			.Description = "Focus is 0, result should be SourceAngle * Spread",
			.PanSource = {
				.SourceAngle = toRad(30.0f),
				.GroupAngle = toRad(-45.0f)
			},
			.Parameters = {
				.Focus = 0.0f,
				.Spread = 0.8f
			},
			.ExpectedResultDegrees = 24.0f
		},
		{
			.Description = "Spread is 1, result is interpolated angle",
			.PanSource = {
				.SourceAngle = toRad(30.0f),
				.GroupAngle = toRad(-45.0f)
			},
			.Parameters = {
				.Focus = 0.5f,
				.Spread = 1.0f
			},
			.ExpectedResultDegrees = -7.5f
		},
		{
			.Description = "General case",
			.PanSource = {
				.SourceAngle = toRad(60.0f),
				.GroupAngle = toRad(-60.0f)
			},
			.Parameters = {
				.Focus = 0.7f,
				.Spread = 0.5f
			},
			.ExpectedResultDegrees = -12.0f
		},
		{
			.Description = "Negative angles",
			.PanSource = {
				.SourceAngle = toRad(-90.0f),
				.GroupAngle = toRad(90.0f)
			},
			.Parameters = {
				.Focus = 0.25f,
				.Spread = 0.75f
			},
			.ExpectedResultDegrees = -33.75f
		},
		{
			.Description = "SourceAngle equals GroupAngle",
			.PanSource = {
				.SourceAngle = toRad(45.0f),
				.GroupAngle = toRad(45.0f)
			},
			.Parameters = {
				.Focus = 0.5f,
				.Spread = 0.9f
			},
			.ExpectedResultDegrees = 40.5f
		}
		};

		const float tolerance = 1e-5f; // Tolerance for floating-point comparison

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			// Call the function under test
			const float resultAngleRad = VBAPanner<>::ApplyFocusAndSpread(testCase.PanSource, testCase.Parameters);

			// Convert the result back to degrees
			float resultAngleDegrees = toDegrees(resultAngleRad);

			// Normalize the angle to the range [-180°, 180°]
			if (resultAngleDegrees > 180.0f)
				resultAngleDegrees -= 360.0f;
			else if (resultAngleDegrees < -180.0f)
				resultAngleDegrees += 360.0f;

			// Compare the result to the expected value
			EXPECT_NEAR(resultAngleDegrees, testCase.ExpectedResultDegrees, tolerance)
				<< "Result angle: " << resultAngleDegrees << " degrees, expected: " << testCase.ExpectedResultDegrees << " degrees";
		}
	}


	TEST_F(VBAPTest, ProcessVBAPData)
	{
		// TODO: At Focus 1, spread 1 (or 0.5?), channel mapping should be direct from source to output, or do we still want to spread evenly?

		struct CustomTraits : VBAPStandartTraits
		{
			using AngleType = double;
		};

		using CustomPannerType = typename VBAPanner<CustomTraits>;
		CustomPannerType panner;

		// Quadraphonic channel layout
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		struct ProcessVBAPDataTestCase
		{
			std::string Description;
			CustomTraits::AngleType PanAngleDegrees;
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

			CustomPannerType::VBAPData data;
			// 1 channel group with 2 virtual sources at positions { -90, 90 } in radians
			ASSERT_TRUE(data.Initialize(ChannelMap::FromNumChannels(1), 2));

			ASSERT_NEAR(data.ChannelGroups[0].VirtualSources[0].Angle, toRad(CustomTraits::AngleType(-90.0)), CustomTraits::AngleType(1e-6));
			ASSERT_NEAR(data.ChannelGroups[0].VirtualSources[1].Angle, toRad(CustomTraits::AngleType(90.0)), CustomTraits::AngleType(1e-6));

			CustomPannerType::PanUpdateData positionData
			{
				.PanAngle = toRad(testCase.PanAngleDegrees),
				.Focus = testCase.Focus,
				.Spread = testCase.Spread
			};

			CustomTraits::ChannelGains gains;

			panner.ProcessVBAPData(data, positionData, [&gains](uint32 channel) -> auto& { return gains; });

			ASSERT_TRUE(testCase.ExpectedGains.size() <= gains.size());
			for (size_t i = 0; i < testCase.ExpectedGains.size(); ++i)
			{
				EXPECT_NEAR(gains[i], testCase.ExpectedGains[i], 1e-4f) << "at index " << i;
			}
		}
	}
	
} // namespace JPL