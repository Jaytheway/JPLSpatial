//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/MinimalQuat.h"

#include "JPLSpatial/Panning/VBAPanning2D.h"
#include "JPLSpatial/Panning/VBAPanning3D.h"
#include "JPLSpatial/Panning/VBAPEx.h"
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/Algo/Algorithm.h"


#include "../Utility/TestUtils.h"
#include "../Utility/Vec3D.h"

#include <gtest/gtest.h>
#include <array>
#include <algorithm>
#include <concepts>
#include <format>
#include <numbers>
#include <set>
#include <memory>
#include <ranges>
#include <vector>
#include <string_view>

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
		const std::vector<NamedChannelMask> mChannelMasks
		{
			{ ChannelMask::Invalid },
			{ ChannelMask::Mono },
			{ ChannelMask::Stereo },
			{ ChannelMask::Quad },
			{ ChannelMask::Surround_4_1 },
			{ ChannelMask::Surround_5_1 },
			{ ChannelMask::Surround_6_1 },
			{ ChannelMask::Surround_7_1 },
		};

		struct NamedChannelLayout
		{
			std::string_view Name;
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
			void AddChannelGains(std::span<const float> sourceGains)
			{
				for (uint32 ch = 0; ch < Points.size(); ++ch)
					Points[ch].Intensity += sourceGains[ch];
			}

			//// Accumulate multiple source channel gains per output channel
			//void AddChannelGains(std::span<const float> sourceChannelGains)
			//{
			//	for (const ChannelGains& sourceGains : sourceChannelGains)
			//		AddChannelGains(sourceGains);
			//}

			void Reset()
			{
				Points.clear();
			}

			std::vector<IntencityPoint<Vec3>> Points;
		};
	};

	TEST_F(VBAPTest, InitializeLUT_PrecomputedGains)
	{
		// Initialize VBAP panner
		VBAPanner2D<> panner;

		struct VBAPTestCase
		{
			std::string_view description;
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

		// TODO: reconsider the testing methodology. Test LUT quantization and variance first

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

			std::vector<float> lutGains(panner.GetNumChannels(), 0.0f);

			for (const auto& angleTest : testCase.angleTests)
			{
				const float testAngleDeg = angleTest.testAngleDegrees;
				const float testAngleRad = toRad(testAngleDeg);
#if 0
				const int pos = panner.GetLUT()->AngleToLUTPosition(testAngleRad);

				// Retrieve gains from LUT
				const uint32_t offset = panner.GetNumChannels() * pos;
				std::vector<float> lutGains(panner.GetNumChannels());
				for (uint32 ch = 0; ch < panner.GetNumChannels(); ++ch)
				{
					lutGains[ch] = panner.GetLUT()->GetLUTValue(offset + ch);
				}
#else
				const Vec2 direction(sinf(testAngleRad), -cosf(testAngleRad));
				panner.GetLUT()->GetSpeakerGains(direction, lutGains);
#endif

				const std::vector<float>& expectedGains = angleTest.expectedGains;
#if 0
				SCOPED_TRACE(std::format("Angle {} degrees, Position {}", testAngleDeg, pos));
#else
				SCOPED_TRACE(std::format("Angle {} degrees", testAngleDeg));
#endif
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

		for (const NamedChannelMask& test : mChannelMasks)
		{
			SCOPED_TRACE(std::format("Channel Map: {}", test.Name));

			const ChannelMap channelMap = test.Layout;
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
			const NamedChannelMask ChannelMap;

			struct ChannelGroupExpectedData
			{
				float Angle;
			};

			const std::vector<ChannelGroupExpectedData> ExpectedChannelGroups;
		};

		// First virtual source of a channel is positioned according to the following formula:
		// ChannelAngle - HalfChannelWidth + HalfVirtualSourceWidth
		// (channel groups and virtual sources are sorted by index the channel appears in the channel set/map)
		const std::vector<VBAPDataTest> VBAPDataTestCases
		{
			{
				.ChannelMap = { ChannelMask::Invalid },
				.ExpectedChannelGroups = {}
			},
			{
				.ChannelMap = { ChannelMask::Mono },
				.ExpectedChannelGroups = {
					{
						.Angle = 0.0f,
					}
				}
			},
			{
				.ChannelMap = { ChannelMask::Stereo },
				.ExpectedChannelGroups = {
					{
						.Angle = -90.0f,
					},
					{
						.Angle = 90.0f,
					}
				}
			},
			{
				// 90 degree per channel
				// 45 degrees per virtual source
				// -22.5 degrees virtual sources offset
				.ChannelMap = { ChannelMask::Quad },
				.ExpectedChannelGroups = {
					{
						.Angle = -45.0f,
					},
					{
						.Angle = 45.0f,
					},
					{
						.Angle = -135.0f,
					},
					{
						.Angle = 135.0f,
					},
				}
			},
			{
				// 72 degree per channel
				// 36 degrees per virtual source
				// -18 degrees virtual sources offset
				.ChannelMap = { ChannelMask::Surround_5_1 },
				.ExpectedChannelGroups = {
					{
						.Angle = -72.0f,
					},
					{
						.Angle = 72.0f,
					},
					{
						.Angle = 0.0f,
					},
					{
						.Angle = -144.0f,
					},
					{
						.Angle = 144.0f,
					}
				}
			}
		};

		VBAPanner2D<> panner;
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		typename VBAPanner2D<>::SourceLayoutType data;

		// Just test the fact that SourceLayout internally initializes minimum of 2 virtual sources per channel,
		// the rest of data we can test fot the actual 2 virtual sources test case
		{
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromChannelMask(ChannelMask::Stereo), data));
			EXPECT_GE(data.GetNumVirtualSources(), 4);
		}

		for (const auto& test : VBAPDataTestCases)
		{
			const ChannelMap channelMap = test.ChannelMap.Layout;
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
			std::string_view Description;
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

		// This vector component double and channel gains as span
		// is more-or-less how JPLSpatial UE integration is setup
		struct TraitsOverride : VBAPBaseTraits<Vec3D>
		{
			using ChannelGains = std::span<float>;
		};

		using PannerType = VBAPanner2D<TraitsOverride>;
		PannerType panner;

		// Quadraphonic channel layout
		ASSERT_TRUE(panner.InitializeLUT(ChannelMap::FromNumChannels(4)));

		struct ProcessVBAPDataTestCase
		{
			std::string_view Description;
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

		static constexpr float gainToleranse = 1e-4f;

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			typename PannerType::SourceLayoutType data;
			// 1 channel group with 2 virtual sources at positions { -90, 90 } in radians
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromNumChannels(1), data));

			const double panRad = static_cast<double>(toRad(testCase.PanAngleDegrees));
			typename PannerType::PanUpdateData positionData
			{
				.SourceDirection = Vec3D{ std::sin(panRad), 0.0, -std::cos(panRad) },
				.Focus = testCase.Focus,
				.Spread = testCase.Spread
			};

			std::array<float, TraitsOverride::MAX_CHANNEL_MIX_MAP_SIZE> gainsData;
			std::span<float> gains(gainsData.data(), 1 * panner.GetNumChannels());
			std::ranges::fill(gains, 0.0f);

			panner.ProcessVBAPData(data, positionData, gains);

			ASSERT_TRUE(testCase.ExpectedGains.size() <= gains.size());
			for (size_t i = 0; i < testCase.ExpectedGains.size(); ++i)
			{
				EXPECT_NEAR(gains[i], testCase.ExpectedGains[i], gainToleranse) << "at index " << i;
			}
		}

		{
			SCOPED_TRACE("2 source channels");
			static constexpr uint32 numSourceChannels = 2;

			typename PannerType::SourceLayoutType data;
			// 1 channel group with 2 virtual sources at positions { -90, 90 } in radians
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromNumChannels(numSourceChannels), data));

			const double panRad = static_cast<double>(toRad(0.0f));
			typename PannerType::PanUpdateData positionData
			{
				.SourceDirection = Vec3D{ std::sin(panRad), 0.0, -std::cos(panRad) },
				.Focus = 1.0f,
				.Spread = 1.0f
			};

			std::array<float, TraitsOverride::MAX_CHANNEL_MIX_MAP_SIZE> gainsData;
			std::span<float> gains(gainsData.data(), numSourceChannels * panner.GetNumChannels());
			std::ranges::fill(gains, 0.0f);

			const uint32 numOutputChannels = panner.GetNumChannels();

			panner.ProcessVBAPData(data, positionData, gains);

			// Expect left channel to output only to FL and BL, and the same amount
			EXPECT_GT(  gainsData[0], 1e-4f);
			EXPECT_NEAR(gainsData[0], gainsData[2], gainToleranse);
			EXPECT_NEAR(gainsData[1], 0.0f, 1e-4f);
			EXPECT_NEAR(gainsData[3], 0.0f, 1e-4f);

			// Expect right channel to output only to FR and BR, and the same amount
			EXPECT_GT(  gainsData[numOutputChannels + 1], 1e-4f);
			EXPECT_NEAR(gainsData[numOutputChannels + 1], gainsData[numOutputChannels + 3], gainToleranse);
			EXPECT_NEAR(gainsData[numOutputChannels + 0], 0.0f, 1e-4f);
			EXPECT_NEAR(gainsData[numOutputChannels + 2], 0.0f, 1e-4f);
		}

		{
			PannerType stereoPanner;
			stereoPanner.InitializeLUT(ChannelMap::FromNumChannels(2));

			SCOPED_TRACE("Mono source in front, Stereo target, equal gains");
			static constexpr uint32 numSourceChannels = 1;

			typename PannerType::SourceLayoutType data;
			// 1 channel group with 2 virtual sources at positions { -90, 90 } in radians
			ASSERT_TRUE(stereoPanner.InitializeSourceLayout(ChannelMap::FromNumChannels(numSourceChannels), data));

			typename PannerType::PanUpdateData positionData
			{
				.SourceDirection = Vec3D{ 0.0, 0.0, -1.0 },
				.Focus = 0.0f,
				.Spread = 0.5f
			};

			StaticArray<float, TraitsOverride::MAX_CHANNEL_MIX_MAP_SIZE> gains(numSourceChannels * stereoPanner.GetNumChannels(), 0.0f);

			const uint32 numOutputChannels = stereoPanner.GetNumChannels();

			stereoPanner.ProcessVBAPData(data, positionData, gains);

			// Expect left and right channel to have equal gains
			EXPECT_NEAR(gains[0], gains[1], gainToleranse);
		}

		//! We don't really care where the LUT puts the direction straight UP,
		//! it's user's/integrator's choice where to shift the direction edge-cases 
#if 0
		{
			SCOPED_TRACE("Source above the listener");
			static constexpr uint32 numSourceChannels = 1;

			typename PannerType::SourceLayoutType data;
			ASSERT_TRUE(panner.InitializeSourceLayout(ChannelMap::FromNumChannels(numSourceChannels), data));

			typename PannerType::PanUpdateData positionData
			{
				.SourceDirection = Vec3D(0.0f, 1.0f, 0.0f),
				.Focus = 0.0f,
				.Spread = 0.0f
			};

			std::array<float, TraitsOverride::MAX_CHANNEL_MIX_MAP_SIZE> gainsData;
			std::span<float> gains(gainsData.data(), numSourceChannels* panner.GetNumChannels());
			std::ranges::fill(gains, 0.0f);

			const uint32 numOutputChannels = panner.GetNumChannels();

			panner.ProcessVBAPData(data, positionData, gains);

			// Expecting forward as a fallback
			const std::array<float, 4> expectedGains{ 0.7071f, 0.7071f, 0.0f, 0.0f };

			// Expect left channel to output only to FL and BL, and the same amount
			EXPECT_NEAR(gainsData[0], expectedGains[0], gainToleranse);
			EXPECT_NEAR(gainsData[1], expectedGains[1], gainToleranse);
			EXPECT_NEAR(gainsData[3], expectedGains[2], gainToleranse);
			EXPECT_NEAR(gainsData[3], expectedGains[3], gainToleranse);
		}
#endif
	}

	TEST_F(VBAPTest, VBAPPannedGainsL2Normalized)
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
				const uint32 numChannels = sourceChannels.GetNumChannels() - sourceChannels.HasLFE();

				std::array<float, VBAPStandartTraits::MAX_CHANNEL_MIX_MAP_SIZE> gainsData;
				std::span<float> gains(gainsData.data(), numChannels * panner.GetNumChannels());

				panner.ProcessVBAPData(data, params, gains);

				SCOPED_TRACE(std::format("Source number of channels: {} | Parameters: {}",
										 numChannels, paramsToString(params)));

				typename VBAPStandartTraits::ChannelGains accumulatedGains;
				accumulatedGains.fill(0.0f);

				// Accumulate per output channel gains
				for (uint32 i = 0; i < numChannels * panner.GetNumChannels(); i += panner.GetNumChannels())
				{
					std::span<const float> channelGains = gains.subspan(i, panner.GetNumChannels());
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

	TEST_F(VBAPTest, WorksWithDifferentVec3Types)
	{
		const auto sourceChannelMap = ChannelMap::FromChannelMask(ChannelMask::Quad);
		const auto targetChannelMap = ChannelMap::FromChannelMask(ChannelMask::Quad);

		struct VirtualSourcesTestCase
		{
			std::string_view Description;
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

	TEST_F(VBAPTest, SourceLayoutsLeaveNoGapsInTargetSpeakerLayouts)
	{
		static constexpr float tolerance = 1e-4f;

		for (const auto& [targetLayoutName, targetLayoutMap] : mChannelMasks)
		{
			if (not targetLayoutMap.IsValid() || targetLayoutMap.GetNumChannels() < 2)
				continue;

			VBAPanner2D<> panner;
			ASSERT_TRUE(panner.InitializeLUT(targetLayoutMap)) << targetLayoutName;

			for (const auto& [sourceLayoutName, sourceLayoutMap] : mChannelMasks)
			{
				if (not sourceLayoutMap.IsValid())
					continue;

				typename VBAPanner2D<>::SourceLayoutType sourceLayout;
				ASSERT_TRUE(panner.InitializeSourceLayout(sourceLayoutMap, sourceLayout)) << sourceLayoutName;

				const float minDistanceBetweenSamples = sourceLayout.GetMinDistanceBetweenSamples();
				const float shortestSpeakerAperture = panner.GetShortestSpeakerAperture();

				EXPECT_LT(minDistanceBetweenSamples - tolerance, shortestSpeakerAperture)
					<< "Source: " << sourceLayoutName << " " << Math::ToDegrees(minDistanceBetweenSamples) << " Num VSs: " << sourceLayout.GetNumVirtualSources()
					<< " -> Target: " << targetLayoutName << " " << Math::ToDegrees(shortestSpeakerAperture);
			}
		}
	}
} // namespace JPL