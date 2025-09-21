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

#include "JPLSpatial/Algo/Algorithm.h"
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/MinimalQuat.h"

#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Panning/VBAPanning2D.h"
#include "JPLSpatial/Panning/VBAPanning3D.h"
#include "JPLSpatial/Panning/VBAPEx.h"
#include "JPLSpatial/ChannelMap.h"


#include "JPLSpatial/Math/Vec3Math.h"
#include "JPLSpatial/Math/MinimalBasis.h"
#include "JPLSpatial/Math/MinimalQuat.h"

#include "../Utility/TestUtils.h"

#include <gtest/gtest.h>
#include <numbers>
#include <algorithm>
#include <format>
#include <sstream>
#include <set>
#include <concepts>
#include <memory>
#include <ranges>

#include "../Utility/Vec3Types.h"

// The test takes quite a while,
// this flag is to skp it temporarily
#define JPL_ENABLE_LUT_VALIDATION_TEST 0

namespace JPL
{
	static constexpr auto toRad(std::floating_point auto degree) { return static_cast<decltype(degree)>(degree * (std::numbers::pi / 180.0)); };
	static constexpr auto toDegrees(std::floating_point auto rads) { return static_cast<decltype(rads)>(rads * (180.0 / std::numbers::pi)); };


	class VBAP3DTest : public testing::Test
	{
	protected:
		using Vec3i = SpeakerTriangulation::Vec3i;
		using Vec3 = MinimalVec3;
		static constexpr float epsilon = 1e-6f;

	protected:
		VBAP3DTest() = default;

		void SetUp() override
		{
		}

		void TearDown() override
		{
		}

	protected:
		struct NamedChannelLayout
		{
			std::string Name;
			ChannelMap Layout;
		};

		const std::vector<NamedChannelLayout> mAllChannelMapLayouts
		{
			{ "IVNALID", ChannelMap::FromChannelMask(ChannelMask::Invalid) },

			//========================================
			{ "Mono", ChannelMap::FromChannelMask(ChannelMask::Mono) },
			{ "Stereo", ChannelMap::FromChannelMask(ChannelMask::Stereo) },
			{ "LCR", ChannelMap::FromChannelMask(ChannelMask::LCR) },
			{ "LRS", ChannelMap::FromChannelMask(ChannelMask::LRS) },
			{ "LCRS", ChannelMap::FromChannelMask(ChannelMask::LCRS) },
			{ "Quad", ChannelMap::FromChannelMask(ChannelMask::Quad) },
			{ "Pentagonal", ChannelMap::FromChannelMask(ChannelMask::Pentagonal) },

			{ "Octagonal", ChannelMap::FromChannelMask(ChannelMask::Octagonal) },

			//========================================
			{ "Surround 4.1", ChannelMap::FromChannelMask(ChannelMask::Surround_4_1) },
			{ "Surround 5.0", ChannelMap::FromChannelMask(ChannelMask::Surround_5_0) },
			{ "Surround 5.1", ChannelMap::FromChannelMask(ChannelMask::Surround_5_1) },
			{ "Surround 6.0", ChannelMap::FromChannelMask(ChannelMask::Surround_6_0) },
			{ "Surround 6.1", ChannelMap::FromChannelMask(ChannelMask::Surround_6_1) },

			//========================================
			// DTS surround setups
			{ "Surround 7.0", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0) },
			{ "Surround 7.1", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1) },

			//========================================
			{ "Surround 5.0.2", ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_2) },
			{ "Surround 5.1.2", ChannelMap::FromChannelMask(ChannelMask::Surround_5_1_2) },
			{ "Surround 5.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_4) },
			{ "Surround 5.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_5_1_4) },

			//========================================
			// Dolby Atmos surround setups
			{ "Surround 7.0.2", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_2) },
			{ "Surround 7.1.2", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_2) },
			{ "Surround 7.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_4) },
			{ "Surround 7.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_4) },

			{ "Surround 7.0.6", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_6) },
			{ "Surround 7.1.6", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_6) },

			//========================================
			// Atmos surround setups
			{ "Surround 9.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_9_0_4) },
			{ "Surround 9.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_9_1_4) },
			{ "Surround 9.0.6", ChannelMap::FromChannelMask(ChannelMask::Surround_9_0_6) },
			{ "Surround 9.1.6", ChannelMap::FromChannelMask(ChannelMask::Surround_9_1_6) }
		};

		const std::vector<NamedChannelLayout> mValid3DLayouts
		{
			//========================================
			{ "Surround 5.0.2", ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_2) },
			{ "Surround 5.1.2", ChannelMap::FromChannelMask(ChannelMask::Surround_5_1_2) },
			{ "Surround 5.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_4) },
			{ "Surround 5.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_5_1_4) },

			//========================================
			// Dolby Atmos surround setups
			{ "Surround 7.0.2", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_2) },
			{ "Surround 7.1.2", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_2) },
			{ "Surround 7.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_4) },
			{ "Surround 7.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_4) },

			{ "Surround 7.0.6", ChannelMap::FromChannelMask(ChannelMask::Surround_7_0_6) },
			{ "Surround 7.1.6", ChannelMap::FromChannelMask(ChannelMask::Surround_7_1_6) },

			//========================================
			// Atmos surround setups
			{ "Surround 9.0.4", ChannelMap::FromChannelMask(ChannelMask::Surround_9_0_4) },
			{ "Surround 9.1.4", ChannelMap::FromChannelMask(ChannelMask::Surround_9_1_4) },
			{ "Surround 9.0.6", ChannelMap::FromChannelMask(ChannelMask::Surround_9_0_6) },
			{ "Surround 9.1.6", ChannelMap::FromChannelMask(ChannelMask::Surround_9_1_6) }
		};

		bool IsValid3DLayout(ChannelMap channelLayout) const
		{
			return std::ranges::find_if(mValid3DLayouts, [channelLayout](const NamedChannelLayout& chl)
			{
				return channelLayout == chl.Layout;
			}) != std::ranges::end(mValid3DLayouts);
		}
	};

#if JPL_USE_SIMPLE_SPEAKER_MESHING
	TEST_F(VBAP3DTest, TriangleCrossing)
	{
		const std::vector<Vec3> channelVectors
		{
			{ 0.0f, 0.0f, 1.0f },			// forward
			{ -1.0f, 0.0f, 0.0f  },			// left
			{ 1.0f, 0.0f, 0.0f },			// right
			{ 0.0f, 0.7071f, -0.7071f },	// forward up
			{ 0.0f, -0.7071f, 0.7071f },	// backward down
		};

		// forward-left-right
		// forward-forward_up-backward_down
		const Vec3i triA{ 0, 1, 2 };
		const Vec3i triB{ 0, 3, 4 };

		//? Not sure why these are considered "non crossing", but in the paper they are, need to clarify this
		const int crossing = SpeakerTriangulation::TestCrossing(triA, triB, channelVectors);
		EXPECT_EQ(crossing, 0);
	}

	//==========================================================================
	TEST_F(VBAP3DTest, TriangleCrossing_ExtraCases)
	{
		constexpr float s = 0.70710678f;          // √½

		const std::vector<Vec3> channelVectors = {
			/* 0 */ {  1.0f,  0.0f,  0.0f },   // Right
			/* 1 */ { -1.0f,  0.0f,  0.0f },   // Left
			/* 2 */ {  0.0f,  1.0f,  0.0f },   // Up
			/* 3 */ {  0.0f, -1.0f,  0.0f },   // Down
			/* 4 */ {  0.0f,  0.0f, -1.0f },   // Forward
			/* 5 */ {  0.0f,  0.0f,  1.0f },   // Backward
			/* 6 */ {     s,  0.0f, -s   },   // Front-Right
			/* 7 */ {    -s,  0.0f, -s   },   // Front-Left
			/* 8 */ {     s,  0.0f,  s   },   // Back-Right
			/* 9 */ {    -s,  0.0f,  s   },   // Back-Left
			/*10 */ {     s,     s,  0.0f }    // Right-Up
		};

		// 1. Baseline – no crossing (single shared vertex)
		{
			Vec3i triA{ 0, 1, 4 };
			Vec3i triB{ 0, 2, 5 };
			EXPECT_EQ(SpeakerTriangulation::TestCrossing(triA, triB, channelVectors), 0);
		}

		// 2. Genuine crossing
		{
			Vec3i triA{ 0, 2, 4 };
			Vec3i triB{ 1, 3, 6 };
			EXPECT_NE(SpeakerTriangulation::TestCrossing(triA, triB, channelVectors), 0);
		}

		// 3. False-positive trap: great-circle intersection is outside both minor arcs
		{
			Vec3i triA{ 1, 7, 4 };      // Left, Front-Left, Forward
			Vec3i triB{ 6,10, 3 };      // Front-Right, Right-Up, Down
			EXPECT_EQ(SpeakerTriangulation::TestCrossing(triA, triB, channelVectors), 0);
		}

		// 4. Shared edge – adjacent faces
		{
			Vec3i triA{ 0, 4, 2 };
			Vec3i triB{ 4, 0, 3 };
			EXPECT_EQ(SpeakerTriangulation::TestCrossing(triA, triB, channelVectors), 0);
		}
	}
#endif // JPL_USE_SIMPLE_SPEAKER_MESHING

	TEST_F(VBAP3DTest, TriangulateSpeakerLayout)
	{
		// Utility to generate constexpr speaker mesh data for the common speaker setups.
		// Only for development purposes.
		static auto printEmbedding = [](std::span<const Vec3> verts, std::span<const Vec3i> faces, std::string_view title)
		{
			// Printer can be swapped, e.g. in case we want to print for file
			Printer<PrintToCout> printer;

			auto printFloat = [](float value) -> std::string
			{
				return value == 0.0f ? "0.0000f" : std::format("{:.4f}f", value);
			};

			printer.PrintLine(std::format("static constexpr Data {} = {{", title));;
			printer.PrintLine("\tstd::to_array({");

			for (auto i = 0; i < verts.size(); ++i)
			{
				const auto& vert = verts[i];
				printer.PrintLine(std::format("\t\tVec3{{ {}, {}, {} {}"
											  , printFloat(vert.X)
											  , printFloat(vert.Y)
											  , printFloat(vert.Z)
											  , (i == verts.size() - 1 ? "}" : "},")));
			}

			printer.PrintLine("\t}),");

			printer.PrintLine("\tstd::to_array({");
			for (auto i = 0; i < faces.size(); ++i)
			{
				const auto& face = faces[i];
				printer.PrintLine(std::format("\t\tVec3i{{ {}, {}, {} {}"
											  , face[0]
											  , face[1]
											  , face[2]
											  , (i == faces.size() - 1 ? " }" : " },")));
			}

			printer.PrintLine("\t})");
			printer.PrintLine("};");
			printer.PrintLine("");
		};

		auto runSpekerMeshTest = [this](const NamedChannelLayout& test)
		{
			SCOPED_TRACE(test.Name);

			std::vector<Vec3> vertices;
			std::vector<Vec3i> triangles;

			const bool bSuccess = SpeakerTriangulation::TriangulateSpeakerLayout<&VBAPStandartTraits::GetChannelVector, Vec3>(test.Layout, vertices, triangles);

			if (IsValid3DLayout(test.Layout))
			{
				EXPECT_TRUE(bSuccess);
			}
			else
			{
				EXPECT_FALSE(bSuccess);
				return;
			}

#if 0		// Debugging
			if (bSuccess)
			{
				SaveOBJ(vertices, triangles, std::format("{}.obj", test.Name));
				printEmbedding(vertices, triangles, test.Name);
			}
#endif
		};

		for (const NamedChannelLayout& test : mAllChannelMapLayouts)
			runSpekerMeshTest(test);
	}

#if JPL_ENABLE_LUT_VALIDATION_TEST
	TEST_F(VBAP3DTest, VBAP3D_BuildsValidLUT)
	{
		auto testLUTFor = [this](ChannelMap targetLayout)
		{
			using PannerType = VBAPanner3D<VBAPStandartTraitsBaseBAP::ELUTSize::KB_983>>;

			PannerType panner;

			if (!IsValid3DLayout(targetLayout))
			{
				EXPECT_FALSE(panner.InitializeLUT(targetLayout));
				return;
			}

			ASSERT_TRUE(panner.InitializeLUT(targetLayout));

			const auto* LUT = panner.GetLUT();
			ASSERT_TRUE(LUT != nullptr);

			size_t invalidLUTCells = 0;

			auto isValidIndex = [](uint32 i)
			{
				PannerType::LUTCodec::EncodedType dx = i & VBAPanner3D<>::LUTCodec::cAxisMask;
				PannerType::LUTCodec::EncodedType dy = (i >> VBAPanner3D<>::LUTCodec::cBitsPerAxis) & VBAPanner3D<>::LUTCodec::cAxisMask;

				return dx != VBAPanner3D<>::LUTCodec::cAxisMask && dy != VBAPanner3D<>::LUTCodec::cAxisMask;
			};

			for (uint32 i = 0; i < LUT->Speakers.size(); ++i)
			{
				if (!isValidIndex(i))
					continue; // skip padded cells that has no direction

				const auto& speakers = LUT->Speakers[i];

				const bool bDuplicateSpeakers =
					speakers[0] == speakers[1] ||
					speakers[1] == speakers[2] ||
					speakers[2] == speakers[0];

				const Vec3 direction = VBAPanner3D<>::LUTCodec::Decode<Vec3>(i);

				EXPECT_FALSE(bDuplicateSpeakers)
					<< std::format("Found duplicate speakers {{{}, {}, {}}} in triplet for LUT index [{}], direction: {}, {}, {}",
								   speakers[0], speakers[1], speakers[2],
								   i,
								   direction.X, direction.Y, direction.Z);

				invalidLUTCells += bDuplicateSpeakers;
			}

			EXPECT_TRUE(invalidLUTCells == 0) << std::format("LUT contains {}/{} invalid speaker triplets", invalidLUTCells, LUT->Speakers.size());

			invalidLUTCells = 0;

			for (uint32 i = 0; i < LUT->Gains.size(); ++i)
			{
				if (!isValidIndex(i))
					continue; // skip padded cells that has no direction

				const auto& gains = LUT->Gains[i];

				const Vec3 direction = VBAPanner3D<>::LUTCodec::Decode<Vec3>(i);

				const bool bGainNearlyZero = Math::IsNearlyZero(gains[0] + gains[1] + gains[2]);
				const bool bHasNegativeGains = gains[0] < 0.0f || gains[1] < 0.0f || gains[2] < 0.0f;

				EXPECT_FALSE(bGainNearlyZero || bHasNegativeGains)
					<< std::format("Invalid gains {{{}, {}, {}}} for speaker triplet for LUT index [{}], direction: {}, {}, {}",
								   gains[0], gains[1], gains[2],
								   i,
								   direction.X, direction.Y, direction.Z);

				invalidLUTCells += (bGainNearlyZero || bHasNegativeGains);
			}

			EXPECT_TRUE(invalidLUTCells == 0) << std::format("LUT contains {}/{} invalid speaker gains", invalidLUTCells, LUT->Gains.size());
		};


		for (const NamedChannelLayout& test : mAllChannelMapLayouts)
		{
			SCOPED_TRACE(test.Name);
			testLUTFor(test.Layout);
		}
	}
#endif

#if 0
	TEST_F(VBAP3DTest, VBAPVisualization)
	{
		using ChannelPoints = std::vector<GroupedPoint<Vec3>>;

		static auto assignGroup = [](std::span<const Vec3> vectors, int group)
		{
			ChannelPoints points;
			points.reserve(vectors.size());
			for (const Vec3& v : vectors)
				points.emplace_back(v, group);
			return points;
		};

		std::vector<IntencityPoint<Vec3>> channelVis;

		auto generateLayout = [&channelVis]<class Params>(
			ChannelMap targetLayout,
			ChannelMap sourceLayout,
			const Params& params) -> ChannelPoints
		{
			VBAPanner3D<> panner;
			if (!panner.InitializeLUT(targetLayout))
				return{};

			ChannelPoints points;
			auto onChannelGenerated = [&points](std::span<const Vec3> channelVSs, uint32 channelId) mutable
			{
				auto p = assignGroup(channelVSs, channelId);
				points.insert(points.end(), p.begin(), p.end());
			};

			const int numChannels = sourceLayout.GetNumChannels() - sourceLayout.HasLFE();
			std::vector<typename VBAPStandartTraits::ChannelGains> sourceChannelGains(numChannels);
			for (auto& chg : sourceChannelGains)
				chg.fill(0.0f);

			auto getSourcChannelGroupGains = [&sourceChannelGains](uint32 sourceChannelIndex) -> typename VBAPanner3D<>::ChannelGainsRef
			{
				return sourceChannelGains[sourceChannelIndex];
			};

			VBAPanner3D<>::SourceLayoutType data;
			panner.InitializeSourceLayout(sourceLayout, data);

			panner.ProcessVBAPData(data, params, getSourcChannelGroupGains, onChannelGenerated);

			// Create speaker energy visualization
			channelVis.clear();
			targetLayout.ForEachChannel([&channelVis](EChannel channel/*, uint32 index*/)
			{
				if (channel != EChannel::LFE)
					channelVis.emplace_back(VBAPStandartTraits::GetChannelVector(channel), 0.0f);
			});

			for (const auto& sourceGains : sourceChannelGains)
			{
				for (uint32 ch = 0; ch < channelVis.size(); ++ch)
					channelVis[ch].Intensity += sourceGains[ch];
			}

			return points;
		};

		auto generateSingleDirectionVis = [&channelVis](ChannelMap targetLayout, const Vec3& direction) -> ChannelPoints
		{
			VBAPanner3D<> panner;
			if (!panner.InitializeLUT(targetLayout))
				return{};

			VBAPanner3D<>::SourceLayoutType data;
			panner.InitializeSourceLayout(ChannelMap::FromChannelMask(ChannelMask::Mono), data);

			typename VBAPStandartTraits::ChannelGains sourceGains;
			sourceGains.fill(0.0f);

			//! test specific direction
			panner.GetSpeakerGains(direction, sourceGains);

			// Normalize
			//? should already be normalized
			//VBAPanner3D<>::NormalzieAccumulatedGains(sourceGains);

			ChannelPoints points;
			points.push_back(assignGroup(std::span<const Vec3>{ &direction, 1 }, 0)[0]);

			// Create speaker energy visualization
			channelVis.clear();
			targetLayout.ForEachChannel([&channelVis](EChannel channel/*, uint32 index*/)
			{
				if (channel != EChannel::LFE)
					channelVis.emplace_back(VBAPStandartTraits::GetChannelVector(channel), 0.0f);
			});

			for (uint32 ch = 0; ch < channelVis.size(); ++ch)
				channelVis[ch].Intensity += sourceGains[ch];

			return points;
		};

		const auto layout54 = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_4);
		const auto layoutSurround = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0);
		const auto layoutQuad = ChannelMap::FromChannelMask(ChannelMask::Quad);
		const auto layoutMono = ChannelMap::FromChannelMask(ChannelMask::Mono);
		const auto layoutStereo = ChannelMap::FromChannelMask(ChannelMask::Stereo);
		const auto layoutLCR = ChannelMap::FromChannelMask(ChannelMask::LCR);


#if 0
		const typename VBAPanner3D<>::PanUpdateDataWithOrientation params
		{
			.Pan = {
				.SourceDirection = Vec3(0.0f, 0.0f, -1.0f).Normalize(),
				.Focus = 0.0f,
				.Spread = 0.5f,
			},
			.Orientation = {
				.Up = sourceRotation.Y,//Vec3(0.0f, 1.0f, 0.0f),
				.Forward = sourceRotation.Z //Vec3(1.0f, 0.0f, 0.0f) // emitter pointing right
			}
		};
#endif
		const typename VBAPanner3D<>::PanUpdateData params
		{
			.SourceDirection = Vec3(1.0f, 1.0f, -1.0f).Normalize(), // checking there's no roll
			.Focus = 0.0f,
			.Spread = 0.5f,
		};

		ChannelPoints points = generateLayout(layout54, layoutStereo, params);
		//ChannelPoints points = generateSingleDirectionVis(layout54, Vec3(-0.5f, -1.0f, 0.3f).Normalize());

#if 0	//! override visualization to debug 2, 5, 6 triplet

		const Vec3 gains = ComputeVBAP(Vec3(0.0f, 0.1f, -1.0f).Normalize(),
									   SpeakerMap::Surround_5_0_2.Vertices[2],
									   SpeakerMap::Surround_5_0_2.Vertices[5],
									   SpeakerMap::Surround_5_0_2.Vertices[6]).Normalize();
		for (auto& sv : channelVis)
			sv.Intensity = 0.0f;
		channelVis[2].Intensity = gains.X;
		channelVis[5].Intensity = gains.Y;
		channelVis[6].Intensity = gains.Z;
#endif

		/*std::cout << "Per channel gains: " << '\n';
		for (uint32 ch = 0; ch < channelVis.size(); ++ch)
			std::cout << std::format("Channel [{}]: {}", ch, channelVis[ch].Intensity) << '\n';*/

			// for the visualization with plot_vbap.py
		SaveToJSON(points, channelVis, "visdata.json");
	}
#endif

	TEST_F(VBAP3DTest, PanningNoRoll)
	{
		// Testing channel rotation quat/basis math

		// canonical centre of channel ring
		static constexpr Vec3 FWD = { 0, 0, -1 }; // engine forwardm change to +Z if needed
		static constexpr Vec3 UP = { 0, 1, 0 };
		static constexpr float yawRad = std::numbers::pi_v<float> * 0.5f;

		const std::vector<Vec3> testPanDiections
		{
			Vec3(1.0f, 0.0f, 0.0f),
			Vec3(-1.0f, 0.0f, 0.0f),
			Vec3(0.0f, 1.0f, 0.0f),
			Vec3(0.0f, -1.0f, 0.0f),
			Vec3(0.0f, 0.0f, 1.0f),
			Vec3(0.0f, 0.0f, -1.0f),

			Vec3(0.0f, 1.0f, 1.0f),
			Vec3(0.0f, 1.0f, -1.0f),
			Vec3(0.0f, -1.0f, 1.0f),
			Vec3(0.0f, -1.0f, -1.0f),

			Vec3(1.0f, 1.0f, 0.0f),
			Vec3(1.0f, -1.0f, 0.0f),
			Vec3(-1.0f, 1.0f, 0.0f),
			Vec3(-1.0f, -1.0f, 0.0f),

			Vec3(1.0f,  0.0f, 1.0f),
			Vec3(1.0f,  0.0f, -1.0f),
			Vec3(-1.0f, 0.0f, 1.0f),
			Vec3(-1.0f, 0.0f, -1.0f),

			Vec3(1.0f, 1.0f, 1.0f),
			Vec3(1.0f, 1.0f, -1.0f),
			Vec3(1.0f, -1.0f, 1.0f),
			Vec3(1.0f, -1.0f, -1.0f),

			Vec3(-1.0f, 1.0f, 1.0f),
			Vec3(-1.0f, 1.0f, -1.0f),
			Vec3(-1.0f, -1.0f, 1.0f),
			Vec3(-1.0f, -1.0f, -1.0f),
		};

		// Steereo source channels spread left and rigth
		auto qYawL = Quat<Vec3>::Rotation(UP, -yawRad); // left (world-Y)
		auto qYawR = Quat<Vec3>::Rotation(UP, +yawRad); // right

		static constexpr Vec3 forward(0.0f, 0.0f, -1.0f);

		for (const Vec3& panDir : testPanDiections)
		{
			// Normalizing here instead of on construction to be able
			// to print more readable `panDir` in case of test fail

			const auto qPan = Math::QuatLookAt(Normalized(panDir), UP);

			const Quat<Vec3> qL = (qPan * qYawL);
			const Quat<Vec3> qR = (qPan * qYawR);
			const Vec3 L = qL.Rotate(FWD);
			const Vec3 R = qR.Rotate(FWD);

			// L.Y and R.Y must be the roughly equal if things working correctly
			EXPECT_NEAR(L.Y, R.Y, 1e-6f)
				<< std::format("Pan direction: {}, {}, {}", panDir.X, panDir.Y, panDir.Z);
		}
	}

	TEST_F(VBAP3DTest, VBAPPannedGainsL2Normalzied)
	{
		VBAPanner3D<> panner;

		using ParametersType = typename VBAPanner3D<>::PanUpdateData;

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

		for (const NamedChannelLayout& test : mValid3DLayouts)
		{
			SCOPED_TRACE(std::format("Target Speaker Layout: {}", test.Name));

			if (!test.Layout.IsValid() || test.Layout.GetNumChannels() < 2)
				continue;

			// TODO: for now we skip large target channel count layouts,
			// because they have tiny min aperture, which requires > max samples of virtual sources
			// to cover the space without holes.
			//! this may be fixed when we skip center channel. Maybe "wides" as well?
			// We could also ditrch the RountUpBy4 for number of rings per source channel,
			// and keep at least 2
			if (test.Layout.GetNumChannels() >= 13)
				continue;

			ASSERT_TRUE(panner.InitializeLUT(test.Layout));

			auto testResultingGainSum = [&panner](
				const ChannelMap& sourceChannels,
				const ParametersType& params)
			{
				typename VBAPanner3D<>::SourceLayoutType data;
				ASSERT_TRUE(panner.InitializeSourceLayout(sourceChannels, data));

				const uint32 numTargetChannels = panner.GetNumChannels();
				const uint32 numChannels = sourceChannels.GetNumChannels();

				std::vector<typename VBAPStandartTraits::ChannelGains> sourceChannelGains(numChannels);
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

				/*if (Math::IsNearlyEqual(params.SourceDirection, Vec3(0, 0, 1)))
				{
					speakerVis.SetLayout(panner.GetChannelMap());
					speakerVis.AddChannelGains(sourceChannelGains);

					SaveToJSON(sourceVis.Points, speakerVis.Points, "visdata.json");
				}*/
			};

			forEachTestParam(testResultingGainSum);
		}
	}

	TEST_F(VBAP3DTest, ProcessVirtualSources)
	{
		static constexpr auto targetChannelMap = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_2);
		static constexpr auto targetChannelCount = targetChannelMap.GetNumChannels();

		struct VirtualSourcesTestCase
		{
			std::string Description;
			std::vector<float> VirtualSourceAnglesDegrees;
			std::array<float, targetChannelCount> ExpectedGains;
		};

		// Testing for Quadraphonic output channel map
		const std::vector<VirtualSourcesTestCase> testCases =
		{
			{
				"Single virtual source at 0 degrees",
				{ 0.0f },
				{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f }
			},
			{
				"Multiple virtual sources at 0 degrees",
				{ 0.0f, 0.0f, 0.0f },
				{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f }
			},
			{
				"Virtual sources at speaker angles",
				{ -45.0f, 45.0f, -135.0f, 135.0f },
				{ 0.5f, 0.5f, 0.0f, 0.5f, 0.5f, 0.0f, 0.0f }
			}
		};

		VBAPanner3D<> panner;

		static constexpr auto tolerance = static_cast<float>(VBAPanner3D<>::PanType::LUTCodec::cMaxComponentError);

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			ASSERT_TRUE(panner.InitializeLUT(targetChannelMap));

			// Prepare virtual sources
			std::vector<typename VBAPanner3D<>::VirtualSource> virtualSources;
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
				EXPECT_NEAR(outGains[i], testCase.ExpectedGains[i], tolerance) << "at index " << i;
			}
		}
	}

	TEST_F(VBAP3DTest, ProcessVBAPData)
	{
		static constexpr auto targetChannelMap = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_2);
		static constexpr auto targetChannelCount = targetChannelMap.GetNumChannels();

		// TODO: At Focus 1, spread 1 (or 0.5?), channel mapping should be direct from source to output, or do we still want to spread evenly?

		using PannerType = typename VBAPanner3D<>;
		PannerType panner;

		// Quadraphonic channel layout
		ASSERT_TRUE(panner.InitializeLUT(targetChannelMap));

		struct ProcessVBAPDataTestCase
		{
			std::string Description;
			float PanAngleDegrees;
			float Spread;
			float Focus;
			std::array<float, targetChannelCount> ExpectedGains; // Expected gains for the ChannelGroup's gains
		};

		const std::vector<ProcessVBAPDataTestCase> testCases = {
		{
			.Description = "PanAngle 0 degrees",
			.PanAngleDegrees = 0.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f }
		},
		{
			.Description = "PanAngle 90 degrees",
			.PanAngleDegrees = 90.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 0.7071f, 0.0f, 0.0f, 0.7071f, 0.0f, 0.0f }
		},
		{
			.Description = "PanAngle -90 degrees",
			.PanAngleDegrees = -90.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.7071f, 0.0f, 0.0f, 0.7071f, 0.0f, 0.0f, 0.0f }
		},
		{
			.Description = "PanAngle 180 degrees",
			.PanAngleDegrees = 180.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 0.0f, 0.0f, 0.7071f, 0.7071f, 0.0f, 0.0f }
		},
		{
			.Description = "PanAngle 45 degrees",
			.PanAngleDegrees = 45.0f,
			.Spread = 1.0f,
			.Focus = 1.0f,
			.ExpectedGains = { 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f }
		}
			// Add more tests if needed...
		};

		static constexpr auto tolerance = static_cast<float>(VBAPanner3D<>::PanType::LUTCodec::cMaxComponentError);

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
			gains.fill(0.0f);

			panner.ProcessVBAPData(data, positionData, [&gains](uint32 channel) -> auto& { return gains; });

			ASSERT_TRUE(testCase.ExpectedGains.size() <= gains.size());
			for (size_t i = 0; i < testCase.ExpectedGains.size(); ++i)
			{
				EXPECT_NEAR(gains[i], testCase.ExpectedGains[i], tolerance) << "at index " << i;
			}
		}
	}


	TEST_F(VBAP3DTest, WorksWithDifferentVec3Types)
	{
		static constexpr auto sourceChannelMap = ChannelMap::FromChannelMask(ChannelMask::Quad);
		static constexpr auto targetChannelMap = ChannelMap::FromChannelMask(ChannelMask::Surround_5_0_2);
		static constexpr auto targetChannelCount = targetChannelMap.GetNumChannels();

		struct VirtualSourcesTestCase
		{
			std::string Description;
			std::vector<float> VirtualSourceAnglesDegrees;
			std::array<float, targetChannelCount> ExpectedGains;
		};

		// Testing for Quadraphonic output channel map
		const std::vector<VirtualSourcesTestCase> testCases =
		{
			{
				"Single virtual source at 0 degrees",
				{ 0.0f },
				{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f }
			},
			{
				"Multiple virtual sources at 0 degrees",
				{ 0.0f, 0.0f, 0.0f },
				{ 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f }
			},
			{
				"Virtual sources at speaker angles",
				{ -45.0f, 45.0f, -135.0f, 135.0f },
				{ 0.5f, 0.5f, 0.0f, 0.5f, 0.5f, 0.0f, 0.0f }
			}
		};

		auto testVec3Type = [&]<class Vec3Type>()
		{
			using PannerType = VBAPanner3D<VBAPBaseTraits<Vec3Type>>;
			PannerType panner;

			static constexpr auto tolerance = static_cast<float>(VBAPanner3D<>::PanType::LUTCodec::cMaxComponentError);

			for (const auto& testCase : testCases)
			{
				SCOPED_TRACE(testCase.Description);

				ASSERT_TRUE(panner.InitializeLUT(targetChannelMap));

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
					EXPECT_NEAR(outGains[i], testCase.ExpectedGains[i], tolerance) << "at index " << i;
				}
			}
		};

		testVec3Type.operator()<JPH::Vec3>();
		testVec3Type.operator()<glm::vec3>();
		testVec3Type.operator()<MinimalVec3>();
	}
} // namespace JPL