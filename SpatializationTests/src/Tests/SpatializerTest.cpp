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

#include "JPLSpatial/Core.h"
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/SpatialManager.h"
#include "JPLSpatial/DistanceAttenuation.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Memory/Memory.h"

#include "../Utility/TestMemoryLeakDetector.h"

#include <gtest/gtest.h>

#include <format>
#include <vector>

namespace JPL
{
	using namespace Spatial;

	class SpatializationTest : public testing::Test
	{
	protected:
		using Vec3Type = MinimalVec3;

		SpatializationTest() = default;

	protected:
		TestLeakDetector mLeakDetector;

		void SetUp() override
		{
			mLeakDetector.SetUp();
		}

		void TearDown() override
		{
			mLeakDetector.TearDown();
		}
	};

#if 0
	TEST_F(SpatializationTest, Vec3)
	{
		Vec3 v1(4.0f, 2.0f, 1.0f);
		EXPECT_FLOAT_EQ(v1.mF[0], 4.0f);
		EXPECT_FLOAT_EQ(v1.mF[1], 2.0f);
		EXPECT_FLOAT_EQ(v1.mF[2], 1.0f);
		EXPECT_FLOAT_EQ(v1.mF[3], 1.0f);

		EXPECT_EQ(Vec3(4.0, 3.0f, 2.0f), Vec3(4.0, 3.0f, 2.0f));
		EXPECT_FALSE(Vec3(4.0, 3.0f, 2.0f) == Vec3(0.0, 3.0f, 2.0f));

		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) * Vec3(0.0f, 1.0f, 2.0f), Vec3(0.0f, 2.0f, 2.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) * 2.0f, Vec3(8.0f, 4.0f, 2.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) / 2.0f, Vec3(2.0f, 1.0f, 0.5f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) *= 2.0f, Vec3(8.0f, 4.0f, 2.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) *= Vec3(0.0f, 1.0f, 2.0f), Vec3(0.0f, 2.0f, 2.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) /= 2.0f, Vec3(2.0f, 1.0f, 0.5f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) + Vec3(0.0f, 1.0f, 2.0f), Vec3(4.0f, 3.0f, 3.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) += Vec3(0.0f, 1.0f, 2.0f), Vec3(4.0f, 3.0f, 3.0f));
		EXPECT_EQ(-Vec3(4.0f, 2.0f, 1.0f), Vec3(-4.0f, -2.0f, -1.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) - Vec3(0.0f, 1.0f, 2.0f), Vec3(4.0f, 1.0f, -1.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) -= Vec3(0.0f, 1.0f, 2.0f), Vec3(4.0f, 1.0f, -1.0f));
		EXPECT_EQ(Vec3(4.0f, 2.0f, 1.0f) / Vec3(4.0f, 1.0f, 2.0f), Vec3(1.0f, 2.0f, 0.5f));

		EXPECT_FLOAT_EQ(Vec3(4.0f, 2.0f, 1.0f).Dot(Vec3(0.0f, 1.0f, 2.0f)), 4.0f);
		EXPECT_EQ(Vec3(4.0f, 9.0f, 16.0f).Sqrt(), Vec3(2.0f, 3.0f, 4.0f));
		EXPECT_FLOAT_EQ(Vec3(1.0f, 2.0f, 3.0f).LengthSq(), 14.0f);
		EXPECT_FLOAT_EQ(Vec3(1.0f, 2.0f, 3.0f).Length(), std::sqrt(14.0f));

		const float length = std::sqrt(14.0f);
		EXPECT_EQ(Vec3(1.0f, 2.0f, 3.0f).Normalized(), Vec3(1.0f / length, 2.0f / length, 3.0f / length));

		EXPECT_EQ(Vec3(-1.0f, 2.0f, -3.0f).GetSign(), Vec3(-1.0f, 1.0f, -1.0f));
		EXPECT_EQ(Vec3(-1.0f, 2.0f, -3.0f).Abs(), Vec3(1.0f, 2.0f, 3.0f));
		EXPECT_EQ(Vec3(-1.0f, 2.0f, -3.0f).Reciprocal(), Vec3(1.0f / -1.0f, 1.0f / 2.0f, 1.0f / -3.0f));
	}
#endif

#if 0
	TEST_F(SpatializationTest, DistanceAttenuation)
	{
		using namespace Spatialization;

		struct DistanceAttenuationTestCase
		{
			std::string Description;
			AttenuationModel AttenuationModel;
			float Distance;
			DistanceAttenuationParameters Parameters;
			float ExpectedAttenuation;
		};

		const std::vector<DistanceAttenuationTestCase> testCases = {
		{
			.Description = "AttenuationModel::None with any distance",
			.AttenuationModel = AttenuationModel::None,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		},
		{
			.Description = "Inverse Model within range",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.2f
		},
		{
			.Description = "Inverse Model below MinDistance",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 0.5f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		},
		{
			.Description = "Inverse Model above MaxDistance",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 15.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.1f
		},
		{
			.Description = "Linear Model within range",
			.AttenuationModel = AttenuationModel::Linear,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.5555556f
		},
		{
			.Description = "Linear Model below MinDistance",
			.AttenuationModel = AttenuationModel::Linear,
			.Distance = 0.5f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		},
		{
			.Description = "Linear Model above MaxDistance",
			.AttenuationModel = AttenuationModel::Linear,
			.Distance = 15.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.0f
		},
		{
			.Description = "Exponential Model within range",
			.AttenuationModel = AttenuationModel::Exponential,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.2f
		},
		{
			.Description = "Exponential Model below MinDistance",
			.AttenuationModel = AttenuationModel::Exponential,
			.Distance = 0.5f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		},
		{
			.Description = "Exponential Model above MaxDistance",
			.AttenuationModel = AttenuationModel::Exponential,
			.Distance = 15.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.1f
		},
		{
			.Description = "Invalid Parameters (MinDistance >= MaxDistance)",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 10.0f,
				.MaxDistance = 1.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		}
			// Add more test cases as needed
		};

		static constexpr float tolerance = 1e-5f;

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			const float attenuation = ComputeDistanceAttenuation(
				testCase.AttenuationModel,
				testCase.Distance,
				testCase.Parameters
			);

			EXPECT_NEAR(attenuation, testCase.ExpectedAttenuation, tolerance)
				<< "AttenuationModel: " << testCase.AttenuationModel
				<< ", Distance: " << testCase.Distance
				<< ", MinDistance: " << testCase.Parameters.MinDistance
				<< ", MaxDistance: " << testCase.Parameters.MaxDistance
				<< ", Rolloff: " << testCase.Parameters.Rolloff;
		}
	}
#endif

#if 0
	TEST_F(SpatializationTest, DistanceAttenuationVectorized)
	{
		using namespace Spatialization;

		struct DistanceAttenuationTestCase
		{
			std::string Description;
			AttenuationModel AttenuationModel;
			float Distance;
			DistanceAttenuationParameters Parameters;
			float ExpectedAttenuation;
		};

		const std::vector<DistanceAttenuationTestCase> testCases = {
		{
			.Description = "Inverse Model within range",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.2f
		},
		{
			.Description = "Inverse Model below MinDistance",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 0.5f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		},
		{
			.Description = "Inverse Model above MaxDistance",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 15.0f,
			.Parameters = {
				.MinDistance = 1.0f,
				.MaxDistance = 10.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 0.1f
		},
		{
			.Description = "Invalid Parameters (MinDistance >= MaxDistance)",
			.AttenuationModel = AttenuationModel::Inverse,
			.Distance = 5.0f,
			.Parameters = {
				.MinDistance = 10.0f,
				.MaxDistance = 1.0f,
				.Rolloff = 1.0f
			},
			.ExpectedAttenuation = 1.0f
		}
			// Add more test cases as needed
		};

		static constexpr float tolerance = 1e-5f;

		std::vector<float> distance;
		std::vector<float> minDistance;
		std::vector<float> maxDistance;
		std::vector<float> rolloff;
		std::vector<float> attenuationResults;

		auto testNumberOfOperations = [&](uint32 numOperations)
		{
			if (numOperations == 0)
				return;

			ASSERT_TRUE(numOperations < testCases.size());

			distance.resize(numOperations);
			minDistance.resize(numOperations);
			maxDistance.resize(numOperations);
			rolloff.resize(numOperations);
			attenuationResults.resize(numOperations);

			for (uint32 i = 0; i < numOperations; ++i)
			{
				distance[i] = testCases[i].Distance;
				minDistance[i] = testCases[i].Parameters.MinDistance;
				maxDistance[i] = testCases[i].Parameters.MaxDistance;
				rolloff[i] = testCases[i].Parameters.Rolloff;
			}
			ComputeResult<DistanceAttenuation>(attenuationResults, AttenuationModel::Inverse, distance, minDistance, maxDistance, rolloff);

			for (uint32 i = 0; i < numOperations; ++i)
			{
				const auto& testCase = testCases[i];

				SCOPED_TRACE(std::format("{}, Number of Operations: {}", testCase.Description, numOperations));
				
				const float attenuation = attenuationResults[i];

				EXPECT_NEAR(attenuation, testCase.ExpectedAttenuation, tolerance)
						<< "AttenuationModel: " << testCase.AttenuationModel
						<< ", Distance: " << testCase.Distance
						<< ", MinDistance: " << testCase.Parameters.MinDistance
						<< ", MaxDistance: " << testCase.Parameters.MaxDistance
						<< ", Rolloff: " << testCase.Parameters.Rolloff;
			}
		};

		for (uint32 i = 0; i < testCases.size(); ++i)
			testNumberOfOperations(i);
	}
#endif

	/*TEST_F(SpatializationTest, SourceAngleAttenuation)
	{
		FAIL();
	}

	TEST_F(SpatializationTest, ListenerAngleAttenuation)
	{
		FAIL();
	}*/

	TEST_F(SpatializationTest, CreateDeleteSource)
	{
		SpatialManager<Vec3Type> spatializer;

		EXPECT_EQ(spatializer.mSourceStuff.size(), 0);
		EXPECT_FALSE(spatializer.DeleteSource(SourceId()));

		// Create 1st source data
		SourceId id1 = spatializer.CreateSource({});
		EXPECT_TRUE(id1.IsValid());
		EXPECT_TRUE(id1);
		EXPECT_EQ(spatializer.mSourceStuff.size(), 1);

		// Create 2nd source data
		SourceId id2 = spatializer.CreateSource({});
		EXPECT_TRUE(id2.IsValid());
		EXPECT_TRUE(id2);
		EXPECT_EQ(spatializer.mSourceStuff.size(), 2);

		// Delete 1st source data
		EXPECT_TRUE(spatializer.DeleteSource(id1));
	}

	TEST_F(SpatializationTest, AdvanceSimulation)
	{
		struct AdvanceSimTestCase
		{
			std::string Description;
			Vec3Type SourcePosition;
			const std::pmr::vector<typename AttenuationCurve::Point> AttenuationCurvePoints
			{
					{.Distance = 0.0f, .Value = 1.0f, .FunctionType = Curve::EType::Linear},
					{.Distance = 10.0f, .Value = 0.5f, .FunctionType = Curve::EType::Linear}
			};
			float ExpectedAttenuationValue;
			std::pmr::vector<float> ExpectedGains;

			SourceId Source;
			AttenuationCurveRef AttCurve;
		};
	
		std::vector<AdvanceSimTestCase> testCases
		{
			{
				.Description = "Source in front of the listener",
				.SourcePosition = Vec3Type(0.0f, 0.0f, -5.0f),
				.ExpectedAttenuationValue = 0.75f,
				.ExpectedGains = { 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				.Description = "Source above the listener",
				.SourcePosition = Vec3Type(0.0f, 5.0f, 0.0f),
				.ExpectedAttenuationValue = 0.75f,
				.ExpectedGains = { 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
			{
				.Description = "Source left of the listener",
				.SourcePosition = Vec3Type(-5.0f, 0.0f, 0.0f),
				.ExpectedAttenuationValue = 0.75f,
				.ExpectedGains = { 0.7071f, 0.0f, 0.7071f, 0.0f }
			},
			{
				.Description = "Source on top of the listener",
				.SourcePosition = Vec3Type(0.0f, 0.0f, 0.0f),
				.ExpectedAttenuationValue = 1.0f,
				.ExpectedGains = { 0.7071f, 0.7071f, 0.0f, 0.0f }
			},
		};

		SpatialManager<Vec3Type> spatializer;
		const ChannelMap quadChannels = ChannelMap::FromChannelMask(ChannelMask::Quad);

		for (auto& testCase : testCases)
		{

			SourceId source = spatializer.CreateSource(
				SourceInitParameters{
					.NumChannels = 1,
					.NumTargetChannels = quadChannels.GetNumChannels(),
					.PanParameters = {.Focus = 0.0f, .Spread = 0.0f }
				});
	
			ASSERT_TRUE(source.IsValid());
			testCase.Source = source;

			Position<Vec3Type> sourcePos{
				.Location = testCase.SourcePosition,
				.Orientation = OrientationData<Vec3Type>::Identity()
			};
			ASSERT_TRUE(spatializer.SetSourcePosition(source, sourcePos));

			// Prepare attenuation
			auto curve = make_pmr_shared<AttenuationCurve>();
			curve->Points = testCase.AttenuationCurvePoints;
			curve->SortPoints();
			testCase.AttCurve = spatializer.GetDirectPathService().AssignAttenuationCurve(spatializer.GetDirectEffectHandle(source), curve);

			// Prepare panning
			// TODO: test PanningService on its own
			const auto* panner = spatializer.GetPanningService().CreatePannerFor(quadChannels);
			ASSERT_TRUE(panner != nullptr);
			//const bool targetCacheInitialized = spatializer.GetPanningService().AddSourceTargets(spatializer.GetPanEffectHandle(source), {quadChannels});
			//ASSERT_TRUE(targetCacheInitialized);
		}

		// TODO: test the cases where panner and/or channel gains are not initialzied
		// TODO: test 0 channel sources and 0 channel targets

		// Process all the data
		spatializer.AdvanceSimulation();

		// TODO: maybe compute or precompute max possible error for given LUT resolution
		static constexpr float tolerance = 4e-3f;

		for (const auto& testCase : testCases)
		{
			//! Note: SCOPED_TRACE allocate and messes up our leak detector,
			//! just print the description on failed EXPECT/ASSERT as a workaround
			// SCOPED_TRACE(testCase.Description);

			// Test attenuation
			const float attenuation = spatializer.GetDistanceAttenuation(testCase.Source, testCase.AttCurve);
			EXPECT_NEAR(attenuation, testCase.ExpectedAttenuationValue, tolerance) << testCase.Description;

			// Test panning
			const auto channelGains = spatializer.GetChannelGains(testCase.Source, quadChannels);
			ASSERT_FALSE(channelGains.empty()) << testCase.Description;

			static constexpr uint32 sourceChannel = 0;

			const auto& channelGainsRef = channelGains;
			
			for (uint32 targetChannel = 0; targetChannel < testCase.ExpectedGains.size(); ++targetChannel)
			{
				EXPECT_NEAR(channelGainsRef[sourceChannel * testCase.ExpectedGains.size() + targetChannel],
							testCase.ExpectedGains[targetChannel], tolerance) << testCase.Description << "\n Target channel: " << targetChannel;
			}
		}
	}

#if 0 //? probably obsolete
	TEST_F(SpatializationTest, ProcessDirtySources)
	{
		using namespace Spatialization;

		struct DirtySorucesTestCase
		{
			std::string Description;
			uint32 NumSources;
			std::set<uint32> DirtyIndices;
		};

		constexpr auto createDirtyIniceSet = [](uint32 numIndices)
		{
			std::set<uint32> dirtyIndices;
			for (uint32 i = 0; i < numIndices; ++i)
				dirtyIndices.insert(i);
			return dirtyIndices;
		};

		const std::vector<DirtySorucesTestCase> testCases = {
		{
			.Description = "No dirty sources",
			.NumSources = 6,
			.DirtyIndices = {}
		},
		{
			.Description = "All sources dirty",
			.NumSources = 6,
			.DirtyIndices = { 0, 1, 2, 3, 4, 5 }
		},
		{
			.Description = "First source dirty",
			.NumSources = 6,
			.DirtyIndices = { 0 }
		},
		{
			.Description = "Last source dirty",
			.NumSources = 6,
			.DirtyIndices = { 5 }
		},
		{
			.Description = "Some sources are dirty",
			.NumSources = 6,
			.DirtyIndices = { 1, 3, 4 }
		},
		{
			.Description = "Internal static compute buffer wrap around",
			.NumSources = static_cast<uint32>(DistanceAttenuationData::Capacity()) + 5,
			.DirtyIndices = createDirtyIniceSet(static_cast<uint32>(DistanceAttenuationData::Capacity()) + 5)
		},
		{
			.Description = "1000 dirty sources",
			.NumSources = 1000,
			.DirtyIndices = createDirtyIniceSet(1000)
		},
		};

		static const Vec3Type dirtyPosition = Vec3Type(0.0f, 5.0f, 0.0f);
		static constexpr float nonDirtyAttenuation = SourceOutputDataMock{}.DistanceAttenuation;
		static constexpr float dirtyAttenuation = 0.2f;

		static constexpr float tolerance = 1e-5f;

		for (const auto& testCase : testCases)
		{
			SCOPED_TRACE(testCase.Description);

			SpatialManager<Vec3Type> spatializer;

			std::vector<SourceId> testSources(testCase.NumSources);
			for (auto& id : testSources)
				id = spatializer.CreateSource({});

			for (uint32 dirtyIndex : testCase.DirtyIndices)
			{
				ASSERT_TRUE(dirtyIndex < testSources.size());
				ASSERT_TRUE(spatializer.SetSourcePosition(testSources[dirtyIndex], dirtyPosition));

				// We can use the same parameters, since we only test "dirtiness"
				ASSERT_TRUE(spatializer.SetAttenuationParameters(
					testSources[dirtyIndex],
					AttenuationModel::Inverse,
					{
						.MinDistance = 1.0f,
						.MaxDistance = 10.0f,
						.Rolloff = 1.0f
					}));
			}

			{
				SCOPED_TRACE("Before calling AdvanceSimulation");

				for (const SourceId source : testSources)
				{
					EXPECT_NEAR(spatializer.GetDistanceAttenuation(source),
								nonDirtyAttenuation,
								tolerance)
						<< "Source: " << source << " should have old attenuation.";
				}
			}

			spatializer.AdvanceSimulation();

			{
				SCOPED_TRACE("After calling AdvanceSimulation");

				for (uint32 i = 0; i < testSources.size(); ++i)
				{
					const SourceId source = testSources[i];
					if (testCase.DirtyIndices.contains(i))
					{
						EXPECT_NEAR(spatializer.GetDistanceAttenuation(source),
									dirtyAttenuation,
									tolerance)
							<< "Source: " << source << " should have new attenuation.";
					}
					else
					{
						EXPECT_NEAR(spatializer.GetDistanceAttenuation(source),
									nonDirtyAttenuation,
									tolerance)
							<< "Source: " << source << " should have old attenuation.";
					}
				}
			}
		}
	}
#endif

} // namespace JPL