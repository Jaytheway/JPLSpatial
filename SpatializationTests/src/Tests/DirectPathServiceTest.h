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
#include "JPLSpatial/Services/DirectPathService.h"

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/MinimalQuat.h"
#include "JPLSpatial/Math/Position.h"

#include "../Utility/TestMemoryLeakDetector.h"

#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <numbers>
#include <vector>
#include <string>

namespace JPL
{
	class ProcessAngleAttenuationTest : public ::testing::Test
	{
	protected:
		using Vec3 = MinimalVec3;

		static JPL_INLINE Position<Vec3> GetForwardFacingIdentity()
		{
			return Position<Vec3>{
				.Location = Vec3(0, 0, 0),
				.Orientation = OrientationData<Vec3>::IdentityForward()
			};
		}

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
	
	class ProcessDirectPath : public ProcessAngleAttenuationTest
	{
	protected:
		enum class EFacing { Forward, Backward, Left, Right };
		struct ListenerTestCase
		{
			std::string Description;
			Position<Vec3> Position;
		};


		static OrientationData<Vec3> OrientForward(const Vec3& forward)
		{
			return OrientationData<Vec3>{.Up = Vec3(0, 1, 0), .Forward = forward };
		};

		static ListenerTestCase GetListenerCaseFor(EFacing direction)
		{
			switch (direction)
			{
			case ProcessDirectPath::EFacing::Forward:
				return {
					.Description = "Listener at origin, facing forwawrd",
					.Position = {.Location = Vec3(0, 0, 0), .Orientation = OrientForward(Vec3(0, 0, -1)) }
				};
			case ProcessDirectPath::EFacing::Backward:
				return {
						.Description = "Listener at origin, facing backward",
						.Position = {.Location = Vec3(0, 0, 0), .Orientation = OrientForward(Vec3(0, 0, 1)) }
				};
			case ProcessDirectPath::EFacing::Left:
				return {
						.Description = "Listener at origin, facing left",
						.Position = {.Location = Vec3(0, 0, 0), .Orientation = OrientForward(Vec3(-1, 0, 0)) }
				};
			case ProcessDirectPath::EFacing::Right:
				return {
					.Description = "Listener at origin, facing right",
					.Position = {.Location = Vec3(0, 0, 0), .Orientation = OrientForward(Vec3(1, 0, 0)) }
				};
			default:
				return {
					.Description = "Listener at origin, facing forwawrd",
					.Position = {.Location = Vec3(0, 0, 0), .Orientation = OrientForward(Vec3(0, 0, -1)) }
				};
			}
		}

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

	TEST_F(ProcessAngleAttenuationTest, InsideInnerCone)
	{
		// Inputs
		Vec3 position(0.0f, 0.0f, -1.0f); // Directly in front (since forward is -Z)
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		Vec3 forw = referencePoint.Orientation.ToBasisUnsafe().Transform(Vec3(0, 0, 1)); // get forward axis rotation
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(60.0f);
		cone.OuterAngle = Math::ToRadians(120.0f);
		float coneOuterGain = 0.5f;

		// Expected Output
		float expectedAngularGain = 1.0f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginal, expectedAngularGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, BetweenInnerAndOuterCone)
	{
		// Inputs
		// Position at 45 degrees off the forward vector (-Z)
		float angle = Math::ToRadians(45.0f);
		Vec3 position(std::sin(angle), 0.0f, -std::cos(angle));
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(60.0f);
		cone.OuterAngle = Math::ToRadians(120.0f);
		float coneOuterGain = 0.5f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_TRUE(angularGainOriginal > coneOuterGain);
		EXPECT_TRUE(angularGainOriginal < 1.0f);
	}

	TEST_F(ProcessAngleAttenuationTest, OutsideOuterCone)
	{
		// Inputs
		Vec3 position(0.0f, 0.0f, 1.0f); // Directly behind (since forward is -Z)
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(60.0f);
		cone.OuterAngle = Math::ToRadians(120.0f);
		float coneOuterGain = 0.5f;

		// Expected Output
		float expectedAngularGain = coneOuterGain;

		// Compute using original function
		float angularGain = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGain = std::lerp(1.0f, coneOuterGain, angularGain);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGain, expectedAngularGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, InnerAngleEqualsOuterAngle)
	{
		// Inputs
		Vec3 position(1.0f, 0.0f, -1.0f); // Arbitrary position in front-left
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(45.0f);
		cone.OuterAngle = Math::ToRadians(45.0f);
		float coneOuterGain = 0.5f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginal, coneOuterGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, AnglesZero)
	{
		// Inputs
		Vec3 position(0.0f, 0.0f, -1.0f); // Directly in front (-Z)
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = 0.0f;
		cone.OuterAngle = 0.0f;
		float coneOuterGain = 0.5f;

		// Expected Output: Since angles are zero, angularGain should be 1.0f when directly in front
		float expectedAngularGain = coneOuterGain;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginal, expectedAngularGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, AnglesFullCircle)
	{
		// Inputs
		Vec3 position(10.0f, 0.0f, 5.0f); // Arbitrary position
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(360.0f);
		cone.OuterAngle = Math::ToRadians(360.0f);
		float coneOuterGain = 0.5f;

		// Expected Output: angularGain should be 1.0f
		float expectedAngularGain = 1.0f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginal, expectedAngularGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, NegativeAngles)
	{
		// Inputs
		Vec3 position(0.0f, 1.0f, -1.0f); // Arbitrary position in front-up
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(-90.0f);
		cone.OuterAngle = Math::ToRadians(-180.0f);
		float coneOuterGain = 0.5f;

		float expectedAngularGain = 1.0f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Since negative angles may not be valid, ensure both functions handle them consistently
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginal, expectedAngularGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, OuterAngleLessThanInnerAngle)
	{
		// Inputs
		Vec3 position(-1.0f, 0.0f, -1.0f); // Arbitrary position in front-left
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(120.0f);
		cone.OuterAngle = Math::ToRadians(60.0f); // Outer angle greater than inner angle
		float coneOuterGain = 0.5f;

		// Compute using original function
		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		//EXPECT_NEAR(angularGainOriginal, angularGainSIMD, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, DAtCutoffValues)
	{
		// Inputs
		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(60.0f);
		cone.OuterAngle = Math::ToRadians(90.0f);
		float coneOuterGain = 0.5f;

		// Compute cutoff values
		float cutoffInner = std::cos(cone.InnerAngle * 0.5f);
		float cutoffOuter = std::cos(cone.OuterAngle * 0.5f);

		// For angle θ, position vector is:
		// position = [sin(θ), 0, -cos(θ)] (since forward is -Z)

		// Angle corresponding to cutoffInner
		float angleInner = std::acos(cutoffInner);
		Vec3 positionInner(std::sin(angleInner), 0.0f, -std::cos(angleInner));

		// Angle corresponding to cutoffOuter
		float angleOuter = std::acos(cutoffOuter);
		Vec3 positionOuter(std::sin(angleOuter), 0.0f, -std::cos(angleOuter));

		// Compute using original function for cutoffInner
		float angularGainOriginalInner = DirectPathService::ProcessAngleAttenuation(positionInner, referencePoint, cone);
		angularGainOriginalInner = std::lerp(1.0f, coneOuterGain, angularGainOriginalInner);


		// Compute using original function for cutoffOuter
		float angularGainOriginalOuter = DirectPathService::ProcessAngleAttenuation(positionOuter, referencePoint, cone);
		angularGainOriginalOuter = std::lerp(1.0f, coneOuterGain, angularGainOriginalOuter);

		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_NEAR(angularGainOriginalInner, 1.0f, tolerance);
		EXPECT_NEAR(angularGainOriginalOuter, coneOuterGain, tolerance);
	}

	TEST_F(ProcessAngleAttenuationTest, AzimuthOverload)
	{
		std::mt19937 rng(42); // Seed for reproducibility
		std::uniform_real_distribution<float> angleDist(0.0f, std::numbers::pi_v<float> *2.0f);
		std::uniform_real_distribution<float> azimuthDist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
		std::uniform_real_distribution<float> gainDist(0.0f, 1.0f);

		const int numTests = 1000;
		static constexpr float toleranse = 1e-5f;

		// Random reference point
		Position<Vec3> referencePoint = GetForwardFacingIdentity();

		for (int i = 0; i < numTests; ++i)
		{
			// Random positions around the listener
			float azimuth = azimuthDist(rng);
			Vec3 position(std::sin(azimuth), 0.0f, -std::cos(azimuth));

			// Random cone angles and gain
			AttenuationCone cone{
				.InnerAngle = angleDist(rng),
				.OuterAngle = angleDist(rng)
			};

			float coneOuterGain = gainDist(rng);

			// Ensure innerAngle <= outerAngle
			if (cone.InnerAngle > cone.OuterAngle)
				std::swap(cone.InnerAngle, cone.OuterAngle);

			// Compute using azimuth overload
			float angularGainAzimuth = DirectPathService::ProcessAngleAttenuation(azimuth, cone);
			angularGainAzimuth = std::lerp(1.0f, coneOuterGain, angularGainAzimuth);

			// Compute using original function
			float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
			angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);

			EXPECT_NEAR(angularGainAzimuth, angularGainOriginal, toleranse) << "Mismatch at iteration " << i;
		}
	}

#if 0
	TEST_F(ProcessAngleAttenuationTest, SIMDvsBranches_PerformanceTest)
	{
		std::mt19937 rng(42); // Seed for reproducibility
		std::uniform_real_distribution<float> angleDist(0.0f, std::numbers::pi_v<float> *2.0f);
		std::uniform_real_distribution<float> gainDist(0.0f, 1.0f);

		const int numTests = 5000;
		static constexpr float toleranse = 1e-5f;

		// Random reference point
		Position<Vec3> referencePoint = GetForwardFacingIdentity();

		float branchedTime = 0.0f;
		{
			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < numTests; ++i)
			{
				// Random positions around the listener
				const Vec3 position = Vec3::sRandom(rng) * 10.0f;

				// Random cone angles and gain
				AttenuationCone cone{
					.InnerAngle = angleDist(rng),
					.OuterAngle = angleDist(rng),
					.OuterGain = gainDist(rng)
				};

				// Ensure innerAngle <= outerAngle
				if (cone.InnerAngle > cone.OuterAngle)
					std::swap(cone.InnerAngle, cone.OuterAngle);

				// Compute using original function
				volatile float angularGainOriginal = ProcessAngleAttenuationOriginal(position, referencePoint, cone);
				auto l = [angularGainOriginal] {};
			}

			branchedTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 0.001f;
		}

		float SIMDTime = 0.0f;
		{
			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < numTests; ++i)
			{
				// Random positions around the listener
				const Vec3 position = Vec3::sRandom(rng) * 10.0f;

				// Random reference point
				Position<Vec3> referencePoint = GetForwardFacingIdentity();

				// Random cone angles and gain
				AttenuationCone cone{
					.InnerAngle = angleDist(rng),
					.OuterAngle = angleDist(rng),
					.OuterGain = gainDist(rng)
				};

				// Ensure innerAngle <= outerAngle
				if (cone.InnerAngle > cone.OuterAngle)
					std::swap(cone.InnerAngle, cone.OuterAngle);

				// Compute using SIMD function
				volatile float angularGainSIMD = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
				//angularGainSIMD = 
				auto l = [angularGainSIMD] {};
			}

			SIMDTime = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count() * 0.001f;
		}

		EXPECT_TRUE(branchedTime > SIMDTime)
			<< "SIMD is slower than branched version: "
			<< "SIMD time: " << SIMDTime
			<< " | Branched time: " << branchedTime;
	}
#endif

	TEST_F(ProcessAngleAttenuationTest, AzimuthOverload2)
	{
		// Inputs
		// Position at 45 degrees off the forward vector (-Z)
		float azimuth = Math::ToRadians(45.0f);
		Vec3 position(std::sin(azimuth), 0.0f, -std::cos(azimuth));

		Position<Vec3> referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = Math::ToRadians(60.0f);
		cone.OuterAngle = Math::ToRadians(120.0f);
		float coneOuterGain = 0.5f;

		// Compute using original function
		float angularGainAzimuth = DirectPathService::ProcessAngleAttenuation(azimuth, cone);
		angularGainAzimuth = std::lerp(1.0f, coneOuterGain, angularGainAzimuth);

		float angularGainOriginal = DirectPathService::ProcessAngleAttenuation(position, referencePoint, cone);
		angularGainOriginal = std::lerp(1.0f, coneOuterGain, angularGainOriginal);


		// Assertions
		static constexpr float tolerance = 1e-6f;
		EXPECT_TRUE(angularGainOriginal > coneOuterGain);
		EXPECT_TRUE(angularGainOriginal < 1.0f);
		EXPECT_NEAR(angularGainAzimuth, angularGainOriginal, tolerance);
	}

	TEST_F(ProcessDirectPath, ProcessDirectPath_ComputesCorrectDotProducts)
	{
		struct DirectPathTestCase
		{
			std::string Description;
			Vec3 SourcePosition;
			
			ListenerTestCase ListenerCase;

			float ExpectedDot;
			float ExpectedInvDot;
		};

		const std::pmr::vector<DirectPathTestCase> testCases(
		{
			// Listener facing forward
			{
				.Description = "Source in world forward",
				.SourcePosition = Vec3(0.0f, 0.0f, -5.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Forward),

				.ExpectedDot = 1.0f,
				.ExpectedInvDot = -1.0f,
			},
			{
				.Description = "Source above the listener",
				.SourcePosition = Vec3(0.0f, 5.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Forward),

				.ExpectedDot = 0.0f,
				.ExpectedInvDot = 0.0f,
			},
			{
				.Description = "Source in world left",
				.SourcePosition = Vec3(-5.0f, 0.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Forward),

				.ExpectedDot = 0.0f,
				.ExpectedInvDot = 0.0f,
			},
			{
				.Description = "Source on top of the listener",
				.SourcePosition = Vec3(0.0f, 0.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Forward),
				
				.ExpectedDot = 1.0f,
				.ExpectedInvDot = -1.0f,
			},

			// Listener facing backward
			{
				.Description = "Source in world forward",
				.SourcePosition = Vec3(0.0f, 0.0f, -5.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Backward),

				.ExpectedDot = -1.0f,
				.ExpectedInvDot = -1.0f,
			},
			{
				.Description = "Source above the listener",
				.SourcePosition = Vec3(0.0f, 5.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Backward),

				.ExpectedDot = 0.0f,
				.ExpectedInvDot = 0.0f,
			},
			{
				.Description = "Source in world left",
				.SourcePosition = Vec3(-5.0f, 0.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Backward),

				.ExpectedDot = 0.0f,
				.ExpectedInvDot = 0.0f,
			},
			{
				.Description = "Source on top of the listener",
				.SourcePosition = Vec3(0.0f, 0.0f, 0.0f),
				.ListenerCase = GetListenerCaseFor(EFacing::Backward),

				.ExpectedDot = 1.0f,
				.ExpectedInvDot = 1.0f,
			},

			// Listener offset, facing fowrard
			{
				.Description = "Source in world forward",
				.SourcePosition = Vec3(0.0f, 0.0f, -10.0f),
				.ListenerCase = {
					.Description = "Listener moved forward, facing forwawrd",
					.Position = {.Location = Vec3(0, 0, -5), .Orientation = OrientForward(Vec3(0, 0, -1)) }
				},

				.ExpectedDot = 1.0f,
				.ExpectedInvDot = -1.0f,
			},
			{
				.Description = "Source in world diagonal fowrard-right",
				.SourcePosition = Vec3(10.0f, 0.0f, -10.0f),
				.ListenerCase = {
					.Description = "Listener left of the source, facing forwawrd",
					.Position = {.Location = Vec3(0, 0, -10), .Orientation = OrientForward(Vec3(0, 0, -1)) }
				},

				.ExpectedDot = 0.0f,
				.ExpectedInvDot = 0.0f,
			},
			{
				.Description = "Source in world diagonal forward-left",
				.SourcePosition = Vec3(-10.0f, 0.0f, -10.0f),
				.ListenerCase = {
					.Description = "Listener diagonal forward-right of the source, facing forwawrd",
					.Position = {.Location = Vec3(0, 0, -20), .Orientation = OrientForward(Vec3(0, 0, -1)) }
				},

				.ExpectedDot = -0.7071f,
				.ExpectedInvDot = 0.7071f,
			},

			// Listener offset, facing source
			{
				.Description = "Source in world diagonal forward-right",
				.SourcePosition = Vec3(10.0f, 0.0f, -10.0f),
				.ListenerCase = {
					.Description = "Listener diagonal back-left of the source, facing source",
					.Position = {.Location = Vec3(5, 0, -5), .Orientation = OrientForward(Normalized(Vec3(1, 0, -1))) }
				},

				.ExpectedDot = 1.0f,
				.ExpectedInvDot = -0.7071f,
			},
			{
				.Description = "Source in world diagonal forward-left",
				.SourcePosition = Vec3(-10.0f, 0.0f, -10.0f),
				.ListenerCase = {
					.Description = "Listener diagonal fowrard-right of the source, facing source",
					.Position = {.Location = Vec3(0, 0, -20), .Orientation = OrientForward(Normalized(Vec3(-1, 0, 1))) }
				},

				.ExpectedDot = 1.0f,
				.ExpectedInvDot = 0.7071f,
			},
		}, JPL::GetDefaultPmrAllocator()); // just to silence the error for bypassing pmr allocator, only for this case.;

		static constexpr float tolerance = 1e-5f;

		for (const DirectPathTestCase& testCase : testCases)
		{
			const Position<Vec3>& listenerPosition = testCase.ListenerCase.Position;

			SCOPED_TRACE(testCase.ListenerCase.Description);
			SCOPED_TRACE(testCase.Description);

			const Position<Vec3> sourcePosition{
				.Location = testCase.SourcePosition,
				.Orientation = OrientationData<Vec3>::IdentityForward()
			};

			const DirectPathResult<Vec3> result = DirectPathService::ProcessDirectPath(sourcePosition, listenerPosition);

			EXPECT_NEAR(result.DirectionDot, testCase.ExpectedDot, tolerance);
			EXPECT_NEAR(result.InvDirectionDot, testCase.ExpectedInvDot, tolerance);
		}
	}

} // namespace JPL