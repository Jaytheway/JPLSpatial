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


#include <Jolt/Jolt.h>

#include <gtest/gtest.h>
#include <cmath>
#include <random>
#include <numbers>
#include <chrono>

namespace JPL
{
	class ProcessAngleAttenuationTest : public ::testing::Test
	{
	protected:
		static JPL_INLINE float DegreesToRadians(float degrees)
		{
			return degrees * (std::numbers::pi_v<float> / 180.0f);
		}

		static JPL_INLINE JPH::Mat44 GetForwardFacingIdentity()
		{
			return JPH::Mat44::sRotationTranslation(
				JPH::Quat::sRotation(JPH::Vec3::sAxisY(), std::numbers::pi_v<float>).Normalized(),
				JPH::Vec3::sZero());
		}

	};


	TEST_F(ProcessAngleAttenuationTest, InsideInnerCone)
	{
		// Inputs
		JPH::Vec3 position(0.0f, 0.0f, -1.0f); // Directly in front (since forward is -Z)
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		JPH::Vec3 forw = referencePoint.GetRotation().GetAxisZ();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(60.0f);
		cone.OuterAngle = DegreesToRadians(120.0f);
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
		float angle = DegreesToRadians(45.0f);
		JPH::Vec3 position(std::sin(angle), 0.0f, -std::cos(angle));
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(60.0f);
		cone.OuterAngle = DegreesToRadians(120.0f);
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
		JPH::Vec3 position(0.0f, 0.0f, 1.0f); // Directly behind (since forward is -Z)
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(60.0f);
		cone.OuterAngle = DegreesToRadians(120.0f);
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
		JPH::Vec3 position(1.0f, 0.0f, -1.0f); // Arbitrary position in front-left
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(45.0f);
		cone.OuterAngle = DegreesToRadians(45.0f);
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
		JPH::Vec3 position(0.0f, 0.0f, -1.0f); // Directly in front (-Z)
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
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
		JPH::Vec3 position(10.0f, 0.0f, 5.0f); // Arbitrary position
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(360.0f);
		cone.OuterAngle = DegreesToRadians(360.0f);
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
		JPH::Vec3 position(0.0f, 1.0f, -1.0f); // Arbitrary position in front-up
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(-90.0f);
		cone.OuterAngle = DegreesToRadians(-180.0f);
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
		JPH::Vec3 position(-1.0f, 0.0f, -1.0f); // Arbitrary position in front-left
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(120.0f);
		cone.OuterAngle = DegreesToRadians(60.0f); // Outer angle greater than inner angle
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
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(60.0f);
		cone.OuterAngle = DegreesToRadians(90.0f);
		float coneOuterGain = 0.5f;

		// Compute cutoff values
		float cutoffInner = std::cos(cone.InnerAngle * 0.5f);
		float cutoffOuter = std::cos(cone.OuterAngle * 0.5f);

		// For angle θ, position vector is:
		// position = [sin(θ), 0, -cos(θ)] (since forward is -Z)

		// Angle corresponding to cutoffInner
		float angleInner = std::acos(cutoffInner);
		JPH::Vec3 positionInner(std::sin(angleInner), 0.0f, -std::cos(angleInner));

		// Angle corresponding to cutoffOuter
		float angleOuter = std::acos(cutoffOuter);
		JPH::Vec3 positionOuter(std::sin(angleOuter), 0.0f, -std::cos(angleOuter));

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
		std::uniform_real_distribution<float> angleDist(0.0f, std::numbers::pi_v<float> * 2.0f);
		std::uniform_real_distribution<float> azimuthDist(-std::numbers::pi_v<float>, std::numbers::pi_v<float>);
		std::uniform_real_distribution<float> gainDist(0.0f, 1.0f);

		const int numTests = 1000;
		static constexpr float toleranse = 1e-5f;

		// Random reference point
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();

		for (int i = 0; i < numTests; ++i)
		{
			// Random positions around the listener
			float azimuth = azimuthDist(rng);
			JPH::Vec3 position(std::sin(azimuth), 0.0f, -std::cos(azimuth));

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
		JPH::Mat44 referencePoint = GetForwardFacingIdentity();

		float branchedTime = 0.0f;
		{
			auto start = std::chrono::high_resolution_clock::now();

			for (int i = 0; i < numTests; ++i)
			{
				// Random positions around the listener
				const JPH::Vec3 position = JPH::Vec3::sRandom(rng) * 10.0f;

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
				const JPH::Vec3 position = JPH::Vec3::sRandom(rng) * 10.0f;

				// Random reference point
				JPH::Mat44 referencePoint = GetForwardFacingIdentity();

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
		float azimuth = DegreesToRadians(45.0f);
		JPH::Vec3 position(std::sin(azimuth), 0.0f, -std::cos(azimuth));

		JPH::Mat44 referencePoint = GetForwardFacingIdentity();
		AttenuationCone cone;
		cone.InnerAngle = DegreesToRadians(60.0f);
		cone.OuterAngle = DegreesToRadians(120.0f);
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

} // namespace JPL