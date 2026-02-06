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

#include "JPLSpatial/Math/MinimalVec2.h"
#include "JPLSpatial/Math/MinimalVec3.h"

#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Panning/VBAPEx.h"

#include <gtest/gtest.h>
#include <array>
#include <numbers>
#include <cmath>


namespace JPL
{
	class ComputeVBAPTest : public testing::Test
	{
	protected:
		using Vec3i = std::array<uint32, 3>;
		using Vec3 = MinimalVec3;
		static constexpr float epsilon = 1e-6f;

	protected:
		ComputeVBAPTest() = default;

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

		static std::vector<Vec3> GetSpeakerVectors(ChannelMap channelMap)
		{
			std::vector<Vec3> vectors;
			channelMap.ForEachChannel([&vectors](EChannel channel/*, uint32 index*/)
			{
				if (channel != EChannel::LFE)
					vectors.push_back(VBAPStandartTraits::GetChannelVector(channel));
			});
			return vectors;
		}

		// Helper for EXPECT_NEAR with Vec2
		static void ExpectNearVec2(const Vec2& actual,
								   const Vec2& expected,
								   float tolerance,
								   const std::string& message = "")
		{
			EXPECT_NEAR(actual.X, expected.X, tolerance) << message;
			EXPECT_NEAR(actual.Y, expected.Y, tolerance) << message;
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

	//==========================================================================
	/// 3D VBAP Tests
	//==========================================================================
	TEST_F(ComputeVBAPTest, VBAP3D_OrthogonalLoudspeakers_AxisAlignedSource)
	{
		// Loudspeakers forming an orthogonal basis (like Ambisonics)
		Vec3 l1 = Vec3(1.0f, 0.0f, 0.0f); // +X axis
		Vec3 l2 = Vec3(0.0f, 1.0f, 0.0f); // +Y axis
		Vec3 l3 = Vec3(0.0f, 0.0f, 1.0f); // +Z axis

		// Test Case 1: Source aligned with L1
		Vec3 p1 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 gains_p1 = ComputeVBAP(p1, l1, l2, l3); // Call ComputeVBAP for each test point
		ASSERT_TRUE(gains_p1.LengthSquared() > epsilon); // Should be non-zero for valid cases
		EXPECT_NEAR(gains_p1.X, 1.0f, epsilon);
		EXPECT_NEAR(gains_p1.Y, 0.0f, epsilon);
		EXPECT_NEAR(gains_p1.Z, 0.0f, epsilon);

		// Test Case 2: Source aligned with L2
		Vec3 p2 = Vec3(0.0f, 1.0f, 0.0f);
		Vec3 gains_p2 = ComputeVBAP(p2, l1, l2, l3);
		ASSERT_TRUE(gains_p2.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_p2.X, 0.0f, epsilon);
		EXPECT_NEAR(gains_p2.Y, 1.0f, epsilon);
		EXPECT_NEAR(gains_p2.Z, 0.0f, epsilon);

		// Test Case 3: Source aligned with L3
		Vec3 p3 = Vec3(0.0f, 0.0f, 1.0f);
		Vec3 gains_p3 = ComputeVBAP(p3, l1, l2, l3);
		ASSERT_TRUE(gains_p3.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_p3.X, 0.0f, epsilon);
		EXPECT_NEAR(gains_p3.Y, 0.0f, epsilon);
		EXPECT_NEAR(gains_p3.Z, 1.0f, epsilon);
	}

	TEST_F(ComputeVBAPTest, VBAP3D_OrthogonalLoudspeakers_DiagonalSource)
	{
		// Loudspeakers forming an orthogonal basis
		Vec3 l1 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 l2 = Vec3(0.0f, 1.0f, 0.0f);
		Vec3 l3 = Vec3(0.0f, 0.0f, 1.0f);

		// Test Case 4: Source in the middle of L1 and L2 (on XY plane)
		Vec3 p4 = Vec3(1.0f, 1.0f, 0.0f).Normalize(); // Should be (0.707, 0.707, 0)
		Vec3 gains_p4 = ComputeVBAP(p4, l1, l2, l3);
		ASSERT_TRUE(gains_p4.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_p4.X, p4.X, epsilon); // For orthogonal basis, g = p
		EXPECT_NEAR(gains_p4.Y, p4.Y, epsilon);
		EXPECT_NEAR(gains_p4.Z, p4.Z, epsilon);

		// Test Case 5: Source in the middle of the octant (equal contribution)
		Vec3 p5 = Vec3(1.0f, 1.0f, 1.0f).Normalize(); // Should be (0.577, 0.577, 0.577)
		Vec3 gains_p5 = ComputeVBAP(p5, l1, l2, l3); // <--- Call ComputeVBAP for p5
		ASSERT_TRUE(gains_p5.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_p5.X, p5.X, epsilon);
		EXPECT_NEAR(gains_p5.Y, p5.Y, epsilon);
		EXPECT_NEAR(gains_p5.Z, p5.Z, epsilon);
	}

	TEST_F(ComputeVBAPTest, VBAP3D_NonOrthogonalLoudspeakers_SimpleTriangle)
	{
		// Loudspeakers forming a non-orthogonal triangle in the XY plane (Z=0)
		// Original l1, l2, l3 result in a negative determinant (l1 . (l2 x l3) is negative).
		// To ensure positive gains for a source INSIDE the triangle, the loudspeaker triplet
		// passed to ComputeVBAP should form a right-handed system (positive determinant).
		// We'll use (l2_orig, l1_orig, l3_orig) order for ComputeVBAP calls.
		// The gains.X, gains.Y, gains.Z will then correspond to l2_orig, l1_orig, l3_orig respectively.
		Vec3 l1_orig = Vec3(-0.707f, 0.707f, 0.0f).Normalize(); // Approx 135 deg in XY
		Vec3 l2_orig = Vec3(0.707f, 0.707f, 0.0f).Normalize();  // Approx 45 deg in XY
		Vec3 l3_orig = Vec3(0.0f, 0.0f, 1.0f).Normalize();      // Straight up

		// Test Case 6: Source straight front (on XY plane, between l1_orig and l2_orig)
		Vec3 p6 = Vec3(0.0f, 1.0f, 0.0f).Normalize();
		// Pass l2_orig, l1_orig, l3_orig to get positive determinant
		Vec3 gains_p6 = ComputeVBAP(p6, l2_orig, l1_orig, l3_orig);
		ASSERT_TRUE(gains_p6.LengthSquared() > epsilon);
		// Expected: gains.X (for l2_orig) and gains.Y (for l1_orig) should be equal and positive.
		// gains.Z (for l3_orig) should be 0 as p6 is in XY plane.
		EXPECT_NEAR(gains_p6.X, 0.70710678f, epsilon); // Gain for l2_orig
		EXPECT_NEAR(gains_p6.Y, 0.70710678f, epsilon); // Gain for l1_orig
		EXPECT_NEAR(gains_p6.Z, 0.0f, epsilon);      // Gain for l3_orig

		// Test Case 7: Source straight up (aligned with l3_orig)
		Vec3 p7 = Vec3(0.0f, 0.0f, 1.0f).Normalize();
		// Pass l2_orig, l1_orig, l3_orig to get positive determinant
		Vec3 gains_p7 = ComputeVBAP(p7, l2_orig, l1_orig, l3_orig);
		ASSERT_TRUE(gains_p7.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_p7.X, 0.0f, epsilon);
		EXPECT_NEAR(gains_p7.Y, 0.0f, epsilon);
		EXPECT_NEAR(gains_p7.Z, 1.0f, epsilon);

		// Test Case 8: Source in the middle of the triangle (e.g., (0, 0.5, 0.5) normalized)
		Vec3 p8 = Vec3(0.0f, 0.5f, 0.5f).Normalize(); // Example point within the triangle
		// Pass l2_orig, l1_orig, l3_orig to get positive determinant
		Vec3 gains_p8 = ComputeVBAP(p8, l2_orig, l1_orig, l3_orig);
		ASSERT_TRUE(gains_p8.LengthSquared() > epsilon);
		// Verify that gains are positive (since determinant is now positive)
		EXPECT_GT(gains_p8.X, 0.0f); // Gain for l2_orig
		EXPECT_GT(gains_p8.Y, 0.0f); // Gain for l1_orig
		EXPECT_GT(gains_p8.Z, 0.0f); // Gain for l3_orig
		// Due to symmetry, gains for l2_orig and l1_orig should be equal
		EXPECT_NEAR(gains_p8.X, gains_p8.Y, epsilon);


		// --- Test cases using l_a, l_b, l_c ---
		// l_a = (1,0,0), l_b = (0,1,0), l_c = (0.577, 0.577, 0.577) (normalized)
		// The triplet (l_a, l_b, l_c) results in a negative determinant.
		// To achieve positive gains for internal points, we need to order them for a positive determinant.
		// Order (l_b, l_c, l_a) yields a positive determinant (l_b . (l_c x l_a) > 0).
		Vec3 l_a_set = Vec3(1.0f, 0.0f, 0.0f).Normalize();
		Vec3 l_b_set = Vec3(0.0f, 1.0f, 0.0f).Normalize();
		Vec3 l_c_set = Vec3(0.5f, 0.5f, 0.5f).Normalize(); // A loudspeaker not on an axis

		// Source in the middle of l_a_set and l_b_set (XY plane)
		Vec3 p_ab = (l_a_set + l_b_set).Normalize();
		// Pass l_b_set, l_c_set, l_a_set to ComputeVBAP
		Vec3 gains_abc_p_ab = ComputeVBAP(p_ab, l_b_set, l_c_set, l_a_set);
		ASSERT_TRUE(gains_abc_p_ab.LengthSquared() > epsilon);
		// Gains for l_b_set (X) and l_a_set (Z) should be positive, l_c_set (Y) should be 0 as p_ab is on XY plane.
		EXPECT_NEAR(gains_abc_p_ab.Y, 0.0f, epsilon); // Gain for l_c_set
		EXPECT_GT(gains_abc_p_ab.X, 0.0f);           // Gain for l_b_set
		EXPECT_GT(gains_abc_p_ab.Z, 0.0f);           // Gain for l_a_set

		// Source aligned with l_c_set
		Vec3 p_c_aligned = l_c_set;
		// Pass l_b_set, l_c_set, l_a_set to ComputeVBAP
		Vec3 gains_abc_p_c = ComputeVBAP(p_c_aligned, l_b_set, l_c_set, l_a_set);
		ASSERT_TRUE(gains_abc_p_c.LengthSquared() > epsilon);
		EXPECT_NEAR(gains_abc_p_c.X, 0.0f, epsilon); // Gain for l_b_set
		EXPECT_NEAR(gains_abc_p_c.Z, 0.0f, epsilon); // Gain for l_a_set
		EXPECT_NEAR(gains_abc_p_c.Y, 1.0f, epsilon); // Gain for l_c_set (should be 1.0 when aligned)
	}

	TEST_F(ComputeVBAPTest, VBAP3D_NonOrthogonalLoudspeakers_FrontPlane)
	{
		Vec3 l1_orig = Vec3(-0.707f, 0.707f, 0.0f).Normalize(); // Top-left
		Vec3 l2_orig = Vec3(0.707f, 0.707f, 0.0f).Normalize();  // Top-right
		Vec3 l3_orig = Vec3(0.0f, 0.0f, -1.0f).Normalize();     // Front center

		// Source close to the front-center speaker, slightly up
		Vec3 p = Vec3(0.0f, 0.1f, -1.0f).Normalize(); // Example point within the triangle
		Vec3 gains = ComputeVBAP(p, l1_orig, l2_orig, l3_orig);
		ASSERT_TRUE(gains.LengthSquared() > epsilon);

		// Expecting gains to be normalized
		EXPECT_NEAR(gains.Length(), 1.0f, epsilon);

		// Verify that gains are positive (since determinant is now positive)
		EXPECT_GT(gains.X, 0.0f); // Gain for l1_orig
		EXPECT_GT(gains.Y, 0.0f); // Gain for l2_orig
		EXPECT_GT(gains.Z, 0.0f); // Gain for l3_orig
		// Due to symmetry, gains for l2_orig and l1_orig should be equal
		EXPECT_NEAR(gains.X, gains.Y, epsilon);

		// Front channel gain should be the loudest
		EXPECT_GT(gains.Z, gains.X);
	}

	TEST_F(ComputeVBAPTest, VBAP3D_CoplanarLoudspeakers_ErrorHandling)
	{
		// Test Case 1: Loudspeakers that are coplanar (e.g., all on the XY plane)
		Vec3 l1_coplanar = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 l2_coplanar = Vec3(0.0f, 1.0f, 0.0f);
		Vec3 l3_coplanar = Vec3(0.707f, 0.707f, 0.0f); // Also on XY plane

		Vec3 p_coplanar = Vec3(0.0f, 0.0f, 1.0f); // Source pointing out of the plane

		Vec3 gains_coplanar = ComputeVBAP(p_coplanar, l1_coplanar, l2_coplanar, l3_coplanar);
		// In coplanar case, ComputeVBAP should return Vec3(0,0,0) due to determinant being near zero.
		// So LengthSquared() should be near 0.0f.
		ASSERT_NEAR(gains_coplanar.LengthSquared(), 0.0f, epsilon); // <--- FIXED ASSERTION
		EXPECT_NEAR(gains_coplanar.X, 0.0f, epsilon);
		EXPECT_NEAR(gains_coplanar.Y, 0.0f, epsilon);
		EXPECT_NEAR(gains_coplanar.Z, 0.0f, epsilon);


		// Test Case 2: Loudspeakers that are collinear (a more degenerate coplanar case)
		Vec3 l4_collinear = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 l5_collinear = Vec3(0.5f, 0.0f, 0.0f);
		Vec3 l6_collinear = Vec3(-1.0f, 0.0f, 0.0f);

		Vec3 p2_collinear = Vec3(0.0f, 1.0f, 0.0f);
		Vec3 gains_collinear = ComputeVBAP(p2_collinear, l4_collinear, l5_collinear, l6_collinear); // <--- FIXED PARAMETERS
		ASSERT_NEAR(gains_collinear.LengthSquared(), 0.0f, epsilon); // <--- FIXED ASSERTION
		EXPECT_NEAR(gains_collinear.X, 0.0f, epsilon);
		EXPECT_NEAR(gains_collinear.Y, 0.0f, epsilon);
		EXPECT_NEAR(gains_collinear.Z, 0.0f, epsilon);
	}

	TEST_F(ComputeVBAPTest, VBAP3D_ScalingAndNormalization_Equation19)
	{
		// This test demonstrates how to apply Equation 19 (scaling) and verifies it.
		// It's not directly testing Equation 18, but shows the full VBAP process.

		// Loudspeakers forming an orthogonal basis
		Vec3 l1 = Vec3(1.0f, 0.0f, 0.0f);
		Vec3 l2 = Vec3(0.0f, 1.0f, 0.0f);
		Vec3 l3 = Vec3(0.0f, 0.0f, 1.0f);

		// Source in the middle of the octant (equal contribution)
		Vec3 p = Vec3(1.0f, 1.0f, 1.0f).Normalize(); // (0.577, 0.577, 0.577)

		Vec3 gains_unscaled = ComputeVBAP(p, l1, l2, l3);
		ASSERT_TRUE(gains_unscaled.LengthSquared() > epsilon);

		// Verify unscaled gains (should be equal to p components for orthogonal basis)
		EXPECT_NEAR(gains_unscaled.X, p.X, epsilon);
		EXPECT_NEAR(gains_unscaled.Y, p.Y, epsilon);
		EXPECT_NEAR(gains_unscaled.Z, p.Z, epsilon);

		// Now apply Equation 19: g_scaled = (sqrt(C) * g) / sqrt(g1^2 + g2^2 + g3^2)
		// Let C = 1.0 (constant sound power)
		const float C = 1.0f;
		float sum_of_squares_unscaled = gains_unscaled.LengthSquared();
		float norm_factor = std::sqrt(sum_of_squares_unscaled);

		Vec3 gains_scaled;

		// Check for norm_factor being zero to prevent division by zero in case of zero gains
		if (norm_factor > epsilon)
		{
			gains_scaled.X = (std::sqrt(C) * gains_unscaled.X) / norm_factor;
			gains_scaled.Y = (std::sqrt(C) * gains_unscaled.Y) / norm_factor;
			gains_scaled.Z = (std::sqrt(C) * gains_unscaled.Z) / norm_factor;
		}
		else
		{
			gains_scaled = Vec3(0.0f, 0.0f, 0.0f); // Set to zero if unscaled gains are zero
		}


		// For C=1, the scaled gains should form a unit vector (sum of squares = 1)
		float sum_of_squares_scaled = gains_scaled.LengthSquared();
		EXPECT_NEAR(sum_of_squares_scaled, C, epsilon);

		// Also, since p was already a unit vector and basis was orthogonal,
		// the unscaled gains already sum to 1 squared. So scaled should be same as unscaled.
		EXPECT_NEAR(gains_scaled.X, p.X, epsilon);
		EXPECT_NEAR(gains_scaled.Y, p.Y, epsilon);
		EXPECT_NEAR(gains_scaled.Z, p.Z, epsilon);

		// Test with a non-orthogonal case and check scaling
		// Use the reordered L's to ensure positive gains for scaling test.
		Vec3 l_a_scaled = Vec3(1.0f, 0.0f, 0.0f).Normalize();
		Vec3 l_b_scaled = Vec3(0.0f, 1.0f, 0.0f).Normalize();
		Vec3 l_c_scaled = Vec3(0.5f, 0.5f, 0.5f).Normalize(); // A loudspeaker not on an axis

		Vec3 p_test = Vec3(0.2f, 0.8f, 0.1f).Normalize(); // Some arbitrary source
		// Pass l_b_scaled, l_c_scaled, l_a_scaled to ComputeVBAP to ensure positive determinant.
		Vec3 gains_unscaled_non_ortho = ComputeVBAP(p_test, l_b_scaled, l_c_scaled, l_a_scaled);
		ASSERT_TRUE(gains_unscaled_non_ortho.LengthSquared() > epsilon);

		float sum_of_squares_unscaled_non_ortho = gains_unscaled_non_ortho.LengthSquared();
		float norm_factor_non_ortho = std::sqrt(sum_of_squares_unscaled_non_ortho);

		Vec3 gains_scaled_non_ortho;
		if (norm_factor_non_ortho > epsilon)
		{
			gains_scaled_non_ortho.X = (std::sqrt(C) * gains_unscaled_non_ortho.X) / norm_factor_non_ortho;
			gains_scaled_non_ortho.Y = (std::sqrt(C) * gains_unscaled_non_ortho.Y) / norm_factor_non_ortho;
			gains_scaled_non_ortho.Z = (std::sqrt(C) * gains_unscaled_non_ortho.Z) / norm_factor_non_ortho;
		}
		else
		{
			gains_scaled_non_ortho = Vec3(0.0f, 0.0f, 0.0f);
		}

		float sum_of_squares_scaled_non_ortho = gains_scaled_non_ortho.LengthSquared();
		EXPECT_NEAR(sum_of_squares_scaled_non_ortho, C, epsilon);
	}

	//==========================================================================
	/// 2D VBAP Tests
	//==========================================================================
	TEST_F(ComputeVBAPTest, VBAP2D_OrthogonalLoudspeakers_AxisAlignedSource)
	{
		// Loudspeakers at 0 and 90 degrees (orthogonal basis)
		Vec2 l1 = Vec2(1.0f, 0.0f); // +X axis
		Vec2 l2 = Vec2(0.0f, 1.0f); // +Y axis

		// Test Case 1: Source aligned with L1
		Vec2 p1 = Vec2(1.0f, 0.0f).Normalize();
		Vec2 gains1 = ComputeVBAP(p1, l1, l2);
		ExpectNearVec2(gains1, Vec2(1.0f, 0.0f), epsilon, "Source aligned with L1");

		// Test Case 2: Source aligned with L2
		Vec2 p2 = Vec2(0.0f, 1.0f).Normalize();
		Vec2 gains2 = ComputeVBAP(p2, l1, l2);
		ExpectNearVec2(gains2, Vec2(0.0f, 1.0f), epsilon, "Source aligned with L2");
	}

	TEST_F(ComputeVBAPTest, VBAP2D_OrthogonalLoudspeakers_DiagonalSource)
	{
		// Loudspeakers at 0 and 90 degrees (orthogonal basis)
		Vec2 l1 = Vec2(1.0f, 0.0f); // +X axis
		Vec2 l2 = Vec2(0.0f, 1.0f); // +Y axis

		// Test Case 3: Source exactly between L1 and L2 (45 degrees)
		Vec2 p3 = Vec2(1.0f, 1.0f).Normalize(); // Should be (0.707, 0.707)
		Vec2 gains3 = ComputeVBAP(p3, l1, l2);
		// For an orthogonal basis, unscaled gains are simply the source vector components
		ExpectNearVec2(gains3, p3, epsilon, "Source between L1 and L2");

		// Test Case 4: Source at 30 degrees (closer to L1)
		Vec2 p4 = Vec2(std::cos(std::numbers::pi_v<float> / 180.0f * 30.0f), std::sin(std::numbers::pi_v<float> / 180.0f * 30.0f)).Normalize();
		Vec2 gains4 = ComputeVBAP(p4, l1, l2);
		ExpectNearVec2(gains4, p4, epsilon, "Source at 30 degrees");
	}

	TEST_F(ComputeVBAPTest, VBAP2D_NonOrthogonalLoudspeakers_SimpleArc)
	{
		// Loudspeakers at -30 and +30 degrees
		Vec2 l1 = Vec2(std::cos(std::numbers::pi_v<float> / 180.0f * -30.0f), std::sin(std::numbers::pi_v<float> / 180.0f * -30.0f)).Normalize();
		Vec2 l2 = Vec2(std::cos(std::numbers::pi_v<float> / 180.0f * 30.0f), std::sin(std::numbers::pi_v<float> / 180.0f * 30.0f)).Normalize();

		// Test Case 5: Source straight ahead (0 degrees), exactly between L1 and L2
		Vec2 p5 = Vec2(1.0f, 0.0f).Normalize(); // (1, 0)
		Vec2 gains5 = ComputeVBAP(p5, l1, l2);
		// Expected gains for (1,0) source with speakers at +/-30 deg:
		// det = l1.cross(l2) = cos(-30)sin(30) - sin(-30)cos(30) = sin(30 - (-30)) = sin(60) = sqrt(3)/2
		// g1 = p.cross(l2) / det = (1*sin(30) - 0*cos(30)) / sin(60) = sin(30)/sin(60) = (1/2) / (sqrt(3)/2) = 1/sqrt(3)
		// g2 = l1.cross(p) / det = (cos(-30)*0 - sin(-30)*1) / sin(60) = -sin(-30)/sin(60) = sin(30)/sin(60) = 1/sqrt(3)
		float expected_gain = 1.0f / std::sqrt(3.0f); // approx 0.57735
		ExpectNearVec2(gains5, Vec2(expected_gain, expected_gain), epsilon, "Source at 0 degrees");

		// Test Case 6: Source aligned with L1
		Vec2 p6 = l1;
		Vec2 gains6 = ComputeVBAP(p6, l1, l2);
		// When source is aligned with a speaker, its gain should be 1, and others 0.
		// This is a key property of VBAP.
		ExpectNearVec2(gains6, Vec2(1.0f, 0.0f), epsilon, "Source aligned with L1");

		// Test Case 7: Source aligned with L2
		Vec2 p7 = l2;
		Vec2 gains7 = ComputeVBAP(p7, l1, l2);
		ExpectNearVec2(gains7, Vec2(0.0f, 1.0f), epsilon, "Source aligned with L2");

		// Test Case 8: Source slightly outside the span (e.g., -45 degrees)
		// This should result in one positive and one negative gain factor.
		Vec2 p8 = Vec2(std::cos(std::numbers::pi_v<float> / 180.0f * -45.0f), std::sin(std::numbers::pi_v<float> / 180.0f * -45.0f)).Normalize();
		Vec2 gains8 = ComputeVBAP(p8, l1, l2);
		EXPECT_GT(gains8.X, 0.0f) << "Gain for L1 should be positive";
		EXPECT_LT(gains8.Y, 0.0f) << "Gain for L2 should be negative (source outside span)";
	}

	TEST_F(ComputeVBAPTest, VBAP2D_CollinearLoudspeakers_ErrorHandling)
	{
		// Loudspeakers that are collinear (e.g., all on the X axis)
		Vec2 l1 = Vec2(1.0f, 0.0f).Normalize();
		Vec2 l2 = Vec2(-1.0f, 0.0f).Normalize(); // Opposite direction, still collinear

		Vec2 p = Vec2(0.0f, 1.0f).Normalize(); // Source pointing perpendicular to the line

		Vec2 gains = ComputeVBAP(p, l1, l2);
		// For collinear speakers, ComputeVBAP should return {0,0}
		ExpectNearVec2(gains, Vec2(0.0f, 0.0f), epsilon, "Collinear speakers, perpendicular source");

		// Another collinear case (same direction)
		Vec2 l3 = Vec2(0.5f, 0.0f).Normalize();
		Vec2 l4 = Vec2(1.0f, 0.0f).Normalize();
		Vec2 p2 = Vec2(0.0f, 1.0f).Normalize();
		Vec2 gains2 = ComputeVBAP(p2, l3, l4);
		ExpectNearVec2(gains2, Vec2(0.0f, 0.0f), epsilon, "Collinear speakers, same direction");
	}

} // namespace JPL