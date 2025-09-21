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

#include <JPLSpatial/Math/Math.h>
#include <JPLSpatial/Math/MinimalVec3.h>
#include <JPLSpatial/Math/Vec3Math.h>
#include <JPLSpatial/Math/MinimalBasis.h>
#include <JPLSpatial/Math/MinimalQuat.h>
#include <gtest/gtest.h>

#include <vector>
#include <utility>
#include <format>

namespace JPL
{
    class RotationTest : public ::testing::Test
    {
    protected:
        using Vec3 = MinimalVec3;

        static constexpr Vec3 xAxis{ 1.0f, 0.0f, 0.0f };
        static constexpr Vec3 yAxis{ 0.0f, 1.0f, 0.0f };
        static constexpr Vec3 zAxis{ 0.0f, 0.0f, 1.0f };

        // Each axis in both directions
        const std::vector<Vec3> testAxes
        {
            xAxis,
            yAxis,
            zAxis,

            -xAxis,
            -yAxis,
            -zAxis,
        };

        // All possible axis pairs
        const std::vector<std::pair<Vec3, Vec3>> axisPairs
        {
            { xAxis, yAxis },
            { xAxis, zAxis },

            { xAxis, -yAxis },
            { xAxis, -zAxis },

            { -xAxis, yAxis },
            { -xAxis, zAxis },

            { zAxis, yAxis },
            { zAxis, xAxis },

            { zAxis, -yAxis },
            { zAxis, -xAxis },

            { -zAxis, yAxis },
            { -zAxis, xAxis },

            { yAxis, zAxis },
            { yAxis, xAxis },

            { yAxis, -zAxis },
            { yAxis, -xAxis },

            { -yAxis, zAxis },
            { -yAxis, xAxis },
        };

        // Test 45, 90, 180, 360 degree rotation in both directions (in rads)
        const std::vector<float> testAngles
        {
            JPL_HALF_PI * 0.5f,
            JPL_HALF_PI,
            JPL_PI,
            JPL_TWO_PI,

            -JPL_HALF_PI * 0.5f,
            -JPL_HALF_PI,
            -JPL_PI,
            -JPL_TWO_PI,
        };

        // forward axis
        static constexpr Vec3 forward{ 0.0f, 0.0f, -1.0f };
        // up axis
        static constexpr Vec3 up{ 0.0f, 1.0f, 0.0f };
    };

    class SlerpTest : public ::testing::Test
    {
    protected:
        const float FLOAT_TOLERANCE = 1e-4f;
        using Vec3f = MinimalVec3;
    };

    TEST_F(SlerpTest, IdentityAndEndpointTests)
    {
        Vec3f v0(1.0f, 0.0f, 0.0f);
        Vec3f v1(0.0f, 1.0f, 0.0f); // Perpendicular to v0

        // Test t = 0.0
        Vec3f result0 = Math::Slerp(v0, v1, 0.0f);
        EXPECT_NEAR(result0.X, v0.X, FLOAT_TOLERANCE);
        EXPECT_NEAR(result0.Y, v0.Y, FLOAT_TOLERANCE);
        EXPECT_NEAR(result0.Z, v0.Z, FLOAT_TOLERANCE);
        EXPECT_NEAR(result0.Length(), 1.0f, FLOAT_TOLERANCE);

        // Test t = 1.0
        Vec3f result1 = Math::Slerp(v0, v1, 1.0f);
        EXPECT_NEAR(result1.X, v1.X, FLOAT_TOLERANCE);
        EXPECT_NEAR(result1.Y, v1.Y, FLOAT_TOLERANCE);
        EXPECT_NEAR(result1.Z, v1.Z, FLOAT_TOLERANCE);
        EXPECT_NEAR(result1.Length(), 1.0f, FLOAT_TOLERANCE);
    }

    TEST_F(SlerpTest, HalfwayPointBetweenPerpendicularVectors)
    {
        Vec3f v0(1.0f, 0.0f, 0.0f); // +X
        Vec3f v1(0.0f, 0.0f, 1.0f); // +Z (90 degrees from v0)

        // Expected halfway point (45 degrees between X and Z, normalized)
        float expected_val = std::sqrt(2.0f) / 2.0f; // ~0.7071
        Vec3f expected_result(expected_val, 0.0f, expected_val);

        Vec3f result = Math::Slerp(v0, v1, 0.5f);

        EXPECT_NEAR(result.X, expected_result.X, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Y, expected_result.Y, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Z, expected_result.Z, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Length(), 1.0f, FLOAT_TOLERANCE);
    }

    TEST_F(SlerpTest, NearlyParallelVectorsFallback)
    {
        Vec3f v0(0.0f, 0.0f, 1.0f); // +Z
        // A vector very slightly off +Z
        Vec3f v1(0.001f, 0.002f, 1.0f);
        v1 = v1.Normalize(); // Ensure unit length

        Vec3f result = Math::Slerp(v0, v1, 0.5f);

        // The result should be roughly halfway between v0 and v1
        // And very close to (0,0,1) with small X,Y components
        EXPECT_NEAR(result.X, (0.0f + v1.X) / 2.0f, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Y, (0.0f + v1.Y) / 2.0f, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Z, (1.0f + v1.Z) / 2.0f, FLOAT_TOLERANCE);
        EXPECT_NEAR(result.Length(), 1.0f, FLOAT_TOLERANCE);
    }

    TEST_F(SlerpTest, OppositeVectorsSpecialCase)
    {
        Vec3f v0(0.0f, 0.0f, 1.0f); // +Z
        Vec3f v1(0.0f, 0.0f, -1.0f); // -Z (perfectly opposite)

        // At t=0.5, the result should be perpendicular to v0.
        // Given v0=(0,0,1), the chosen rotation axis will be (0,1,0).
        // Rotating (0,0,1) by 90 degrees around (0,1,0) leads to (1,0,0).
        Vec3f result_half = Math::Slerp(v0, v1, 0.5f);
        EXPECT_NEAR(result_half.Length(), 1.0f, FLOAT_TOLERANCE);
        EXPECT_NEAR(result_half.X, 1.0f, FLOAT_TOLERANCE); // Expected X
        EXPECT_NEAR(result_half.Y, 0.0f, FLOAT_TOLERANCE);  // Expected Y
        EXPECT_NEAR(result_half.Z, 0.0f, FLOAT_TOLERANCE);  // Expected Z (on XY plane)

        // Check intermediate point for opposite vectors, e.g., t=0.25 (45 degrees rotation)
        // Rotating (0,0,1) by 45 degrees around (0,1,0) leads to (sin(45), 0, cos(45))
        Vec3f result_qtr = Math::Slerp(v0, v1, 0.25f);
        float expected_qtr_val = std::sqrt(2.0f) / 2.0f; // cos(45 deg) or sin(45 deg)
        EXPECT_NEAR(result_qtr.Length(), 1.0f, FLOAT_TOLERANCE);
        EXPECT_NEAR(result_qtr.X, expected_qtr_val, FLOAT_TOLERANCE); // Expected X
        EXPECT_NEAR(result_qtr.Y, 0.0f, FLOAT_TOLERANCE);             // Expected Y
        EXPECT_NEAR(result_qtr.Z, expected_qtr_val, FLOAT_TOLERANCE); // Expected Z
    }

    TEST_F(SlerpTest, GeneralInterpolationBetweenArbitraryVectors)
    {
        Vec3f v0(0.5f, 0.5f, 0.5f); // Example vector 1
        v0 = v0.Normalize(); // Ensure unit length
        Vec3f v1(-0.5f, 0.5f, 0.5f); // Example vector 2
        v1 = v1.Normalize(); // Ensure unit length

        // Test t = 0.25 (arbitrary intermediate point)
        Vec3f result_25 = Math::Slerp(v0, v1, 0.25f);
        EXPECT_NEAR(result_25.Length(), 1.0f, FLOAT_TOLERANCE);

        // Test t = 0.75 (arbitrary intermediate point)
        Vec3f result_75 = Math::Slerp(v0, v1, 0.75f);
        EXPECT_NEAR(result_75.Length(), 1.0f, FLOAT_TOLERANCE);

        // Verify properties of spherical interpolation (e.g., dot products should make sense)
        // The interpolated vector should be closer to v0 for small t and closer to v1 for large t.
        // The angle between v0 and result_25 should be smaller than between result_25 and v1.
        float angle_v0_v1 = std::acos(DotProduct(v0, v1));
        float angle_v0_result25 = std::acos(DotProduct(v0, result_25));
        float angle_result25_v1 = std::acos(DotProduct(result_25, v1));

        EXPECT_NEAR(angle_v0_result25, 0.25f * angle_v0_v1, FLOAT_TOLERANCE);
        EXPECT_NEAR(angle_result25_v1, 0.75f * angle_v0_v1, FLOAT_TOLERANCE);
    }

    TEST_F(RotationTest, Quat_Basis_IdentityRotationIsEquivalent)
    {
        static constexpr float tolerance = 1e-6f;

        for (const Vec3& rotationAxis : testAxes)
        {
            SCOPED_TRACE(std::format("Axis: {}", (std::stringstream() << rotationAxis).str()));

            const auto identityBasis = Basis<Vec3>::Identity();
            const auto identityQuat = Quat<Vec3>::Identity();

            // Basis and Quat identity rotated axis should not change
            const Vec3 axisBasisTransformed = identityBasis.Transform(rotationAxis);
            const Vec3 axisQuatTransformed = identityQuat.Rotate(rotationAxis);
            EXPECT_TRUE(Math::IsNearlyEqual(axisBasisTransformed, axisQuatTransformed, tolerance))
                << "fwdBasisTransformed: " << axisBasisTransformed << " \n axisQuatTransformed: " << axisQuatTransformed;
        }
    }

    TEST_F(RotationTest, Quat_Basis_QuatToBasisRotation_RotationIsEquivalent)
    {
        static constexpr float tolerance = 1e-6f;

        for (const Vec3& rotationAxis : testAxes)
        {
            for (float rotationAngle : testAngles)
            {
                SCOPED_TRACE(std::format("Axis: {}, angle: {}", (std::stringstream() << rotationAxis).str(), Math::ToDegrees(rotationAngle)));

                const auto rawBasis = Basis<Vec3>::Rotation(rotationAxis, rotationAngle);
                const auto rawQuat = Quat<Vec3>::Rotation(rotationAxis, rotationAngle);
                const auto basisFromQuat = rawQuat.ToBasis();

                // Basis from Quat of the same rotation should be the same as Basis from that rotation
                EXPECT_TRUE(Math::IsNearlyEqual(rawBasis.X, basisFromQuat.X, tolerance)) << "     rawBasis.X: " << rawBasis.X << " \nbasisFromQuat.X: " << basisFromQuat.X;
                EXPECT_TRUE(Math::IsNearlyEqual(rawBasis.Y, basisFromQuat.Y, tolerance)) << "     rawBasis.Y: " << rawBasis.Y << " \nbasisFromQuat.Y: " << basisFromQuat.Y;
                EXPECT_TRUE(Math::IsNearlyEqual(rawBasis.Z, basisFromQuat.Z, tolerance)) << "     rawBasis.Z: " << rawBasis.Z << " \nbasisFromQuat.Z: " << basisFromQuat.Z;

                // Basis and Quat rotated forward vector should be equal
                const Vec3 fwdBasisTransformed = rawBasis.Transform(forward);
                const Vec3 fwdQuatTransformed = rawQuat.Rotate(forward);
                EXPECT_TRUE(Math::IsNearlyEqual(fwdBasisTransformed, fwdQuatTransformed, tolerance))
                    << "fwdBasisTransformed: " << fwdBasisTransformed << " \n fwdQuatTransformed: " << fwdQuatTransformed;

                // Basis and Quat rotated up vector should be equal
                const Vec3 upBasisTransformed = rawBasis.Transform(up);
                const Vec3 upQuatTransformed = rawQuat.Rotate(up);
                EXPECT_TRUE(Math::IsNearlyEqual(upBasisTransformed, upQuatTransformed, tolerance))
                    << "upBasisTransformed: " << upBasisTransformed << " \n upQuatTransformed: " << upQuatTransformed;
            }
        }
    }

    TEST_F(RotationTest, Quat_Basis_FromUpAndForwardEquivalent)
    {
        static constexpr float tolerance = 1e-6f;

        for (const auto& [upAxis, forwardAxis] : axisPairs)
        {
            SCOPED_TRACE(std::format("Up-Axis: {}, Forward-Axis: {}",
                                     (std::stringstream() << upAxis).str(), (std::stringstream() << forwardAxis).str()));

            const auto basis = Basis<Vec3>::FromUpAndForward(upAxis, forwardAxis);

            EXPECT_GT(DotProduct(CrossProduct(basis.X, basis.Y), basis.Z), 0)
                << "Constructed bases is not in right-headed coordinate system.";

            const auto quat = Quat<Vec3>::FromUpAndForward(upAxis, forwardAxis);

            // Basis and Quat rotated forward vector should be equal
            const Vec3 fwdBasisTransformed = basis.Transform(forward);
            const Vec3 fwdQuatTransformed = quat.Rotate(forward);
            EXPECT_TRUE(Math::IsNearlyEqual(fwdBasisTransformed, fwdQuatTransformed, tolerance))
                << "fwdBasisTransformed: " << fwdBasisTransformed << " \n fwdQuatTransformed: " << fwdQuatTransformed;

            // Basis and Quat rotated up vector should be equal
            const Vec3 upBasisTransformed = basis.Transform(up);
            const Vec3 upQuatTransformed = quat.Rotate(up);
            EXPECT_TRUE(Math::IsNearlyEqual(upBasisTransformed, upQuatTransformed, tolerance))
                << "upBasisTransformed: " << upBasisTransformed << " \n upQuatTransformed: " << upQuatTransformed;
        }
    }

    TEST_F(RotationTest, Quat_Basis_InverseTransformIsEquivalent)
    {
        static constexpr float tolerance = 1e-6f;

        for (const Vec3& rotationAxis : testAxes)
        {
            for (float rotationAngle : testAngles)
            {
                SCOPED_TRACE(std::format("Axis: {}, angle: {}", (std::stringstream() << rotationAxis).str(), Math::ToDegrees(rotationAngle)));

                const auto rawBasis = Basis<Vec3>::Rotation(rotationAxis, rotationAngle);
                const auto rawQuat = Quat<Vec3>::Rotation(rotationAxis, rotationAngle);

                // Basis and Quat rotated forward vector should be equal
                const Vec3 fwdBasisInverseTransformed = rawBasis.InverseTransform(forward);
                const Vec3 fwdQuatInverseTransformed = rawQuat.Conjugated().Rotate(forward);
                EXPECT_TRUE(Math::IsNearlyEqual(fwdBasisInverseTransformed, fwdQuatInverseTransformed, tolerance))
                    << "fwdBasisInverseTransformed: " << fwdBasisInverseTransformed << " \n fwdQuatInverseTransformed: " << fwdQuatInverseTransformed;

                // Basis and Quat rotated up vector should be equal
                const Vec3 upBasisInverseTransformed = rawBasis.InverseTransform(up);
                const Vec3 upQuatInverseTransformed = rawQuat.Conjugated().Rotate(up);
                EXPECT_TRUE(Math::IsNearlyEqual(upBasisInverseTransformed, upQuatInverseTransformed, tolerance))
                    << "upBasisInverseTransformed: " << upBasisInverseTransformed << " \n upQuatInverseTransformed: " << upQuatInverseTransformed;
            }
        }
    }

    TEST_F(RotationTest, Quat_Basis_TransformCompositionIsEquivalent)
    {
        static constexpr float tolerance = 1e-6f;

        for (const auto& [upAxisA, forwardAxisA] : axisPairs)
        {
            SCOPED_TRACE(std::format("Up-Axis A: {}, Forward-Axis A: {}",
                                     (std::stringstream() << upAxisA).str(), (std::stringstream() << forwardAxisA).str()));

            const auto basisA = Basis<Vec3>::FromUpAndForward(upAxisA, forwardAxisA);
            const auto quatA = Quat<Vec3>::FromUpAndForward(upAxisA, forwardAxisA);

            for (const auto& [upAxisB, forwardAxisB] : axisPairs)
            {
                SCOPED_TRACE(std::format("Up-Axis B: {}, Forward-Axis B: {}",
                                         (std::stringstream() << upAxisB).str(), (std::stringstream() << forwardAxisB).str()));

                const auto basisB = Basis<Vec3>::FromUpAndForward(upAxisB, forwardAxisB);
                const auto quatB = Quat<Vec3>::FromUpAndForward(upAxisB, forwardAxisB);
                
                {
                    SCOPED_TRACE("Transform composition");

                    const auto basisComp = basisB.Transform(basisA);
                    const auto quatComp = quatB * quatA;

                    // Basis and Quat rotated forward vector should be equal
                    const Vec3 fwdBasisTransformed = basisComp.Transform(forward);
                    const Vec3 fwdQuatTransformed = quatComp.Rotate(forward);

                    EXPECT_TRUE(Math::IsNearlyEqual(fwdBasisTransformed, fwdQuatTransformed, tolerance))
                        << "fwdBasisTransformed: " << fwdBasisTransformed << " \n fwdQuatTransformed: " << fwdQuatTransformed;

                    // Basis and Quat rotated up vector should be equal
                    const Vec3 upBasisTransformed = basisComp.Transform(up);
                    const Vec3 upQuatTransformed = quatComp.Rotate(up);
                    EXPECT_TRUE(Math::IsNearlyEqual(upBasisTransformed, upQuatTransformed, tolerance))
                        << "upBasisTransformed: " << upBasisTransformed << " \n upQuatTransformed: " << upQuatTransformed;
                }
                {
                    SCOPED_TRACE("Inverse Transform composition");

                    const auto basisCompInv = basisB.InverseTransform(basisA);
                    const auto quatCompInv = quatB.Conjugated() * quatA;

                    // Basis and Quat rotated forward vector should be equal
                    const Vec3 fwdBasisTransformed = basisCompInv.Transform(forward);
                    const Vec3 fwdQuatTransformed = quatCompInv.Rotate(forward);

                    EXPECT_TRUE(Math::IsNearlyEqual(fwdBasisTransformed, fwdQuatTransformed, tolerance))
                        << "fwdBasisTransformed: " << fwdBasisTransformed << " \n fwdQuatTransformed: " << fwdQuatTransformed;

                    // Basis and Quat rotated up vector should be equal
                    const Vec3 upBasisTransformed = basisCompInv.Transform(up);
                    const Vec3 upQuatTransformed = quatCompInv.Rotate(up);
                    EXPECT_TRUE(Math::IsNearlyEqual(upBasisTransformed, upQuatTransformed, tolerance))
                        << "upBasisTransformed: " << upBasisTransformed << " \n upQuatTransformed: " << upQuatTransformed;
                }
            }
        }
    }

    TEST_F(RotationTest, Quat_Basis_TransformPointIsEquivalent)
    {
        static constexpr float tolerance = 1e-5f;

        const std::vector<Vec3> points
        {
            Vec3(5, 0, 0),
            Vec3(0, 5, 0),
            Vec3(0, 0, 5),
            Vec3(-5, 0, 0),
            Vec3(0, -5, 0),
            Vec3(0, 0, -5),
        };

        for (const auto& [upAxisA, forwardAxisA] : axisPairs)
        {
            SCOPED_TRACE(std::format("Up-Axis A: {}, Forward-Axis A: {}",
                                     (std::stringstream() << upAxisA).str(), (std::stringstream() << forwardAxisA).str()));

            const auto basisA = Basis<Vec3>::FromUpAndForward(upAxisA, forwardAxisA);
            const auto quatA = Quat<Vec3>::FromUpAndForward(upAxisA, forwardAxisA);

            for (const auto& [upAxisB, forwardAxisB] : axisPairs)
            {
                SCOPED_TRACE(std::format("Up-Axis B: {}, Forward-Axis B: {}",
                                         (std::stringstream() << upAxisB).str(), (std::stringstream() << forwardAxisB).str()));

                const auto basisB = Basis<Vec3>::FromUpAndForward(upAxisB, forwardAxisB);
                const auto quatB = Quat<Vec3>::FromUpAndForward(upAxisB, forwardAxisB);

                const auto basisComp = basisB.Transform(basisA);
                const auto quatComp = quatB * quatA;

                for (const Vec3& point : points)
                {
                    // Basis and Quat rotated forward vector should be equal
                    const Vec3 pointBasisTransformed = basisComp.Transform(point);
                    const Vec3 pointQuatTransformed = quatComp.Rotate(point);

                    EXPECT_TRUE(Math::IsNearlyEqual(pointBasisTransformed, pointQuatTransformed, tolerance))
                        << "pointBasisTransformed: " << pointBasisTransformed << " \n pointQuatTransformed: " << pointQuatTransformed;
                }
            }
        }
    }

} // namespace JPL