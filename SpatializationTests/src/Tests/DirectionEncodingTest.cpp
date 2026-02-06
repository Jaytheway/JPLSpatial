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

#include "JPLSpatial/Math/DirectionEncoding.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/Vec3Pack.h"

#include "JPLSpatial/Panning/VBAPLUT2D.h"

#include "../Utility/TestUtils.h"

#include <gtest/gtest.h>

#include <array>
#include <cmath>
#include <format>
#include <span>
#include <limits>

namespace JPL
{
	TEST(DirectionEncoding, ParseCodeIndices16Bit)
	{
        using Vec3 = MinimalVec3;
        using LUTCodec = Octahedron16Bit; // Note: it's not practical to test > 16-bit
        static constexpr auto stride = LUTCodec::cAxisRange;

        struct Encoding
        {
            typename LUTCodec::EncodedType Code;
            typename LUTCodec::EncodedType DecodedEncoded;
        };
        std::vector<Encoding> invalidEncodings;
        invalidEncodings.reserve(stride * stride);

        // For each cell in octahedron encoding
        for (uint16_t dy = 0; dy < stride; ++dy)
        {
            for (uint16_t dx = 0; dx < stride; ++dx)
            {
                // index -> code
                const LUTCodec::EncodedType code = LUTCodec::CombineComponents(dy, dx);
                    
                // code -> vec3
                const Vec3 dir = LUTCodec::Decode<Vec3>(code);
                    
                // vec3 -> code
                const LUTCodec::EncodedType encoded = LUTCodec::Encode(dir);

                // Take a record of the failed decoding->encoding
                if (LUTCodec::SanitizeCode(code) != encoded)
                    invalidEncodings.emplace_back(code, encoded);
            }
        }

        // Print error if any invalid encodings found
        EXPECT_TRUE(invalidEncodings.empty())
            << std::format("Codec produced {}/{} ({:.5}%) invalid encodings",
                           invalidEncodings.size(), stride * stride, double(invalidEncodings.size()) / (stride * stride) * 100.0);

        if (!invalidEncodings.empty())
        {
            SaveCSV(invalidEncodings, [](std::ofstream& file, const Encoding& e)
            {
                file << e.Code << "," << e.DecodedEncoded;
            },
                    "invalid_codes.csv", "index_code,decoded_encoded");
        }
	}

    TEST(DirectionEncoding, EncodeDecode64Bit)
    {
        // Testing 64-bit Vec3->codec->Vec3 to eliminate precision error factor
        using Vec3 = MinimalVec3;
        using LUTCodec = Octahedron32Bit;

        static auto getDelta = [](const Vec3& a, const Vec3& b)
        {
            return Abs(a - b);
        };

        static constexpr float step = std::numbers::pi_v<float> / 90.0f; // 1 degree
        // Estimated number of directions that will be generated
        static constexpr size_t numDirs = static_cast<size_t>(
            (std::numbers::pi_v<float> * 2.0f / step) *
            (std::numbers::pi_v<float> * 2.0f) / step);

        struct EncodingData
        {
            Vec3 Direction;
            Vec3 EncodedDecoded;
            Vec3 Delta;
        };
        std::vector<EncodingData> invalidEncodings;
        invalidEncodings.reserve(numDirs);

        // Max error detected
        Vec3 deltaMax = Vec3::Zero();
        
        size_t totalDirsTested = 0;

        for (float phi = 0.0f; phi < std::numbers::pi_v<float> * 2.0f; phi += step)
        {
            const auto [sinPhi, cosPhi] = Math::SinCos(phi);

            for (float theta = 0.0f; theta < std::numbers::pi_v<float> * 2.0f; theta += step)
            {
                const auto [sinTheta, cosTheta] = Math::SinCos(theta);

                // Generate test direction
                const Vec3 dir = Normalized(Vec3{
                    sinPhi * cosTheta,
                    sinPhi * sinTheta,
                    cosPhi
                });

                // Encode -> decode test direction
                const LUTCodec::EncodedType encoded = LUTCodec::Encode(dir);
                const Vec3 decoded = LUTCodec::Decode<Vec3>(encoded);

                // Get delta error
                const Vec3 delta = getDelta(dir, decoded);
                static constexpr auto errorTolerance = LUTCodec::cMaxComponentError;
                const bool bAproxEqual =
                    delta.X <= errorTolerance &&
                    delta.Y <= errorTolerance &&
                    delta.Y <= errorTolerance;

                // Take a record of the error
                if (!bAproxEqual)
                {
                    invalidEncodings.emplace_back(dir, decoded, delta);
                    deltaMax.X = std::max(deltaMax.X, delta.X);
                    deltaMax.Y = std::max(deltaMax.Y, delta.Y);
                    deltaMax.Z = std::max(deltaMax.Z, delta.Z);
                }

                totalDirsTested++;
            }
        }

        // Compute delta error variance for each axis
        OnlineVariance deltaVarianceX;
        OnlineVariance deltaVarianceY;
        OnlineVariance deltaVarianceZ;
        for (const auto& [dir, encode, delta] : invalidEncodings)
        {
            deltaVarianceX.Add(delta.X);
            deltaVarianceY.Add(delta.Y);
            deltaVarianceZ.Add(delta.Z);
        }

        // Print error if any invalid directions found
        EXPECT_TRUE(invalidEncodings.empty())
            << std::format("Codec produced {}/{} ({:.5}%) invalid encodings",
                           invalidEncodings.size(), totalDirsTested, double(invalidEncodings.size()) / totalDirsTested * 100.0)
            << std::format("\nDelta mean: {{ {}, {}, {} }} \nDelta variance: {{ {}, {}, {} }}",
                           deltaVarianceX.Mean, deltaVarianceY.Mean, deltaVarianceZ.Mean,
                           deltaVarianceX.GetVariance(), deltaVarianceY.GetVariance(), deltaVarianceZ.GetVariance())
            << std::format("\nDelta max: {{ {}, {}, {} }}",
                           deltaMax.X, deltaMax.Y, deltaMax.Z);


        auto vecToStrRow = [](const Vec3& v) { return std::format("\"{}, {}, {}\"", v.X, v.Y, v.Z); };

        // Save error data to file
        if (!invalidEncodings.empty())
        {
            SaveCSV(invalidEncodings, [vecToStrRow](std::ofstream& file, const EncodingData& e)
            {
                file
                    << vecToStrRow(e.Direction) << ","
                    << vecToStrRow(e.EncodedDecoded) << ","
                    << vecToStrRow(e.Delta) << ","
                    << e.Delta.Length();
            }, "invalid_encoding.csv", "test_dir,encoded_decoded,delta,delta_length");
        }
    }

    TEST(DiamondEncoding, EncodeDecode)
    {
        static auto getDelta = [](const Vec2& a, const Vec2& b)
        {
            return Abs(a - b);
        };

        static constexpr float step = JPL_PI / 360.0f; // 0.5 degree
        // Estimated number of directions that will be generated
        static constexpr size_t numDirs = static_cast<size_t>(JPL_TWO_PI / step);

        struct EncodingData
        {
            Vec2 Direction;
            Vec2 EncodedDecoded;
            Vec2 Delta;
        };
        std::vector<EncodingData> invalidEncodings;
        invalidEncodings.reserve(numDirs);

        // Max error detected
        Vec2 deltaMax = Vec2::Zero();

        size_t totalDirsTested = 0;

        for (float theta = 0.0f; theta < JPL_TWO_PI; theta += step)
        {
            const auto [sinTheta, cosTheta] = Math::SinCos(theta);

            // Generate test direction
            const Vec2 dir = Normalized(Vec2{ sinTheta, -cosTheta });

            // Encode -> decode test direction
            const float encoded = ToDiamond(dir);
            const Vec2 decoded = FromDiamond(encoded);

            // Get delta error
            const Vec2 delta = getDelta(dir, decoded);
            static constexpr auto errorTolerance = 1e-6f;
            const bool bAproxEqual =
                delta.X <= errorTolerance &&
                delta.Y <= errorTolerance;

            // Take a record of the error
            if (!bAproxEqual)
            {
                invalidEncodings.emplace_back(dir, decoded, delta);
                deltaMax.X = std::max(deltaMax.X, delta.X);
                deltaMax.Y = std::max(deltaMax.Y, delta.Y);
            }

            totalDirsTested++;
        }

        // Compute delta error variance for each axis
        OnlineVariance deltaVarianceX;
        OnlineVariance deltaVarianceY;
        for (const auto& [dir, encode, delta] : invalidEncodings)
        {
            deltaVarianceX.Add(delta.X);
            deltaVarianceY.Add(delta.Y);
        }

        // Print error if any invalid directions found
        EXPECT_TRUE(invalidEncodings.empty())
            << std::format("Codec produced {}/{} ({:.5}%) invalid encodings",
                           invalidEncodings.size(), totalDirsTested, double(invalidEncodings.size()) / totalDirsTested * 100.0)
            << std::format("\nDelta mean: {{ {}, {} }} \nDelta variance: {{ {}, {} }}",
                           deltaVarianceX.Mean, deltaVarianceY.Mean,
                           deltaVarianceX.GetVariance(), deltaVarianceY.GetVariance())
            << std::format("\nDelta max: {{ {}, {} }}",
                           deltaMax.X, deltaMax.Y);


        auto vecToStrRow = [](const Vec2& v) { return std::format("\"{}, {}\"", v.X, v.Y); };

        // Save error data to file
        if (!invalidEncodings.empty())
        {
            SaveCSV(invalidEncodings, [vecToStrRow](std::ofstream& file, const EncodingData& e)
            {
                file
                    << vecToStrRow(e.Direction) << ","
                    << vecToStrRow(e.EncodedDecoded) << ","
                    << vecToStrRow(e.Delta) << ","
                    << e.Delta.Length();
            }, "invalid_encoding_2D.csv", "test_dir,encoded_decoded,delta,delta_length");
        }
    }

   
    TEST(DiamondEncoding, ComparableToAtan2)
    {
        static constexpr float step = JPL_PI / 360.0f; // 0.5 degree

        // Estimated number of directions that will be generated
        static constexpr size_t numDirs = static_cast<size_t>(JPL_TWO_PI / step);

        struct EncodingData
        {
            Vec2 Direction;
            Vec2 EncodedDecoded;
            float DeltaDegrees;
        };
        std::vector<EncodingData> invalidEncodings;
        invalidEncodings.reserve(numDirs);

        // Max error detected
        float deltaDegreesMax = 0.0f;

        size_t totalDirsTested = 0;

        for (float theta = 0.0f; theta < JPL_TWO_PI; theta += step)
        {
            const auto [sinTheta, cosTheta] = Math::SinCos(theta);

            // Generate test direction
            const Vec2 dir = Normalized(Vec2{sinTheta, -cosTheta});

            const float atan2Expected = std::atan2(dir.X, dir.Y);

            // Encode -> decode test direction
            const float encoded = ToDiamond(dir);
            const Vec2 decoded = FromDiamond(encoded);

            const float atan2Decoded = std::atan2(decoded.X, decoded.Y);

            // Get delta error
            const float atan2Delta = Math::Abs(atan2Decoded - atan2Expected);

            // Take a record of the error
            if (!Math::IsNearlyZero(atan2Delta, 1e-6f)) // about 0.00326586 degrees
            {
                const float deltaDegrees = Math::ToDegrees(atan2Delta);
                invalidEncodings.emplace_back(dir, decoded, deltaDegrees);
                deltaDegreesMax = std::max(deltaDegreesMax, deltaDegrees);
            }

            totalDirsTested++;
        }

        // Compute delta error variance for each axis
        OnlineVariance deltaVariance;
        for (const auto& [dir, encode, delta] : invalidEncodings)
            deltaVariance.Add(delta);

        // Print error if any invalid directions found
        EXPECT_TRUE(invalidEncodings.empty())
            << std::format("Codec produced {}/{} ({:.5}%) invalid encodings",
                           invalidEncodings.size(), totalDirsTested, double(invalidEncodings.size()) / totalDirsTested * 100.0)
            << std::format("\nDelta mean: {} deg, \nDelta variance: {} deg",
                           deltaVariance.Mean, deltaVariance.GetVariance())
            << std::format("\nDelta max: {} deg", deltaDegreesMax);


        auto vecToStrRow = [](const Vec2& v) { return std::format("\"{}, {}\"", v.X, v.Y); };

        // Save error data to file
        if (!invalidEncodings.empty())
        {
            SaveCSV(invalidEncodings, [vecToStrRow](std::ofstream& file, const EncodingData& e)
            {
                file
                    << vecToStrRow(e.Direction) << ","
                    << vecToStrRow(e.EncodedDecoded) << ","
                    << e.DeltaDegrees;
            }, "invalid_encoding_2D.csv", "test_dir,encoded_decoded,atan2_delta");
        }
    }

	TEST(DiamondEncoding, CorrectionLUT_UniformityVsGroundTruth)
	{
        // This test is esstntially checking:
        // "Can we treat diamond scalar `p` as a linear angle parameter (after phase alignment)?"
        // The answer is `no`, without a correction LUT we get up to 4-6 degrees deviation.
        //
        // This is relevant if we're using Diamond encoding as a cheep atan2,
        // however irrelevant for building a LUT from uniform angles.

        // ---- Budgets from resolutions ----
        struct CorrBudgets
        {
            float MaxAngleDeg;
            float MeanAngleDeg;
            int MaxBin;
            float MeanBin;
        };

        // @param N_bins : main LUT resolution (power of two)
        // @param M_corr : correction LUT size (power of two)
        static auto ComputeCorrBudgets = [](uint32_t N_bins, uint32_t M_corr,
                                            float safetyMax = 1.10f,
                                            float safetyMean = 1.15f) -> CorrBudgets
        {
            // Numeric characteristics of diamond mapping (fixed):
            constexpr float L_MAX_DEG = 458.366235955633f; // max dθ/dp in degrees
            constexpr float L_MEAN_DEG = 375.085622423709f; // mean dθ/dp in degrees

            const float binWidthDeg = 360.0f / static_cast<float>(N_bins);

            const float maxAngle = (L_MAX_DEG / (2.0f * static_cast<float>(M_corr))) * safetyMax;
            const float meanAngle = (L_MEAN_DEG / (4.0f * static_cast<float>(M_corr))) * safetyMean;

            const float safetyMeanBin = 1.30f;

            const int maxBin = static_cast<int>(std::ceil(maxAngle / binWidthDeg));
            const float meanBin = (meanAngle / binWidthDeg) * safetyMeanBin;

            return CorrBudgets{
                .MaxAngleDeg = maxAngle,
                .MeanAngleDeg = meanAngle,
                .MaxBin = maxBin,
                .MeanBin = meanBin
            };
        };

        auto runTestCase = [&](uint32_t N_bins, uint32_t M_corr)
        {
            SCOPED_TRACE(std::format("N_bins: {}, M_corr: {}", N_bins, M_corr));

            // Helpers
            static auto wrapPi = [](float a)
            {
                a = std::fmod(a + JPL_PI, JPL_TWO_PI);
                if (a < 0.0f)
                    a += JPL_TWO_PI;
                return a - JPL_PI;
            };

            static auto angleDelta = [](float a, float b) { return wrapPi(a - b); };

            // Parameters
            const float step = JPL_PI / 360.0f; // 0.5 degree sampling

            ASSERT_EQ((N_bins & (N_bins - 1u)), 0u);
            ASSERT_EQ((M_corr & (M_corr - 1u)), 0u);

            const uint32_t Nmask = N_bins - 1u;
            const uint32_t McMask = M_corr - 1u;

            // Build correction LUT (p -> uhat in [0,1) with 0 degrees at +Z)
            typename VBAP::LUT2D::template Array<float> corrU;
            VBAP::LUT2D::BuildDiamondToAngleNormLUT(corrU, M_corr);
            
            ASSERT_EQ(corrU.size(), M_corr);

            // Phase align "raw diamond" so that +Z maps to 0 degrees
            const float p_fwd = ToDiamond(Vec2{ 0.0f, 1.0f }); // p at +Z

            // Accumulators with OnlineVariance
            OnlineVariance angRawDeg, angCorrDeg;   // angle error in degrees
            OnlineVariance binRaw, binCorr;         // |delta bin| as floats (int cast to float)

            float maxAngleErrRawDeg = 0.0f;
            float maxAngleErrCorrDeg = 0.0f;
            int maxBinErrRaw = 0;
            int maxBinErrCorr = 0;

            // Sweep full circle
            for (float thetaTrue = 0.0f; thetaTrue < JPL_TWO_PI; thetaTrue += step)
            {
                // Unit direction with 0 degrees at +Z (ground truth angle is atan2(x,y))
                const auto [s, c] = Math::SinCos(thetaTrue);
                const Vec2  dir{ s, c }; // (x,y) = (sinθ, cosθ)

                // Ground truth θ in [0, 2π)
                float thetaGroundTruth = std::atan2(dir.X, dir.Y);
                if (thetaGroundTruth < 0.0f)
                    thetaGroundTruth += JPL_TWO_PI;

                // Diamond parameter
                const float p = ToDiamond(dir);

                // 1. Raw diamond (phase-aligned) ---
                //float p_al = p - p_fwd; if (p_al < 0.0f) p_al += 1.0f;
                float p_al = p_fwd - p;
                if (p_al < 0.0f)
                    p_al += 1.0f;

                const float thetaRaw = p_al * JPL_TWO_PI;

                // 2. Corrected via LUT (nearest) ---
                const float pf = p * float(M_corr);
                const uint32_t j = static_cast<uint32_t>(pf + 0.5f) & McMask;
                const float uhat = corrU[j];
                const float theta_corr = uhat * JPL_TWO_PI;

                // Angle errors (degrees), minimal signed on circle
                const float dRawDeg = Math::Abs(angleDelta(thetaRaw, thetaGroundTruth)) * JPL_TO_DEG;
                const float dCorrDeg = Math::Abs(angleDelta(theta_corr, thetaGroundTruth)) * JPL_TO_DEG;

                angRawDeg.Add(dRawDeg);
                angCorrDeg.Add(dCorrDeg);

                maxAngleErrRawDeg = std::max(maxAngleErrRawDeg, dRawDeg);
                maxAngleErrCorrDeg = std::max(maxAngleErrCorrDeg, dCorrDeg);

                // Bin errors for N_bins
                const int idx_gt = static_cast<int>(thetaGroundTruth * JPL_INV_TWO_PI * N_bins + 0.5f) & static_cast<int>(Nmask);
                const int idx_raw = static_cast<int>(thetaRaw * JPL_INV_TWO_PI * N_bins + 0.5f) & static_cast<int>(Nmask);
                const int idx_corr = static_cast<int>(theta_corr * JPL_INV_TWO_PI * N_bins + 0.5f) & static_cast<int>(Nmask);

                auto wrapdiff = [N_bins](int a, int b)
                {
                    int d = a - b;
                    if (d > static_cast<int>(N_bins) / 2)
                        d -= static_cast<int>(N_bins);
                    if (d < -static_cast<int>(N_bins) / 2)
                        d += static_cast<int>(N_bins);

                    return d;
                };
                const int dRawBins = Math::Abs(wrapdiff(idx_raw, idx_gt));
                const int dCorrBins = Math::Abs(wrapdiff(idx_corr, idx_gt));

                binRaw.Add(static_cast<float>(dRawBins));
                binCorr.Add(static_cast<float>(dCorrBins));

                maxBinErrRaw = std::max(maxBinErrRaw, dRawBins);
                maxBinErrCorr = std::max(maxBinErrCorr, dCorrBins);
            }

            // --- Report ---

            const CorrBudgets B = ComputeCorrBudgets(N_bins, M_corr);

            // Optional: print only if over budget
            if (maxAngleErrCorrDeg > B.MaxAngleDeg ||
                angCorrDeg.Mean > B.MeanAngleDeg ||
                maxBinErrCorr > B.MaxBin ||
                binCorr.Mean > B.MeanBin)
            {
                std::cout << "[N=" << N_bins << ", M=" << M_corr << "] " << "\n";

                std::cout << "Angle error RAW   :" << '\n'
                    << "    mean= " << angRawDeg.Mean << " deg" << '\n'
                    << "    var= " << angRawDeg.GetVariance() << '\n'
                    << "    max= " << maxAngleErrRawDeg << " deg"<< "\n";
                std::cout << "Angle error CORR  :" << '\n'
                    << "    mean= " << angCorrDeg.Mean << " (<= " << B.MeanAngleDeg << ")" << "\n"
                    << "    var=" << angCorrDeg.GetVariance() << "\n"
                    << "    max=" << maxAngleErrCorrDeg << " deg" << " (<= " << B.MaxAngleDeg << ")" << "\n";

                // Bin error -> how often do we land into a wrong LUT cell
                std::cout << "Bin error RAW     :" << '\n'
                    << "    mean= " << binRaw.Mean << '\n'
                    << "    var= " << binRaw.GetVariance() << '\n'
                    << "    max= " << maxBinErrRaw << "\n";
                std::cout << "Bin error CORR    :" << '\n'
                    << "    mean=" << binCorr.Mean << " (<= " << B.MeanBin << ")\n"
                    << "    var=" << binCorr.GetVariance() << '\n'
                    << "    max=" << maxBinErrCorr << " (<= " << B.MaxBin << ")" << "\n";
            }

            // --- Expectations ---
            // With M_corr=1024, worst-case angle error should be well under 0.25 degrees
            EXPECT_LE(maxAngleErrCorrDeg, B.MaxAngleDeg);

            // Raw diamond should be much worse than corrected:
            EXPECT_LE(angCorrDeg.Mean, B.MeanAngleDeg);
            EXPECT_LT(angCorrDeg.GetVariance(), angRawDeg.GetVariance());

            // For N=512 (~0.703 degrees/bin), corrected nearest should be within 1 bin anywhere
            EXPECT_LE(maxBinErrCorr, B.MaxBin);

            EXPECT_LE(binCorr.Mean, B.MeanBin); // tiny slack for float

            // Sanity: raw diamond shows several-bin error at diagonals (typically ~6)
            EXPECT_GE(maxBinErrRaw, 3); // loose lower bound, just to ensure we're measuring

        };

        const std::pair<uint32_t, uint32_t> cases[] ={
            {256, 512},
            {512, 1024},
            {1024, 1024},
            {1024, 2048},
            {2048, 2048},
            {1024, 512},
        };

        for (const auto [N_bins, M_corr] : cases)
            runTestCase(N_bins, M_corr);

	}

    TEST(DiamondEncoding, UniformityVsGroundTruth)
    {
        // Testing how relevant Diamond encoding is for builging LUT
        // using uniform steps within Diamond encoding.

        auto runTestCase = [&](uint32_t N_bins)
        {
            SCOPED_TRACE(std::format("N_bins: {}", N_bins));

            static auto wrapPi = [](float a)
            {
                a = std::fmod(a + JPL_PI, JPL_TWO_PI);
                if (a < 0.0f)
                    a += JPL_TWO_PI;
                return a - JPL_PI;
            };

            static auto angleDelta = [](float a, float b) { return wrapPi(a - b); };

            const float stepLinear = 1.0f / N_bins;

            std::vector<float> angles;
            angles.reserve(1.0f / stepLinear);

            // Sweep full circle
            for (float diamond = 0.0f; diamond < 1.0; diamond += stepLinear)
            {
                const Vec2 dir = FromDiamond(diamond);

                float theta = std::atan2(dir.Y, dir.X);
                if (theta < 0.0f)
                    theta += JPL_TWO_PI;

                angles.push_back(theta);
            }

            std::ranges::sort(angles);

            OnlineVariance degDiff;
            float minStep = std::numeric_limits<float>::max();
            float maxStep = 0.0f;

            for (uint32 i = 1; i < angles.size(); ++i)
            {
                const float step = Math::Abs(angleDelta(angles[i], angles[i - 1]));
                degDiff.Add(step);
                minStep = std::min(step, minStep);
                maxStep = std::max(step, maxStep);
            }
            {
                const float step = Math::Abs(angleDelta(angles[0], angles.back()));
                degDiff.Add(step);
                minStep = std::min(step, minStep);
                maxStep = std::max(step, maxStep);
            }


            // --- Expectations ---
            const float expectedStepDeg = Math::ToDegrees(JPL_TWO_PI / N_bins);
            const float expectedVarianceDeg = 0.5f; // 0.5 degree variance is perfectly fine

            const float stepMeanDeg = Math::ToDegrees(degDiff.Mean);
            const float stepVarDeg = Math::Sqrt(degDiff.GetVariance()) * JPL_TO_DEG;
            const float stepMinDeg = Math::ToDegrees(minStep);
            const float stepMaxDeg = Math::ToDegrees(maxStep);
            const float stepMaxGapDeg = stepMaxDeg - stepMinDeg;

            EXPECT_LE(stepMeanDeg - expectedVarianceDeg, expectedStepDeg) << "Variance: " << stepVarDeg;
            EXPECT_LE(stepVarDeg - expectedVarianceDeg, expectedVarianceDeg);

            //std::cout << "N bins: " << N_bins << '\n';
            //std::cout << "Step mean degrees: " << stepMeanDeg << '\n';
            //std::cout << "Step variance degrees: " << stepVarDeg << '\n';
            //std::cout << "Step min / max | gap: " << stepMinDeg << " / " << stepMaxDeg << " | " << stepMaxGapDeg << '\n';
        };

		const uint32_t cases[] = {
			256,
			512,
			1024,
			2048
		};

        for (const auto N_bins : cases)
            runTestCase(N_bins);
    }

    TEST(DirectionEncoding, SIMDEncoding)
    {
        // Testing 64-bit Vec3->codec->Vec3 to eliminate precision error factor
        using Vec3 = MinimalVec3;
        using LUTCodec = Octahedron16Bit;

        static auto getDelta = [](const Vec3Pack& a, const Vec3Pack& b)
        {
            return Abs(a - b);
        };

        static constexpr float step = std::numbers::pi_v<float> / 90.0f; // 1 degree
        // Estimated number of directions that will be generated
        static constexpr size_t numDirs = FloorToSIMDSize(static_cast<size_t>(
            (JPL_TWO_PI / step) *
            (JPL_TWO_PI / step)
            ));

        static constexpr size_t numDirPacks = GetNumSIMDOps(numDirs);

        // Generated directions
        std::vector<Vec3> directions;
        directions.reserve(numDirs);

        for (float phi = 0.0f; phi < JPL_TWO_PI; phi += step)
        {
            if (directions.size() == numDirs)
                break;

            const auto [sinPhi, cosPhi] = Math::SinCos(phi);

            for (float theta = 0.0f; theta < JPL_TWO_PI; theta += step)
            {
                if (directions.size() == numDirs)
                    break;

                const auto [sinTheta, cosTheta] = Math::SinCos(theta);

                // Generate test direction
                directions.push_back(Normalized(Vec3{
                    sinPhi * cosTheta,
                    sinPhi * sinTheta,
                    cosPhi
                }));
            }
        }

        // Pack directions into simd packs
        std::vector<Vec3Pack> directionPacks;
        directionPacks.reserve(numDirPacks);
        for (uint32 i = 0; i < numDirs; i += simd::size())
            directionPacks.emplace_back().load(std::span<const Vec3>(&directions[i], simd::size()));

        // Test encoding->decoding for the generated and packed directions
        struct EncodingData
        {
            Vec3 Direction;
            Vec3 EncodedDecoded;
            Vec3 Delta;
        };
        std::vector<EncodingData> invalidEncodings;
        invalidEncodings.reserve(numDirPacks);

        // Max error detected
        Vec3 deltaMax = Vec3::Zero();

        size_t totalDirsTested = 0;

        for (const Vec3Pack& dirPack : directionPacks)
        {
            // Encode -> decode test direction
            const simd_mask encoded = LUTCodec::Encode(dirPack.X, dirPack.Y, dirPack.Z);

            Vec3Pack decoded;
            LUTCodec::Decode(encoded, decoded.X, decoded.Y, decoded.Z);

            // Get delta error
            const Vec3Pack delta = getDelta(dirPack, decoded);

            std::array<Vec3, simd::size()> unpackedOriginal, unpackedDecoced,unpackedDelta;
            dirPack.store(std::span<Vec3>(unpackedOriginal));
            decoded.store(std::span<Vec3>(unpackedDecoced));
            delta.store(std::span<Vec3>(unpackedDelta));

            for (uint32 i = 0; i < simd::size(); ++i)
            {
                static constexpr auto errorTolerance = LUTCodec::cMaxComponentError;
                const bool bAproxEqual =
                    unpackedDelta[i].X <= errorTolerance &&
                    unpackedDelta[i].Y <= errorTolerance &&
                    unpackedDelta[i].Y <= errorTolerance;

                // Take a record of the error
                if (!bAproxEqual)
                {
                    invalidEncodings.emplace_back(unpackedOriginal[i], unpackedDecoced[i], unpackedDelta[i]);
                    deltaMax.X = std::max(deltaMax.X, unpackedDelta[i].X);
                    deltaMax.Y = std::max(deltaMax.Y, unpackedDelta[i].Y);
                    deltaMax.Z = std::max(deltaMax.Z, unpackedDelta[i].Z);
                }
            }

            totalDirsTested += 4;
        }

        // Compute delta error variance for each axis
        OnlineVariance deltaVarianceX;
        OnlineVariance deltaVarianceY;
        OnlineVariance deltaVarianceZ;
        for (const auto& [dir, encode, delta] : invalidEncodings)
        {
            deltaVarianceX.Add(delta.X);
            deltaVarianceY.Add(delta.Y);
            deltaVarianceZ.Add(delta.Z);
        }

        // Print error if any invalid directions found
        EXPECT_TRUE(invalidEncodings.empty())
            << std::format("Codec produced {}/{} ({:.5}%) invalid encodings",
                            invalidEncodings.size(), totalDirsTested, double(invalidEncodings.size()) / totalDirsTested * 100.0)
            << std::format("\nDelta mean: {{ {}, {}, {} }} \nDelta variance: {{ {}, {}, {} }}",
                            deltaVarianceX.Mean, deltaVarianceY.Mean, deltaVarianceZ.Mean,
                            deltaVarianceX.GetVariance(), deltaVarianceY.GetVariance(), deltaVarianceZ.GetVariance())
            << std::format("\nDelta max: {{ {}, {}, {} }}",
                            deltaMax.X, deltaMax.Y, deltaMax.Z);


        auto vecToStrRow = [](const Vec3& v) { return std::format("\"{}, {}, {}\"", v.X, v.Y, v.Z); };

        // Save error data to file
        if (!invalidEncodings.empty())
        {
            SaveCSV(invalidEncodings, [vecToStrRow](std::ofstream& file, const EncodingData& e)
            {
                file
                    << vecToStrRow(e.Direction) << ","
                    << vecToStrRow(e.EncodedDecoded) << ","
                    << vecToStrRow(e.Delta) << ","
                    << e.Delta.Length();
            }, "invalid_encoding.csv", "test_dir,encoded_decoded,delta,delta_length");
        }
    }
} // namespace JPL