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

#include "JPLSpatial/DistanceAttenuation.h"

#include <gtest/gtest.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <string>

namespace JPL
{
#if 0 //? Changed curves to more or less common shapes, no need for this test for now
    TEST(AttenuationCurveTest, AttenuationCurve)
    {
        struct AttenuationTestCase
        {
            std::string Description;
            std::vector<AttenuationCurve::Point> Points;
            float TestDistance;
            float ExpectedValue;
        };

        const std::vector<AttenuationTestCase> testCases = {
        // **Constant Function Tests**
        {
            .Description = "Constant function with single point",
            .Points = {{.Distance = 0.0f, .Value = 0.7f, .FunctionType = AttenuationCurve::EFunctionType::Constant}},
            .TestDistance = 5.0f,
            .ExpectedValue = 0.7f
        },
        {
            .Description = "Constant function between multiple points",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Constant},
                {.Distance = 10.0f, .Value = 0.8f, .FunctionType = AttenuationCurve::EFunctionType::Constant},
                {.Distance = 20.0f, .Value = 0.6f, .FunctionType = AttenuationCurve::EFunctionType::Constant}
            },
            .TestDistance = 15.0f,
            .ExpectedValue = 0.8f
        },

        // **Linear Function Tests**
        {
            .Description = "Linear interpolation between two points",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.75f
        },
        {
            .Description = "Linear function beyond last point",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 15.0f,
            .ExpectedValue = 0.5f
        },

        // **Exponential Function Tests**
        {
            .Description = "Exponential decay between two points",
             .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Exponential, .Params = { .Exponent = 0.1f / 1.0f }},
                {.Distance = 10.0f, .Value = 0.1f, .FunctionType = AttenuationCurve::EFunctionType::Exponential, .Params = {.Exponent = 0.1f / 1.0f }}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = std::pow(0.1f, 0.5f),
        },

        // **Cubic Function Tests**
        {
            .Description = "Cubic interpolation between two points",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Cubic},
                {.Distance = 10.0f, .Value = 0.0f, .FunctionType = AttenuationCurve::EFunctionType::Cubic}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.5f, // Approximate expected value
        },

        // **Bezier Function Tests**
        {
            .Description = "Bezier interpolation between two points",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Bezier, .Params = {.Bezier = {.ControlPoint1Value = 5.0f}}},
                {.Distance = 10.0f, .Value = 0.0f, .FunctionType = AttenuationCurve::EFunctionType::Bezier, .Params = {.Bezier = {.ControlPoint2Value = 5.0f}}}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.5f, // Approximate expected value
        },

        // **Edge Cases**
        {
            .Description = "Distance before first point",
            .Points = {
                {.Distance = 10.0f, .Value = 0.8f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 20.0f, .Value = 0.6f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.8f // Should use first point's value
        },
        {
            .Description = "Distance after last point",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 15.0f,
            .ExpectedValue = 0.5f // Should use last point's value
        },
        {
            .Description = "Duplicate distances in points",
            .Points = {
                {.Distance = 10.0f, .Value = 0.8f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.6f, .FunctionType = AttenuationCurve::EFunctionType::Linear}, // Duplicate distance
                {.Distance = 20.0f, .Value = 0.4f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 10.0f,
            .ExpectedValue = 0.8f // Depending on implementation
        },

        // **Incorrect Input Values**
        {
            .Description = "Empty points vector",
            .Points = {},
            .TestDistance = 5.0f,
            .ExpectedValue = 1.0f // Default attenuation
        },
        {
            .Description = "Negative distance",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = -5.0f,
            .ExpectedValue = 1.0f // Depending on implementation, could use first point's value
        },
        {
            .Description = "Invalid function type",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = static_cast<AttenuationCurve::EFunctionType>(999)}, // Invalid function type
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 1.0f // Depending on implementation, default to point's value
        },

        // **Edge Case: Zero Distance Range**
        {
            .Description = "Zero distance range between points",
            .Points = {
                {.Distance = 5.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 5.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 1.0f // Depending on implementation
        },

        // **Edge Case: Points Not Sorted**
        {
            .Description = "Points not sorted by distance",
            .Points = {
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.75f // Should sort points internally
        },

        // **Edge Case: Large Number of Points**
        {
            .Description = "Large number of points",
            .Points = []()
        {
            std::vector<AttenuationCurve::Point> pts;
            for (int i = 0; i <= 100; ++i)
            {
                pts.push_back({.Distance = static_cast<float>(i), .Value = 1.0f / (i + 1), .FunctionType = AttenuationCurve::EFunctionType::Linear});
            }
            return pts;
            }(),
            .TestDistance = 50.5f,
            .ExpectedValue = (1.0f / 51 + 1.0f / 52) / 2, // Linear interpolation between points 50 and 51
        },

        // **Edge Case: Negative Values**
        {
            .Description = "Negative attenuation values",
            .Points = {
                {.Distance = 0.0f, .Value = -1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = -0.5f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = -0.75f // Linear interpolation of negative values
        },

        // **Edge Case: Extremely Close Points**
        {
            .Description = "Extremely close points",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 0.0001f, .Value = 0.9999f, .FunctionType = AttenuationCurve::EFunctionType::Linear}
            },
            .TestDistance = 0.00005f,
            .ExpectedValue = 0.99995f // Should handle small differences correctly
        },

        // **Mixed Function Types Tests**
        {
            .Description = "Transition from Linear to Cubic",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Cubic},
                {.Distance = 20.0f, .Value = 0.0f, .FunctionType = AttenuationCurve::EFunctionType::Cubic}
            },
            .TestDistance = 5.0f,
            .ExpectedValue = 0.75f // Linear interpolation
        },
        {
            .Description = "Transition from Linear to Cubic at distance 15.0",
            .Points = {
                {.Distance = 0.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Linear},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Cubic},
                {.Distance = 20.0f, .Value = 0.0f, .FunctionType = AttenuationCurve::EFunctionType::Cubic}
            },
            .TestDistance = 15.0f,
            .ExpectedValue = 0.25f
        },
        {
        .Description = "Transition from Cubic to Bezier",
        .Points = {
                {.Distance = 0.0f, .Value = 0.0f, .FunctionType = AttenuationCurve::EFunctionType::Cubic},
                {.Distance = 10.0f, .Value = 0.5f, .FunctionType = AttenuationCurve::EFunctionType::Bezier, .Params = {.Bezier = {.ControlPoint1Value = 0.5f}}},
                {.Distance = 20.0f, .Value = 1.0f, .FunctionType = AttenuationCurve::EFunctionType::Bezier, .Params = {.Bezier = {.ControlPoint2Value = 0.5f}}}
            },
            .TestDistance = 15.0f,
            .ExpectedValue = 0.75f
        },
        };

        // Helper function to create AttenuationCurve
        auto createCurve = [](const std::vector<AttenuationCurve::Point>& points)
        {
            AttenuationCurve curve;
            curve.Points = points;
            // Ensure points are sorted by distance
            curve.SortPoints();
            return curve;
        };

        for (const auto& testCase : testCases)
        {
            AttenuationCurve curve = createCurve(testCase.Points);
            float result = curve.Evaluate(testCase.TestDistance);
            EXPECT_NEAR(result, testCase.ExpectedValue, 1e-5) << testCase.Description;
        }
    }
#endif

} // namespace JPL