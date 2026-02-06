//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** MiniaudioCpp **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/MiniaudioCpp
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, MiniaudioCpp is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#ifdef JPL_TEST

#include "JPLSpatial/ErrorReporting.h"

#include "Utility/TestMemoryLeakDetector.h"

#if 0
#include "Tests/AttenuationCurveTest.h"
#endif

#if JPL_HAS_BEAM_TRACING
#include "Tests/RayAndPolyTest.h"
//? No need to test quad tree for now, while it's still just a copy of Jolt's
//#include "Tests/QuadTreeTest.h"
#endif
#if JPL_HAS_PATH_TRACING
#include "Tests/SpecularRayTracingTest.h"
//#include "Tests/BDPTTest.h"
#include "Tests/DelayLineTest.h"
#include "Tests/CrossoverFilterTest.h"
#endif

#undef max
#undef min

#include <gtest/gtest.h>

#include <cstdio>
#include <format>
#include <iostream>
#include <source_location>

//==========================================================================
static void JPLTraceCallback(const char* message)
{
	std::cout << message << '\n';
}

#if defined(JPL_ENABLE_ASSERTS) || defined(JPL_ENABLE_ENSURE)
static bool AssertionFailedCallback(const char* inExpression, const char* inMessage, const std::source_location location)
{
	// Print assertion details to the log
	const auto messageString = std::format(
		"ASSERT FAILED in file '{}' at line {}\n"
		"\n"
		"  Function: {}.\n"
		"Expression: {}"
		"{}{}", // message if provided
		location.file_name(),
		location.line(),
		location.function_name(),
		inExpression,
		inMessage ? "\n   Message: " : "",
		inMessage ? inMessage : "");

	// Making sure we flush the output when running CI
	std::fwrite(messageString.data(), 1, messageString.size(), stderr);
	std::fflush(stderr);

	return true; // Trigger breakpoint
};
#endif // JPL_ENABLE_ASSERTS || defined(JPL_ENABLE_ENSURE)

//==========================================================================
static void Main(int argc, char* argv[])
{
#if defined(JPL_TEST_WITH_JOLT)
    JPH::RegisterDefaultAllocator();
#endif

	JPL::SpatialTrace = JPLTraceCallback;
#if defined(JPL_ENABLE_ASSERTS) || defined(JPL_ENABLE_ENSURE)
	JPL::AssertFailed = AssertionFailedCallback;
#endif
}

#if defined(JPL_TEST_WITH_JOLT)
//? Just a temp hack to make sure we have registered default allocation functions
//? when initializing static objects with JPH::STLAllocator
static struct StaticInit
{
    StaticInit() { JPH::RegisterDefaultAllocator();}
} Init;
#endif

//==========================================================================
int main(int argc, char* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	
	Main(argc, argv);

	return RUN_ALL_TESTS();
}

#endif // JPL_TEST
