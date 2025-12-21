//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatialziation **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatialziation
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatialziation is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include "JPLSpatial/ErrorReporting.h"

#include <iostream>
#include <format>
#include <cstdarg>

namespace JPL
{
	static void DummyTrace(const char* message)
	{
		std::cout << message << "\n";
	};

	TraceFunction SpatialTrace = DummyTrace;

#if defined(JPL_ENABLE_ASSERTS) || defined(JPL_ENABLE_ENSURE)

	static bool DummyAssertFailed(const char* /*inExpression*/, const char* /*inMessage*/, const std::source_location)
	{
		return true; // Trigger breakpoint
	};

	AssertFailedFunction AssertFailed = DummyAssertFailed;

#endif // JPL_ENABLE_ASSERTS || defined(JPL_ENABLE_ENSURE)

} // namespace JPL