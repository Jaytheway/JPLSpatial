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


// With simplified approach we don't compute geometry term
// for each bounce, since it cancels out anyway.
// We simply factor it in when connecting subpaths.
#define JPL_SIMPLE 1

//? TMP. for debugging
#define JPL_DEBUG_PRINT 1

#if JPL_DEBUG_PRINT
#include <iostream>
#define LOG_TMP(message) std::cout << message << '\n'
#else
#define LOG_TMP(message)
#endif // !JPL_DEBUG_PRINT

#define JPL_USE_ER 0
#define JPL_ER_SIMPLE 0 && JPL_USE_ER
#define JPL_ER_COMPLEX 1 && JPL_USE_ER && !JPL_ER_SIMPLE

#define JPL_REJECT_SPECULAR 0
#define JPL_ER_SIMPLE_SPECULAR 1 && JPL_ER_SIMPLE && !JPL_REJECT_SPECULAR


namespace JPL
{
	
} // namespace JPL