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

#include "JPLSpatialization/ErrorReporting.h"

#include "Tests/VBAPTest.h"

#undef max
#undef min

#include <iostream>
#include <gtest/gtest.h>

struct vec3
{
    float x, y, z;
};

constexpr std::array<vec3, 20> g_maChannelDirections = {
                      vec3{ 0.0f,     0.0f,    -1.0f    },  /* MA_CHANNEL_NONE */
                      vec3{ 0.0f,     0.0f,    -1.0f    },  /* MA_CHANNEL_MONO */
                      vec3{-0.7071f,  0.0f,    -0.7071f },  /* MA_CHANNEL_FRONT_LEFT */
                      vec3{+0.7071f,  0.0f,    -0.7071f },  /* MA_CHANNEL_FRONT_RIGHT */
                      vec3{ 0.0f,     0.0f,    -1.0f    },  /* MA_CHANNEL_FRONT_CENTER */
                      vec3{ 0.0f,     0.0f,    -1.0f    },  /* MA_CHANNEL_LFE */
                      vec3{-0.7071f,  0.0f,    +0.7071f },  /* MA_CHANNEL_BACK_LEFT */
                      vec3{+0.7071f,  0.0f,    +0.7071f },  /* MA_CHANNEL_BACK_RIGHT */
                      vec3{-0.3162f,  0.0f,    -0.9487f },  /* MA_CHANNEL_FRONT_LEFT_CENTER */
                      vec3{+0.3162f,  0.0f,    -0.9487f },  /* MA_CHANNEL_FRONT_RIGHT_CENTER */
                      vec3{ 0.0f,     0.0f,    +1.0f    },  /* MA_CHANNEL_BACK_CENTER */
                      vec3{-1.0f,     0.0f,     0.0f    },  /* MA_CHANNEL_SIDE_LEFT */
                      vec3{+1.0f,     0.0f,     0.0f    },  /* MA_CHANNEL_SIDE_RIGHT */
                      vec3{ 0.0f,    +1.0f,     0.0f    },  /* MA_CHANNEL_TOP_CENTER */
                      vec3{-0.5774f, +0.5774f, -0.5774f },  /* MA_CHANNEL_TOP_FRONT_LEFT */
                      vec3{ 0.0f,    +0.7071f, -0.7071f },  /* MA_CHANNEL_TOP_FRONT_CENTER */
                      vec3{+0.5774f, +0.5774f, -0.5774f },  /* MA_CHANNEL_TOP_FRONT_RIGHT */
                      vec3{-0.5774f, +0.5774f, +0.5774f },  /* MA_CHANNEL_TOP_BACK_LEFT */
                      vec3{ 0.0f,    +0.7071f, +0.7071f },  /* MA_CHANNEL_TOP_BACK_CENTER */
                      vec3{+0.5774f, +0.5774f, +0.5774f },  /* MA_CHANNEL_TOP_BACK_RIGHT */
};

//==========================================================================
using AllocationCallbackData = std::atomic<uint64_t>;

inline static std::atomic<uint64_t> sMemoryUsedByEngine{ 0 };


//==========================================================================
static void JPLTraceCallback(const char* message)
{
	std::cout << message << '\n';
}

//==========================================================================
static void Main(int argc, char* argv[])
{
    // Printing out some values to embed later
   /* for (const vec3 p : g_maChannelDirections)
        std::cout << std::atan2(p.x, -p.z) << "f, " << '\n';*/

	JPL::Trace = JPLTraceCallback;
}


//==========================================================================
int main(int argc, char* argv[])
{
	testing::InitGoogleTest(&argc, argv);
	
	Main(argc, argv);

	return RUN_ALL_TESTS();
}

#endif // JPL_TEST
