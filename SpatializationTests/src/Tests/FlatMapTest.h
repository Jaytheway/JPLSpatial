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

#include "JPLSpatial/Containers/FlatMap.h"

#include <gtest/gtest.h>


namespace JPL
{
	TEST(FlatMap, TinyTest)
	{
		FlatMap<int, int> fm;
		fm.emplace(1, 10);
		fm.emplace(2, 20);
		fm.emplace(3, 30);

		auto it0 = fm.begin(), it2 = it0 + 2;

		EXPECT_EQ((it2 - it0), 2);
		EXPECT_TRUE((it0 < it2) && !(it2 < it0));

		auto [k, v] = *it2;
		EXPECT_TRUE(k == 3 && v == 30);

		it0->second = 99;
		EXPECT_EQ(fm.at(1), 99);

		fm.erase(it0);
		EXPECT_EQ(fm.begin()->second, 20);
	}
} // namespace JPL