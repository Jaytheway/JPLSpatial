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

#include "JPLSpatial/Math/Vec3Traits.h"

#include <limits>

namespace JPL
{
	/// Compute barycentric coordinates of closest point to origin for infinite line defined by (inA, inB)
	/// Point can then be computed as inA * outU + inB * outV
	/// Returns false if the points inA, inB do not form a line (are at the same point)
	template<CVec3 Vec3>
	inline bool GetBaryCentricCoordinates(const Vec3& inA, const Vec3& inB, float& outU, float& outV)
	{
		static constexpr float epsSqr = std::numeric_limits<float>::epsilon() * std::numeric_limits<float>::epsilon();

		const Vec3 ab = inB - inA;
		const float denominator = LengthSquared(ab);
		if (denominator < (epsSqr))
		{
			// Degenerate line segment, fallback to points
			if (LengthSquared(inA) < LengthSquared(inB))
			{
				// A closest
				outU = 1.0f;
				outV = 0.0f;
			}
			else
			{
				// B closest
				outU = 0.0f;
				outV = 1.0f;
			}
			return false;
		}
		else
		{
			outV = -DotProduct(inA, ab) / denominator;
			outU = 1.0f - outV;
		}
		return true;
	}

	/// Get the closest point to the origin of line (inA, inB)
	/// outSet describes which features are closest: 1 = a, 2 = b, 3 = line segment ab
	template<CVec3 Vec3>
	inline Vec3	GetClosestPointOnLine(const Vec3& inA, const Vec3& inB, uint32_t& outSet)
	{
		float u, v;
		GetBaryCentricCoordinates(inA, inB, u, v);
		if (v <= 0.0f)
		{
			// inA is closest point
			outSet = 0b0001;
			return inA;
		}
		else if (u <= 0.0f)
		{
			// inB is closest point
			outSet = 0b0010;
			return inB;
		}
		else
		{
			// Closest point lies on line inA inB
			outSet = 0b0011;
			return u * inA + v * inB;
		}
	}
} // namespace JPL