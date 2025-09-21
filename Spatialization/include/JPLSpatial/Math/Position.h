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

#include "JPLSpatial/Core.h"
#include "JPLSpatial/Math/MinimalQuat.h"
#include "JPLSpatial/Math/MinimalBasis.h"

namespace JPL
{
	/// Minimum data required to do our orientation math
	template<CVec3Accessible Vec3Type>
	struct Orientation
	{
		// Note: we may just switch to Basis to store all three axes at some point,
		// unless we want to keep this Orientation as customization point
		// for the different axes semantics in different engies.

		Vec3Type Up;
		Vec3Type Forward;

		[[nodiscard]] JPL_INLINE static Orientation<Vec3Type> Identity() noexcept
		{
			return { .Up = Vec3Type(0, 1, 0), .Forward = Vec3Type(0, 0, 1) };
		}

		[[nodiscard]] JPL_INLINE Basis<Vec3Type> ToBasis() const noexcept
		{
			return Math::MakeBasis(Forward, Up);
		}

		// Use this if Up and Forward are guaranteed to be orthogonal
		[[nodiscard]] JPL_INLINE Basis<Vec3Type> ToBasisUnsafe() const
		{
			JPL_ASSERT(Math::IsNearlyZero(DotProduct(Up, Forward)));
			return Basis<Vec3Type>{
				.X = CrossProduct(Up, Forward),
				.Y = Up,
				.Z = Forward
			};
		}

		[[nodiscard]] JPL_INLINE Quat<Vec3Type> ToQuat() const noexcept
		{
			return Math::QuatFromUpAndForward(Up, Forward);
		}
	};

	/// Location and orientation in one place
	template<CVec3Accessible Vec3Type>
	struct Position
	{
		Vec3Type Location;
		Orientation<Vec3Type> Orientation;
	};

	template<CVec3Accessible Vec3Type>
	[[nodiscard]] JPL_INLINE bool operator==(const Orientation<Vec3Type>& lhs, const Orientation<Vec3Type>& rhs) noexcept
	{
		return lhs.Up== rhs.Up && lhs.Forward == rhs.Forward;
	}

	template<CVec3Accessible Vec3Type>
	[[nodiscard]] JPL_INLINE bool operator==(const Position<Vec3Type>& lhs, const Position<Vec3Type>& rhs) noexcept
	{
		return lhs.Location == rhs.Location && lhs.Orientation == rhs.Orientation;
	}
} // namespace JPL