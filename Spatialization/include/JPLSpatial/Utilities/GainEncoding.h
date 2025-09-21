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

namespace JPL
{
	//==========================================================================
	// Compact 24-bit packing for normalzied [0, 1] gain values
	// (little-endian)
	struct Gain24Bit
	{
		static constexpr uint32 cU24max = (1 << 24) - 1;
		static constexpr float cInv = 1.0f / cU24max;

		uint8 Bytes[3];

		constexpr Gain24Bit() = default;

		JPL_INLINE explicit constexpr Gain24Bit(float gainNormalized)
		{
			constexpr auto cScale = static_cast<float>(1 << 24);
			uint32 w24 = static_cast<uint32_t>(gainNormalized * cScale);
			w24 -= (w24 >> 24);

			Bytes[0] = static_cast<uint8>(w24 & 0xFF);
			Bytes[1] = static_cast<uint8>((w24 >> 8) & 0xFF);
			Bytes[2] = static_cast<uint8>((w24 >> 16) & 0xFF);
		}

		JPL_INLINE constexpr Gain24Bit& operator=(float gainNormalized) noexcept
		{
			*this = Gain24Bit{ gainNormalized };
			return *this;
		}

		JPL_INLINE constexpr operator float() const noexcept
		{
			return (static_cast<uint32>(Bytes[0])
				| (static_cast<uint32>(Bytes[1]) << 8)
				| (static_cast<uint32>(Bytes[2]) << 16))
				* cInv;
		}
	};

	//==========================================================================
	// 16-bit packing for normalzied [0, 1] gain values
	struct Gain16Bit
	{
		static constexpr uint32 cU16max = (1 << 16) - 1;
		static constexpr float cInv = 1.0f / cU16max;

		uint16 Bytes;

		constexpr Gain16Bit() = default;

		JPL_INLINE explicit constexpr Gain16Bit(float gainNormalized)
		{
			constexpr auto cScale = static_cast<float>(1 << 16);
			auto w16 = static_cast<uint32>(gainNormalized * cScale);
			w16 -= (w16 >> 16);
			Bytes = w16;
		}

		JPL_INLINE constexpr Gain16Bit& operator=(float gainNormalized) noexcept
		{
			*this = Gain16Bit{ gainNormalized };
			return *this;
		}

		JPL_INLINE constexpr operator float() const noexcept
		{
			return Bytes * cInv;
		}
	};

	//==========================================================================
	namespace UnitTests
	{
		// Constexpr helper: absolute difference
		constexpr bool approx(float a, float b, float tol) noexcept
		{
			return (a > b ? a - b : b - a) <= tol;
		}

		constexpr float tol16 = Gain16Bit::cInv; // one LSB in amplitude
		constexpr float tol24 = Gain24Bit::cInv;

		// Mid-range sanity
		static_assert(approx(static_cast<float>(Gain16Bit{ 0.50f }), 0.50f, tol16), "Gain16 mid-range round-trip failed");
		static_assert(approx(static_cast<float>(Gain24Bit{ 0.25f }), 0.25f, tol24), "Gain24 mid-range round-trip failed");

		// Edge cases
		static_assert(static_cast<float>(Gain16Bit{ 0.0f }) == 0.0f, "Gain16 zero failed");
		static_assert(approx(static_cast<float>(Gain24Bit{ 1.0f }), 1.0f, tol24 * 2), "Gain24 full-scale failed");
	} // namespace UnitTests

} // namespace JPL