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

#pragma once

#include "JPLSpatial/Core.h"
#include "JPLSpatial/Utilities/Hash.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Math/SIMD.h"

#include <span>

namespace JPL
{
	/// A very experimental specular path definition

	//======================================================================
	/// Specular path identifier representing a sequence of reflections
	/// between a given source and listener.
	// TODO: we may or may not want to keep id list for the entire path
	//		to check for source/listener matches
	struct SpecularPathId
	{
		uint32 Id = Hash::cStartSeed;

		// If the RISM is implemented as a depth-first ray traversal,
		// the path identifier can be efficiently incrementally
		// constructed with each successive ray reflection.
		JPL_INLINE constexpr void AddVertex(uint32 id) { HashCombine32(Id, id); }

		JPL_INLINE constexpr void FinalizeHash() { Id = Murmur3Finalize(Id); }

		// TODO: we may want to call FinalizeHash when comparing, or when converting to std::hash
		[[nodiscard]] constexpr bool operator==(const SpecularPathId&) const = default;
	};

	//======================================================================
	// We simulate propagation in 4 frequency bands:
	//	0-176 Hz, 176-775 Hz, 775-3408 Hz, and 3408-22050 Hz
	//	..or as center frequencies: 59 Hz, 369 Hz, 1625 Hz, 8669 Hz
	using EnergyBands = simd;

	//======================================================================
	class AirAbsorption
	{
	public:
		// Default air attenuation coefficients per meter at temperature 21C, humidity 45%, standard air pressure
		inline static const EnergyBands cDefaultCoeffs_dB{
			0.000115f,	// 0.00011533632007753075
			0.002130f,	// 0.002130442215493568
			0.007863f,	// 0.007863092294756541
			0.129334f	// 0.12933369065280596
		};
		// effective gain factors per meter { 0.999987, 0.999755, 0.999095, 0.98522 }

		static void ComputeForDistance(float distance, EnergyBands& outAttenuation)
		{
			JPL_ASSERT(distance >= 0.0f);
			outAttenuation = cDefaultCoeffs_dB * distance;
		}
	};

	//======================================================================
	template<CVec3 Vec3Type>
	struct alignas(JPL_CACHE_LINE_SIZE) SpecularPathData
	{
		// Sequence of identifiers of source, surfaces, and listener
		std::span<int> Nodes; // { source, s0, s1..., listener }
		//! Note: memory is owned by SpecularPathCache object

		// Combined energy intencity dB loss
		EnergyBands Energy;

		// IS position or direction
		Vec3Type ImageSource;

		bool bValid;
	};

} // namespace JPL


namespace std
{
	template <>
	struct hash<JPL::SpecularPathId>
	{
		[[nodiscard]] constexpr std::size_t operator()(const JPL::SpecularPathId& key) const
		{
			return static_cast<std::size_t>(key.Id);
		}
	};
} // namespace std
