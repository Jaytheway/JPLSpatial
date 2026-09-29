//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPL Spatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2026 Jaroslav Pevno, JPL Spatial is offered under the terms of the ISC license:
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
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"

#include <algorithm>
#include <array>
#include <span>
#include <ranges>

namespace JPL
{
	using AbsorptionCoeffs = simd;
	using FreqBandCenters = simd;

	//======================================================================
	// Currently we simulate propagation in 4 frequency bands:
	//	20-176 Hz, 176-775 Hz, 775-3408 Hz, and 3408-22050 Hz
	//	..or as center frequencies: 59 Hz, 369 Hz, 1625 Hz, 8669 Hz
	using EnergyBands = simd;

	//======================================================================
	/// Split frequencies  points for crossover
	struct SplitFrequencies
	{
		float F1 = 176.0f, F2 = 775.0f, F3 = 3408.0f;

		[[nodiscard]] constexpr bool operator==(const SplitFrequencies&) const noexcept = default;
	};

	/// At the moment JPL Spatial uses these frequency sptits for sound propagation.
	inline constexpr SplitFrequencies cDefaultFrequencySplits{ .F1 = 176.0f, .F2 = 775.0f, .F3 = 3408.0f };

	//======================================================================
	[[nodiscard]] JPL_INLINE FreqBandCenters ComputeBandCenters(const SplitFrequencies& splitFrequenciesHz, float nyquist = 22050.0f)
	{
		const simd lowerEdge(20.0f, splitFrequenciesHz.F1, splitFrequenciesHz.F2, splitFrequenciesHz.F3);
		const simd upperEdge(splitFrequenciesHz.F1, splitFrequenciesHz.F2, splitFrequenciesHz.F3, nyquist);
		return Math::Sqrt(lowerEdge * upperEdge);
	}

	template<std::floating_point T, std::size_t N> requires(N != std::dynamic_extent)
	[[nodiscard]] JPL_INLINE std::array<T, N + 1> ComputeBandCenters(std::span<const T, N> splitFrequenciesHz, T nyquist = T(22050.0))
	{
		std::array<T, N + 1> centers{ T(20.0) }; // init first element as lower bound frequency
		std::ranges::copy(splitFrequenciesHz, &centers[1]);

		for (uint32 i = 0; i < centers.size() - 1; ++i)
			centers[i] *= splitFrequenciesHz[i];
		centers.back() *= nyquist;

		for (T& c : centers)
			c = Math::Sqrt(c);

		return centers;
	}

	[[nodiscard]] JPL_INLINE FreqBandCenters ComputeBandLowerThirdCenters(const SplitFrequencies& splitFrequenciesHz, float nyquist = 22050.0f)
	{
		const simd lowerEdge(20.0f, splitFrequenciesHz.F1, splitFrequenciesHz.F2, splitFrequenciesHz.F3);
		const simd upperEdge(splitFrequenciesHz.F1, splitFrequenciesHz.F2, splitFrequenciesHz.F3, 22050.0f);

		const simd bandRatio = upperEdge / lowerEdge;
		const simd firstThirdCenter = lowerEdge * pow(bandRatio, simd(1.0f / 6.0f)); // cbrt -> sqrt

		return firstThirdCenter;
	}

	[[nodiscard]] JPL_INLINE SplitFrequencies SplitFrequenciesFromBandCenters(const FreqBandCenters& bandCenters, float nyquist = 22050.0f)
	{
		float centers2[4]{};
		(bandCenters * bandCenters).store(centers2);

		const float f3 = centers2[3] / nyquist;
		const float f2 = centers2[2] / f3;
		const float f1 = centers2[1] / f2;

		return SplitFrequencies{ .F1 = f1, .F2 = f2, .F3 = f3 };
	}


	[[nodiscard]] JPL_INLINE FreqBandCenters ComputeBandLowerThirdCenters(const FreqBandCenters& bandCenters, float nyquist = 22050.0f)
	{
		return ComputeBandLowerThirdCenters(SplitFrequenciesFromBandCenters(bandCenters, nyquist), nyquist);
	}

	/// The bands are 0-176 Hz, 176-775 Hz, 775-3408 Hz, and 3408-22050 Hz
	/// Centers: 59 Hz, 369 Hz, 1625 Hz, 8669 Hz
	inline const FreqBandCenters cBandCenters = round(ComputeBandCenters(cDefaultFrequencySplits));

} // namespace JPL