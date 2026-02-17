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
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"

#include <cmath>
#include <concepts>
#include <numbers>
#include <type_traits>

namespace JPL
{
	template<class T>
	concept CFloatOrSIMD = std::same_as<float, std::remove_cvref_t<T>> || std::same_as<simd, std::remove_cvref_t<T>>;

	template<CFloatOrSIMD T>
	JPL_INLINE T dBToGain(const T& dB) noexcept
	{
		// Our base of `pow` is always 10, 
		// so we don't need half the error handlign checks of `pow`.

		// This is effectively a cheaper version of pow(10.0f, dB / 20.0f)
		static const T ln10div20 = std::numbers::ln10_v<float> * 0.05f;
		if constexpr (std::same_as<T, simd>)
			return exp(ln10div20 * dB);
		else
			return std::exp(ln10div20 * dB);
	}

	template<CFloatOrSIMD T>
	JPL_INLINE T dBToIntencity(const T& dB) noexcept
	{
		// A cheaper version of pow(10.0f, dB / 10.0f)
		static const T ln10div10 = std::numbers::ln10_v<float> * 0.1f;
		if constexpr (std::same_as<T, simd>)
			return exp(ln10div10 * dB);
		else
			return std::exp(ln10div10 * dB);
	}

	template<CFloatOrSIMD T>
	JPL_INLINE T GainTodB(const T& gainFactor) noexcept
	{
		if constexpr (std::same_as<T, simd>)
			return -20.0f * log10(gainFactor);
		else
			return -20.0f * std::log10(gainFactor);
	}

	template<CFloatOrSIMD T>
	JPL_INLINE T IntencityTodB(const T& intencityFactor) noexcept
	{
		if constexpr (std::same_as<T, simd>)
			return -10.0f * log10(intencityFactor);
		else
			return -10.0f * std::log10(intencityFactor);
	}
} // namespace JPL