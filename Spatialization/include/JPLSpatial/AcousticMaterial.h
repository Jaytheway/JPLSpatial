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
#include "JPLSpatial/FrequencyBands.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/DecibelsAndGain.h"

#include <algorithm>
#include <cmath>
#include <concepts>
#include <ostream>
#include <string_view>
#include <unordered_map>
#include <utility>

namespace JPL
{
	template<CFloatOrSIMD T>
	JPL_INLINE T ReflectionLoss_dB(const T& alpha)
	{
		if constexpr (std::same_as<T, simd>)
		{
			const T reflectance = max(T(1e-6f), T(1.0f) - alpha);
			return IntencityTodB(reflectance);
		}
		else
		{
			const T reflectance = std::max(T(1e-6f), T(1.0f) - alpha);
			return IntencityTodB(reflectance);
		}
	}

	/// Acoustical properties of a material
	struct AcousticMaterial
	{
		using List = std::unordered_map<uint32, AcousticMaterial>;
		
		using AbsorptionCoeffs = JPL::AbsorptionCoeffs;
		using FreqBandCenters = JPL::FreqBandCenters;

		// The bands are 0-176 Hz, 176-775 Hz, 775-3408 Hz, and 3408-22050 Hz
		inline static const FreqBandCenters cBandCenters = JPL::cBandCenters;
		// TODO: user-provided frequency bands

		std::string_view Name;
		uint32 ID = 0;

		// TODO: recompute these when user defined Acoustic Material coefficients changed
		
		// Crude average in dB scalse
		float AbsorptionAverage_dB = 0.0f;
		
		// Crude average
		float AbsorptionAverage = 0.0f;
		
		// 1.0 - AbsorptionAverage
		float AbsorptionAverageOneMinus = 1.0f;

		AbsorptionCoeffs Coeffs; // alpha coefficient
		AbsorptionCoeffs Absorption_dB;
		AbsorptionCoeffs AmplitudeFactors;

		static const AcousticMaterial* Get(uint32 materialID);
		static const AcousticMaterial* Get(std::string_view materialName);
		static const AcousticMaterial& GetDefault();
		static const List& GetListOfMaterials() { return sList; }

		JPL_INLINE void Accumulate(AbsorptionCoeffs& inOutCoefMultiplier) const
		{
			inOutCoefMultiplier *= simd(1.0f) - Coeffs;
		}

		/// If material by the `name` exists, set material coefficients,
		/// add new material otherwise.
		static void SetMaterial(std::string_view name, const AbsorptionCoeffs& coeffs);
		
		/// Make material without adding it to global static list.
		static std::pair<uint32, AcousticMaterial> MakeMaterial(std::string_view name, const AbsorptionCoeffs& coeffs);

	private:
		// Make sure the List is initialized anyone tries to use it
		static List sList;
	};

	inline std::ostream& operator<<(std::ostream& os, const AcousticMaterial& v)
	{
		os << v.Name << ": ";
		os << v.Coeffs;
		return os;
	}
} // namespace JPL