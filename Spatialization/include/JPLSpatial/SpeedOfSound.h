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

#include "Math/Math.h"

namespace JPL
{
	//==========================================================================
	/// Speed of sound utility
	class SpeedOfSound
	{
	public:
		// Speed of sound at reference temperature (20C)
		static constexpr float cReference = 343.2f;

		//======================================================================
		/// Compute speed of sound for `temperature` in m/s.
		/// @param temperatureC is temperature in C, to calculate speed of sound for
		[[nodiscard]] static constexpr float ForTemperature(float temperatureC)
		{
			// ISO 9613-1, Equation A.5
			// Note: speed of sound changes with humidity up to 0.3%,
			// which is considered negligible.

			constexpr float zeroC_Kelvin = 273.15f;
			constexpr float invReferenceTemp = 1.0f / (20.0f + zeroC_Kelvin);
			return cReference * Math::Sqrt((zeroC_Kelvin + temperatureC) * invReferenceTemp);
		}

		// TODO: speed of sound in uncommon environments like liquids and space
	};

	// TODO: move JPL_SPEED_OF_SOUND here, or include this header in Math header where JPL_SPEED_OF_SOUND currently is

} // namespace JPL
