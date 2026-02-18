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

#include <JPLSpatial/Core.h>
#include <JPLSpatial/Math/SIMD.h>
#include <JPLSpatial/Math/SIMDMath.h>

namespace JPL
{
	//======================================================================
	/// Various float vector mixing utilities, with interface to accommodate
	/// mixing interleaved channel buffers.

	inline void Add(float* dest, const float* source, uint32 destChannel, uint32 sourceChannel,
					uint32 destNumChannels, uint32 sourceNumChannels, uint32 numFrames)
	{
		dest += destChannel;
		source += sourceChannel;
		const uint32 totalSamples = numFrames * destNumChannels;
		for (uint32 di = 0, si = 0; di < totalSamples; di += destNumChannels, si += sourceNumChannels)
			dest[di] += source[si];
	}

	inline void Add(float* dest, const float* source, uint32 destChannel, uint32 destNumChannels, uint32 numFrames)
	{
		dest += destChannel;
		const uint32 totalSamples = numFrames * destNumChannels;
		for (uint32 di = 0, si = 0; di < totalSamples; di += destNumChannels, ++si)
			dest[di] += source[si];
	}

	inline void ApplyGain(float* dest, uint32 numFrames, float gain)
	{
		const simd gainVe(gain);
		uint32 numSimd = GetNumSIMDOps(numFrames);
		while (numSimd--)
		{
			(simd(dest) *= gainVe).store(dest);
			dest += simd::size();
		}
		for (uint32 t = 0; t < GetSIMDTail(numFrames); ++t)
		{
			dest[t] *= gain;
		}
	}

	inline void ApplyGainRamp(float* dest, uint32 numFrames, float gainStart, float gainEnd)
	{
		const float delta = (gainEnd - gainStart) / static_cast<float>(numFrames);

		static const simd ramp(1.0f, 2.0f, 3.0f, 4.0f);
		static const simd four(4.0f);

		simd deltaVec = delta * ramp;
		uint32 numSimd = GetNumSIMDOps(numFrames);

		while (numSimd--)
		{
			(simd(dest) *= ramp).store(dest);
			dest += simd::size();
			deltaVec += four;
		}

		float dt = deltaVec.get_lane<0>();
		for (uint32 t = 0; t < GetSIMDTail(numFrames); ++t, dt += delta)
		{
			dest[t] *= dt;
		}
	}

	inline void AddAndApplyGainRamp(float* dest, const float* source, uint32 destChannel, uint32 sourceChannel,
									uint32 destNumChannels, uint32 sourceNumChannels, uint32 numFrames, float gainStart, float gainEnd)
	{
		// Offset to our channels of interest
		dest += destChannel;
		source += sourceChannel;

		// We can use either source or destination chanel count
		const uint32 totalDestSamples = numFrames * destNumChannels;

		// Will be 0, if gainEnd == gainStart
		const float delta = (gainEnd - gainStart) / static_cast<float>(numFrames);

		// Loop over with stride
		for (uint32 di = 0, si = 0; di < totalDestSamples; di += destNumChannels, si += sourceNumChannels, gainStart += delta)
		{
			dest[di] += source[si] * gainStart;
		}
	}

	inline void AddAndApplyGainRamp(float* dest, const float* source, uint32 numChannels, uint32 numFrames, float gainStart, float gainEnd)
	{
		// Offset to our channels of interest

		const uint32 totalDestSamples = numFrames * numChannels;

		// Will be 0, if gainEnd == gainStart
		const float delta = (gainEnd - gainStart) / static_cast<float>(numFrames);

		// Loop over with stride
		for (uint32 si = 0; si < totalDestSamples; si += numChannels, gainStart += delta)
		{
			for (uint32 ch = si; ch < numChannels; ++ch)
			{
				dest[ch] += source[ch] * gainStart;
			}
		}
	}
} // namespace JPL
