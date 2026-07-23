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

#include <JPLSpatial/Core.h>
#include <JPLSpatial/Auralization/CrossoverFilter.h>
#include <JPLSpatial/Auralization/ReverbDesign.h>
#include <JPLSpatial/Math/DecibelsAndGain.h>
#include <JPLSpatial/Math/SIMD.h>
#include <JPLSpatial/Utilities/RealtimeObject.h>

#include <span>

namespace JPL
{
	//==========================================================================
	/// Multi-channel reverb processing bus implementing a feedback delay network (FDN)
	/// with four-band attenuation filters for dynamic RT60 control.
	class ReverbBus
	{
		static constexpr uint32 cFDNChannels = 16; // FDN order
		static constexpr uint32 cScratchSize = 480;

		// Arbitrary min/max reverb time range
		static constexpr float cMinRT60 = 0.03f;
		static constexpr float cMaxRT60 = 10.0f;

		static constexpr float cMinSlope = -60.0f / cMinRT60;
		static constexpr float cMaxSlope = -60.0f / cMaxRT60;

		//======================================================================
		/// Crossover filter bank applying per band decay gains within FDN
		struct AttenuationFilter
		{
			JPL_INLINE void Prepare(float sampleRate, const SplitFrequencies& splits)
			{
				Crossover.Prepare(sampleRate, splits);
			}
			
			[[nodiscard]] JPL_INLINE float Process(float sample)
			{
				return Crossover.ProcessSample(sample, DecayGains);
			}
			
			/// @param delaySeconds Delay-line length in seconds.
			/// @param decaySlopeDbPerSecond Reverberation decay slope in dB/s.
			JPL_INLINE void UpdateParameters(float delaySeconds, const simd& decaySlopeDbPerSecond)
			{
				const simd dbLossPerLoop = decaySlopeDbPerSecond * delaySeconds;
				DecayGains = dBToGain(dbLossPerLoop);
			}

			FourBandCrossover Crossover;
			simd DecayGains{ 0.999f };
		};

		using FDN = MultiChannelMixedFeedback<cFDNChannels, AttenuationFilter>;

	public:
		//======================================================================
		ReverbBus() = default;
		~ReverbBus() noexcept = default;

		/// Set reverberation time in four frequency bands
		/// as per split frequencies set in Prepare()
		/// Must be called from a single non-audio thread.
		/// May run concurrently with ProcessInterleaved().
		void SetRT60(const simd& newRT60Seconds);

		/// Set reverberation time in four frequency bands
		/// as decay slope -dB/s.
		/// Must be called from a single non-audio thread.
		/// May run concurrently with ProcessInterleaved().
		void SetDecaySlope(const simd& newDecaySlope);

		/// Prepare FDN and filters for processing.
		/// Must be called from a single non-audio thread before the audio thread
		/// begins calling ProcessInterleaved().
		/// 
		/// @param sampleRate	sampling rate of the process block
		/// @param splits		split frequencies for the attenuation decay crossover filter
		void Prepare(float sampleRate, const SplitFrequencies& splits = {});

		/// Process interleaved block of samples, producing interleaved output.
		/// Real-time safe. May be called from one audio thread after Prepare().
		///
		/// @param input Interleaved input. Multichannel input is mixed down to
		/// mono before entering the FDN.
		/// @param output Interleaved output buffer.
		/// @param numFrames Number of frames in both buffers. Input and output
		/// channel counts are inferred from each buffer's size.
		void ProcessInterleaved(std::span<const float> input, std::span<float> output, uint32 numFrames);

	private:
		FDN mFDN;
	
		using SafeRT60 = JPL::RealtimeObject<simd, RealtimeObjectOptions::nonRealtimeMutatable>;
		using SafeRT60Write = typename SafeRT60::ScopedAccess<JPL::ThreadType::nonRealtime>;
		using SafeRT60Read = typename SafeRT60::ScopedAccess<JPL::ThreadType::realtime>;
		SafeRT60 mDecaySlopeDbPerSecond{ -60.0f / 1.0f};
	};
} // namespace JPL
