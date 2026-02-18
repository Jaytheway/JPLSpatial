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
#include <JPLSpatial/Auralization/DelayLine.h>
#include <JPLSpatial/Auralization/CrossoverFilter.h>
#include <JPLSpatial/Math/SIMD.h>
#include <JPLSpatial/Memory/Memory.h>
#include <JPLSpatial/Utilities/RealtimeObject.h>
#include <JPLSpatial/Utilities/SmoothedValue.h>

#include <vector>
#include <span>

namespace JPL
{
	//==================================================================================
	/// Direct Sound Effect Bus transfering direct sound propagaion data to audio
	/// rendering thread. It uses its own delay line with at least 2 seconds buffer
	/// space and performs rendering of the direct sound using interpolating delay line
	/// taps, simulating propagation delay, applying pan and propagation filtering.
	/// The memory is managed by the default global JPL PMR allocator.
	class DirectSoundEffect
	{
	public:
		DirectSoundEffect(float sampleRate,
						  uint32 numSourceChannels,
						  uint32 numOutputChannels,
						  const simd initialFilterGains,
						  float initialDelayTimeInSeconds,
						  std::span<const float> initialMixMap);
		~DirectSoundEffect();

		/// Must be called from audio thread.
		/// Number of input channels inferred from `intput.size() / numFrames`.
		/// Number of output channels inferred from `output.size() / numFrames`.
		/// Both channel counts must match what as passed to constructor.
		/// 
		/// @input and @output must have the same nubmer of frames
		void ProcessInterleaved(std::span<const float> input, std::span<float> output, uint32 numFrames);

		/// Must be called from non-RT thread
		void UpdateParameters(const simd& filterGains, float delayTimeSeconds, std::span<const float> channelMixMap);

	private:
		//==============================================================================
		// Data modified by non-realtime thread
		struct DirectEffectTargetData
		{
			explicit DirectEffectTargetData(std::pmr::memory_resource* memoryResource)
				: FilterGains(1.0f)
				, DelayTime(0.0f)
				, ChannelGains(memoryResource)
			{
			}

			JPL::simd FilterGains;
			float DelayTime; // in samples

			std::pmr::vector<float> ChannelGains;
		};

		//==============================================================================
		// Data modified by realtime thread
		class ChannelRTData final
		{
		public:
			static constexpr uint32 cScratchSize = 480;

			using TapType = JPL::DelayTap<JPL::Thiran1stInterpolator>;
			using DelayLineType = JPL::DelayLine<cScratchSize + ChannelRTData::TapType::InterpolatorType::InputLength>;

			ChannelRTData() = default;
			~ChannelRTData() = default;

		public:
			// Propagation delay
			TapType Tap;
			pmr_unique_ptr<DelayLineType> DelayLine;

			// Propagation filtering
			JPL::FourBandCrossover Filter;
		};

		//==============================================================================
		static constexpr uint32 cScratchSize = ChannelRTData::cScratchSize;

		using SafeDirectEffectTD = JPL::RealtimeObject<DirectEffectTargetData>;
		using SafeDirectEffectTDWrite = typename SafeDirectEffectTD::ScopedAccess<JPL::ThreadType::nonRealtime>;
		using SafeDirectEffectTDRead = typename SafeDirectEffectTD::ScopedAccess<JPL::ThreadType::realtime>;

	private:
		SafeDirectEffectTD mTargetData;

		float mSampleRate;

		// After initialization must be accessed only from RT thread
		std::span<ChannelRTData> mChannels;
		JPL::SmoothedValue<float> mDelayTime;
		std::pmr::vector<float> mCurrentGains;
		simd mFilterGains;

	};
} // namespace JPL
