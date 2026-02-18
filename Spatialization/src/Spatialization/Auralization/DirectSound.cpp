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

#include <JPLSpatial/Auralization/ChannelMixing.h>
#include <JPLSpatial/Auralization/DirectSound.h>
#include <JPLSpatial/Containers/StaticArray.h>

#include <algorithm>
#include <cstring>

// Disable/enable features for testing/debugging
#define JPL_ER_USE_DELAY 1
#define JPL_ER_USE_FILTER 1
#define JPL_ER_USE_PAN 1

namespace JPL
{
	DirectSoundEffect::DirectSoundEffect(float sampleRate,
										 uint32 numSourceChannels,
										 uint32 numOutputChannels,
										 const simd initialFilterGains,
										 float initialDelayTimeInSeconds,
										 std::span<const float> initialMixMap)
		: mTargetData(DirectEffectTargetData(JPL::GetDefaultMemoryResource()), JPL::GetDefaultMemoryResource())
		, mCurrentGains(JPL::GetDefaultMemoryResource())
	{
		JPL_ASSERT(initialMixMap.size() == numSourceChannels * numOutputChannels);

		mSampleRate = sampleRate;
		
		mCurrentGains.resize(numSourceChannels * numOutputChannels);
		std::ranges::copy(initialMixMap, mCurrentGains.begin());
		
		// Convert delay time from seconds to to samples
		const float initialDelayTime = initialDelayTimeInSeconds * mSampleRate;

		mDelayTime = JPL::SmoothedValue<float>::CreateExpSmoothing(initialDelayTime, initialDelayTime, cScratchSize);
		mFilterGains = initialFilterGains;
		{
			// Initialize target data
			SafeDirectEffectTDWrite targetData(mTargetData);
			targetData->FilterGains = mFilterGains;
			targetData->DelayTime = initialDelayTime;
			targetData->ChannelGains = mCurrentGains;
		}

		// Allocate space for channel data
		mChannels = { GetDefaultPmrAllocator().allocate_object<ChannelRTData>(numSourceChannels), numSourceChannels };
		
		// Construct channel data objects
		for (uint32 i = 0; i < mChannels.size(); ++i)
		{
			new(&mChannels[i]) ChannelRTData();
		}

		// Initialize channels
		for (ChannelRTData& channel : mChannels)
		{
			channel.Tap = ChannelRTData::DelayLineType::CreateTap<typename ChannelRTData::TapType>(0.0f);

			// TODO: maybe reuse the same delay line for all channels?
			// TODO: get max delay time from some config
			channel.DelayLine = make_pmr_unique<typename ChannelRTData::DelayLineType>(static_cast<uint32>(sampleRate * 2.0f));

			// TODO: get frequency splits from some config as well
			channel.Filter.Prepare(sampleRate, {});
		}
	}

	DirectSoundEffect::~DirectSoundEffect()
	{
		for (ChannelRTData& channel : mChannels)
		{
			channel.~ChannelRTData();
		}
		GetDefaultPmrAllocator().deallocate_object(mChannels.data(), mChannels.size());
	}

	void DirectSoundEffect::ProcessInterleaved(std::span<const float> input,
											   std::span<float> output,
											   uint32 numFrames)
	{
		const uint32 numInChannels = input.size() / numFrames;
		const uint32 numOutChannels = output.size() / numFrames;
		
		// Sanity checks
		JPL_ASSERT(numInChannels == mChannels.size());
		JPL_ASSERT(numInChannels * numOutChannels == mCurrentGains.size());

		uint32_t framesProcessed = 0;
		while (numFrames > 0)
		{
			const uint32_t framesToProcess = std::min(numFrames, cScratchSize);

			std::span<const float> inputChunk = input.subspan(framesProcessed * numInChannels);
			std::span<float> outputChunk = output.subspan(framesProcessed * numOutChannels);

			framesProcessed += framesToProcess;

			// Push new samples to delay lines
			const uint32 samplesToProcess = inputChunk.size();
			for (uint32 channelIndex = 0; channelIndex < numInChannels; ++channelIndex)
			{
				auto& delayLine = *mChannels[channelIndex].DelayLine;
				const float* in = inputChunk.data() + channelIndex;

				for (uint32_t s = 0; s < samplesToProcess; s += numInChannels)
				{
					const float sample = in[s];
					delayLine.Push(sample);
				}
			}

			// ..after the above, we have pushed 'scratchSize' sample into the delay line
			// so we need to take this offset into account when reading taps.

			StaticArray<float, cScratchSize> buffer(framesToProcess);

			// Grab the updated data
			simd targetFilterGains;
			float targetDelayTime;
			{
				SafeDirectEffectTDRead tdRead(mTargetData);
				targetFilterGains = tdRead->FilterGains;
				targetDelayTime = tdRead->DelayTime;
			}

			// Prepare buffer for all the delay times we need
			StaticArray<float, cScratchSize> delayTimesBuffer(framesToProcess);
			mDelayTime.Target = targetDelayTime;
			for (uint32_t s = 0; s < framesToProcess; ++s)
			{
				delayTimesBuffer[s] = static_cast<float>(mDelayTime.GetNext());
			}

			// Read and process channel taps
			for (uint32 inChannelIndex = 0; inChannelIndex < numInChannels; ++inChannelIndex)
			{
				ChannelRTData& channel = mChannels[inChannelIndex];

#if JPL_ER_USE_DELAY
				// Process delay
				{
					// Add scratch size that we are "behind"
					int delayOffset = static_cast<int>(framesToProcess);

					// We have to do this sample by sample because the timespan
					// may be larger than framesToProcess due to delay time interpolation.
					// Essentially it may "move us out" of the window we request from the delay line.
					for (uint32_t s = 0; s < framesToProcess; ++s)
					{
						channel.Tap.SetDelay(static_cast<float>(delayOffset) + delayTimesBuffer[s]);

						// Wite delayed sample into the scratch buffer
						buffer[s] = channel.Tap.Process(*channel.DelayLine);

						// Advance the read offset
						delayOffset -= 1;
					}

				}
#else
				//? Temp testing just panning
				for (uint32_t d = 0; d < framesToProcess; ++d)
				{
					buffer[d] = channel.DelayLine->GetReadWindow<1>(framesToProcess - d);
				}
#endif

#if JPL_ER_USE_FILTER
				// Process this tap with filters
				channel.Filter.ProcessBlock(buffer, mFilterGains, targetFilterGains, buffer);
#endif
				float* currentChannelGains = &mCurrentGains[inChannelIndex * numOutChannels];

				JPL_ASSERT(numOutChannels <= 32, "Number of target channel excede static buffer. Increase buffer size, if this is not an error.");
				StaticArray<float, 32> targetChannelGains(numOutChannels);
				{
					SafeDirectEffectTDRead targetData(mTargetData);
					std::memcpy(targetChannelGains.data(), &targetData->ChannelGains[inChannelIndex * numOutChannels], numOutChannels * sizeof(float));
				}

				// Mix/add this input channel to each of the ouptut channels
				for (uint32_t outChannelIndex = 0; outChannelIndex < numOutChannels; ++outChannelIndex)
				{
					// TODO: we could improve this if we apply gain ramp first to contiguous 'buffer',
					//		then add it to the interleaved output. Or setup deinterleaved callback.
#if JPL_ER_USE_PAN

					AddAndApplyGainRamp(outputChunk.data(),
										buffer.data(),
										outChannelIndex, 0,
										numOutChannels, 1,
										framesToProcess,
										currentChannelGains[outChannelIndex],
										targetChannelGains[outChannelIndex]);
#else
					Add(outputChunk.data(),
						buffer.data(),
						outChannelIndex,
						numOutChannels,
						framesToProcess);
#endif

					currentChannelGains[outChannelIndex] = targetChannelGains[outChannelIndex];
				}
			}

			mFilterGains = targetFilterGains;

			numFrames -= framesToProcess;
		} // while (numFrames > 0)
	}

	void DirectSoundEffect::UpdateParameters(const simd& filterGains, float delayTime, std::span<const float> channelMixMap)
	{
		SafeDirectEffectTDWrite targetData(mTargetData);
		JPL_ASSERT(channelMixMap.size() == targetData->ChannelGains.size());

		targetData->FilterGains = filterGains;
		targetData->DelayTime = delayTime * mSampleRate;
		std::ranges::copy(channelMixMap, targetData->ChannelGains.begin());
	}

} // namespace JPL
