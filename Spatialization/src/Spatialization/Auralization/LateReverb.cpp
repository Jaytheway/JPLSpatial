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

#include "JPLSpatial/Auralization/LateReverb.h"

#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/Auralization/ChannelMixing.h"

#include <algorithm>
#include <bit>
#include <cstring>

namespace JPL
{
	void ReverbBus::SetRT60(const simd& newRT60Seconds)
	{
		const simd decaySlope = simd(-60.0f) / clamp(newRT60Seconds, cMinRT60, cMaxRT60);
		*SafeRT60Write(mDecaySlopeDbPerSecond) = decaySlope;
	}

	void ReverbBus::SetDecaySlope(const simd& newDecaySlope)
	{
		*SafeRT60Write(mDecaySlopeDbPerSecond) = clamp(newDecaySlope, cMinSlope, cMaxSlope);
	}

	void ReverbBus::Prepare(float sampleRate, const SplitFrequencies& splits)
	{
		mFDN.Prepare(sampleRate, splits);
		// Set initial values
		// (safe to use realtime read here becaues realtiem is not running yet)
		mFDN.UpdateFilterParameters(*SafeRT60Read(mDecaySlopeDbPerSecond));
	}

	/// Performance note:
	///	- most of the CPU time takes crossover
	/// ...which in the future may be replaced with simpler decay filter
	void ReverbBus::ProcessInterleaved(std::span<const float> input, std::span<float> output, uint32 numFrames)
	{
		// Update realtime parameters
		mFDN.UpdateFilterParameters(*SafeRT60Read(mDecaySlopeDbPerSecond));

		using FDNChannels = typename FDN::Channels;

		const uint32 numInChannels = input.size() / numFrames;
		const uint32 numOutChannels = output.size() / numFrames;

		JPL_ASSERT(numOutChannels <= cFDNChannels);

		// Downmix normalization factor
		const float channelNorm = 1.0f / numInChannels;

		// Process frames in chunks of cScratchSize
		uint32 framesProcessed = 0;
		while (numFrames > 0)
		{
			const uint32 framesToProcess = std::min(numFrames, cScratchSize);

			std::span<const float> inputChunk = input.subspan(framesProcessed * numInChannels);
			std::span<float> outputChunk = output.subspan(framesProcessed * numOutChannels);

			framesProcessed += framesToProcess;

			StaticArray<float, cScratchSize> buffer(framesToProcess);
			if (numInChannels > 1)
			{
				// Initialize to 0, because DownmixToMono adds to output
				buffer.assign(framesProcessed, 0.0f);

				// Downmix input to mono
				DownmixToMono(buffer.data(), inputChunk.data(), numInChannels, framesToProcess * numInChannels);

				// Normalize downmixed frames
				ApplyGain(buffer.data(), framesToProcess, channelNorm);
			}
			else
			{
				// Copy input to scratch buffer
				std::memcpy(buffer.data(), inputChunk.data(), sizeof(float) * framesToProcess);
			}

			FDNChannels fdnInput;

			for (uint32 frameIdx = 0, outSampleIdx = 0; frameIdx < buffer.size(); ++frameIdx, outSampleIdx += numOutChannels)
			{
				Matrix::FDNInput::InjectNormalized(buffer[frameIdx], fdnInput);

				FDNChannels fdnOutput = mFDN.Process(fdnInput);

				auto outputFrame = outputChunk.subspan(outSampleIdx, numOutChannels);

				if constexpr (std::has_single_bit(cFDNChannels))
				{
					Matrix::FDNOutputMixer::Mix(fdnOutput, outputFrame);
				}
				else
				{
					Matrix::FDNOutputProjection<>::Mix(fdnOutput, outputFrame);
				}
			}

			numFrames -= framesToProcess;
		}
	}

} // namespace JPL
