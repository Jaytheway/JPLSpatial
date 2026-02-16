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

#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Auralization/FractionalDelay.h"
#include "JPLSpatial/Utilities/AbstractIndex.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <numbers>
#include <vector>
#include <span>

namespace JPL
{
	//==========================================================================
	/// Forward declaration
	template<uint32_t MaxWindow>
	class DelayLine;

	//==========================================================================
	/// Simple delay tap that supports sub-sample interpolation
	/// via Interpolator template interface
	template<class InterpolatorT = Thiran1stInterpolator>
	struct DelayTap
	{
	public:
		using InterpolatorType = InterpolatorT;

		int DelaySample = 0; // whole-sample part
		InterpolatorType Interpolator;

		/// Set delay is in samples, can be fractional
		inline void SetDelay(float delay)
		{
			JPL_ASSERT(delay >= 0.0f && "Delay must be non-negative");
			DelaySample = static_cast<uint32_t>(delay);
			const float delayFraction = delay - DelaySample;
			Interpolator.SetFraction(delayFraction);
		}

		/// Process delay interpolation and get the delayed sample
		template<class DelayLine> requires(DelayLine::WindowSize >= InterpolatorType::InputLength)
		inline float Process(const DelayLine& delayLine)
		{
			auto data = delayLine.template GetReadWindow<InterpolatorType::InputLength>(DelaySample);
			return Interpolator.Process(data);
		}
	};

	//==========================================================================
	/// Delay Tap crossfading from previous to target sample position
	/// using simple cosine crossfade
	struct DelayTapXFade
	{
		static constexpr uint32_t InputLength = 1;

		using InterpolatorType = DelayTapXFade;

		/// Set crossfade fraction
		/// @param alpha	: must be in (0, 1)
		inline void SetCrossfade(float alpha)
		{
			mGainA = 0.5f * (1.0f + cosf(std::numbers::pi_v<float> * alpha));
			mGainB = 1.0f - mGainA;
		}

		/// Set target delay is in samples,
		/// the old delay tap time is crossfaded with the new time
		inline void SetDelay(float delay)
		{
			JPL_ASSERT(delay >= 0.0f && "Delay must be non-negative");

			//! For now we use discrete values,
			// since we crossfade smoothely
			// we may not need to interpolate samples
			mLastDelay = mNewDelay;
			mNewDelay = static_cast<int>(delay);

			// TODO: if we were in the middle of a crossfade,
			// do we want to swap the gains and "continue"
			// crossfading action into the new delay time?
			// std::swap(mGainA, mGainB);
			// ..fade out from mGainA current value...
			// 
			// OR: avoid setting new target delay in the middle of a crossfade!
		}

		/// Process delay crossfade and get the delayed sample
		template<class DelayLine> requires(DelayLine::WindowSize >= InputLength)
		inline float Process(const DelayLine& delayLine)
		{
			const float tapA = delayLine.GetReadWindow<InputLength>(mLastDelay);
			const float tapB = delayLine.GetReadWindow<InputLength>(mNewDelay);
			return tapA * mGainA + tapB * mGainB;
		}

	private:
		int mLastDelay = 0;
		int mNewDelay = 0;
		float mGainA = 1.0f;
		float mGainB = 0.0f;
	};

	//==========================================================================
	/// DelayLine is a ring buffer suitable for multi-tap delays.
	///
	/// Template parameter `MaxWindow` defines the maximum `InputLength`
	/// of the Interpolator this DelayLine can serve
	template<uint32_t MaxWindow = 2>
	class DelayLine
	{
	public:
		static constexpr uint32_t WindowSize = MaxWindow;

	private:
		RingIndexBackwardFast<WindowSize> mIndex;
		std::vector<float> mBuffer; // size = Ring + WindowSize

	public:
		// @param minSize: minimum required size of the delay in samples,
		// the actual lengh of the delay buffer is going to be next power of 2
		explicit DelayLine(uint32_t minSize)
			: mIndex(minSize)
			, mBuffer(mIndex.GetTotalSize(), 0.0f)
		{
		}

		template<class TapType> requires (TapType::InterpolatorType::InputLength <= WindowSize)
		static TapType CreateTap(float initialDelay = 0.0f)
		{
			TapType tap;
			tap.SetDelay(initialDelay);
			return tap;
		}

		template<class Interpolator> requires (Interpolator::InputLength <= WindowSize)
		static DelayTap<Interpolator> CreateTap(float initialDelay = 0.0f)
		{
			DelayTap<Interpolator> tap;
			tap.SetDelay(initialDelay);
			return tap;
		}

		inline uint32_t GetSize() const { return mIndex.GetRingSize(); }
		inline uint32_t GetWriteIndex() const { return mIndex.GetCurrent().WriteIndex; }
		inline const float* raw() const { return mBuffer.data(); }

		inline void Clear()
		{
			std::fill(mBuffer.begin(), mBuffer.end(), 0.0f);
			mIndex.Reset();
		}

		// Push one live sample into the ring buffer
		inline void Push(float sample)
		{
			const auto [writeIndex, mirrorIndex] = mIndex--;

			// Write into ring and mirror
			mBuffer[writeIndex] = sample;
			mBuffer[mirrorIndex] = sample;
		}

		// Return span to a contiguous data window that start `intDelay` samples
		// The returned data is guaranteed to be of length `max(WindowLength, 1)`
		template<uint32_t WindowLength> requires (WindowLength > 1 && WindowLength <= WindowSize)
		inline std::span<const float, WindowLength> GetReadWindow(uint32_t intDelay) const
		{
#if 0 // Ring index writing forward
			// start of contiguous `MaxWindow` window at an offset from the WriteIndex
			const uint32_t readIndex = mIndex.GetOffset(intDelay + WindowLength);
#else
			// +1 because offset is from the current write position,
			// while with delay 0 we need last written sample
			const uint32_t readIndex = mIndex.GetOffset(intDelay + 1);
#endif
			return std::span<const float, WindowLength>(&mBuffer[readIndex], WindowLength);
		}

		// Return sample at `intDelay`
		template<uint32_t WindowLength> requires (WindowLength <= 1)
		inline float GetReadWindow(uint32_t intDelay) const
		{
#if 0 // Ring index writing forward
			static constexpr uint32_t windowLength = 1;
			// start of contiguous `MaxWindow` window at an offset from the WriteIndex
			const uint32_t readIndex = mIndex.GetOffset(intDelay + windowLength);
#else
			// +1 because offset is from the current write position,
			// while with delay 0 we need last written sample
			const uint32_t readIndex = mIndex.GetOffset(intDelay + 1);
#endif
			return mBuffer[readIndex];
		}
	};

} // namespace JPL
