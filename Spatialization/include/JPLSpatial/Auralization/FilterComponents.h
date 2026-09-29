//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPL Spatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno 2026, JPL Spatial is offered under the terms of the ISC license:
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

#include "JPLSpatial/Math/DecibelsAndGain.h"

#include <cmath>
#include <span>
#include <vector>

namespace JPL
{
	//==========================================================================
	namespace FilterUtils
	{
		template<class FilterType>
		void ComputeFilterResponse(const FilterType& filter, float sampleRate, std::vector<float>& outMagnitudes);

		template<class FilterType>
		void ComputeFilterResponse_dB(const FilterType& filter, float sampleRate, std::vector<float>& outMagnitudes);

		//======================================================================
		/// Simple helper to iterate over a semitone-spaced grid.
		/// 
		/// Implements of std::ranges::view_interface.
		/// Can be iterated over in a loop, or used with std::ranges/views utils:
		/// for (auto frequency : SemitoneGridView(optionalMinFrequency, sampleRate)
		///		foo(frequency);
		/// 
		template<std::floating_point T>
		class SemitoneGridView;
	} // namespace FilterUtils

	//==========================================================================
	/// Cached data for parameter updates
	namespace Cache
	{
		struct Shelf
		{
			float t;		// tan(PI * frequency / sampleRate)
			float sqrtK;	// sqrt(gain)
			float gain;
		};
	} // namespace Cache

	//==========================================================================
	namespace Topology
	{
		struct OnePole
		{
			using Sample = float;

			Sample b0 = 1.0f, b1 = 0.0f;
			Sample /*a0 = 0.0f,*/ a1 = 0.0f;

			// Calculate magnitude at the given normalized frequency
			[[nodiscard]] inline Sample CalculateResponse(float normalziedFrequency) const;

			[[nodiscard]] static JPL_INLINE OnePole MakeLowShelf(float frequency, float sampleRate, float gain);
			[[nodiscard]] static JPL_INLINE OnePole MakeHighShelf(float frequency, float sampleRate, float gain);

			[[nodiscard]] static JPL_INLINE OnePole MakeLowShelf(float normalizedFrequency, float gain);
			[[nodiscard]] static JPL_INLINE OnePole MakeHighShelf(float normalizedFrequency, float gain);

			[[nodiscard]] static inline OnePole MakeLowShelf(Cache::Shelf cache);
			[[nodiscard]] static inline OnePole MakeHighShelf(Cache::Shelf cache);
		};

		struct Biquad
		{
			using Sample = float;

			Sample b0 = 1.0f, b1 = 0.0f, b2 = 0.0f;
			Sample a1 = 0.0f, a2 = 0.0f;

			[[nodiscard]] inline std::complex<double> CalculateResponse(float normalizedFrequency) const;

			[[nodiscard]] static inline Biquad Combine(const OnePole& low, const OnePole& high, float broadbandGain);

			// RBJ with fixed slope=1
			[[nodiscard]] static JPL_INLINE Biquad MakeHighShelf(double normalizedFrequency, double gainDb);
			[[nodiscard]] static inline Biquad MakeHighShelf(const Cache::BiquadShelf& cache, double gainDb);

			template<class StateType>
			void ProcessInterpolating(std::span<Sample> samples, StateType& state, Biquad& previous) const;

			template<class StateType>
			void ProcessInterpolating(std::span<Sample> samples, uint32 stride, StateType& state, Biquad& previous) const;

			JPL_INLINE void SetCoefficients(const Topology::Biquad& other);
		};
	} // namespace Topology

	//==========================================================================
	namespace State
	{
		struct DirectFormIOnePole
		{
			using Sample = float;
			Sample x1 = 0.0f, y1 = 0.0f;

			JPL_INLINE void Reset() { x1 = y1 = 0.0f; }

			[[nodiscard]] JPL_INLINE Sample Process(Sample x, const Topology::OnePole& onePole)
			{
				const Sample y =
					(onePole.b0 * x) +
					(onePole.b1 * x1) -
					(onePole.a1 * y1);
				x1 = x;
				y1 = y;
				return y;
			}
		};

		struct DirectFormI
		{
			using Sample = float;
			Sample x1 = 0.0f, x2 = 0.0f, y1 = 0.0f, y2 = 0.0f;

			JPL_INLINE void Reset() { x1 = x2 = y1 = y2 = 0.0f; }

			[[nodiscard]] JPL_INLINE Sample Process(Sample x, const Topology::Biquad& biquad)
			{
				const Sample y =
					biquad.b0 * x +
					biquad.b1 * x1 +
					biquad.b2 * x2 -
					biquad.a1 * y1 -
					biquad.a2 * y2;
				x2 = x1;
				y2 = y1;
				x1 = x;
				y1 = y;
				return y;
			}
		};

		struct TransposedDirectFormII
		{
			using Sample = float;
			Sample s1 = 0.0f, s2 = 0.0f;

			JPL_INLINE void Reset() { s1 = s2 = 0.0f; }

			[[nodiscard]] JPL_INLINE Sample Process(Sample x, const Topology::Biquad& biquad)
			{
				const Sample y = biquad.b0 * x + s1;
				s1 = biquad.b1 * x - biquad.a1 * y + s2;
				s2 = biquad.b2 * x - biquad.a2 * y;
				return y;
			}
		};
	} // namespace State
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	namespace FilterUtils
	{
		//======================================================================
		template<std::floating_point T>
		class SemitoneGridView : public std::ranges::view_interface<SemitoneGridView<T>>
		{
		public:
			static constexpr T cSemitoneRatio = T(1.0594630943592953); // pow(2, 1/12)
			static constexpr T cMinFrequency = T(20.0);

		public:
			constexpr explicit SemitoneGridView(T minFrequency, T maxFrequency)
				: mMin(std::max(cMinFrequency, minFrequency)), mMax(maxFrequency)
			{
			}
			constexpr explicit SemitoneGridView(T maxFrequency)
				: SemitoneGridView(cMinFrequency, maxFrequency)
			{
			}

			class iterator;

			constexpr iterator begin() const noexcept
			{
				return iterator(mMin, mMax, mMin > mMax);
			}

			constexpr iterator end() const noexcept
			{
				return iterator(mMax, mMax, /* end */ true);
			}

			[[nodiscard]] constexpr std::size_t size() const noexcept
			{
				if (mMax < mMin)
					return 0;

				if (mMax == mMin)
					return 1;

				if (std::is_constant_evaluated())
				{
					std::size_t s = 0;
					T frequency = mMin;

					while (frequency <= mMax)
					{
						++s;
						frequency *= cSemitoneRatio;
					}

					return s;
				}
				else
				{
					return static_cast<std::size_t>(std::floor(T(12) * std::log2(mMax / mMin))) + 1;
				}
			}

		public:
			class iterator
			{
			public:
				using iterator_concept = std::forward_iterator_tag;
				using iterator_category = std::forward_iterator_tag;
				using value_type = T;
				using difference_type = std::ptrdiff_t;

			public:
				constexpr iterator() : mCurrent(0), mMaxValue(0), bIsEnd(true) {}
				constexpr iterator(T start, T max, bool end = false)
					: mCurrent(start), mMaxValue(max), bIsEnd(end)
				{
				}

				constexpr T operator*() const noexcept { return mCurrent; }

				constexpr iterator& operator++() noexcept
				{
					if (not bIsEnd)
					{
						mCurrent *= cSemitoneRatio;
						bIsEnd = mCurrent - mMaxValue > T(1e-9);
					}
					return *this;
				}

				constexpr iterator operator++(int) noexcept
				{
					iterator tmp = *this;
					++(*this);
					return tmp;
				}

				constexpr bool operator==(const iterator& other) const noexcept
				{
					if (bIsEnd and other.bIsEnd)
						return true;
					if (bIsEnd or other.bIsEnd)
						return false;
					return JPL::Math::IsNearlyEqual(mCurrent, other.mCurrent, T(1e-9));
				}

			private:
				T mCurrent;
				T mMaxValue;
				bool bIsEnd;
			};

		private:
			T mMin;
			T mMax;
		};

		//======================================================================
	namespace Impl
	{
		template<bool bDecibels, class FilterType>
		void ComputeFilterResponse(const FilterType& filter, float sampleRate, std::vector<float>& outMagnitudes)
		{
				const auto semitoneGrid = SemitoneGridView(sampleRate * 0.5f);

			outMagnitudes.clear();
				outMagnitudes.reserve(semitoneGrid.size());

			const float invSampleRate = 1.0f / sampleRate;

				auto projection = [](auto&& v)
				{
					using Func = decltype(&FilterType::CalculateResponse);
					static constexpr bool bIsComplex =
						std::same_as<std::invoke_result_t<Func, FilterType, float>, std::complex<double>> or
						std::same_as<std::invoke_result_t<Func, FilterType, double>, std::complex<double>>;

					if constexpr (bIsComplex)
			{
						return std::abs(v);
			}
					else
					{
						return v;
					}
				};

				for (float frequency : semitoneGrid)
					outMagnitudes.push_back(projection(filter.CalculateResponse(frequency * invSampleRate)));

			if constexpr (bDecibels)
			{
				for (float& magnitude : outMagnitudes)
					magnitude = -GainTodB(magnitude);
			}
		}
	} // namespace Impl

		//======================================================================
		template<class FilterType>
		void ComputeFilterResponse(const FilterType& filter, float sampleRate, std::vector<float>& outMagnitudes)
		{
			Impl::ComputeFilterResponse<false>(filter, sampleRate, outMagnitudes);
		}

		template<class FilterType>
		void ComputeFilterResponse_dB(const FilterType& filter, float sampleRate, std::vector<float>& outMagnitudes)
		{
			Impl::ComputeFilterResponse<true>(filter, sampleRate, outMagnitudes);
		}
	} // namespace FilterUtils

	//==========================================================================
	inline auto Topology::OnePole::CalculateResponse(float normalziedFrequency) const -> Sample
	{
		const Sample omega = JPL_TWO_PI * normalziedFrequency;
		const Sample cosOmega = ::cosf(omega);
		const Sample numSq = (b0 * b0) + (b1 * b1) + (2.0f * b0 * b1 * cosOmega);
		const Sample denSq = 1.0f + (a1 * a1) + (2.0f * a1 * cosOmega);
		return ::sqrtf(numSq / denSq);
	}

	JPL_INLINE Topology::OnePole Topology::OnePole::MakeLowShelf(float frequency, float sampleRate, float gain)
	{
		return MakeLowShelf(frequency / sampleRate, gain);
	}

	JPL_INLINE Topology::OnePole Topology::OnePole::MakeLowShelf(float normalizedFrequency, float gain)
	{
		const float t = ::tanf(JPL_PI * normalizedFrequency);
		const float sqrtK = ::sqrtf(gain);
		return MakeLowShelf(Cache::Shelf{ .t = t, .sqrtK = sqrtK, .gain = gain });
	}

	inline Topology::OnePole Topology::OnePole::MakeLowShelf(Cache::Shelf cache)
	{
		const Sample raw_b0 = (cache.t * cache.sqrtK) + 1.0f;
		const Sample raw_b1 = (cache.t * cache.sqrtK) - 1.0f;
		const Sample raw_a0 = (cache.t / cache.sqrtK) + 1.0f;
		const Sample raw_a1 = (cache.t / cache.sqrtK) - 1.0f;

		// Pre-normalize coefficients
		const Sample norm = Sample(1.0f) / raw_a0;
		return OnePole{
			.b0 = raw_b0 * norm,
			.b1 = raw_b1 * norm,
			.a1 = raw_a1 * norm
		};
	}


	JPL_INLINE Topology::OnePole Topology::OnePole::MakeHighShelf(float frequency, float sampleRate, float gain)
	{
		return MakeHighShelf(frequency / sampleRate, gain);
	}

	JPL_INLINE Topology::OnePole Topology::OnePole::MakeHighShelf(float normalizedFrequency, float gain)
	{
		const float t = ::tanf(JPL_PI * normalizedFrequency);
		const float sqrtK = ::sqrtf(gain);
		return MakeHighShelf(Cache::Shelf{ .t = t, .sqrtK = sqrtK, .gain = gain });
	}

	inline Topology::OnePole Topology::OnePole::MakeHighShelf(Cache::Shelf cache)
	{
		const Sample raw_b0 = (cache.t * cache.sqrtK) + cache.gain;
		const Sample raw_b1 = (cache.t * cache.sqrtK) - cache.gain;
		const Sample raw_a0 = (cache.t * cache.sqrtK) + 1.0f;
		const Sample raw_a1 = (cache.t * cache.sqrtK) - 1.0f;

		// Pre-normalize coefficients
		const Sample norm = Sample(1.0f) / raw_a0;
		return OnePole{
			.b0 = raw_b0 * norm,
			.b1 = raw_b1 * norm,
			.a1 = raw_a1 * norm
		};
	}

	//==========================================================================
	inline std::complex<double> Topology::Biquad::CalculateResponse(float normalizedFrequency) const
	{
		// DSPFilters by Vinnie Falco (MIT)
		using Complex = std::complex<double>;
		const double omega = 2 * std::numbers::pi_v<double> *normalizedFrequency;
		const Complex czn1 = std::polar(1.0, -omega);
		const Complex czn2 = std::polar(1.0, -2.0 * omega);
		Complex ch(1.0);
		Complex cbot(1.0);

		Complex ct(b0);
		Complex cb(1.0);
		ct = ct + static_cast<double>(b1) * czn1;
		ct = ct + static_cast<double>(b2) * czn2;
		cb = cb + static_cast<double>(a1) * czn1;
		cb = cb + static_cast<double>(a2) * czn2;
		ch *= ct;
		cbot *= cb;

		return ch / cbot;
	}

	inline Topology::Biquad Topology::Biquad::Combine(const OnePole& low, const OnePole& high, float broadbandGain)
	{
		return Biquad{
			.b0 = broadbandGain * (low.b0 * high.b0),
			.b1 = broadbandGain * (low.b0 * high.b1 + low.b1 * high.b0),
			.b2 = broadbandGain * (low.b1 * high.b1),
			.a1 = low.a1 + high.a1,
			.a2 = low.a1 * high.a1
		};
	}

	template<class StateType>
	inline void Topology::Biquad::ProcessInterpolating(std::span<Sample> samples, StateType& state, Topology::Biquad& previous) const
	{
		if (samples.empty())
			return;

		const Sample t = 1.0f / samples.size();
		const Sample db0 = t * (b0 - previous.b0);
		const Sample db1 = t * (b1 - previous.b1);
		const Sample db2 = t * (b2 - previous.b2);
		const Sample da1 = t * (a1 - previous.a1);
		const Sample da2 = t * (a2 - previous.a2);

		for (Sample& sample : samples)
		{
			previous.b0 += db0;
			previous.b1 += db1;
			previous.b2 += db2;
			previous.a1 += da1;
			previous.a2 += da2;
			sample = state.Process(sample, previous);
		}

		previous = *this;
	}

	template<class StateType>
	inline void Topology::Biquad::ProcessInterpolating(std::span<Sample> samples, uint32 stride, StateType& state, Biquad& previous) const
	{
		if (samples.empty())
			return;

		const uint32 numFrames = 1 + (samples.size() - 1) / stride;
		const Sample t = 1.0f / static_cast<Sample>(numFrames);
		const Sample db0 = t * (b0 - previous.b0);
		const Sample db1 = t * (b1 - previous.b1);
		const Sample db2 = t * (b2 - previous.b2);
		const Sample da1 = t * (a1 - previous.a1);
		const Sample da2 = t * (a2 - previous.a2);

		for (uint32 s = 0; s < samples.size(); s += stride)
		{
			previous.b0 += db0;
			previous.b1 += db1;
			previous.b2 += db2;
			previous.a1 += da1;
			previous.a2 += da2;
			samples[s] = state.Process(samples[s], previous);
		}

		previous = *this;
	}

	JPL_INLINE void Topology::Biquad::SetCoefficients(const Topology::Biquad& other)
	{
		*this = other;
	}

} // namespace JPL
