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
#include "JPLSpatial/Auralization/ChannelMixing.h"
#include <JPLSpatial/Auralization/DelayLine.h>
#include <JPLSpatial/Math/Math.h>
#include <JPLSpatial/Math/SIMD.h>

#include <algorithm>
#include <array>
#include <bit>
#include <concepts>
#include <ranges>
#include <span>
#include <random>

namespace JPL
{
	//==========================================================================
	/// Reverb design components & utilities.
	/// Largely inspired by wonderful article by Geraint Luff "Let's Write A Reverb"
	/// 
	/// (example of usage is at the very end of the file)

	//==========================================================================
	namespace Detail
	{
		inline constexpr auto cPrimes =[]() -> std::array<uint32, 2'000>
		{
			constexpr uint32 N = 2'000;
			
			auto isPrime = [](uint32 num)
			{
				if (num <= 1) return false;
				if (num == 2 || num == 3) return true;
				if (num % 2 == 0 || num % 3 == 0) return false;
				for (uint32 i = 5; i * i <= num; i += 6)
					if (num % i == 0 || num % (i + 2) == 0) return false;
				return true;
			};

			std::array<uint32, N> primes{};
			std::size_t count = 0;
			uint32 candidate = 2;

			while (count < N)
			{
				if (isPrime(candidate))
				{
					primes[count] = candidate;
					count++;
				}
				candidate++;
			}
			return primes;
		}();

		[[nodiscard]] JPL_INLINE uint32 CeilToPrime(uint32 number)
		{
			auto found = std::ranges::lower_bound(cPrimes, number);
			JPL_ASSERT(found != std::ranges::end(cPrimes));
			return *found;
		}

		constexpr uint32 cMaxFDNOrder = 64;
		constexpr uint32 cMaxDelayInSamples = cPrimes.back();

		template<uint32 FDNOrder>
		constexpr bool cValidFDNOrder = FDNOrder > 0 and FDNOrder <= cMaxFDNOrder;

		//======================================================================
		template<class Sample>
		[[nodiscard]] constexpr Sample GenRandomSign(uint32 outChannel, uint32 fdnChannel)
		{
			// Deterministic pseudo-random +/- pattern
			uint32 x = fdnChannel + 0x9E3779B9u * (outChannel + 1);
			x ^= x >> 16; x *= 0x7FEB352Du; 
			x ^= x >> 15; x *= 0x846CA68Bu;
			x ^= x >> 16;
			return (x & 1u) ? Sample(1) : Sample(-1);
		}

		template<class Sample, std::size_t NumFDNChannels> requires (Detail::cValidFDNOrder<NumFDNChannels>)
		static constexpr std::array<Sample, NumFDNChannels> MakeSignMatrix()
		{
			std::array<Sample, NumFDNChannels> signs;
			for (uint32 i = 0; i < NumFDNChannels; ++i)
				signs[i] = Detail::GenRandomSign<Sample>(0u, i);
			return signs;
		}

		template<class Sample, std::size_t NumFDNChannels, uint32 MaxOutputChannels>
			requires(Detail::cValidFDNOrder<NumFDNChannels> and MaxOutputChannels > 0)
		static constexpr std::array<std::array<Sample, NumFDNChannels>, MaxOutputChannels> MakeSignMatrix()
		{
			std::array<std::array<Sample, NumFDNChannels>, MaxOutputChannels> signs;
			for (uint32 o = 0; o < MaxOutputChannels; ++o)
			{
				for (uint32 i = 0; i < NumFDNChannels; ++i)
					signs[o][i] = Detail::GenRandomSign<Sample>(o + 1, i);
			}
			return signs;
		}
	}
	
	//==========================================================================
	namespace Matrix
	{
		//======================================================================
		template<class Sample, std::size_t Size>
		JPL_INLINE void Multiply(const std::array<Sample, Size>& dataA, std::array<Sample, Size>& dataBOut)
		{
			if constexpr (std::same_as<Sample, float> and Size >= simd::size())
			{
				static constexpr uint32 simdFloor = FloorToSIMDSize(Size);

				for (uint32 i = 0; i < simdFloor; i += simd::size())
				{
					(simd(&dataBOut[i]) * simd(dataA[i])).store(&dataBOut[i]);
				}

				if constexpr (Size > simdFloor)
				{
					for (uint32 i = simdFloor; i < Size; ++i)
						dataBOut[i] *= dataA[i];
				}
			}
			else
			{
				for (uint32 i = 0; i < Size; ++i)
					dataBOut[i] *= dataA[i];
			}
		}

		template<class Sample>
		JPL_INLINE void Multiply(std::span<Sample> data, Sample value)
		{
			if constexpr (std::same_as<Sample, float>)
			{
				const uint32 simdFloor = FloorToSIMDSize(data.size());

				uint32 i = 0;
				const simd v(value);
				for (; i < simdFloor; i += simd::size())
				{
					(simd(&data[i]) * v).store(&data[i]);
				}

				for (; i < data.size(); ++i)
					data[i] *= value;
			}
			else
			{
				for (uint32 i = 0; i < data.size(); ++i)
					data[i] *= value;
			}
		}

		template<class Sample, std::size_t Size>
		JPL_INLINE void Multiply(std::array<Sample, Size>& data, Sample value)
		{
			if constexpr (std::same_as<Sample, float> and Size >= simd::size())
			{
				static constexpr uint32 simdFloor = FloorToSIMDSize(Size);
				
				const simd v(value);
				
				for (uint32 i = 0; i < simdFloor; i += simd::size())
				{
					(simd(&data[i]) * v).store(&data[i]);
				}

				if constexpr (Size > simdFloor)
				{
					for (uint32 i = simdFloor; i < Size; ++i)
						data[i] *= value;
				}
			}
			else
			{
				for (uint32 i = 0; i < Size; ++i)
					data[i] *= value;
			}
		}

		template<class Sample, std::size_t Size>
		JPL_INLINE void Add(std::array<Sample, Size>& data, Sample value)
		{
			if constexpr (std::same_as<Sample, float> and Size >= simd::size())
			{
				static constexpr uint32 simdFloor = FloorToSIMDSize(Size);

				const simd v(value);

				for (uint32 i = 0; i < simdFloor; i += simd::size())
				{
					(simd(&data[i]) + v).store(&data[i]);
				}

				if constexpr (Size > simdFloor)
				{
					for (uint32 i = simdFloor; i < Size; ++i)
						data[i] += value;
				}
			}
			else
			{
				for (uint32 i = 0; i < Size; ++i)
					data[i] += value;
			}
		}

		template<class Sample, std::size_t Size>
		JPL_INLINE void Add(const std::array<Sample, Size>& dataA, std::array<Sample, Size>& dataBOut)
		{
			if constexpr (std::same_as<Sample, float> and Size >= simd::size())
			{
				static constexpr uint32 simdFloor = FloorToSIMDSize(Size);

				for (uint32 i = 0; i < simdFloor; i += simd::size())
				{
					(simd(&dataBOut[i]) + simd(dataA[i])).store(&dataBOut[i]);
				}

				if constexpr (Size > simdFloor)
				{
					for (uint32 i = simdFloor; i < Size; ++i)
						dataBOut[i] += dataA[i];
				}
			}
			else
			{
				for (uint32 i = 0; i < Size; ++i)
					dataBOut[i] += dataA[i];
			}
		}

		template<class Sample, std::size_t Size>
		[[nodiscard]] JPL_INLINE Sample ReduceSum(const std::array<Sample, Size>& data)
		{
			Sample sum(0);
			if constexpr (std::same_as<Sample, float> and Size >= simd::size())
			{
				static constexpr uint32 simdFloor = FloorToSIMDSize(Size);

				for (uint32 i = 0; i < simdFloor; i += simd::size())
				{
					sum += simd(&data[i]).reduce();
				}

				if constexpr (Size > simdFloor)
				{
					for (uint32 i = simdFloor; i < Size; ++i)
						sum += data[i];
				}
			}
			else
			{
				for (Sample s : data)
					sum += s;
			}
			return sum;
		}

		template<class Sample, std::size_t Size>
		JPL_INLINE void AssignRandomSign(std::array<Sample, Size>& data)
		{
			static constexpr auto signs = Detail::MakeSignMatrix<Sample, Size>();
			Multiply(signs, data);
		}

		template<std::size_t MaxOutSize, class Sample, std::size_t Size>
		JPL_INLINE void AssignRandomSign(std::array<Sample, Size>& data, uint32 outIndex)
		{
			static constexpr auto signs = Detail::MakeSignMatrix<Sample, Size, MaxOutSize>();
			Multiply(signs[outIndex], data);
		}

		//======================================================================
		class Householder
		{
		public:
			template<class Sample, uint32 Size> requires(Detail::cValidFDNOrder<Size>)
			static void MixInPlace(std::array<Sample, Size>& arr)
			{
				static constexpr Sample cMultiplier{ -2.0 / Size };
				const Sample sum = ReduceSum(arr) * cMultiplier;
				Add(arr, sum);
			}

			// Special case for 16th order
			template<class Sample>
			static void MixInPlace(std::array<Sample, 16>& arr)
			{
				if constexpr (std::same_as<Sample, float>)
				{
					auto mix4 = [](float* v)
					{
						simd vec(v);
						const float sum = vec.reduce();

						// Matrix with +1 on diagonal, -1 elsewhere:
						// y_i = 2 * x_i - sum
						vec = Math::FMA(simd(2.0f), vec, simd(-sum));
						vec.store(v);
					};

					// First mix within each group of 4.
					for (uint32 g = 0; g < 16; g += 4)
						mix4(&arr[g]);

					// Then mix across the 4 groups for each lane.
					for (uint32 lane = 0; lane < 4; ++lane)
					{
						float v[4] = {
							arr[lane + 0 * 4],
							arr[lane + 1 * 4],
							arr[lane + 2 * 4],
							arr[lane + 3 * 4],
						};

						mix4(v);
						(simd(v) * simd(0.25f)).store(v);

						arr[lane + 0 * 4] = v[0];
						arr[lane + 1 * 4] = v[1];
						arr[lane + 2 * 4] = v[2];
						arr[lane + 3 * 4] = v[3];
					}
				}
				else
				{
					auto mix4 = [](Sample* v)
					{
						const Sample sum = v[0] + v[1] + v[2] + v[3];

						// Matrix with +1 on diagonal, -1 elsewhere:
						// y_i = 2 * x_i - sum
						for (uint32 i = 0; i < 4; ++i)
							v[i] = Math::FMA(Sample(2), v[i], -sum);
					};

					// First mix within each group of 4.
					for (uint32 g = 0; g < 16; g += 4)
						mix4(arr + g);

					// Then mix across the 4 groups for each lane.
					for (uint32 lane = 0; lane < 4; ++lane)
					{
						Sample v[4] = {
							arr[lane + 0 * 4],
							arr[lane + 1 * 4],
							arr[lane + 2 * 4],
							arr[lane + 3 * 4],
						};

						mix4(v);

						arr[lane + 0 * 4] = v[0] * Sample(0.25);
						arr[lane + 1 * 4] = v[1] * Sample(0.25);
						arr[lane + 2 * 4] = v[2] * Sample(0.25);
						arr[lane + 3 * 4] = v[3] * Sample(0.25);
					}
				}
			}
		};

		//======================================================================
		class Hadamard
		{
		private:
			template<class Sample, uint32 Size>
			static void RecursiveUnscaled(std::span<Sample, Size> data)
			{
				if (Size <= 1)
					return;
				
				static constexpr uint32 hSize = Size / 2;

				// Two (unscaled) Hadamards of half the size
				Hadamard::RecursiveUnscaled(data.template subspan<0, hSize>());
				Hadamard::RecursiveUnscaled(data.template subspan<hSize, hSize>());

				if constexpr (std::same_as<Sample, float> and hSize >= simd::size())
				{
					static constexpr uint32 simdFloor = FloorToSIMDSize(hSize);
					for (uint32 i = 0; i < simdFloor; i += simd::size())
					{
						const simd a(&data[i]);
						const simd b(&data[i + hSize]);
						(a + b).store(&data[i]);
						(a - b).store(&data[i + hSize]);
					}

					// Process potential tail
					if constexpr (hSize > simdFloor)
					{
						for (uint32 i = simdFloor; i < hSize; ++i)
						{
							const Sample a = data[i];
							const Sample b = data[i + hSize];
							data[i] = (a + b);
							data[i + hSize] = (a - b);
						}
					}

				}
				else
				{
					// Combine the two halves using sum/difference
					for (uint32 i = 0; i < hSize; ++i)
					{
						const Sample a = data[i];
						const Sample b = data[i + hSize];
						data[i] = (a + b);
						data[i + hSize] = (a - b);
					}
				}
			}

		public:
			template<class Sample, std::size_t Size> requires(Detail::cValidFDNOrder<Size> and std::has_single_bit(Size))
			static JPL_INLINE void MixInPlace(std::array<Sample, Size>& data)
			{
				RecursiveUnscaled(std::span<Sample, Size>(data));
				static constexpr Sample scalingFactor = Math::Sqrt(1.0 / Size);
				Multiply(data, scalingFactor);
			}
		};

		//======================================================================
		struct FDNInput
		{
		public:
			template<class Sample, std::size_t NumFDNChannels> requires(Detail::cValidFDNOrder<NumFDNChannels>)
			static JPL_INLINE void InjectNormalized(Sample monoInput, std::array<Sample, NumFDNChannels>& outFDNInput)
			{
				static constexpr Sample norm = Math::Sqrt(Sample(1) / Sample(NumFDNChannels));
				outFDNInput.fill(monoInput * norm);
				AssignRandomSign(outFDNInput);
			}
		};

		//======================================================================
		// Hadamard mixer
		// If num FDN channels guaranteed to be power of 2,
		// this is a more stable choice than FDNOutputProjection
		// 
		// Hadamard output mixer needs power-of-two FDN size.
		struct FDNOutputMixer
		{
			template<class Sample, std::size_t NumFDNChannels>
				requires(Detail::cValidFDNOrder<NumFDNChannels> and std::has_single_bit(NumFDNChannels))
			static void Mix(std::array<Sample, NumFDNChannels> fdn, std::span<Sample> out)
			{
				// Decorrelate / spread the tank states first.
				Matrix::Hadamard::MixInPlace(fdn);

				const uint32 numOutputChannels = out.size();

				for (uint32 i = 0; i < NumFDNChannels; ++i)
				{
					const uint32 outChannel = i % numOutputChannels;
					out[outChannel] += fdn[i];
				}

				static constexpr Sample invFDNChannels = Sample(1.0) / NumFDNChannels;

				// Hadamard is already normalized by 1 / sqrt(NumFDNChannels).
				// This compensates for folding multiple FDN channels into each output.
				const Sample foldNorm =
					Math::Sqrt(static_cast<Sample>(numOutputChannels) * invFDNChannels);

				Multiply(out, foldNorm);
			}
		};

		//======================================================================
		template<uint32 MaxOutputChannels = 16u> requires(MaxOutputChannels > 0)
		struct FDNOutputProjection
		{
			template<class Sample, std::size_t NumFDNChannels> requires(Detail::cValidFDNOrder<NumFDNChannels>)
			static void Mix(const std::array<Sample, NumFDNChannels>& fdn, std::span<Sample> out)
			{
				JPL_ASSERT(out.size() <= MaxOutputChannels);


				for (uint32 o = 0; o < out.size(); ++o)
				{
					std::array<Sample, NumFDNChannels> fdnWithRandSign = fdn;
					AssignRandomSign<MaxOutputChannels>(fdnWithRandSign, 0);

					out[o] = ReduceSum(fdnWithRandSign);
				}

				static constexpr Sample norm = Math::Sqrt(Sample(1) / Sample(NumFDNChannels));

				Multiply(out, norm);
			}
		};

	} // namespace Matrix
	
	//==========================================================================
	struct Feedback
	{
		float DecayGain = 0.85f;
		float DelayMs = 80.0f;
		uint32 DelaySamples = 0;
		DelayLine<1> Delay;

		JPL_INLINE void Prepare(float sampleRate)
		{
			DelaySamples = DelayMs * 0.001f * sampleRate;
			Delay.Resize(DelaySamples + 1);
			Delay.Clear();
		}

		[[nodiscard]] JPL_INLINE float Process(float sample)
		{
			const float delayed = Delay.GetReadWindow<1>(DelaySamples);
			const float sum = Math::FMA(delayed, DecayGain, sample);
			Delay.Push(sum);
			return delayed;
		}
	};

	//==================================================================================
	// Using prime distribution within Min/Max range
	template<uint32 NumChannels, class AttenuationFilter> requires(Detail::cValidFDNOrder<NumChannels>)
	struct MultiChannelFeedbackBase
	{
	private:
		// A nice sounding set in range [1500, 4500]
		static constexpr uint32 cPrimeDelayLengths[] ={
			2927, 2593, 2273, 3697, 1877, 3877, 2477, 3461,
			1609, 3779, 3541, 4259, 1669, 3539, 3637, 4013,
			3121, 4003, 1627, 3733, 3511, 4093, 2411, 3011,
			2801, 2017, 4013, 1621, 3023, 2161, 4201, 3079,
			1783, 4027, 1613, 1907, 2129, 3797, 2543, 1579,
			3967, 2551, 3833, 4111, 3059, 4159, 1753, 1601,
			2423, 1993, 4493, 2707, 3719, 3547, 3769, 4219,
			2389, 2087, 1709, 3229, 3083, 3391, 3259, 2791
		};
	public:
		using Channels = std::array<float, NumChannels>;

#if 0
		uint32 DelaySamplesMin = 1500;
		uint32 DelaySamplesMax = 4500;
#endif

		template<class ...FilterArgs>
		void Prepare(float sampleRate, FilterArgs&&...filterPrepareArgs)
		{
			mInvSampleRate = 1.0f / sampleRate;
#if 1
			std::ranges::copy(cPrimeDelayLengths | std::views::take(NumChannels), mDelaySamples.begin());
#else

			auto first = std::ranges::lower_bound(cPrime, DelaySamplesMin);
			auto last = std::ranges::lower_bound(cPrime, DelaySamplesMax);
			JPL_ASSERT(std::distance(first, last) >= NumChannels);

			std::mt19937 rng{ std::random_device{}() };
			std::ranges::sample(std::ranges::subrange(first, last), mDelaySamples.begin(), NumChannels, rng);
			std::ranges::shuffle(mDelaySamples, rng);
#endif

			for (uint32 c = 0; c < NumChannels; ++c)
			{
				mDelays[c].Resize(mDelaySamples[c] + 1);
				mDelays[c].Clear();

				mAttenuationFilters[c].Prepare(sampleRate, std::forward<FilterArgs>(filterPrepareArgs)...);
			}
		}

		template<class ...FilterArgs>
		void UpdateFilterParameters(FilterArgs&&...filterUpdateArgs)
		{
			for (uint32 c = 0; c < NumChannels; ++c)
			{
				// Recalculate exact gain for this specific loop length
				const float delaySeconds = mDelaySamples[c] * mInvSampleRate;
				mAttenuationFilters[c].UpdateParameters(delaySeconds, std::forward<FilterArgs>(filterUpdateArgs)...);
			}
		}

	protected:
		[[nodiscard]] JPL_INLINE float ApplyAttenuationFilter(float sample, uint32 fdnIndex)
		{
			return mAttenuationFilters[fdnIndex].Process(sample);
		}

	protected:
		std::array<uint32, NumChannels> mDelaySamples;
		std::array<DelayLine<1>, NumChannels> mDelays;
		std::array<AttenuationFilter, NumChannels> mAttenuationFilters;
		float mInvSampleRate = 1.0f / 48'000.0f;
	};

	//==========================================================================
	template<uint32 NumChannels, class AttenuationFilter> requires (Detail::cValidFDNOrder<NumChannels>)
	class MultiChannelFeedback : public MultiChannelFeedbackBase<NumChannels, AttenuationFilter>
	{
	public:
		using Base = MultiChannelFeedbackBase<NumChannels, AttenuationFilter>;
		using Channels = typename Base::Channels;

		using Base::Base;

		Channels Process(Channels input)
		{
			Channels delayed;
			for (uint32 c = 0; c < NumChannels; ++c)
			{
				delayed[c] = Base::mDelays[c].template GetReadWindow<1>(Base::mDelaySamples[c]);
			}

			for (uint32 c = 0; c < NumChannels; ++c)
			{
				const float sum = Base::ApplyAttenuationFilter(delayed[c] + input[c], c);
				Base::mDelays[c].Push(sum);
			}

			return delayed;
		}
	};
	
	//==========================================================================
	template<uint32 NumChannels, class AttenuationFilter> requires (Detail::cValidFDNOrder<NumChannels>)
	class MultiChannelMixedFeedback : public MultiChannelFeedbackBase<NumChannels, AttenuationFilter>
	{
	public:
		using Base = MultiChannelFeedbackBase<NumChannels, AttenuationFilter>;
		using Channels = typename Base::Channels;

		using Base::Base;

		Channels Process(const Channels& input)
		{
			Channels delayed;
			for (uint32 c = 0; c < NumChannels; ++c)
			{
				delayed[c] = Base::mDelays[c].template GetReadWindow<1>(Base::mDelaySamples[c]);
			}

			// Mix using a Householder matrix
			Channels mixed = delayed;
			Matrix::Householder::MixInPlace(mixed);

			for (uint32 c = 0; c < NumChannels; ++c)
			{
				const float sum = Base::ApplyAttenuationFilter(mixed[c] + input[c], c);
				Base::mDelays[c].Push(sum);
			}

			return delayed;
		}
	};

	//==========================================================================
	template<uint32 NumChannels = 8> requires(Detail::cValidFDNOrder<NumChannels>)
	struct DiffusionStep
	{
		using Channels = std::array<float, NumChannels>;

		float DelayMsRange = 50.0f;

		void Prepare(float sampleRate)
		{
			static constexpr float invNumChannels = 1.0f / NumChannels;
			const float delaySamplesRange = DelayMsRange * 0.001f * sampleRate;

			auto randomInRange = [](float min, float max)
			{
				static std::mt19937 mt(std::random_device{}());
				return std::uniform_real_distribution<float>{ min, max }(mt);
			};

			for (uint32 c = 0; c < NumChannels; ++c)
			{
				const float rangeLow = delaySamplesRange * c * invNumChannels;
				const float rangeHigh = delaySamplesRange * (c + 1) * invNumChannels;
				mDelaySamples[c] = Detail::CeilToPrime(static_cast<uint32>(randomInRange(rangeLow, rangeHigh)));
				mDelays[c].Resize(mDelaySamples[c] + 1);
				mDelays[c].Clear();
			}
		}

		Channels Process(Channels input)
		{
			// Delay
			Channels delayed;
			for (uint32 c = 0; c < NumChannels; ++c)
			{
				mDelays[c].Push(input[c]);
				delayed[c] = mDelays[c].template GetReadWindow<1>(mDelaySamples[c]);
			}

			// Mix with a Hadamard matrix
			Channels mixed = delayed;
			Matrix::Hadamard::MixInPlace(mixed);

			// Flip some polarities
			Matrix::AssignRandomSign(mixed);

			return mixed;
		}

	private:
		std::array<uint32, NumChannels> mDelaySamples;
		std::array<DelayLine<1>, NumChannels> mDelays;
	};

	//==========================================================================
	template<uint32 NumChannels = 8, uint32 StepCount = 4> requires(Detail::cValidFDNOrder<NumChannels> and StepCount > 0)
	struct DiffuserBase
	{
	protected:
		DiffuserBase() = default;

	public:
		using Channels = std::array<float, NumChannels>;
		using Step = DiffusionStep<NumChannels>;

		std::array<Step, StepCount> Steps;

		void Prepare(float sampleRate)
		{
			for (Step& step : Steps)
				step.Prepare(sampleRate);
		}

		Channels Process(Channels samples)
		{
			for (Step& step : Steps)
				samples = step.Process(samples);
			return samples;
		}
	};

	//==========================================================================
	template<uint32 NumChannels = 8, uint32 StepCount = 4> requires(Detail::cValidFDNOrder<NumChannels> and StepCount > 0)
	struct DiffuserEqualLengths : DiffuserBase<NumChannels, StepCount>
	{
		using Step = DiffusionStep<NumChannels>;

		std::array<Step, StepCount> Steps;

		DiffuserEqualLengths(float totalDiffusionMs)
		{
			const float delayMsRange = totalDiffusionMs / StepCount;
			for (Step& step : Steps)
				step.DelayMsRange = delayMsRange;
		}
	};

	//==========================================================================
	template<uint32 NumChannels = 8, uint32 StepCount = 4> requires (Detail::cValidFDNOrder<NumChannels> and StepCount > 0)
	struct DiffuserHalfLengths : DiffuserBase<NumChannels, StepCount>
	{
		using Step = DiffusionStep<NumChannels>;

		std::array<Step, StepCount> Steps;

		DiffuserHalfLengths(float diffusionMs)
		{
			for (Step& step : Steps)
			{
				diffusionMs *= 0.5;
				step.DelayMsRange = diffusionMs;
			}
		}
	};

	//==================================================================================
	/// Example
#if 0
	template<uint32 NumChannels = 8, uint32 NumDiffusionSteps = 4> requires(Detail::cValidFDNOrder<NumChannels>)
	class FDNReverb
	{
	public:
		using Channels = std::array<float, NumChannels>;

		FDNReverb(float roomSizeMs, const simd& rt60, float dry = 0.0f, float wet = 1.0f)
			: mDiffuser(roomSizeMs)
			, mDryAmount(dry)
			, mWetAmount(wet)
			, mDiffuserAmount(wet * 0.5f)
			, mUseDiffuser(true)
		{
			mFeedback.RT60 = rt60;
		}

		inline void Prepare(float sampleRate)
		{
			mSampleRate = sampleRate;
			mFeedback.Prepare(sampleRate);
			mDiffuser.Prepare(sampleRate);
		}

		[[nodiscard]] inline Channels Process(Channels input)
		{
			Channels output;
			if (mUseDiffuser)
			{
				Channels diffuse = mDiffuser.Process(input);
				Channels longLasting = mFeedback.Process(diffuse);

				//! this seems to help a bit to reduce the "doubling" effect of tapping diffuser to the output
				//Matrix::Hadamard<float, NumChannels>::MixInPlace(diffuse.data());
				Matrix::Householder::MixInPlace(diffuse);

				Matrix::Multiply(input, mDryAmount);
				Matrix::Multiply(diffuse, mDiffuserAmount);
				Matrix::Multiply(longLasting, mWetAmount);
				Matrix::Add(input, diffuse);
				Matrix::Add(diffuse, longLasting);
				output = longLasting;
			}
			else
			{
				Channels longLasting = mFeedback.Process(input);

				Matrix::Multiply(input, mDryAmount);
				Matrix::Multiply(longLasting, mWetAmount);
				Matrix::Add(input, longLasting);
				output = longLasting;
			}
			return output;
		}

		JPL_INLINE void SetDryLevel(float newDryLevel) { mDryAmount = newDryLevel; }
		JPL_INLINE void SetWetLevel(float newWetLevel) { mWetAmount = newWetLevel; }
		[[nodiscard]] JPL_INLINE float GetCurrentRT60() const { return mFeedback.RT60; }
		JPL_INLINE void SetDiffuserTapLevel(float newLevel) { mDiffuserAmount = newLevel; }
		JPL_INLINE void SetUseDiffuser(bool shouldUse) { mUseDiffuser = shouldUse; }
		JPL_INLINE void SetRT60(const simd& newRT60) { mFeedback.UpdateDecayGains(clamp(newRT60, 0.03f, 10.0f), mSampleRate); }

	private:
		MultiChannelMixedFeedback<NumChannels> mFeedback;
		DiffuserEqualLengths<NumChannels, NumDiffusionSteps> mDiffuser;
		float mDryAmount, mWetAmount, mDiffuserAmount;
		bool mUseDiffuser;
		float mSampleRate = 48'000.0f;
	};
#endif
} // namespace JPL
