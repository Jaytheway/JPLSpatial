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

#include <JPLSpatial/Auralization/DelayLine.h>
#include <JPLSpatial/Auralization/CrossoverFilter.h>
#include <JPLSpatial/Math/SIMD.h>
#include <JPLSpatial/Memory/Memory.h>
#include <JPLSpatial/Utilities/RealtimeObject.h>
#include <JPLSpatial/Utilities/SmoothedValue.h>

#include <atomic>
#include <cstdint>
#include <ranges>
#include <vector>
#include <span>
#include <memory_resource>
#include <memory>

namespace JPL
{
	//==================================================================================
	/// Early Reflection Bus transfering ER data to audio rendering thread.
	/// It uses its own delay line with at least 2 seconds buffer space and performs
	/// rendering of early reflection using interpolating delay line taps, simulating
	/// propagation delay, applying pan and propagation filtering.
	/// The memory is managed by internal memory pool and default global JPL PMR allocator.
	class ERBus
	{
		static constexpr uint32_t cScratchSize = 480;
	public:
		//==============================================================================
		// Parameters to create/update early reflection taps
		struct ERUpdateData
		{
			JPL::simd FilterGains;
			std::vector<float> Gains;
			float Delay;
			uint32_t Id;
		};

	private:
		//==============================================================================
		using TapType = JPL::DelayTap<JPL::Thiran1stInterpolator>;

		enum ERState : int
		{
			Stopped = 0,
			Rendering,
			FadingOut
		};

		// Data modified by non-realtime thread
		struct TargetERData
		{
			JPL::simd FilterGains;
			float* ChannelGains;
			float DelayTime; // in samples
			mutable ERState State;
		};

		// Data modified by realtime thread
		class RealtimeData final
		{
		public:
			RealtimeData() = default;
			~RealtimeData() = default;

			// Non-copyable
			RealtimeData(const RealtimeData&) = delete;
			RealtimeData& operator=(const RealtimeData&) = delete;

		public:
			// Propagation delay
			TapType Tap;
			JPL::SmoothedValue<float> DelayTime;

			// Propagation filtering
			JPL::simd FilterGains;
			JPL::FourBandCrossover Filter;

			// Panning
			float* ChannelGains = nullptr;

			// Render state
			std::atomic<ERState> State;
		};

		struct ER
		{
			TargetERData TargetData;		// Non-realtime mutable
			RealtimeData* RealtimeState;	// Realtime mutable
			uint32_t Id;					// Immutable

			inline bool operator==(const typename ERBus::ER& other)
			{
				return other.Id == Id;
			}

			inline bool operator==(const typename ERBus::ERUpdateData& other)
			{
				return other.Id == Id;
			}
		};

		//==============================================================================
		// Wrapper to control the copy/move/destroy behavior within RealtimeObject
		// This is effectively a special allocator that holds the ERs.
		// 
		// RealtimeObject copies the internal object,
		// and we need a special way to copy data from our pointers,
		// sinse we're not using vectors for channel gains
		class ERStorage
		{
		public:
			// Default constructor is deleted
			// because we always need a memory resource
			ERStorage() = delete;

			explicit ERStorage(std::pmr::memory_resource& resource);
			ERStorage(std::pmr::memory_resource& resource, uint32_t numChannels);

			ERStorage(const ERStorage& other);
			ERStorage& operator=(const ERStorage& other);

			ERStorage(ERStorage&& other) noexcept;
			ERStorage& operator=(ERStorage&& other) noexcept;

			~ERStorage();

			using AllocatorType = std::pmr::polymorphic_allocator<>;
			AllocatorType GetAllocator() const { return mAllocator; }

			void UpdateChannelGains(float*& destination, const float* source);
			float* AllocateCopyChannelGains(const float* source);
			void DeallocateChannelGains(float* gains);

			const std::pmr::vector<ER>& GetERs() const { return ERs; }
			std::pmr::vector<ER>& GetERs() { return ERs; }

			ER* FindERByID(uint32_t id);

			template<std::ranges::range UpdateData>
			void PartitionERs(const UpdateData& updateERs, std::span<ER>& outFoundERs, std::span<ER>& outNotFoundERs);

			TargetERData MakeTargetERData(const JPL::simd& filterGains,
										  const float* channelGains,
										  float delayInSamples,
										  ERState state);

			void UpdateTargetERData(TargetERData& er,
									const JPL::simd& filterGains,
									const float* channelGains,
									float delayInSamples,
									ERState state);

			// Returns number of elements removed
			template <class Predicate>
			std::size_t EraseIf(Predicate predicate);

			// Returns number of elements removed
			template <class Predicate>
			std::size_t EraseIf(std::size_t begin, std::size_t count, Predicate predicate);

			std::size_t Erase(std::size_t begin, std::size_t count);

			// Returns new ER count
			std::size_t InsertNewERs(std::pmr::vector<ER>&& newERs);

		private:
			float* AllocateChannelGains();
			void AllocateCopyChannelGains(const std::pmr::vector<ER>& source);

		private:
			std::pmr::polymorphic_allocator<float> mAllocator;
			std::pmr::vector<ER> ERs;
			uint32_t NumChannels = 0;
		};

		using SafeERs = JPL::RealtimeObject<ERStorage>;
		using SafeERsWrite = typename SafeERs::ScopedAccess<JPL::ThreadType::nonRealtime>;
		using SafeERsRead = typename SafeERs::ScopedAccess<JPL::ThreadType::realtime>;

	public:
		ERBus();
		ERBus(float sampleRate, uint32_t numChannels);
		~ERBus();

		/// Must be called from single non-audio thread
		void Prepare(float sampleRate, uint32_t numChannels);

		/// Must be called from audio thread
		/// input and output must have the same nubmer of channels and samples
		void ProcessInterleaved(std::span<const float> input, std::span<float> output, uint32_t numChannels, uint32_t numSamples);

		uint32_t GetNumChannels() const { return mNumChannels; }

		// Add/remove/update early reflection taps
		/// Must be called from the same non-audio thread
		void SetTaps(std::span<const ERUpdateData> newERs);

		//std::atomic<float> RMS{ 0.0f };

		inline uint32_t GetNumTaps() const { return mNumTaps; }

	private:
		RealtimeData* AllocateRTData();
		RealtimeData* AllocateRTData(const JPL::simd& filterGains, float delayInSamples, ERState state);
		void DeallocateRTData(RealtimeData* data);

	private:
		// Memory pool to reduce number of allocations
		std::pmr::unsynchronized_pool_resource mPool;

		using DelayLineType = JPL::DelayLine<cScratchSize + TapType::InterpolatorType::InputLength>;
		pmr_unique_ptr<DelayLineType> mDelayLine;

		SafeERs mERs;
		uint32_t mNumTaps = 0;

		uint32_t mNumChannels = 0;
		float mSampleRate = 48'000.0f;
	};
} // namespace JPL
