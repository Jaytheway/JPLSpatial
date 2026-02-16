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

#include "JPLSpatial/Auralization/EarlyReflectionsBus.h"

#include "JPLSpatial/Auralization/ChannelMixing.h"

#include <algorithm>
#include <cstring>
#include <ranges>
#include <iterator>
#include <utility>

// Disable/enable features for testing/debugging
#define JPL_ER_USE_DELAY 1
#define JPL_ER_USE_FILTER 1
#define JPL_ER_USE_PAN 1

namespace JPL
{
	namespace stdr = std::ranges;
	namespace stdv = std::views;

	ERBus::ERBus()
		: mPool(
			std::pmr::pool_options{
				.max_blocks_per_chunk = 256, //! experimenting with the this
				.largest_required_pool_block = sizeof(RealtimeData)
			})
		, mERs(ERStorage(*std::pmr::get_default_resource(), 2), &mPool)
	{
	}

	ERBus::ERBus(float sampleRate, uint32_t numChannels)
		: mPool(
			std::pmr::pool_options{
				.max_blocks_per_chunk = 32,
				.largest_required_pool_block = sizeof(RealtimeData)
			})
		, mERs(ERStorage(*std::pmr::get_default_resource(), numChannels), &mPool)
	{
		Prepare(sampleRate, numChannels);
	}

	ERBus::~ERBus()
	{
		//! No need to explicitely deallocate what we have allocated,
		//! std::pmr::unsynchronized_pool_resource will
		//! deallocate all the allcoated memory on its own
	}

	void ERBus::Prepare(float sampleRate, uint32_t numChannels)
	{
		mNumChannels = numChannels;

		// for now initialize to 2 secodns (next pow2 is 128k samples per channel)
		mDelayLine = make_pmr_unique<DelayLineType>(static_cast<uint32_t>(sampleRate * 2.0f)); //! ERs are downmixed to mono

		// We need to set the number of channels for the taps
		//! if number of channels changes, we need to make sure
		//! to clear the taps before setting new value, since
		//! allocations depend on the number of channels
		mERs.nonRealtimeReplace(mPool, numChannels);
	}

	void ERBus::ProcessInterleaved(std::span<const float> input,
								   std::span<float> output,
								   uint32_t numChannels,
								   uint32_t numSamples)
	{
		// Downmix normalization factor
		const float channelNorm = 1.0f / numChannels;

		// Process samples in chunks of cScratchSize

		uint32_t samplesProcessed = 0;
		while (numSamples > 0)
		{
			const uint32_t samplesToProcess = std::min(numSamples, cScratchSize);
			const float invSamplesToProcess = 1.0f / static_cast<float>(samplesToProcess);

			std::span<const float> inputChunk = input.subspan(samplesProcessed * numChannels);
			std::span<float> outputChunk = output.subspan(samplesProcessed * numChannels);

			samplesProcessed += samplesToProcess;

			for (uint32_t s = 0; s < samplesToProcess; ++s)
			{
				// Downmix to mono
				float sample = 0.0f;
				for (uint32_t ch = 0; ch < numChannels; ++ch)
					sample += inputChunk[s * numChannels + ch];
				sample *= channelNorm;

				// Push new sample to delay line
				mDelayLine->Push(sample);
			}

			// ..after the above, we have pushed 'scratchSize' sample into the delay line
			// so we need to take this offset into account when reading taps.

			float scratchBuffer[cScratchSize]{};
			std::span<float> buffer(scratchBuffer, samplesToProcess);

#if not JPL_ER_USE_DELAY
			//? Temp testing just panning
			for (uint32_t d = 0; d < samplesToProcess; ++d)
			{
				buffer[d] = mDelayLine->GetReadWindow<1>(samplesToProcess - d);
			}
#endif
			// Grab the updated data
			SafeERsRead erRead(mERs);
			const auto& ers = erRead->GetERs();
			// TODO: (if non-realtime thread ever waits for too long for realtime to release data,
			//			we can preallocate some space and copy the TargetData into it here to release sooner;
			//			BUT: make sure non-realtime doesn't delete the RealtimeState while it's processed here)

			// Read and process ER taps
			for (const ER& er : ers)
			{
				const auto& targetData = er.TargetData;

				// We must not touch realtime state if we're stopped,
				// it may be deleted already
				// (this tells that non-realtime thread recieved this update)
				if (targetData.State == ERState::Stopped)
					continue;

				// Guaranteed to be not nullptr if the state is not Stopped
				auto& realtimeData = *er.RealtimeState;

				// After we stopp, non-realtime thread may take time
				// to remove the stale, faded-out tap
				// Note: this will let through target state Rendering,
				// which means that this tap was "restarted"
				const ERState currentRTState = realtimeData.State.load(std::memory_order_relaxed);
				if (currentRTState == ERState::Stopped && targetData.State == ERState::FadingOut)
					continue;

				// Update the current rendering state
				realtimeData.State.store(targetData.State, std::memory_order_release);

#if JPL_ER_USE_DELAY
				// Process delay
				{
					// Add scratch size that we are "behind"
					int delayOffset = static_cast<int>(samplesToProcess);

					realtimeData.DelayTime.Target = targetData.DelayTime;

					for (uint32_t s = 0; s < samplesToProcess; ++s) // TODO: can/should we do this in batch?
					{
						realtimeData.Tap.SetDelay(static_cast<float>(delayOffset) + static_cast<float>(realtimeData.DelayTime.GetNext()));

						// Wite delayed sample into the scratch buffer
						buffer[s] = realtimeData.Tap.Process(*mDelayLine);

						// Advance the read offset
						delayOffset -= 1;
					}
				}
#endif

#if JPL_ER_USE_FILTER
				// Process this tap with filters
				realtimeData.Filter.ProcessBlock(buffer, realtimeData.FilterGains, targetData.FilterGains, buffer);
				realtimeData.FilterGains = targetData.FilterGains;
				//! (this filter is most expansive part of the process block - literaly 80% of the processing)
				//! HOWEVER: with AVX2, which actually uses FMAs, we get more than 2x performance! 20-30 taps ~0.5ms
#endif

				// Mix/add to the ouptut
				for (uint32_t outChannel = 0; outChannel < numChannels; ++outChannel)
				{
					// TODO: we could improve this if we apply gain ramp first to contiguous 'buffer',
					//		then add it to the interleaved output. Or setup deinterleaved callback.
#if JPL_ER_USE_PAN

					AddAndApplyGainRamp(outputChunk.data(),
										buffer.data(),
										outChannel, 0,
										numChannels, 1,
										samplesToProcess,
										realtimeData.ChannelGains[outChannel],
										targetData.ChannelGains[outChannel]);
#else
					Add(outputChunk.data(),
						buffer.data(),
						outChannel, 0,
						numChannels, 1,
						samplesToProcess);
#endif

					realtimeData.ChannelGains[outChannel] = targetData.ChannelGains[outChannel];
				}

				// If we were fading out, flag that we're done by this point
				ERState targetState = ERState::FadingOut;
				realtimeData.State.compare_exchange_strong(targetState, ERState::Stopped, std::memory_order_release);
			}

			numSamples -= samplesToProcess;
		} // while (numSamples > 0)
	}

	// Simple utility for convenience
	template<stdr::range Container>
	static auto FindERByID(Container& container, uint32_t id)
	{
		// This preserves the constness
		using ReturnValue = std::remove_reference_t<decltype(*container.begin())>;
		auto found = stdr::find_if(container, [id](const auto& er) { return er.Id == id; });
		return found == stdr::end(container)
			? static_cast<ReturnValue*>(nullptr)
			: &(*found);
	}

	void ERBus::SetTaps(std::span<const ERUpdateData> newERData)
	{
		// List of ERs to delete
		// (must be outside of the SafeERsWrite scope)
		std::pmr::vector<RealtimeData*> eraseList;

		// 1. First and foremost clean up the faded out ERs
		{
			SafeERsWrite ersAcquire(mERs);
			auto& ers = ersAcquire->GetERs();

			// Reserve max size we could possibly need
			// for all ERs potentially to be deleted
			eraseList.reserve(ers.size());
			
			for (ER& er : ers)
			{
				if (er.RealtimeState->State.load(std::memory_order_acquire) == ERState::Stopped &&
					er.TargetData.State != ERState::Rendering) // add to erase list both, FadingOut and Stopped
				{
					er.TargetData.State = ERState::Stopped;
					eraseList.push_back(er.RealtimeState);
				}
			}
		} // ...after this scope reatlime thread won't touch RealtimeState if TargetData.State is Stopped...
		
		{
			SafeERsWrite ersAcquire(mERs);
			auto& ers = ersAcquire->GetERs();

			// 2. Remove from the list of ERs and deallocate RealtimeState
			// for ERs marked to stop on the previous update and which have faded out
			ersAcquire->EraseIf([&eraseList](const ER& er)
			{
#if defined(JPL_ENABLE_ASSERTS)
				if (er.TargetData.State == ERState::Stopped)
				{
					// The realtime thread should had finished before this point,
					// if this assert fails, eitehr we didn't wait for it before setting TargetData.State,
					// or something strange has happend.
					JPL_ASSERT(er.RealtimeState->State.load(std::memory_order_acquire) == ERState::Stopped);
					
					// Sanity check
					JPL_ASSERT(stdr::find(eraseList, er.RealtimeState) != stdr::end(eraseList));
					return true;
				}
				else
				{
					return false;
				}
#else
				return er.TargetData.State == ERState::Stopped;
#endif
			});

			// ...now it should be safe to delete stale realtime states
			for (auto* realtimeState : eraseList)
			{
				DeallocateRTData(realtimeState);
			}
			eraseList.clear();

			// Working with partitioned ranges improves the performance by about 1.5-2.0x
			std::span<ER> ersToUpdate;
			std::span<ER> ersStale;
			ersAcquire->PartitionERs(newERData, ersToUpdate, ersStale);

			for (ER& staleER : ersStale)
			{
				// 3. Mark for fade-out ERs that we no longer need.
				// We need to: Fade-out -> Flag when finish -> Delete tap

				const ERState currentState = staleER.RealtimeState->State.load(std::memory_order_acquire);
				if (currentState == ERState::Stopped)
				{
					staleER.TargetData.State = ERState::Stopped; // We'll delete on the next update
				}
				else if (currentState != ERState::FadingOut)
				{
					std::fill(staleER.TargetData.ChannelGains, staleER.TargetData.ChannelGains + mNumChannels, 0.0f);
					staleER.TargetData.State = ERState::FadingOut;
				}
			}

			// Temp buffer for new ERs.
			// It's faster to append new buffer,
			// than to push directly to "working" data,
			// which would increase the size of the vector to search for.
			// Also we can use the same allcoator we're moving new things "to".
			std::pmr::vector<ER> newERs(ersAcquire->GetAllocator());
			newERs.reserve(newERData.size());

			// 4. Add new ERs, or update existing
			for (const ERUpdateData& newER : newERData)
			{
				if (auto* toUpdateER = FindERByID(ersToUpdate, newER.Id))
				{
					// If a tap was requrested to update, but it is also stale (stopped),
					// we need to wait for it to finish fading out and then mark for deletion
					// on the next update.
					// And create a new one.
					
					const ERState currentRealtimeState = toUpdateER->RealtimeState->State.load(std::memory_order_acquire);
					if (currentRealtimeState == ERState::FadingOut)
					{
						// Wait to finish fading out
						// (in practice this never spins)
						while (toUpdateER->RealtimeState->State.load(std::memory_order_acquire) != ERState::Stopped)
						{}
					}

					// 5. Update existing
					ersAcquire->UpdateTargetERData(
						toUpdateER->TargetData,
						newER.FilterGains,
						newER.Gains.data(),
						newER.Delay * mSampleRate,
						ERState::Rendering
					);
				}
				else // ERs "to add" won't be in 'ersToUpdate', niether in 'ersStale'
				{
					// 6. Create new ERs

					// ER channel gains must be provided for this bus' channel count
					JPL_ASSERT(newER.Gains.size() == mNumChannels);

					const float delayInSamples = newER.Delay * mSampleRate;

					newERs.emplace_back(
						ersAcquire->MakeTargetERData(
							newER.FilterGains,
							newER.Gains.data(),
							delayInSamples,
							ERState::Rendering
						),
						AllocateRTData(
							newER.FilterGains,
							delayInSamples,
							ERState::Rendering),
						newER.Id
					);
				}
			}

			// 7. Append-move the newly created ERs
			const std::size_t newERCount = ersAcquire->InsertNewERs(std::move(newERs));
			mNumTaps = static_cast<uint32_t>(newERCount);

		} // ER Data update is released for the realtime thread
	}

	auto ERBus::AllocateRTData() -> RealtimeData*
	{
		std::pmr::polymorphic_allocator<float> allocator(&mPool);

		// Allocate object
		auto* realtimeData = allocator.new_object<RealtimeData>();

		// Allocate channel gains
		realtimeData->ChannelGains = allocator.allocate(mNumChannels);
		std::fill(realtimeData->ChannelGains, realtimeData->ChannelGains + mNumChannels, 0.0f);

		return realtimeData;
	}

	auto ERBus::AllocateRTData(const JPL::simd& filterGains, float delayInSamples, ERState state) -> RealtimeData*
	{
		RealtimeData* realtimeData = AllocateRTData();
		realtimeData->FilterGains = filterGains;
		realtimeData->Filter.Prepare(mSampleRate);
		realtimeData->Tap = DelayLineType::CreateTap<TapType>(delayInSamples);
		realtimeData->DelayTime = JPL::SmoothedValue<float>::CreateExpSmoothing(delayInSamples, delayInSamples, cScratchSize);
		realtimeData->State.store(state, std::memory_order_release);

		return realtimeData;
	}

	void ERBus::DeallocateRTData(RealtimeData* data)
	{
		std::pmr::polymorphic_allocator<float> allocator(&mPool);

		// Deallocate channel gains
		allocator.deallocate(data->ChannelGains, mNumChannels);

		// Deallocate object
		allocator.delete_object(data);
	}

	ERBus::ERStorage::ERStorage(std::pmr::memory_resource& resource)
		: mAllocator(&resource)
		, ERs(&resource)
	{
	}

	ERBus::ERStorage::ERStorage(std::pmr::memory_resource& resource, uint32_t numChannels)
		: mAllocator(&resource)
		, NumChannels(numChannels)
		, ERs(&resource)
	{
	}

	ERBus::ERStorage::ERStorage(const ERStorage& other)
		: mAllocator(other.mAllocator)
		, ERs(other.ERs, other.mAllocator)
		, NumChannels(other.NumChannels)
	{
		AllocateCopyChannelGains(other.ERs);
	}

	ERBus::ERStorage& ERBus::ERStorage::operator=(const ERStorage& other)
	{
		// Must use the same memory resource
		assert(mAllocator == other.mAllocator);

		if (NumChannels != 0)
		{
			for (ER& er : ERs)
				mAllocator.deallocate(er.TargetData.ChannelGains, NumChannels);
		}

		ERs = other.ERs;
		NumChannels = other.NumChannels;

		AllocateCopyChannelGains(other.ERs);
		return *this;
	}

	ERBus::ERBus::ERStorage::ERStorage(ERStorage&& other) noexcept
		: mAllocator(other.mAllocator)
		, ERs(std::move(other.ERs), other.mAllocator)
		, NumChannels(other.NumChannels)

	{
		other.NumChannels = 0;
	}

	ERBus::ERStorage& ERBus::ERStorage::operator=(ERStorage&& other) noexcept
	{
		// Must use the same memory resource
		assert(mAllocator == other.mAllocator);

		if (NumChannels != 0)
		{
			for (ER& er : ERs)
				mAllocator.deallocate(er.TargetData.ChannelGains, NumChannels);
		}

		ERs = std::move(other.ERs);
		NumChannels = other.NumChannels;
		other.NumChannels = 0;

		return *this;
	}

	ERBus::ERStorage::~ERStorage()
	{
		if (NumChannels != 0)
		{
			for (ER& er : ERs)
				mAllocator.deallocate(er.TargetData.ChannelGains, NumChannels);
		}
	}

	void ERBus::ERStorage::UpdateChannelGains(float*& destination, const float* source)
	{
		std::memcpy(destination, source, sizeof(float) * NumChannels);
	}

	float* ERBus::ERStorage::AllocateCopyChannelGains(const float* source)
	{
		float* newGains = AllocateChannelGains();
		if (newGains)
			UpdateChannelGains(newGains, source);
		return newGains;
	}

	void ERBus::ERStorage::DeallocateChannelGains(float* gains)
	{
		if (gains)
			mAllocator.deallocate(gains, NumChannels);
	}

	auto ERBus::ERStorage::FindERByID(uint32_t id) -> ER*
	{
		return JPL::FindERByID(ERs, id);
	}

	template<std::ranges::range UpdateData>
	void ERBus::ERStorage::PartitionERs(const UpdateData& updateERs,
										std::span<ER>& outFoundERs,
										std::span<ER>& outNotFoundERs)
	{
		auto last = std::partition(ERs.begin(), ERs.end(), [&updateERs](const ER& er)
		{
			return JPL::FindERByID(updateERs, er.Id);
		});
		outFoundERs = { ERs.begin(), last };
		outNotFoundERs = { last, ERs.end() };
	}

	typename ERBus::TargetERData ERBus::ERStorage::MakeTargetERData(const JPL::simd& filterGains,
																	const float* channelGains,
																	float delayInSamples,
																	ERState state)
	{
		return TargetERData{
			.FilterGains = filterGains,
			.ChannelGains = AllocateCopyChannelGains(channelGains),
			.DelayTime = delayInSamples,
			.State = state
		};
	}

	void ERBus::ERStorage::UpdateTargetERData(TargetERData& er,
											  const JPL::simd& filterGains,
											  const float* channelGains,
											  float delayInSamples,
											  ERState state)
	{
		er.FilterGains = filterGains;
		UpdateChannelGains(er.ChannelGains, channelGains);
		er.DelayTime = delayInSamples;
		er.State = state;
	}

	float* ERBus::ERStorage::AllocateChannelGains()
	{
		return NumChannels != 0
			? mAllocator.allocate(NumChannels)
			: static_cast<float*>(nullptr);
	}

	void ERBus::ERStorage::AllocateCopyChannelGains(const std::pmr::vector<ER>& source)
	{
		if (NumChannels == 0)
			return;

		for (uint32_t i = 0; i < ERs.size(); ++i)
		{
			ERs[i].TargetData.ChannelGains = AllocateChannelGains();
			std::memcpy(
				ERs[i].TargetData.ChannelGains,
				source[i].TargetData.ChannelGains,
				sizeof(float) * NumChannels);
		}
	}

	template<class Predicate>
	inline std::size_t ERBus::ERStorage::EraseIf(Predicate predicate)
	{
		const std::size_t oldSize = ERs.size();

		auto first = std::partition(ERs.begin(), ERs.end(), [&predicate](const ER& er)
		{
			// Negate the predicate to move satisfying elements to the end
			return !predicate(er);
		});

		// We need to deallcoate channel gains
		for (const ER& er : std::span{ first, ERs.end() })
			DeallocateChannelGains(er.TargetData.ChannelGains);

		ERs.erase(first, ERs.end());

		return oldSize - ERs.size();
	}

	template<class Predicate>
	std::size_t ERBus::ERStorage::EraseIf(std::size_t begin, std::size_t count, Predicate predicate)
	{
		const std::size_t oldSize = ERs.size();

		if ((begin + count) > oldSize)
			return oldSize;

		auto beginI = ERs.begin() + begin;
		auto endI = ERs.begin() + (begin + count);
		auto first = std::partition(beginI, endI, [&predicate](const ER& er)
		{
			// Negate the predicate to move satisfying elements to the end
			return !predicate(er);
		});

		endI = ERs.begin() + (begin + count);

		// We need to deallcoate channel gains
		for (const ER& er : std::span{ first, endI })
			DeallocateChannelGains(er.TargetData.ChannelGains);

		ERs.erase(first, endI);

		return oldSize - ERs.size();
	}

	std::size_t ERBus::ERStorage::Erase(std::size_t begin, std::size_t count)
	{
		const std::size_t oldSize = ERs.size();

		if ((begin + count) > oldSize)
			return oldSize;

		// We need to deallcoate channel gains
		for (const ER& er : std::span{ ERs.data() + begin, count })
			DeallocateChannelGains(er.TargetData.ChannelGains);

		ERs.erase(ERs.begin() + begin, ERs.begin() + (begin + count));

		return oldSize - ERs.size();
	}

	std::size_t ERBus::ERStorage::InsertNewERs(std::pmr::vector<ER>&& newERs)
	{
		ERs.reserve(ERs.size() + newERs.size());
		ERs.insert(
			ERs.end(),
			std::make_move_iterator(newERs.begin()),
			std::make_move_iterator(newERs.end())
		);
		return ERs.size();
	}
} // namespace JPL
