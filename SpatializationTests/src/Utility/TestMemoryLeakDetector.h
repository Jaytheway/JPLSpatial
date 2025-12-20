//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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

#include "JPLSpatial/Core.h"
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Memory/Memory.h"

#include <gtest/gtest.h>

#include <memory_resource>
#include <bitset>
#include <cstddef>

// Enables global new/delete overrides and testing for any "leaks" bypassing PMR
#ifndef JPL_TEST_GLOBAL_NEW_LEAKS
#define JPL_TEST_GLOBAL_NEW_LEAKS 1
#endif // !JPL_TEST_GLOBAL_NEW_LEAKS

namespace JPL
{
	//======================================================================
	class MallocatorCounting
	{
		static constexpr std::size_t cDefaultAlignment = JPL_DEFAULT_NEW_ALIGNMENT;
		static_assert(cDefaultAlignment >= sizeof(std::size_t));

		static constexpr std::size_t cTagSize = sizeof(std::size_t);
		static_assert(cTagSize <= cDefaultAlignment);
	public:
		MallocatorCounting() = default;

		JPL_INLINE void					ResetCounters();
		[[nodiscard]] JPL_INLINE void* Allocate(std::size_t inSize);
		[[nodiscard]] inline void* Reallocate(void* inBlock, std::size_t inOldSize, std::size_t inNewSize);
		JPL_INLINE void					Free(void* inBlock);
		[[nodiscard]] JPL_INLINE void* AlignedAllocate(std::size_t inSize, std::size_t inAlignment);
		JPL_INLINE void					AlignedFree(void* inBlock, std::size_t inAlignment);

	private:
		[[nodiscard]] JPL_INLINE std::byte* UpdateSizeRecord(void* block, std::size_t newSize);
		[[nodiscard]] JPL_INLINE void* AddAllocation(void* block, std::size_t size, std::size_t alignment);
		[[nodiscard]] JPL_INLINE void* SubtractAllocation(void* inBlock, std::size_t alignment);

	public:
		std::size_t InUse = 0;
		std::size_t MaxUsage = 0;
	};
#if JPL_TEST_GLOBAL_NEW_LEAKS
	// Single global object we use just for testing
	MallocatorCounting& GetGlobalNewDeleteCounter();

#ifdef JPL_TEST_GLOBAL_NEW_LEAKS_DEF
	MallocatorCounting& GetGlobalNewDeleteCounter()
	{
		static MallocatorCounting gGlobalNewDeleteCounter;
		return gGlobalNewDeleteCounter;
	}
#endif
#endif
} // namespace JPL

#if JPL_TEST_GLOBAL_NEW_LEAKS
void* operator new (size_t inCount);
void operator delete (void* inPointer) noexcept;
void* operator new[](size_t inCount);
void operator delete[](void* inPointer) noexcept;
void* operator new (size_t inCount, std::align_val_t inAlignment);
void operator delete (void* inPointer, std::align_val_t inAlignment) noexcept;
void* operator new[](size_t inCount, std::align_val_t inAlignment);
void operator delete[](void* inPointer, std::align_val_t inAlignment) noexcept;

#ifdef JPL_TEST_GLOBAL_NEW_LEAKS_DEF
void* operator new (size_t inCount) { return JPL::GetGlobalNewDeleteCounter().Allocate(inCount); }
void operator delete (void* inPointer) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }
void* operator new[](size_t inCount) { return JPL::GetGlobalNewDeleteCounter().Allocate(inCount); }
void operator delete[](void* inPointer) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }
void* operator new (size_t inCount, std::align_val_t inAlignment) { return JPL::GetGlobalNewDeleteCounter().AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); }
void operator delete (void* inPointer, std::align_val_t inAlignment) noexcept { JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<size_t>(inAlignment)); }
void* operator new[](size_t inCount, std::align_val_t inAlignment) { return JPL::GetGlobalNewDeleteCounter().AlignedAllocate(inCount, static_cast<size_t>(inAlignment)); }
void operator delete[](void* inPointer, std::align_val_t inAlignment) noexcept { JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<size_t>(inAlignment)); }
#endif
#endif

namespace JPL
{
	//======================================================================
	JPL_INLINE void MallocatorCounting::ResetCounters()
	{
		InUse = 0;
		MaxUsage = 0;
	}

	JPL_INLINE void* MallocatorCounting::Allocate(std::size_t inSize)
	{
		auto* p = Mallocator::Allocate(inSize + cDefaultAlignment);
		return AddAllocation(p, inSize, cDefaultAlignment);
	}

	inline void* MallocatorCounting::Reallocate(void* inBlock, std::size_t inOldSize, std::size_t inNewSize)
	{
		if (inBlock == nullptr)
			return Allocate(inNewSize);

		if (inOldSize > inNewSize)
			InUse -= inOldSize - inNewSize;
		else
			InUse += inNewSize - inOldSize;

		if (InUse > MaxUsage)
			MaxUsage = InUse;

		auto* p = (std::byte*)inBlock - cDefaultAlignment;

		//! Note: this may not play well if initial alignment was > cDefaultAlignment
		p = (std::byte*)Mallocator::Reallocate(p, inOldSize + cDefaultAlignment, inNewSize + cDefaultAlignment);
		return UpdateSizeRecord(p, inNewSize) + cDefaultAlignment;
	}

	JPL_INLINE void MallocatorCounting::Free(void* inBlock)
	{
		if (inBlock == nullptr)
			return;

		auto* p = SubtractAllocation(inBlock, cDefaultAlignment);
		Mallocator::Free(p);
	}

	JPL_INLINE void* MallocatorCounting::AlignedAllocate(std::size_t inSize, std::size_t inAlignment)
	{
		JPL_ASSERT(inAlignment > cDefaultAlignment);

		// We are wasting quite a bit here if alighment is much greater than sizeof(std::size_t)
		// but it's fine for testing

		const std::size_t alignment = inAlignment;
		auto* p = Mallocator::AlignedAllocate(inSize + alignment, alignment);
		return AddAllocation(p, inSize, alignment);
	}

	JPL_INLINE void MallocatorCounting::AlignedFree(void* inBlock, std::size_t inAlignment)
	{
		if (inBlock == nullptr)
			return;

		JPL_ASSERT(inAlignment > cDefaultAlignment);
		auto* p = SubtractAllocation(inBlock, inAlignment);
		Mallocator::AlignedFree(p);
	}

	JPL_INLINE std::byte* MallocatorCounting::UpdateSizeRecord(void* block, std::size_t newSize)
	{
		auto* p = (std::byte*)block;
		*(std::size_t*)(p) = newSize;
		return p;
	}

	JPL_INLINE void* MallocatorCounting::AddAllocation(void* block, std::size_t size, std::size_t alignment)
	{
		InUse += size;
		if (InUse > MaxUsage)
			MaxUsage = InUse;

		return UpdateSizeRecord(block, size) + alignment;

	}

	JPL_INLINE void* MallocatorCounting::SubtractAllocation(void* inBlock, std::size_t alignment)
	{
		auto* p = ((std::byte*)inBlock) - alignment;
		const auto size = *(std::size_t*)(p);

		// It's possible `InUse` counter was reset
		// while some memory hasn't been deallocated yet
		InUse = InUse >= size ? InUse - size : 0;

		return p;
	}


#if JPL_TEST_GLOBAL_NEW_LEAKS
	//======================================================================
	// Checking if scope leaves allocation side effects
	struct ScopedNewDeleteCounter
	{
		std::size_t PrevInUse = 0;
		std::size_t PrevMaxUsage = 0;

		inline void GetCounters(std::size_t& outInUse, std::size_t& outMaxUsage) const
		{
			outInUse = GetGlobalNewDeleteCounter().InUse;
			outMaxUsage = GetGlobalNewDeleteCounter().MaxUsage;
		}

		ScopedNewDeleteCounter()
		{
			PrevInUse = GetGlobalNewDeleteCounter().InUse;
			PrevMaxUsage = GetGlobalNewDeleteCounter().MaxUsage;
			GetGlobalNewDeleteCounter().ResetCounters();
		}

		~ScopedNewDeleteCounter()
		{
			GetGlobalNewDeleteCounter().InUse = PrevInUse;
			GetGlobalNewDeleteCounter().MaxUsage = PrevMaxUsage;
		}
	};
#endif

	//======================================================================
	/// RAII override for default pmr resource
	struct ScopedStdPmrResourceOverride
	{
		std::pmr::memory_resource* Previous;

		inline explicit ScopedStdPmrResourceOverride(std::pmr::memory_resource* resource)
			: Previous(std::pmr::get_default_resource())
		{
			std::pmr::set_default_resource(resource);
		}

		inline ~ScopedStdPmrResourceOverride()
		{
			std::pmr::set_default_resource(Previous);
		}
	};
	
	//======================================================================
	//! NOTE: GTest's SCOPED_TRACE allocates and messes up the leak detector,
	//!		- either ignore
	class TestLeakDetector
	{
	public:
		enum EOptions : int
		{
			Default = 0,				// only check if anything hasn't been deallocated
			CheckNonJPLMaxUsage_IsZero, // check std pmr as well as global new/delete
			CheckJPLMaxUsage_IsNonZero,	// check that anything had been allocated with jpl pmr
			SkipGlobalNewDelete,		// skip checking leaks for global new/delete (only relevant if JPL_TEST_GLOBAL_NEW_LEAKS is defined as non zero)
		};

		~TestLeakDetector() = default;

		explicit TestLeakDetector(int options = EOptions::Default)
			: mOptions(options)
		{
		}

	public:
		// Must be called from test SetUp
		void SetUp()
		{
			// Reset stack allocator
			mBufResource.release();

			// For the sake of sanity, ensure destructors have been called 
			if (mPmrStdOverride)
				mStackAllocator.delete_object(mPmrStdOverride);

			if (mPmrJPLOverride)
				mStackAllocator.delete_object(mPmrJPLOverride);

			if (mNewDeleteOverride)
				mStackAllocator.delete_object(mNewDeleteOverride);

			// Reset counters
			mPmrStdCounter.InUse = 0;
			mPmrStdCounter.MaxUsage = 0;
			mPmrJPLCounter.InUse = 0;
			mPmrJPLCounter.MaxUsage = 0;

			// Override default global std pmr
			mPmrStdOverride = mStackAllocator.new_object<ScopedStdPmrResourceOverride>(&mPmrStdCounter);
			// Override default JPLSpatial pmr
			mPmrJPLOverride = mStackAllocator.new_object<ScopedGlobalMemoryResource>(&mPmrJPLCounter);

#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Insert our counter into global new/delete overide.
			// This includes:
			// - all pmr stuff using default resource
			// - global new/delete
			// - std::allocator (internally calls new/delete)
			mNewDeleteOverride = mStackAllocator.new_object<ScopedNewDeleteCounter>();
#endif

			// Sanity check
			ASSERT_TRUE(std::pmr::get_default_resource() == &mPmrStdCounter);
			ASSERT_TRUE(GetDefaultMemoryResource() == &mPmrJPLCounter);
			ASSERT_NE(std::pmr::get_default_resource(), GetDefaultMemoryResource());
		}
		
		// Must be called from test TearDown
		void TearDown()
		{
#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Get the counters before resetting the override object
			mNewDeleteOverride->GetCounters(mDefaultNewDeleteInUse, mDefaultNewDeleteMaxUsage);
#endif

			// - InUse = 0: indicates leaks, either deallocated from different memory resource
			// or not deallocated at all.
			// 
			// - MaxUsage: shows if any allocations happened in the given context.

#if JPL_TEST_GLOBAL_NEW_LEAKS
			// Default new/delete spill
			if (!mOptions.test(SkipGlobalNewDelete))
			{
				EXPECT_EQ(mDefaultNewDeleteInUse, 0) << "Allocations bypassed PMR and leaked.";
				if (mOptions.test(CheckNonJPLMaxUsage_IsZero))
					EXPECT_EQ(mDefaultNewDeleteMaxUsage, 0) << "Allocations bypassed PMR.";
			}
#endif

			// Pmr Std spill
			EXPECT_EQ(mPmrStdCounter.InUse, 0) << "PMR allocations bypassed JPL PMR and leaked.";
			if (mOptions.test(CheckNonJPLMaxUsage_IsZero))
				EXPECT_EQ(mPmrStdCounter.MaxUsage, 0) << "PMR allocations bypassed JPL PMR.";

			// JPL Pmr Std
			EXPECT_EQ(mPmrJPLCounter.InUse, 0) << "JPL PMR allocations leaked.";
			if (mOptions.test(CheckJPLMaxUsage_IsNonZero))
				EXPECT_GT(mPmrJPLCounter.MaxUsage, 0) << "JPL PMR allocations detected.";

			// Revert overrides

#if JPL_TEST_GLOBAL_NEW_LEAKS
			mStackAllocator.delete_object(mNewDeleteOverride);
			mNewDeleteOverride = nullptr;
#endif
			mStackAllocator.delete_object(mPmrJPLOverride);
			mPmrJPLOverride = nullptr;

			mStackAllocator.delete_object(mPmrStdOverride);
			mPmrStdOverride = nullptr;
		}

	private:
		std::bitset<32> mOptions = EOptions::Default;

		// Using stack allocator to bypass heap that we're testing
		std::byte mBuffer[256];
		std::pmr::monotonic_buffer_resource mBufResource{ &mBuffer, sizeof(mBuffer), std::pmr::null_memory_resource() };
		std::pmr::polymorphic_allocator<> mStackAllocator{ &mBufResource };

		PmrMallocResource mPmrStdMalloc;
		PmrMallocResource mPmrJPLMalloc;
		CountingResource mPmrStdCounter{ &mPmrStdMalloc };
		CountingResource mPmrJPLCounter{ &mPmrStdMalloc };

		ScopedStdPmrResourceOverride* mPmrStdOverride = nullptr;
		ScopedGlobalMemoryResource* mPmrJPLOverride = nullptr;
#if JPL_TEST_GLOBAL_NEW_LEAKS
		ScopedNewDeleteCounter* mNewDeleteOverride = nullptr;
		std::size_t mDefaultNewDeleteInUse = 0;
		std::size_t mDefaultNewDeleteMaxUsage = 0;
#endif

	};

	class ScopedTestLeakDetector
	{
		TestLeakDetector mLeakDetector;
	public:
		inline explicit ScopedTestLeakDetector(int options = TestLeakDetector::EOptions::Default)
			: mLeakDetector(options)
		{
			mLeakDetector.SetUp();
		}

		inline ~ScopedTestLeakDetector()
		{
			mLeakDetector.TearDown();
		}
	};

} // namespace JPL