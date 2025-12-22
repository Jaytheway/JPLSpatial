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

#ifdef JPL_COMPILER_MSVC
#include <malloc.h>
#else
#include <cstdlib>
#endif
#include <memory>
#include <type_traits>
#include <concepts>
#include <memory_resource>
#include <cstddef>

namespace JPL
{
	//======================================================================
	/// Default memory resource used throughout the library
	//! (we may or may not want to make this thread_local)
	inline std::pmr::memory_resource* gDefaultMemoryResource = std::pmr::get_default_resource();
	inline std::pmr::memory_resource* GetDefaultMemoryResource() noexcept { return gDefaultMemoryResource; }

	//======================================================================
	/// RAII override library-wide default memory resource
	class ScopedGlobalMemoryResource
	{
	public:
		explicit ScopedGlobalMemoryResource(std::pmr::memory_resource* resource)
			: mPrevious(gDefaultMemoryResource)
		{
			gDefaultMemoryResource = resource;
		}

		~ScopedGlobalMemoryResource()
		{
			gDefaultMemoryResource = mPrevious;
		}

	private:
		std::pmr::memory_resource* mPrevious;
	};

	//======================================================================
	/// A simple deleter to be used for std::unique_ptr and alike,
	/// that uses custom allocator, which must match the allocator
	/// used to allocate memory for the object.
	template<class T, class AllocatorType = std::allocator<T>>
	struct AllocatorDeleter
	{
		using AForT = typename std::allocator_traits<AllocatorType>::template rebind_alloc<T>;
		using Traits = std::allocator_traits<AForT>;

		AllocatorDeleter() noexcept(std::is_nothrow_default_constructible_v<AForT>) = default;

		explicit AllocatorDeleter(const AllocatorType& allocator) noexcept(std::is_nothrow_constructible_v<AForT, const AllocatorType&>)
			: mAllocator(allocator)
		{
		}

		// Accept already-rebound allocator too
		explicit AllocatorDeleter(const AForT& allcoator) noexcept(std::is_nothrow_copy_constructible_v<AForT>) requires (!std::same_as<AllocatorType, AForT>)
			: mAllocator(allcoator)
		{
		}

		JPL_INLINE void operator()(T* ptr) noexcept
		{
			if (!ptr)
				return;
			Traits::destroy(mAllocator, ptr);
			Traits::deallocate(mAllocator, ptr, 1);
		}

	private:
		[[no_unique_address]] AForT mAllocator{};
	};
	
	//======================================================================
	/// Specialization for PMR, since pmr allocator is not move-assignalbe,
	/// isntead of allocator, we store pointer to its memory resourece.
	template<class T>
	class PmrDeleter
	{
	public:
		using Allocator = std::pmr::polymorphic_allocator<T>;
		using Traits = std::allocator_traits<Allocator>;

		PmrDeleter() = default;
		explicit PmrDeleter(std::pmr::memory_resource* resource) noexcept
			: mResource(resource)
		{
		}

		JPL_INLINE void operator()(T* p) noexcept
		{
			if (!p)
				return;
			Allocator allocator{ mResource };
			Traits::destroy(allocator, p);
			Traits::deallocate(allocator, p, 1);
		}
	private:
		std::pmr::memory_resource* mResource = GetDefaultMemoryResource();
	};
	// Sanity check
	static_assert(std::is_move_assignable_v<PmrDeleter<int>>);

	namespace Internal
	{
		template<class Allocator>
		inline constexpr bool is_pmr_alloc_v =
			std::is_same_v<
				typename std::allocator_traits<Allocator>::template rebind_alloc<std::byte>,
				std::pmr::polymorphic_allocator<std::byte>
			>;
	}

	//======================================================================
	/// Utility helper to wrap usage of custom allocator with std::unique_ptr
	/// to allocate object and pass AllocatorDeleter<T> as deleter to
	/// deallocate using the same allocator.
	template<class T, class Allocator, class... Args>
	[[nodiscard]] auto allocate_unique(const Allocator& allocator, Args&&... args)
	{
		using AForT = typename std::allocator_traits<Allocator>::template rebind_alloc<T>;
		using Traits = std::allocator_traits<AForT>;
		AForT rebound(allocator);

		T* raw = Traits::allocate(rebound, 1);

		if constexpr (std::is_nothrow_constructible_v<T, Args...>)
		{
			Traits::construct(rebound, raw, std::forward<Args>(args)...);
		}
		else
		{
			try
			{
				Traits::construct(rebound, raw, std::forward<Args>(args)...);
			} catch (...)
			{
				Traits::deallocate(rebound, raw, 1);
				throw;
			}
		}

		if constexpr (Internal::is_pmr_alloc_v<std::remove_cvref_t<Allocator>>)
		{
			using Deleter = PmrDeleter<T>;
			return std::unique_ptr<T, Deleter>(raw, Deleter(allocator.resource()));
		}
		else
		{
			using Deleter = AllocatorDeleter<T, Allocator>;
			return std::unique_ptr<T, Deleter>(raw, Deleter(rebound));
		}
	}

	//======================================================================
	/// Just a shorter alias
	template<class T>
	using PmrAllocator = std::pmr::polymorphic_allocator<T>;

	template<class T = std::byte>
	[[nodiscard]] JPL_INLINE PmrAllocator<T> GetDefaultPmrAllocator() { return PmrAllocator<T>(GetDefaultMemoryResource());  }

	//======================================================================
	/// Allocate new object from deafult global memory resource used in JPLSpatial
	template<class T, class ...Args>
	[[nodiscard]] JPL_INLINE T* DefaultNew(Args&& ...args)
	{
		return GetDefaultPmrAllocator<>().new_object<T>(std::forward<Args>(args)...);
	}

	/// Delete object allcoated from deafult global memory resource. The object must had been allocated by calling `DefaultNew` 
	template<class T>
	JPL_INLINE void DefaultDelete(T* object)
	{
		GetDefaultPmrAllocator<>().delete_object(object);
	}

	//======================================================================
	/// std::make_shared wrapper using our global memory resource
	template<class T, class... Args>
	[[nodiscard]] JPL_INLINE std::shared_ptr<T> make_pmr_shared(Args&&... args)
	{
		return std::allocate_shared<T>(GetDefaultPmrAllocator<T>(), std::forward<Args>(args)...);
	}
	
	/// std::make_shared wrapper using out global memory resource.
	/// This overload takes pointer to object created by DefaultNew<T>()
	/// which is then deleted with PmrDeleter from default memory resource.
	template<class T>
	[[nodiscard]] JPL_INLINE std::shared_ptr<T> make_pmr_shared(T* ptr)
	{
		return std::shared_ptr<T>(ptr, PmrDeleter<T>(GetDefaultMemoryResource()), GetDefaultPmrAllocator<T>());
	}

	template<class T, class Y>
	JPL_INLINE void reset_pmr_shared(std::shared_ptr<T>& sharedPtr, Y* ptr)
	{
		sharedPtr.reset(ptr, PmrDeleter<Y>(GetDefaultMemoryResource()), GetDefaultPmrAllocator<Y>());
	}
	template<class T>
	JPL_INLINE void reset_pmr_shared(std::shared_ptr<T>& sharedPtr)
	{
		sharedPtr.reset(static_cast<T*>(nullptr), PmrDeleter<T>(GetDefaultMemoryResource()), GetDefaultPmrAllocator<T>());
	}

	/// Alias for unique ptr using pmr allocator and deleter
	template<class T>
	using pmr_unique_ptr = std::unique_ptr<T, PmrDeleter<T>>;

	/// std::make_unique wrapper using our global memory resource
	template<class T, class... Args>
	[[nodiscard]] JPL_INLINE pmr_unique_ptr<T> make_pmr_unique(Args&&... args)
	{
		return allocate_unique<T>(GetDefaultPmrAllocator<T>(), std::forward<Args>(args)...);
	}

	//======================================================================
	/// Simple std::memory_resource wrapper that counts memory usage.
	/// Can be used for debugging and for dynamic memory resources
	/// adjustment based on the usage.
	class CountingResource : public std::pmr::memory_resource
	{
	public:
		explicit CountingResource(std::pmr::memory_resource* upstream)
			: Upstream(upstream)
		{
		}

	private:
		[[nodiscard]] JPL_INLINE void* do_allocate(std::size_t bytes, std::size_t align) override
		{
			void* p = Upstream->allocate(bytes, align);
			InUse += bytes;
			if (InUse > MaxUsage)
				MaxUsage = InUse;

			return p;
		}

		JPL_INLINE void do_deallocate(void* p, std::size_t bytes, std::size_t align) override
		{
			InUse -= bytes;
			Upstream->deallocate(p, bytes, align);
		}

		[[nodiscard]] JPL_INLINE bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override
		{
			return this == &other;
		}

	public:
		std::pmr::memory_resource* Upstream;
		std::size_t InUse = 0;
		std::size_t MaxUsage = 0;
	};

	//======================================================================
	class Mallocator
	{
	public:
		[[nodiscard]] JPL_INLINE static void* Allocate(std::size_t inSize)
		{
			JPL_ASSERT(inSize > 0);
			return std::malloc(inSize);
		}

		[[nodiscard]] JPL_INLINE static void* Reallocate(void* inBlock, [[maybe_unused]] std::size_t inOldSize, std::size_t inNewSize)
		{
			JPL_ASSERT(inNewSize > 0);
			return std::realloc(inBlock, inNewSize);
		}

		JPL_INLINE static void Free(void* inBlock)
		{
			std::free(inBlock);
		}

		[[nodiscard]] JPL_INLINE static void* AlignedAllocate(std::size_t inSize, std::size_t inAlignment)
		{
			JPL_ASSERT(inSize > 0 && inAlignment > 0);

#if defined(JPL_PLATFORM_WINDOWS)
			// Microsoft doesn't implement posix_memalign
			return _aligned_malloc(inSize, inAlignment);
#else
			void* block = nullptr;
			JPL_SUPPRESS_WARNING_PUSH
			JPL_GCC_SUPPRESS_WARNING("-Wunused-result")
			JPL_CLANG_SUPPRESS_WARNING("-Wunused-result")
			(void)posix_memalign(&block, inAlignment, inSize);
			JPL_SUPPRESS_WARNING_POP
			return block;
#endif
		}

		JPL_INLINE static void AlignedFree(void* inBlock)
		{
#if defined(JPL_PLATFORM_WINDOWS)
			_aligned_free(inBlock);
#else
			free(inBlock);
#endif
		}
	};
	
	//======================================================================
	class PmrMallocResource : public std::pmr::memory_resource
	{
	public:
		PmrMallocResource() = default;

	private:
		[[nodiscard]] JPL_INLINE void* do_allocate(std::size_t bytes, std::size_t align) override
		{
			if (align > JPL_DEFAULT_NEW_ALIGNMENT)
				return Mallocator::AlignedAllocate(bytes, align);
			else
				return Mallocator::Allocate(bytes);
		}

		JPL_INLINE void do_deallocate(void* p, std::size_t bytes, std::size_t align) override
		{
			if (align > JPL_DEFAULT_NEW_ALIGNMENT)
				Mallocator::AlignedFree(p);
			else
				Mallocator::Free(p);
		}

		[[nodiscard]] JPL_INLINE bool do_is_equal(const std::pmr::memory_resource& other) const noexcept override
		{
			return this == &other;
		}
	};
} // namespace JPL