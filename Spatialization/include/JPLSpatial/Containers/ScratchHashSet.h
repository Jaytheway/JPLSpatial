//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2026 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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
#include <JPLSpatial/ErrorReporting.h>

#include <bit>
#include <cstring>
#include <cstddef>
#include <functional>
#include <limits>
#include <type_traits>

namespace JPL
{
    template<class T, class HashFunction, class Equals = std::equal_to<T>> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    class ScratchHashSet32;

    // Set that expects already hashed keys as inputs
    using ScratchHashSetIdentity = ScratchHashSet32<uint32, std::identity>;

    //==========================================================================
    /// Simple utility mainly for ensuring uniqueness of ranges of elements.
    /// HasFunction must return uint32_t hash of T.
    /// Uses provided memory buffer, no allocations/deallocations internally.
    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    class ScratchHashSet32 // TODO: convert to a proper hash table
    {
    public:
        ScratchHashSet32() = default;

        ScratchHashSet32(void* memory,
                         std::size_t memoryBytes,
                         uint32 expectedCount,
                         const HashFunction& hash = {},
                         const Equals& equals = {});

        [[nodiscard]] static JPL_INLINE std::size_t GetRequiredMemorySize(uint32 expectedCount);

        inline void Init(void* memory, std::size_t memoryBytes, uint32 expectedCount);

        inline bool Insert(const T& key);

        [[nodiscard]] inline bool Contains(const T& key) const;

        // Does not deallocate memory, only clears the occupancy flags and size counter
        JPL_INLINE void Clear();

        [[nodiscard]] JPL_INLINE uint32 size() const { return mSize; }
        [[nodiscard]] JPL_INLINE uint32 capacity() const { return mCapacity; }

    private:
        [[nodiscard]] static JPL_INLINE std::size_t GetCapacityForCount(uint32 expectedCount);

    private:
        [[no_unique_address]] HashFunction mHasher{};
        [[no_unique_address]] Equals mEquals{};
        uint32* mKeys = nullptr;
        uint8* mOcupied = nullptr;
        uint32 mCapacity = 0;
        uint32 mMask = 0;
        uint32 mSize = 0;
    };
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    inline ScratchHashSet32<T, HashFunction, Equals>::ScratchHashSet32(void* memory,
                                                                       std::size_t memoryBytes,
                                                                       uint32 expectedCount,
                                                                       const HashFunction& hash,
                                                                       const Equals& equals)
        : mHasher(hash)
        , mEquals(equals)
    {
        Init(memory, memoryBytes, expectedCount);
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    JPL_INLINE std::size_t ScratchHashSet32<T, HashFunction, Equals>::GetRequiredMemorySize(uint32 expectedCount)
    {
        const std::size_t capacity = GetCapacityForCount(expectedCount);
        return capacity * sizeof(T)     // keys
            + capacity * sizeof(uint8); // flags
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    JPL_INLINE std::size_t ScratchHashSet32<T, HashFunction, Equals>::GetCapacityForCount(uint32 expectedCount)
    {
        const std::size_t minCapacity = expectedCount > 0 ? expectedCount * 2llu : 2llu;
        JPL_ASSERT(minCapacity <= std::numeric_limits<uint32>::max()); // sanity check
        return std::bit_ceil(minCapacity);
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    inline void ScratchHashSet32<T, HashFunction, Equals>::Init(void* memory, std::size_t memoryBytes, uint32 expectedCount)
    {
        mCapacity = static_cast<uint32>(GetCapacityForCount(expectedCount));
        mMask = mCapacity - 1u;
        mSize = 0;

        const std::size_t slotsBytes = static_cast<std::size_t>(mCapacity) * sizeof(T);
        const std::size_t flagsBytes = static_cast<std::size_t>(mCapacity) * sizeof(uint8);

        JPL_ASSERT(slotsBytes + flagsBytes <= memoryBytes);

        std::byte* ptr = static_cast<std::byte*>(memory);
        mKeys = reinterpret_cast<T*>(ptr);
        mOcupied = reinterpret_cast<uint8*>(ptr + slotsBytes);

        std::memset(mOcupied, 0, flagsBytes);
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    inline bool ScratchHashSet32<T, HashFunction, Equals>::Insert(const T& key)
    {
        uint32 index = mHasher(key) & mMask;

        for (uint32 probe = 0; probe < mCapacity; ++probe)
        {
            if (not mOcupied[index])
            {
                mOcupied[index] = 1;
                mKeys[index] = key;
                ++mSize;
                return true;
            }

            if (mEquals(mKeys[index], key))
            {
                return false;
            }

            index = (index + 1u) & mMask;
        }

        JPL_ASSERT(false);
        return false;
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    inline bool ScratchHashSet32<T, HashFunction, Equals>::Contains(const T& key) const
    {
        uint32 index = mHasher(key) & mMask;

        for (uint32 probe = 0; probe < mCapacity; ++probe)
        {
            if (not mOcupied[index])
            {
                return false;
            }

            if (mEquals(mKeys[index], key))
            {
                return true;
            }

            index = (index + 1u) & mMask;
        }

        return false;
    }

    template<class T, class HashFunction, class Equals> requires(std::is_trivial<T>::value and std::is_copy_constructible_v<T>)
    JPL_INLINE void ScratchHashSet32<T, HashFunction, Equals>::Clear()
    {
        std::memset(mOcupied, 0, static_cast<std::size_t>(mCapacity) * sizeof(uint8));
        mSize = 0;
    }
} // namespace JPL

