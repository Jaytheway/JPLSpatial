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

#include <JPLSpatial/Core.h>
#include <JPLSpatial/ErrorReporting.h>

#include <algorithm>
#include <iterator>
#include <type_traits>

namespace JPL
{
    //==========================================================================
    // A simple utility for small buffers of trivial types
    template<class T, std::size_t Capacity> requires(std::is_trivial<T>::value && std::is_copy_constructible_v<T>)
    class StaticArray
    {
    public:
        static_assert(std::is_object_v<T>,  "The C++ Standard forbids containers of non-object types "
                                            "because of [container.requirements].");

        using element_type = T;
        using value_type = std::remove_cvref_t<T>;
        using size_type = std::size_t;
        using difference_type = ptrdiff_t;
        using pointer = T*;
        using const_pointer = const T*;
        using reference = T&;
        using const_reference = const T&;

        using iterator = pointer;
        using const_iterator = const_pointer;
        using reverse_iterator = std::reverse_iterator<iterator>;
        using const_reverse_iterator = std::reverse_iterator<const_iterator>;

        //==========================================================================
        constexpr StaticArray() noexcept = default;
        constexpr StaticArray(const StaticArray&) noexcept = default;
        constexpr StaticArray(StaticArray&&) noexcept = default;
        constexpr StaticArray& operator=(const StaticArray&) noexcept = default;
        constexpr StaticArray& operator=(StaticArray&&) noexcept = default;
        constexpr ~StaticArray() noexcept = default;

        constexpr JPL_INLINE explicit StaticArray(std::size_t size) noexcept
            : mSize(size)
        {
            JPL_ASSERT(size <= Capacity);
        }

        constexpr JPL_INLINE explicit StaticArray(std::size_t size, T defaultValue) noexcept
            : mSize(size)
        {
            JPL_ASSERT(size <= Capacity);
            std::ranges::fill_n(mStorage, size, defaultValue);
        }

        //==========================================================================
        // Capacity
        [[nodiscard]] constexpr JPL_INLINE size_type        size() const { return mSize; }
        [[nodiscard]] constexpr JPL_INLINE size_type        size_bytes() const { return mSize * sizeof(value_type); }
        [[nodiscard]] constexpr JPL_INLINE bool             empty() const { return size() == 0; }

        // iteration
        [[nodiscard]] constexpr JPL_INLINE iterator         begin() noexcept { return mSize ? &mStorage[0] : nullptr; }
        [[nodiscard]] constexpr JPL_INLINE iterator         end() noexcept { return mSize ? &mStorage[mSize] : nullptr; }

        [[nodiscard]] constexpr JPL_INLINE const_iterator   begin() const noexcept { return mSize ? &mStorage[0] : nullptr; }
        [[nodiscard]] constexpr JPL_INLINE const_iterator   end() const noexcept { return mSize ? &mStorage[mSize] : nullptr; }
        
        [[nodiscard]] constexpr JPL_INLINE const_iterator   cbegin() const noexcept { return begin(); }
        [[nodiscard]] constexpr JPL_INLINE const_iterator   cend() const noexcept { return end(); }

        [[nodiscard]] constexpr JPL_INLINE reverse_iterator rbegin() const noexcept { return reverse_iterator{ end() }; }
        [[nodiscard]] constexpr JPL_INLINE reverse_iterator rend() const noexcept { return reverse_iterator{ begin() }; }

        [[nodiscard]] constexpr JPL_INLINE reference        front() noexcept { JPL_ASSERT(mSize > 0); return mStorage[0]; }
        [[nodiscard]] constexpr JPL_INLINE reference        back() noexcept { JPL_ASSERT(mSize > 0); return mStorage[mSize - 1]; }

        [[nodiscard]] constexpr JPL_INLINE const_reference  front() const noexcept { JPL_ASSERT(mSize > 0); return mStorage[0]; }
        [[nodiscard]] constexpr JPL_INLINE const_reference  back() const noexcept { JPL_ASSERT(mSize > 0); return mStorage[mSize - 1]; }

        [[nodiscard]] constexpr JPL_INLINE pointer          data() noexcept { return mSize ? &mStorage[0] : nullptr; }
        [[nodiscard]] constexpr JPL_INLINE const_pointer    data() const noexcept { mSize ? &mStorage[0] : nullptr; }

        //==========================================================================
        [[nodiscard]] constexpr JPL_INLINE reference operator[](const size_type offset) noexcept
        {
            JPL_ASSERT(offset < mSize);
            return mStorage[offset];
        }

        [[nodiscard]] constexpr JPL_INLINE const_reference operator[](const size_type offset) const noexcept
        {
            JPL_ASSERT(offset < mSize);
            return mStorage[offset];
        }

        constexpr JPL_INLINE void resize(std::size_t newSize) noexcept
        {
            JPL_ASSERT(newSize <= Capacity);
            mSize = newSize;
        }

        constexpr JPL_INLINE void resize(std::size_t newSize, const T& valueToInsert) noexcept
        {
            JPL_ASSERT(newSize <= Capacity);
            if (newSize > mSize)
            {
                std::fill_n(&mStorage[mSize], newSize - mSize, valueToInsert);
            }
            mSize = newSize;
        }

        constexpr JPL_INLINE void push_back(const T& value)
        {
            JPL_ASSERT(mSize < Capacity);
            mStorage[mSize] = value;
        }

        template <class... Args>
        [[nodiscard]] constexpr JPL_INLINE reference emplace_back(Args&&... args)
        {
            JPL_ASSERT(mSize < Capacity);
            mStorage[mSize] = T{ std::forward<Args>(args)... };
            return mStorage[mSize];
        }

        constexpr JPL_INLINE bool pop_back(T& outValue) noexcept
        {
            if (mSize > 0)
            {
                mSize--;
                outValue = mStorage[mSize];
                return true;
            }
            else
            {
                return false;
            }
        }

        constexpr JPL_INLINE bool pop_back() noexcept
        {
            if (mSize > 0)
            {
                mSize--;
                return true;
            }
            else
            {
                return false;
            }
        }

    private:
        size_type mSize = 0;
        T mStorage[Capacity];
    };
} // namespace JPL
