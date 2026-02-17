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

#include <cstdint>
#include <span>
#include <concepts>

namespace JPL
{
    // Simple utility helper to combine hash
    struct Hash
    {
        static constexpr uint32_t cStartSeed = 0x811c9dc5u;

        JPL_INLINE constexpr uint32_t Combine(uint32_t value);
        [[nodiscard]] JPL_INLINE constexpr uint32_t Finalize() const;

    protected:
        uint32_t mSeed = cStartSeed;
    };

    // Start uint32_t seed = 0x811C9DC5u (Hash::cStartSeed)
    JPL_INLINE constexpr void HashCombine32(uint32_t& seed, uint32_t id32)
    {
        seed ^= id32 + 0x9e3779b9u + (seed << 6) + (seed >> 2);
     
        // FNV-1a prime - extra diffusion
        // (optional, if observing collisions)
        //seed *= 16777619u;
    }

    // Start uint32_t seed = 0x811C9DC5u (Hash::cStartSeed)
    JPL_INLINE constexpr void HashCombine32(uint32_t& seed, uint64_t id64)
    {
        // fold 64 -> 32  (XOR upper & lower halves)
        const uint32_t v = uint32_t(id64) ^ uint32_t(id64 >> 32);
        HashCombine32(seed, v);
    }

    [[nodiscard]] JPL_INLINE constexpr uint32_t Murmur3Finalize(uint32_t hash)
    {
        hash ^= hash >> 13;
        hash *= 0x85ebca6bu;
        hash ^= hash >> 16;
        return hash;
    }

    // 32-bit hash of an ordered sequence of 64-bit IDs
    template<std::integral T> requires (sizeof(T) == 8)
    [[nodiscard]] constexpr inline uint32_t HashIntSequence(std::span<const T> ids)
    {
        uint32_t h = Hash::cStartSeed; // FNV offset basis

        for (T id : ids)
        {
            // fold 64 -> 32  (XOR upper & lower halves)
            uint32_t v = uint32_t(id) ^ uint32_t(id >> 32);

            // (good enough for a few hundreds of elements,
            // switch to HasCombine32 if not enough)
            h ^= v;         // FNV-1a step
            h *= 16777619u; // 32-bit prime
        }

        return Murmur3Finalize(h);
    }

    // 32-bit hash of an ordered sequence of 32-bit IDs
    template<std::integral T> requires (sizeof(T) == 4)
    [[nodiscard]] constexpr inline uint32_t HashIntSequence32(std::span<const T> ids)
    {
        uint32_t h = Hash::cStartSeed;
        for (T v : ids)
            HashCombine32(h, static_cast<uint32_t>(v));

        return Murmur3Finalize(h);
    }
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
    JPL_INLINE constexpr uint32_t Hash::Combine(uint32_t value) { HashCombine32(mSeed, value); return mSeed; }
    [[nodiscard]] JPL_INLINE constexpr uint32_t Hash::Finalize() const { return Murmur3Finalize(mSeed); }
} // namespace JPL
