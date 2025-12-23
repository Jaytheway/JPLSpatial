//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2025 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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

#include <bit>
#include <limits>

#if defined(JPL_USE_SSE)
#include <immintrin.h>
#include <emmintrin.h>
#include <cstring> // std::memcpy
#if defined (JPL_USE_SSE4_1)
#include <smmintrin.h>
#endif
#elif defined(JPL_USE_NEON)
#ifdef JPL_COMPILER_MSVC
#include <intrin.h>
#include <arm64_neon.h>
#else
#include <arm_neon.h>
#endif
#else
#include <algorithm>
#include <cmath> 
#endif

#include <type_traits>
#include <ostream>
#include <span>

namespace JPL
{
	// Forward declaration
	struct simd_mask;

	//==========================================================================
	/// Minimal 4-wide 32-bit float vector implementation for SIMD
	struct [[nodiscard]] alignas(JPL_VECTOR_ALIGNMENT) simd
	{
		// Underlying native vector type
#if defined(JPL_USE_SSE)
		using Type = __m128;
#elif defined(JPL_USE_NEON)
		using Type = float32x4_t;
#else
		using Type = std::array<float, 4>;
#endif

		simd() noexcept = default;
		simd(const simd&) noexcept = default;
		simd& operator=(const simd&)  noexcept = default;
		JPL_INLINE simd(Type value)  noexcept : mNative(value) {}
		JPL_INLINE simd(float value) noexcept;
		JPL_INLINE simd(float v0, float v1, float v2, float v3) noexcept;
		JPL_INLINE simd(const float* mem);

		/// Vector with all zeros
		static JPL_INLINE simd zero() noexcept;
		
		/// Vector with all NaN's
		static JPL_INLINE simd nan() noexcept;

		static JPL_INLINE simd inf() noexcept;

		/// Frequently used constants
		static const simd c_0;
		static const simd c_1;
		static const simd c_0p5;

		/// Get number of element of the vector
		static constexpr std::size_t size() noexcept { return 4; }

		/// Load values from memory into simd, 'mem' should point to memory with at least size() number of floats 
		JPL_INLINE void load(const float* mem);

		/// Store values from simd to provided memory location
		JPL_INLINE void store(float* mem) const;

		/// Get float component by index
		JPL_INLINE float operator [] (uint32 index) const noexcept;

		/// Get float component by index known at compile-time
		template<uint32 LaneIndex> requires (LaneIndex < 4)
		JPL_INLINE float get_lane() const noexcept;

		/// Multiply two float vectors (component wise)
		JPL_INLINE simd	operator * (const simd& other) const noexcept;

		/// Multiply vector with float
		JPL_INLINE simd	operator * (float value) const noexcept;

		/// Multiply vector with float
		friend JPL_INLINE simd operator * (float value, const simd& other) noexcept;

		/// Divide vector by float
		JPL_INLINE simd	operator / (float value) const noexcept;

		/// Multiply vector with float
		JPL_INLINE simd& operator *= (float value) noexcept;

		/// Multiply vector with vector
		JPL_INLINE simd& operator *= (const simd& other) noexcept;

		/// Divide vector by float
		JPL_INLINE simd& operator /= (float value) noexcept;

		/// Add two float vectors (component wise)
		JPL_INLINE simd	operator + (const simd& other) const noexcept;

		/// Add two float vectors (component wise)
		JPL_INLINE simd& operator += (const simd& other) noexcept;

		/// Negate
		JPL_INLINE simd	operator - () const noexcept;

		/// Subtract two float vectors (component wise)
		JPL_INLINE simd	operator - (const simd& other) const noexcept;

		/// Subtract two float vectors (component wise)
		JPL_INLINE simd& operator -= (const simd& other) noexcept;

		/// Divide (component wise)
		JPL_INLINE simd	operator / (const simd& other) const noexcept;

		/// Component-wise logical OR
		JPL_INLINE void operator |= (const simd& other) noexcept;

		/// Component-wise logical XOR
		JPL_INLINE void operator ^= (const simd& other) noexcept;

		/// Component-wise logical AND
		JPL_INLINE void operator &= (const simd& other) noexcept;

		/// Returns sum of all components
		JPL_INLINE float reduce() const noexcept;

		/// Returns max component
		JPL_INLINE float reduce_max() const noexcept;

		/// Returns min component
		JPL_INLINE float reduce_min() const noexcept;

		JPL_INLINE /*explicit*/ operator Type() const noexcept { return mNative; }

		/// Convert each component from a float to an int
		JPL_INLINE simd_mask to_mask() const noexcept;

		/// Reinterpret simd as a simd_mask (doesn't change the bits)
		JPL_INLINE simd_mask as_mask() const noexcept;

		/// Component-wise select, returns 'a' if mask is true, 'b' otherwise
		static JPL_INLINE simd select(const simd_mask& mask, const simd& a, const simd& b) noexcept;

		/// Reciprocal vector (1 / value) for each of the components
		static JPL_INLINE simd reciprocal(const simd& vec) noexcept;

		inline friend std::ostream& operator << (std::ostream& inStream, const simd& vec)
		{
			float data[4];
			vec.store(data);
			inStream << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3];
			return inStream;
		}

		Type mNative;
	};
	static_assert(std::is_trivial<simd>(), "simd supposed to be a trivial type.");

	//==========================================================================
	struct [[nodiscard]] alignas(JPL_VECTOR_ALIGNMENT) simd_mask
	{
		// Underlying vector type
#if defined(JPL_USE_SSE)
		using Type = __m128i;
#elif defined(JPL_USE_NEON)
		using Type = uint32x4_t;
#else
		using Type = std::array<uint32,4>;
#endif

		simd_mask() noexcept = default;
		simd_mask(const simd_mask&) noexcept = default;
		simd_mask& operator=(const simd_mask&) noexcept = default;
		JPL_INLINE simd_mask(Type value) noexcept : mNative(value) {}
		JPL_INLINE simd_mask(uint32 value) noexcept;
		JPL_INLINE simd_mask(uint32 v0, uint32 v1, uint32 v2, uint32 v3) noexcept;
		JPL_INLINE simd_mask(const uint32* mem);

		static JPL_INLINE simd_mask replicate(int value)  noexcept;
		static JPL_INLINE simd_mask replicate(uint32 value)  noexcept;

		static JPL_INLINE simd_mask zero() noexcept;

		/// Get number of element of the vector
		static constexpr std::size_t size() noexcept { return 4; }

		/// Component set to this value is true
		inline static constexpr uint32 cTrueValue = 0xffffffffu;

		/// Load values from memory into simd, 'mem' should point to memory with at least size() number of uint32 
		JPL_INLINE void load(const uint32* mem);

		/// Store values from simd to provided memory location
		JPL_INLINE void store(uint32* mem) const;
		
		/// Test if all of the components are true
		JPL_INLINE bool all_of() const noexcept;

		/// Test if any of the components are true
		JPL_INLINE bool any_of() const noexcept;

		/// Test if none of the components are true
		JPL_INLINE bool none_of() const noexcept;

		/// Count the number of components that are true
		JPL_INLINE int reduce_count() const noexcept;

		/// Get index of the first component that is true
		JPL_INLINE int reduce_min_index() const noexcept;
		/// Get index of the last component that is true
		JPL_INLINE int reduce_max_index() const noexcept;

		/// Store if [0] is true in bit 0, [1] in bit 1, [2] in bit 2 and [3] in bit 3
		JPL_INLINE int GetTrues() const noexcept;

		/// Component-wise multiplies each of the 4 integer components with an integer (discards any overflow)
		JPL_INLINE simd_mask operator * (const simd_mask& other) const noexcept;

		/// Component-wise multiplies each of the 4 integer components with an integer (discards any overflow)
		JPL_INLINE simd_mask& operator *= (const simd_mask& other) noexcept;

		/// Component-wise add an integer value to all integer components (discards any overflow)
		JPL_INLINE simd_mask operator + (const simd_mask& other) const noexcept;

		/// Component-wise add two integer vectors (component wise)
		JPL_INLINE simd_mask& operator += (const simd_mask& other) noexcept;

		/// Component-wise subtract two simd_mask vectors
		JPL_INLINE simd_mask operator - (const simd_mask& other) const noexcept;

		/// Component-wise subtract two simd_mask vectors
		JPL_INLINE simd_mask& operator -= (const simd_mask& other) noexcept;

		/// Component-wise logical NOT
		JPL_INLINE simd_mask operator ~ () const noexcept;

		/// Component-wise logical OR
		JPL_INLINE void operator |= (const simd_mask& other) noexcept;

		/// Component-wise logical XOR
		JPL_INLINE void operator ^= (const simd_mask& other) noexcept;

		/// Component-wise logical AND
		JPL_INLINE void operator &= (const simd_mask& other) noexcept;

		JPL_INLINE simd_mask operator >> (const uint32 count) const noexcept;
		JPL_INLINE simd_mask operator << (const uint32 count) const noexcept;

		template<uint Count> requires(Count <= 31)
		JPL_INLINE simd_mask shr() const noexcept;

		template<uint Count> requires(Count <= 31)
		JPL_INLINE simd_mask shl() const noexcept;

		template<uint Count> requires(Count <= 31)
		JPL_INLINE simd_mask ashr() const noexcept;

		JPL_INLINE simd_mask adds(const simd_mask& other) const noexcept;
		JPL_INLINE simd_mask subs(const simd_mask& other) const noexcept;

		JPL_INLINE /*explicit*/ operator Type() const noexcept { return mNative; };

		/// Convert each component from an int to a float
		JPL_INLINE simd to_simd() const noexcept;

		/// Reinterpret simd_mask as a simd (doesn't change the bits)
		JPL_INLINE simd as_simd() const noexcept;

		inline friend std::ostream& operator << (std::ostream& inStream, const simd_mask& vec)
		{
			uint32 data[4];
			vec.store(data);
			inStream << data[0] << ", " << data[1] << ", " << data[2] << ", " << data[3];
			return inStream;
		}

		Type mNative;
	};
	static_assert(std::is_trivial<simd_mask>(), "simd_mask supposed to be a trivial type.");

	//==========================================================================
	/// Component-wise logical OR
	JPL_INLINE simd operator | (const simd& a, const simd& b) noexcept;
	/// Component-wise logical XOR
	JPL_INLINE simd operator ^ (const simd& a, const simd& b) noexcept;
	/// Component-wise logical AND
	JPL_INLINE simd operator & (const simd& a, const simd& b) noexcept;

	/// Component-wise comparison
	JPL_INLINE simd_mask operator == (const simd& a, const simd& b) noexcept;
	JPL_INLINE simd_mask operator != (const simd& a, const simd& b) noexcept { return ~(a == b); }

	JPL_INLINE simd_mask operator < (const simd& a, const simd& b) noexcept;
	JPL_INLINE simd_mask operator <= (const simd& a, const simd& b) noexcept;
	JPL_INLINE simd_mask operator > (const simd& a, const simd& b) noexcept;
	JPL_INLINE simd_mask operator >= (const simd& a, const simd& b) noexcept;

	/// Component-wise comparison
	JPL_INLINE simd_mask operator == (const simd_mask& a, const simd_mask& b) noexcept;
	JPL_INLINE simd_mask operator != (const simd_mask& a, const simd_mask& b)  noexcept { return ~(a == b); }

	/// Component-wise logical OR
	JPL_INLINE simd_mask operator | (const simd_mask& a, const simd_mask& b) noexcept;
	/// Component-wise logical XOR
	JPL_INLINE simd_mask operator ^ (const simd_mask& a, const simd_mask& b) noexcept;
	/// Component-wise logical AND
	JPL_INLINE simd_mask operator & (const simd_mask& a, const simd_mask& b) noexcept;

	//==========================================================================
	namespace Math
	{
		/// Component-wise square root
		JPL_INLINE simd Sqrt(const simd& vec) noexcept;

		/// Component-wise inverse square root (1 / sqrt(x))
		JPL_INLINE simd InvSqrt(const simd& vec) noexcept;
	}
	
	/// Element-wise max
	JPL_INLINE simd max(const simd& a, const simd& b) noexcept;

	/// Element-wise min
	JPL_INLINE simd min(const simd& a, const simd& b) noexcept;

	JPL_INLINE simd abs(const simd& vec) noexcept;

	/// Element-wise clamp
	JPL_INLINE simd clamp(const simd& value, const simd& minV, const simd& maxV) noexcept;

	/// Element-wise fused multiply-add
	JPL_INLINE simd fma(const simd& mul1, const simd& mul2, const simd& addV) noexcept;

	/// Element-wise floor
	JPL_INLINE simd floor(const simd& vec) noexcept;

	/// Element-wise round to nearest integer value
	JPL_INLINE simd round(const simd& vec) noexcept;

	/// Interleave two low lanes with the two high lanes (e.g. { 0, 1, 2, 3 } -> { 0, 2, 1, 3 })
	JPL_INLINE simd interleave_lohi(const simd& vec) noexcept;

	/// Combine two low lanes of input `a` and `b` as { a0, a1, b0, b1 }
	JPL_INLINE simd combine_lo(const simd& a, const simd& b) noexcept;
	
	/// Combine two high lanes of input `a` and `b` as { a2, a3, b2, b3 }
	JPL_INLINE simd combine_hi(const simd& a, const simd& b) noexcept;

	/// Combine two low lanes of `a` and two high lanes from `b` as { a0, a1, b2, b3 }
	JPL_INLINE simd combine_lohi(const simd& a, const simd& b) noexcept;

	/// Reverse the order of the lanes
	JPL_INLINE simd reverse(const simd& vec) noexcept;

	/// Element-wise max
	JPL_INLINE simd_mask max(const simd_mask& a, const simd_mask& b) noexcept;

	/// Element-wise min
	JPL_INLINE simd_mask min(const simd_mask& a, const simd_mask& b) noexcept;

	/// Element-wise clamp
	JPL_INLINE simd_mask clamp(const simd_mask& value, const simd_mask& minV, const simd_mask& maxV) noexcept;

	/// Element-wise max, signed
	JPL_INLINE simd_mask maxs(const simd_mask& a, const simd_mask& b) noexcept;
	
	/// Element-wise min, signed
	JPL_INLINE simd_mask mins(const simd_mask& a, const simd_mask& b) noexcept;

	/// Element-wise clamp, signed
	JPL_INLINE simd_mask clamps(const simd_mask& value, const simd_mask& minV, const simd_mask& maxV) noexcept;

} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL
{
	//==========================================================================
	inline const simd simd::c_0 = simd::zero();
	inline const simd simd::c_1 = 1.0f;
	inline const simd simd::c_0p5 = 0.5f;

	//==========================================================================
	JPL_INLINE simd::simd(float value) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_set1_ps(value);
#elif defined(JPL_USE_NEON)
		mNative = vdupq_n_f32(value);
#else
		mNative[0] = value;
		mNative[1] = value;
		mNative[2] = value;
		mNative[3] = value;
#endif
	}

	JPL_INLINE simd::simd(float v0, float v1, float v2, float v3) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_set_ps(v3, v2, v1, v0);
#elif defined(JPL_USE_NEON)
		mNative = vdupq_n_f32(0);
		mNative = vsetq_lane_f32(v0, mNative, 0);
		mNative = vsetq_lane_f32(v1, mNative, 1);
		mNative = vsetq_lane_f32(v2, mNative, 2);
		mNative = vsetq_lane_f32(v3, mNative, 3);
#else
		mNative[0] = v0;
		mNative[1] = v1;
		mNative[2] = v2;
		mNative[3] = v3;
#endif
	}

	JPL_INLINE simd::simd(const float* mem)
	{
		load(mem);
	}

	JPL_INLINE simd simd::zero() noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_setzero_ps();
#elif defined(JPL_USE_NEON)
		return vdupq_n_f32(0);
#else
		return simd(0, 0, 0, 0);
#endif
	}

	JPL_INLINE simd simd::nan() noexcept
	{
		return simd(std::numeric_limits<float>::quiet_NaN());
	}

	JPL_INLINE simd simd::inf() noexcept
	{
		return simd(std::numeric_limits<float>::infinity());
	}

	JPL_INLINE void simd::load(const float* mem)
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_loadu_ps(mem);
#elif defined(JPL_USE_NEON)
		mNative = vld1q_f32(mem);
#else
		mNative = { mem[0], mem[1], mem[2], mem[3] };
#endif
	}

	JPL_INLINE void simd::store(float* mem) const
	{
#if defined(JPL_USE_SSE)
		_mm_storeu_ps(mem, mNative);
#elif defined(JPL_USE_NEON)
		vst1q_f32(mem, mNative);
#else
		mem[0] = mNative[0];
		mem[1] = mNative[1];
		mem[2] = mNative[2];
		mem[3] = mNative[3];
#endif
	}

	JPL_INLINE float simd::operator[](uint32 index) const noexcept
	{
		JPL_ASSERT(index < 4);
#if defined(JPL_USE_SSE)
#if defined(JPL_USE_AVX)
		__m128i sel = _mm_set1_epi32(static_cast<int>(index));
		__m128  p = _mm_permutevar_ps(mNative, sel);
		return _mm_cvtss_f32(p);
#else
		alignas(16) float temp[4];
		_mm_store_ps(temp, mNative);
		return temp[index];
#endif
#elif defined(JPL_USE_NEON)
		alignas(16) float temp[4];
		vst1q_f32(temp, mNative);
		return temp[index];
#else
		return mNative[index];
#endif
	}

	template<uint32 LaneIndex> requires (LaneIndex < 4)
	JPL_INLINE float simd::get_lane() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_cvtss_f32(_mm_shuffle_ps(mNative, mNative, _MM_SHUFFLE(0, 0, 0, LaneIndex)));
#elif defined(JPL_USE_NEON)
		return vgetq_lane_f32(mNative, LaneIndex);
#else
		return mNative[LaneIndex];
#endif
	}

	JPL_INLINE simd simd::operator * (const simd& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_mul_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vmulq_f32(mNative, other.mNative);
#else
		return simd(
			mNative[0] * other.mNative[0],
			mNative[1] * other.mNative[1],
			mNative[2] * other.mNative[2],
			mNative[3] * other.mNative[3]
		);
#endif
	}

	JPL_INLINE simd simd::operator * (float value) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_mul_ps(mNative, _mm_set1_ps(value));
#elif defined(JPL_USE_NEON)
		return vmulq_n_f32(mNative, value);
#else
		return simd(mNative[0] * value, mNative[1] * value, mNative[2] * value, mNative[3] * value);
#endif
	}

	JPL_INLINE simd operator * (float value, const simd& vec) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_mul_ps(_mm_set1_ps(value), vec.mNative);
#elif defined(JPL_USE_NEON)
		return vmulq_n_f32(vec.mNative, value);
#else
		return simd(
			value * vec.mNative[0],
			value * vec.mNative[1],
			value * vec.mNative[2],
			value * vec.mNative[3]
		);
#endif
	}

	JPL_INLINE simd simd::operator / (float value) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_div_ps(mNative, _mm_set1_ps(value));
#elif defined(JPL_USE_NEON)
		return vdivq_f32(mNative, vdupq_n_f32(value));
#else
		return simd(
			mNative[0] / value,
			mNative[1] / value,
			mNative[2] / value,
			mNative[3] / value
		);
#endif
	}

	JPL_INLINE simd& simd::operator *= (float value) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_mul_ps(mNative, _mm_set1_ps(value));
#elif defined(JPL_USE_NEON)
		mNative = vmulq_n_f32(mNative, value);
#else
		for (int i = 0; i < 4; ++i)
			mNative[i] *= value;
#endif
		return *this;
	}

	JPL_INLINE simd& simd::operator *= (const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_mul_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vmulq_f32(mNative, other.mNative);
#else
		for (int i = 0; i < 4; ++i)
			mNative[i] *= other.mNative[i];
#endif
		return *this;
	}

	JPL_INLINE simd& simd::operator /= (float value) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_div_ps(mNative, _mm_set1_ps(value));
#elif defined(JPL_USE_NEON)
		mNative = vdivq_f32(mNative, vdupq_n_f32(value));
#else
		for (int i = 0; i < 4; ++i)
			mNative[i] /= value;
#endif
		return *this;
	}

	JPL_INLINE simd simd::operator + (const simd& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_add_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vaddq_f32(mNative, other.mNative);
#else
		return simd(
			mNative[0] + other.mNative[0],
			mNative[1] + other.mNative[1],
			mNative[2] + other.mNative[2],
			mNative[3] + other.mNative[3]
		);
#endif
	}

	JPL_INLINE simd& simd::operator += (const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_add_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vaddq_f32(mNative, other.mNative);
#else
		for (int i = 0; i < 4; ++i)
			mNative[i] += other.mNative[i];
#endif
		return *this;
	}

	JPL_INLINE simd simd::operator - () const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_sub_ps(_mm_setzero_ps(), mNative);
#elif defined(JPL_USE_NEON)
#if 1 // ifdef JPL_CROSS_PLATFORM_DETERMINISTIC
		return vsubq_f32(vdupq_n_f32(0), mNative);
#else
		return vnegq_f32(mNative);
#endif
#else
#if 1 // ifdef JPL_CROSS_PLATFORM_DETERMINISTIC
		return simd(
			0.0f - mNative[0],
			0.0f - mNative[1],
			0.0f - mNative[2],
			0.0f - mNative[3]
		);
#else
		return simd(-mNative[0], -mNative[1], -mNative[2], -mNative[3]);
#endif
#endif
	}

	JPL_INLINE simd simd::operator - (const simd& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_sub_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vsubq_f32(mNative, other.mNative);
#else
		return simd(
			mNative[0] - other.mNative[0],
			mNative[1] - other.mNative[1],
			mNative[2] - other.mNative[2],
			mNative[3] - other.mNative[3]
		);
#endif
	}

	JPL_INLINE simd& simd::operator -= (const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_sub_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vsubq_f32(mNative, other.mNative);
#else
		for (int i = 0; i < 4; ++i)
			mNative[i] -= other.mNative[i];
#endif
		return *this;
	}

	JPL_INLINE simd simd::operator / (const simd& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_div_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vdivq_f32(mNative, other.mNative);
#else
		return simd(
			mNative[0] / other.mNative[0],
			mNative[1] / other.mNative[1],
			mNative[2] / other.mNative[2],
			mNative[3] / other.mNative[3]
		);
#endif
	}

	JPL_INLINE void simd::operator|=(const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_or_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vreinterpretq_f32_u32(
			vorrq_u32(
				vreinterpretq_u32_f32(mNative),
				vreinterpretq_u32_f32(other.mNative)
			)
		);
#else
		*this = (as_mask() | other.as_mask()).as_simd();
#endif
	}

	JPL_INLINE void simd::operator^=(const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_xor_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vreinterpretq_f32_u32(
			veorq_u32(
				vreinterpretq_u32_f32(mNative),
				vreinterpretq_u32_f32(other.mNative)
			)
		);
#else
		*this = (as_mask() ^ other.as_mask()).as_simd();
#endif
	}

	JPL_INLINE void simd::operator&=(const simd& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_and_ps(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vreinterpretq_f32_u32(
			vandq_u32(
				vreinterpretq_u32_f32(mNative),
				vreinterpretq_u32_f32(other.mNative)
			)
		);
#else
		*this = (as_mask() & other.as_mask()).as_simd();
#endif
	}

	JPL_INLINE simd operator|(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_or_ps(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_f32_u32(
			vorrq_u32(
				vreinterpretq_u32_f32(a.mNative),
				vreinterpretq_u32_f32(b.mNative)
			)
		);
#else
		return (a.as_mask() | b.as_mask()).as_simd();
#endif
	}

	JPL_INLINE simd operator^(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_xor_ps(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_f32_u32(
			veorq_u32(
				vreinterpretq_u32_f32(a.mNative),
				vreinterpretq_u32_f32(b.mNative)
			)
		);
#else
		return (a.as_mask() ^ b.as_mask()).as_simd();
#endif
	}

	JPL_INLINE simd operator&(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_and_ps(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_f32_u32(
			vandq_u32(
				vreinterpretq_u32_f32(a.mNative),
				vreinterpretq_u32_f32(b.mNative)
			)
		);
#else
		return (a.as_mask() & b.as_mask()).as_simd();
#endif
	}

	JPL_INLINE float simd::reduce() const noexcept
	{
#if defined(JPL_USE_SSE)
		Type shuf = _mm_shuffle_ps(mNative, mNative, _MM_SHUFFLE(2, 3, 0, 1));
		Type sums = _mm_add_ps(mNative, shuf);
		shuf = _mm_movehl_ps(shuf, sums);
		sums = _mm_add_ss(sums, shuf);
		return _mm_cvtss_f32(sums);
#elif defined (JPL_USE_NEON)
		return vaddvq_f32(mNative); // AArch64
#else
		return mNative[0] + mNative[1] + mNative[2] + mNative[3];
#endif
	}

	JPL_INLINE float simd::reduce_max() const noexcept
	{
#if defined(JPL_USE_SSE)
		Type shuf1 = _mm_shuffle_ps(mNative, mNative, _MM_SHUFFLE(0, 1, 2, 3));
		Type max1 = _mm_max_ps(mNative, shuf1);
		Type shuf2 = _mm_shuffle_ps(max1, max1, _MM_SHUFFLE(1, 0, 3, 2));
		max1 = _mm_max_ps(max1, shuf2);
		return _mm_cvtss_f32(max1);
#elif defined (JPL_USE_NEON)
		float32x2_t v_pair = vpmax_f32(vget_low_f32(mNative), vget_high_f32(mNative));
		float32x2_t max_pair = vpmax_f32(v_pair, v_pair);
		return vget_lane_f32(max_pair, 0);
#else
		return std::max({ mNative[0], mNative[1], mNative[2], mNative[3] });
#endif
	}

	JPL_INLINE float simd::reduce_min() const noexcept
	{
#if defined(JPL_USE_SSE)
		Type shuf1 = _mm_shuffle_ps(mNative, mNative, _MM_SHUFFLE(0, 1, 2, 3));
		Type min1 = _mm_min_ps(mNative, shuf1);
		Type shuf2 = _mm_shuffle_ps(min1, min1, _MM_SHUFFLE(1, 0, 3, 2));
		min1 = _mm_min_ps(min1, shuf2);
		return _mm_cvtss_f32(min1);
#elif defined (JPL_USE_NEON)
		float32x2_t v_pair = vpmin_f32(vget_low_f32(mNative), vget_high_f32(mNative));
		float32x2_t min_pair = vpmin_f32(v_pair, v_pair);
		return vget_lane_f32(min_pair, 0);
#else
		return std::min({ mNative[0], mNative[1], mNative[2], mNative[3] });
#endif
	}

	JPL_INLINE simd_mask simd::to_mask() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_cvttps_epi32(mNative);
#elif defined(JPL_USE_NEON)
		// Fixup from https://github.com/DLTcollab/sse2neon (MIT)
		auto _sse2neon_cvtps_epi32_fixup = [](float32x4_t f, int32x4_t cvt)
		{
			/* Detect values >= 2147483648.0f (out of INT32 range) */
			float32x4_t max_f = vdupq_n_f32(2147483648.0f);
			uint32x4_t overflow = vcgeq_f32(f, max_f);

			/* Detect NaN: x != x for NaN values */
			uint32x4_t is_nan = vmvnq_u32(vceqq_f32(f, f));

			/* Combine: any overflow or NaN should produce INT32_MIN */
			uint32x4_t need_indefinite = vorrq_u32(overflow, is_nan);

			/* Blend: select INT32_MIN where needed */
			int32x4_t indefinite = vdupq_n_s32(INT32_MIN);
			return vbslq_s32(need_indefinite, indefinite, cvt);
		};

		int32x4_t cvt = vcvtq_s32_f32(mNative);
		return vreinterpretq_u32_s32(_sse2neon_cvtps_epi32_fixup(mNative, cvt));
		//return vcvtq_u32_f32(mNative);
#else
		return {
			uint32(mNative[0]),
			uint32(mNative[1]),
			uint32(mNative[2]),
			uint32(mNative[3])
		};
#endif
	}

	JPL_INLINE simd_mask simd::as_mask() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_f32(mNative);
#else
		return std::bit_cast<simd_mask>(*this);
#endif
	}

	JPL_INLINE simd simd::select(const simd_mask& mask, const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE4_1)
		return _mm_blendv_ps(b.mNative, a.mNative, _mm_castsi128_ps(mask.mNative));
#elif defined(JPL_USE_SSE)
		Type mf = _mm_castsi128_ps(mask.mNative);
		Type t0 = _mm_andnot_ps(mf, b.mNative);	// (~m) & a
		Type t1 = _mm_and_ps(mf, a.mNative);	// m & b
		return _mm_or_ps(t0, t1);
#elif defined(JPL_USE_NEON)
		return vbslq_f32(
			vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(mask.mNative), 31)),
			a.mNative,
			b.mNative);
#else
		return {
			mask.mNative[0] ? a.mNative[0] : b.mNative[0],
			mask.mNative[1] ? a.mNative[1] : b.mNative[1],
			mask.mNative[2] ? a.mNative[2] : b.mNative[2],
			mask.mNative[3] ? a.mNative[3] : b.mNative[3]
		};
#endif
	}

	JPL_INLINE simd simd::reciprocal(const simd& vec) noexcept
	{
		return simd(1.0f) / vec;
	}

	//==========================================================================
	JPL_INLINE simd_mask::simd_mask(uint32 value) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_set1_epi32(static_cast<int>(value));
#elif defined(JPL_USE_NEON)
		mNative = vdupq_n_u32(value);
#else
		mNative[0] = value;
		mNative[1] = value;
		mNative[2] = value;
		mNative[3] = value;
#endif
	}

	JPL_INLINE simd_mask::simd_mask(uint32 v0, uint32 v1, uint32 v2, uint32 v3) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_set_epi32(
			static_cast<int>(v3),
			static_cast<int>(v2),
			static_cast<int>(v1),
			static_cast<int>(v0)
		);
#elif defined(JPL_USE_NEON)
		mNative = vdupq_n_u32(0);
		mNative = vsetq_lane_u32(v0, mNative, 0);
		mNative = vsetq_lane_u32(v1, mNative, 1);
		mNative = vsetq_lane_u32(v2, mNative, 2);
		mNative = vsetq_lane_u32(v3, mNative, 3);
#else
		mNative[0] = v0;
		mNative[1] = v1;
		mNative[2] = v2;
		mNative[3] = v3;
#endif
	}

	JPL_INLINE simd_mask::simd_mask(const uint32* mem)
	{
		load(mem);
	}

	JPL_INLINE simd_mask simd_mask::replicate(int value) noexcept
	{
#if defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vdupq_n_s32(value));
#else
		// for SSE/AVX it's the same as in constructor
		return simd_mask(static_cast<uint32>(value));
#endif
	}

	JPL_INLINE simd_mask simd_mask::replicate(uint32 value) noexcept
	{
		return simd_mask(value);
	}

	JPL_INLINE simd_mask simd_mask::zero() noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_setzero_si128();
#elif defined(JPL_USE_NEON)
		return vdupq_n_u32(0);
#else
		return simd_mask(0, 0, 0, 0);
#endif
	}

	JPL_INLINE void simd_mask::load(const uint32* mem)
	{
#if defined(JPL_USE_SSE)
		std::memcpy(&mNative, mem, sizeof(mNative));
#elif defined(JPL_USE_NEON)
		mNative = vld1q_u32(mem);
#else
		mNative = { mem[0], mem[1], mem[2], mem[3] };
#endif
	}

	JPL_INLINE void simd_mask::store(uint32* mem) const
	{
#if defined(JPL_USE_SSE)
		std::memcpy(mem, &mNative, sizeof(mNative));
#elif defined(JPL_USE_NEON)
		vst1q_u32(mem, mNative);
#else
		mem[0] = mNative[0];
		mem[1] = mNative[1];
		mem[2] = mNative[2];
		mem[3] = mNative[3];
#endif
	}

	JPL_INLINE bool simd_mask::all_of() const noexcept
	{
		return GetTrues() == 0b1111;
	}

	JPL_INLINE bool simd_mask::any_of() const noexcept
	{
		return GetTrues() != 0;
	}

	JPL_INLINE bool simd_mask::none_of() const noexcept
	{
		return GetTrues() == 0;
	}

	JPL_INLINE int simd_mask::reduce_count() const noexcept
	{
#if defined(JPL_USE_SSE)
		return std::popcount(static_cast<uint32>(_mm_movemask_ps(_mm_castsi128_ps(mNative))));
#elif defined(JPL_USE_NEON)
		return vaddvq_u32(vshrq_n_u32(mNative, 31));
#else
		return (mNative[0] >> 31) + (mNative[1] >> 31) + (mNative[2] >> 31) + (mNative[3] >> 31);
#endif
	}

	JPL_INLINE int simd_mask::reduce_min_index() const noexcept
	{
		uint32 m = static_cast<uint32>(GetTrues());
		// std::countr_zero(0) == number of bits (32), so guard m==0 once.
		return m ? static_cast<int>(std::countr_zero(m)) : -1;
	}

	JPL_INLINE int simd_mask::reduce_max_index() const noexcept
	{
		uint32 m = static_cast<uint32>(GetTrues());
		// For m==0, std::countl_zero(m) == 32 -> 31-32 == -1
		return 31 - static_cast<int>(std::countl_zero(m));
	}

	JPL_INLINE int simd_mask::GetTrues() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_movemask_ps(_mm_castsi128_ps(mNative));
#elif defined(JPL_USE_NEON)
		static_assert(JPL_CPU_ADDRESS_BITS == 64 && "32-bit arm doesn't support this.");
		uint32x4_t bits = vshrq_n_u32(mNative, 31);
		const uint32x4_t w = { 1u, 2u, 4u, 8u };
		return static_cast<int>(vaddvq_u32(vmulq_u32(bits, w)));
#else
		return (mNative[0] >> 31)
			| ((mNative[1] >> 31) << 1)
			| ((mNative[2] >> 31) << 2)
			| ((mNative[3] >> 31) << 3);
#endif
	}

	JPL_INLINE simd_mask simd_mask::operator*(const simd_mask& other) const noexcept
	{
#if defined(JPL_USE_SSE4_1)
		return _mm_mullo_epi32(mNative, other.mNative);
#elif defined(JPL_USE_SSE)
		Type tmp1 = _mm_mul_epu32(mNative, other.mNative);
		Type tmp2 = _mm_mul_epu32(_mm_srli_si128(mNative, 4), _mm_srli_si128(other.mNative, 4));
		return _mm_unpacklo_epi32(
			_mm_shuffle_epi32(tmp1, _MM_SHUFFLE(0, 0, 2, 0)),
			_mm_shuffle_epi32(tmp2, _MM_SHUFFLE(0, 0, 2, 0))
		);
#elif defined(JPL_USE_NEON)
		return vmulq_u32(mNative, other.mNative);
#else
		return {
			mNative[0] * other.mNative[0],
			mNative[1] * other.mNative[1],
			mNative[2] * other.mNative[2],
			mNative[3] * other.mNative[3]
		};
#endif
	}

	JPL_INLINE simd_mask& simd_mask::operator*=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE4_1)
		mNative = _mm_mullo_epi32(mNative, other.mNative);
#elif defined(JPL_USE_SSE)
		Type tmp1 = _mm_mul_epu32(mNative, other.mNative);
		Type tmp2 = _mm_mul_epu32(_mm_srli_si128(mNative, 4), _mm_srli_si128(other.mNative, 4));
		mNative = _mm_unpacklo_epi32(
			_mm_shuffle_epi32(tmp1, _MM_SHUFFLE(0, 0, 2, 0)),
			_mm_shuffle_epi32(tmp2, _MM_SHUFFLE(0, 0, 2, 0))
		);
#elif defined(JPL_USE_NEON)
		mNative = vmulq_u32(mNative, other.mNative);
#else
		mNative[0] *= other.mNative[0];
		mNative[1] *= other.mNative[1];
		mNative[2] *= other.mNative[2];
		mNative[3] *= other.mNative[3];
#endif
		return *this;
	}

	JPL_INLINE simd_mask simd_mask::operator+(const simd_mask& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_add_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vaddq_u32(mNative, other.mNative);
#else
		return {
			mNative[0] + other.mNative[0],
			mNative[1] + other.mNative[1],
			mNative[2] + other.mNative[2],
			mNative[3] + other.mNative[3]
		};
#endif
	}

	JPL_INLINE simd_mask& simd_mask::operator+=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_add_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vaddq_u32(mNative, other.mNative);
#else
		mNative[0] += other.mNative[0];
		mNative[1] += other.mNative[1];
		mNative[2] += other.mNative[2];
		mNative[3] += other.mNative[3];
#endif
		return *this;
	}

	JPL_INLINE simd_mask simd_mask::operator-(const simd_mask& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_sub_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vsubq_u32(mNative, other.mNative);
#else
		return simd_mask(
			mNative[0] - other.mNative[0],
			mNative[1] - other.mNative[1],
			mNative[2] - other.mNative[2],
			mNative[3] - other.mNative[3]
		);
#endif
	}

	JPL_INLINE simd_mask& simd_mask::operator-=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE)

		mNative = _mm_sub_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vsubq_u32(mNative, other.mNative);
#else
		mNative[0] -= other.mNative[0];
		mNative[1] -= other.mNative[1];
		mNative[2] -= other.mNative[2];
		mNative[3] -= other.mNative[3];
#endif
		return *this;
	}

	JPL_INLINE simd_mask operator==(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(_mm_cmpeq_ps(a.mNative, b.mNative));
#elif defined(JPL_USE_NEON)
		return vceqq_f32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] == b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] == b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] == b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] == b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask operator<(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(_mm_cmplt_ps(a.mNative, b.mNative));
#elif defined(JPL_USE_NEON)
		return vcltq_f32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] < b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] < b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] < b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] < b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask operator<=(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(_mm_cmple_ps(a.mNative, b.mNative));
#elif defined(JPL_USE_NEON)
		return vcleq_f32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] <= b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] <= b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] <= b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] <= b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask operator>(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(_mm_cmpgt_ps(a.mNative, b.mNative));
#elif defined(JPL_USE_NEON)
		return vcgtq_f32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] > b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] > b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] > b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] > b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask operator>=(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castps_si128(_mm_cmpge_ps(a.mNative, b.mNative));
#elif defined(JPL_USE_NEON)
		return vcgeq_f32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] >= b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] >= b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] >= b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] >= b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask operator==(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_cmpeq_epi32(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vceqq_u32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] == b.mNative[0] ? cTrueValue : 0,
			a.mNative[1] == b.mNative[1] ? cTrueValue : 0,
			a.mNative[2] == b.mNative[2] ? cTrueValue : 0,
			a.mNative[3] == b.mNative[3] ? cTrueValue : 0
		};
#endif
	}

	JPL_INLINE simd_mask simd_mask::operator ~ () const noexcept
	{
#if defined(JPL_USE_AVX512)
		return _mm_ternarylogic_epi32(mNative, mNative, mNative, 0b01010101);
#elif defined(JPL_USE_SSE)
		return *this ^ simd_mask(0xffffffff);
#elif defined(JPL_USE_NEON)
		return vmvnq_u32(mNative);
#else
		return { ~mNative[0], ~mNative[1], ~mNative[2], ~mNative[3] };
#endif
	}

	JPL_INLINE void simd_mask::operator|=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_or_si128(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vorrq_u32(mNative, other.mNative);
#else
		mNative[0] |= other.mNative[0];
		mNative[1] |= other.mNative[1];
		mNative[2] |= other.mNative[2];
		mNative[3] |= other.mNative[3];
#endif
	}

	JPL_INLINE void  simd_mask::operator^=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_xor_si128(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = veorq_u32(mNative, other.mNative);
#else
		mNative[0] ^= other.mNative[0];
		mNative[1] ^= other.mNative[1];
		mNative[2] ^= other.mNative[2];
		mNative[3] ^= other.mNative[3];
#endif
	}

	JPL_INLINE void  simd_mask::operator&=(const simd_mask& other) noexcept
	{
#if defined(JPL_USE_SSE)
		mNative = _mm_and_si128(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		mNative = vandq_u32(mNative, other.mNative);
#else
		mNative[0] &= other.mNative[0];
		mNative[1] &= other.mNative[1];
		mNative[2] &= other.mNative[2];
		mNative[3] &= other.mNative[3];
#endif
	}

	JPL_INLINE simd_mask simd_mask::operator>>(const uint32 count) const noexcept
	{
		JPL_ASSERT(count <= 31);
#if defined(JPL_USE_SSE)
		__m128i c = _mm_cvtsi32_si128(static_cast<int>(count));
		return _mm_srl_epi32(mNative, c);
#elif defined(JPL_USE_NEON)
		int32x4_t sh = vdupq_n_s32(-static_cast<int32_t>(count));
		return vshlq_u32(mNative, sh);
#else
		return simd_mask(
			mNative[0] >> count,
			mNative[1] >> count,
			mNative[2] >> count,
			mNative[3] >> count
		);
#endif
	}

	JPL_INLINE simd_mask simd_mask::operator<<(const uint32 count) const noexcept
	{
		JPL_ASSERT(count <= 31);
#if defined(JPL_USE_SSE)
		__m128i c = _mm_cvtsi32_si128(static_cast<int>(count));
		return _mm_sll_epi32(mNative, c);
#elif defined(JPL_USE_NEON)
		int32x4_t sh = vdupq_n_s32(static_cast<int32_t>(count));
		return vshlq_u32(mNative, sh);
#else
		return simd_mask(
			mNative[0] << count,
			mNative[1] << count,
			mNative[2] << count,
			mNative[3] << count
		);
#endif
	}

	template<uint Count> requires(Count <= 31)
	JPL_INLINE simd_mask simd_mask::shr() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_srli_epi32(mNative, Count);
#elif defined(JPL_USE_NEON)
		return vshrq_n_u32(mNative, Count);
#else
		return simd_mask(
			mNative[0] >> Count,
			mNative[1] >> Count,
			mNative[2] >> Count,
			mNative[3] >> Count
		);
#endif
	}

	template<uint Count> requires(Count <= 31)
	JPL_INLINE simd_mask simd_mask::shl() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_slli_epi32(mNative, Count);
#elif defined(JPL_USE_NEON)
		return vshlq_n_u32(mNative, Count);
#else
		return simd_mask(
			mNative[0] << Count,
			mNative[1] << Count,
			mNative[2] << Count,
			mNative[3] << Count
		);
#endif
	}

	template<uint Count> requires(Count <= 31)
	JPL_INLINE simd_mask simd_mask::ashr() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_srai_epi32(mNative, Count);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vshrq_n_s32(vreinterpretq_s32_u32(mNative), Count));
#else
		return {
			static_cast<uint32>(static_cast<int32>(mNative[0]) >> Count),
			static_cast<uint32>(static_cast<int32>(mNative[1]) >> Count),
			static_cast<uint32>(static_cast<int32>(mNative[2]) >> Count),
			static_cast<uint32>(static_cast<int32>(mNative[3]) >> Count)
		};
#endif
	}

	JPL_INLINE simd_mask simd_mask::adds(const simd_mask& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_add_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vaddq_s32(
			vreinterpretq_s32_u32(mNative),
			vreinterpretq_s32_u32(other.mNative)));
#else
		return simd_mask(mNative[0] + other.mNative[0],
						 mNative[1] + other.mNative[1],
						 mNative[2] + other.mNative[2],
						 mNative[3] + other.mNative[3]);
#endif
	}

	JPL_INLINE simd_mask simd_mask::subs(const simd_mask& other) const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_sub_epi32(mNative, other.mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vsubq_s32(
			vreinterpretq_s32_u32(mNative),
			vreinterpretq_s32_u32(other.mNative)));
#else
		return simd_mask(mNative[0] - other.mNative[0],
						 mNative[1] - other.mNative[1],
						 mNative[2] - other.mNative[2],
						 mNative[3] - other.mNative[3]);
#endif
	}

	JPL_INLINE simd simd_mask::to_simd() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_cvtepi32_ps(mNative);
#elif defined(JPL_USE_NEON)
		return vcvtq_f32_s32(vreinterpretq_s32_u32(mNative));
		//return vcvtq_f32_u32(mNative);
#else
		return {
			static_cast<float>(mU32[0]),
			static_cast<float>(mU32[1]),
			static_cast<float>(mU32[2]),
			static_cast<float>(mU32[3])
		};
#endif
	}

	JPL_INLINE simd simd_mask::as_simd() const noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castsi128_ps(mNative);
#elif defined(JPL_USE_NEON)
		return vreinterpretq_f32_u32(mNative);
#else
		return std::bit_cast<simd>(*this);
#endif
	}

	JPL_INLINE simd_mask operator|(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_or_si128(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vorrq_u32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] | b.mNative[0],
			a.mNative[1] | b.mNative[1],
			a.mNative[2] | b.mNative[2],
			a.mNative[3] | b.mNative[3]
		};
#endif
	}

	JPL_INLINE simd_mask operator^(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_xor_si128(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return veorq_u32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] ^ b.mNative[0],
			a.mNative[1] ^ b.mNative[1],
			a.mNative[2] ^ b.mNative[2],
			a.mNative[3] ^ b.mNative[3]
		};
#endif
	}

	JPL_INLINE simd_mask operator&(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_and_si128(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vandq_u32(a.mNative, b.mNative);
#else
		return {
			a.mNative[0] & b.mNative[0],
			a.mNative[1] & b.mNative[1],
			a.mNative[2] & b.mNative[2],
			a.mNative[3] & b.mNative[3]
		};
#endif
	}

	//==========================================================================
	namespace Math
	{
		JPL_INLINE simd Sqrt(const simd& vec) noexcept
		{
#if defined(JPL_USE_SSE)
			return _mm_sqrt_ps(vec.mNative);
#elif defined(JPL_USE_NEON)
			return vsqrtq_f32(vec.mNative);
#else
			return simd(std::sqrt(mNative[0]), std::sqrt(mNative[1]), std::sqrt(mNative[2]), std::sqrt(mNative[3]));
#endif
		}

		JPL_INLINE simd InvSqrt(const simd& vec) noexcept
		{
			// Number of Newton's refinements.
			// 0: estimate, 1: ~16b, 2: ~23-24b (near float)
			static constexpr int NR = 2;
			
			using Type = typename simd::Type;

#if defined(JPL_USE_SSE)

			Type y = _mm_rsqrt_ps(vec); // approx 1/sqrt(x)

			auto newton = [](const Type& x, const Type& y)
			{
				static const Type half = _mm_set1_ps(0.5f);
				static const Type thal = _mm_set1_ps(1.5f);
#if defined(JPL_USE_FMADD)
				// y = y * (1.5 - 0.5*x*y*y)
				Type xy = _mm_mul_ps(x, y);
				Type xy2 = _mm_mul_ps(xy, y);
				Type term = _mm_fnmadd_ps(half, xy2, thal);
				return _mm_mul_ps(y, term);
#else
				Type y2 = _mm_mul_ps(y, y);
				Type term = _mm_sub_ps(thal, _mm_mul_ps(_mm_mul_ps(half, x), y2));
				return _mm_mul_ps(y, tern);
#endif
			};

			if constexpr (NR >= 1)
			{
				y = newton(vec, y);
			}
			if constexpr (NR >= 2)
			{
				y = newton(vec, y);
			}

			return y;
#elif defined (JPL_USE_NEON)
			Type y = vrsqrteq_f32(vec);  // approx 1/sqrt(x)

			auto newton = [](const Type& x, const Type& y)
			{
#if 0
				// vrsqrtsq_f32(a, b) ~= (3 - a * b) / 2, here a = x * y, b = y
				Type y2 = vmulq_f32(y, y);
				Type xy2 = vmulq_f32(x, y2);
				Type term = vrsqrtsq_f32(xy2, y);
#else
				static const Type half = vdupq_n_f32(0.5f);
				static const Type thal = vdupq_n_f32(1.5f);
				Type xy = vmulq_f32(x, y);
				Type xy2 = vmulq_f32(xy, y);
				Type term = vfnmaddq_f32(half, xy2, thal);
#endif
				return vmulq_f32(y, term);
			};

			if constexpr (NR >= 1)
			{
				y = newton(vec, y);
			}
			if constexpr (NR >= 2)
			{
				y = newton(vec, y);
			}

			return y;
#else
			return simd(
				1.0f / std::sqrt(vec.mNative[0]),
				1.0f / std::sqrt(vec.mNative[1]),
				1.0f / std::sqrt(vec.mNative[2]),
				1.0f / std::sqrt(vec.mNative[3])
			);
#endif
		}
	}

	JPL_INLINE simd max(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_max_ps(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vmaxq_f32(a.mNative, b.mNative);
#else
		return simd(std::max(a.mNative[0], b.mNative[0]),
					std::max(a.mNative[1], b.mNative[1]),
					std::max(a.mNative[2], b.mNative[2]),
					std::max(a.mNative[3], b.mNative[3]));
#endif
	}

	JPL_INLINE simd min(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_min_ps(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vminq_f32(a.mNative, b.mNative);
#else
		return simd(std::min(a.mNative[0], b.mNative[0]),
					std::min(a.mNative[1], b.mNative[1]),
					std::min(a.mNative[2], b.mNative[2]),
					std::min(a.mNative[3], b.mNative[3]));
#endif
	}

	JPL_INLINE simd abs(const simd& vec) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_and_ps(vec.mNative, simd_mask(0x7fffffff).as_simd()); // v & (inv_sign_mask)
#elif defined(JPL_USE_NEON)
		return vabsq_f32(vec.mNative);
#else
		return { abs(vec.mNative[0]), abs(vec.mNative[1]), abs(vec.mNative[2]), abs(vec.mNative[3]) };
#endif
	}

	JPL_INLINE simd clamp(const simd& value, const simd& minV, const simd& maxV) noexcept
	{
		return max(min(value, maxV), minV);
	}

	JPL_INLINE simd fma(const simd& mul1, const simd& mul2, const simd& addV) noexcept
	{
#if defined(JPL_USE_SSE)
#ifdef JPL_USE_FMADD
		return _mm_fmadd_ps(mul1.mNative, mul2.mNative, addV.mNative);
#else
		return _mm_add_ps(_mm_mul_ps(mul1.mNative, mul2.mNative), addV.mNative);
#endif
#elif defined(JPL_USE_NEON)
		return vmlaq_f32(addV.mNative, mul1.mNative, mul2.mNative);
#else
		return Vec3(mul1.mNative[0] * mul2.mNative[0] + addV.mNative[0],
					mul1.mNative[1] * mul2.mNative[1] + addV.mNative[1],
					mul1.mNative[2] * mul2.mNative[2] + addV.mNative[2],
					mul1.mNative[3] * mul2.mNative[3] + addV.mNative[3]);
#endif
	}

	JPL_INLINE simd floor(const simd& vec) noexcept
	{
		// Error handling...
		const simd vec_abs = abs(vec);
		const simd_mask is_nan = vec != vec;
		const simd_mask is_inf = vec_abs == simd::inf();
		const simd_mask is_zero = vec_abs == simd::zero();

#if defined (JPL_USE_SSE)
#if defined (JPL_USE_SSE4_1)
		simd floored = _mm_floor_ps(vec.mNative);
#else
		// trunc toward 0
		simd truncated = vec.to_mask().to_simd();
		// floor = trunc - (trunc > v ? 1 : 0)
		simd mask = (truncated > vec).as_simd() & simd(1.0f);
		simd floored = truncated - mask;
#endif
		return simd::select(is_nan | is_inf | is_zero, vec, floored);
#elif defined (JPL_USE_NEON)
		auto floored = vrndmq_f32(vec.mNative);
		return simd::select(is_nan | is_inf | is_zero, vec, floored);
#else
		return {
			std::floorf(vec.mNative[0]),
			std::floorf(vec.mNative[1]),
			std::floorf(vec.mNative[2]),
			std::floorf(vec.mNative[3])
		};
#endif
	}

	JPL_INLINE simd round(const simd& vec) noexcept
	{
#if defined(JPL_USE_SSE4_1)
		return _mm_round_ps(vec, _MM_FROUND_TO_NEAREST_INT | _MM_FROUND_NO_EXC);
#elif defined(JPL_USE_SSE)
		// trunc(x + copysign(0.5, x))
		simd_mask sign = vec.as_mask() & simd_mask(0x80000000);
		return (vec + (simd(0.5f) | sign.as_simd())).to_mask().to_simd();
#elif defined(JPL_USE_NEON)
		return vrndnq_f32(vec.mNative);
#else
		return {
			std::roundf(vec.mNative[0]),
			std::roundf(vec.mNative[1]),
			std::roundf(vec.mNative[2]),
			std::roundf(vec.mNative[3])
		};
#endif
	}

	JPL_INLINE simd interleave_lohi(const simd& vec) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_shuffle_ps(vec.mNative, vec.mNative, _MM_SHUFFLE(3, 1, 2, 0));
#elif defined(JPL_USE_NEON)
		auto vecs = vzip_f32(vget_low_f32(vec.mNative), vget_high_f32(vec.mNative));
		return vcombine_f32(vecs.val[0], vecs.val[1]);
#else
		return { vec.mNative[0], vec.mNative[2], vec.mNative[1], vec.mNative[3] };
#endif
	}

	JPL_INLINE simd combine_lo(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castsi128_ps(
			_mm_unpacklo_epi64(
				_mm_castps_si128(a.mNative),
				_mm_castps_si128(b.mNative)
			)
		);
#elif defined(JPL_USE_NEON)
		return vcombine_f32(vget_low_f32(a.mNative), vget_low_f32(b.mNative));
#else
		return { a.mNative[0], a.mNative[1],  b.mNative[0], b.mNative[1] };
#endif
	}

	JPL_INLINE simd combine_hi(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_castsi128_ps(
			_mm_unpackhi_epi64(
				_mm_castps_si128(a.mNative),
				_mm_castps_si128(b.mNative)
			)
		);
#elif defined(JPL_USE_NEON)
		return vcombine_f32(vget_high_f32(a.mNative), vget_high_f32(b.mNative));
#else
		return { a.mNative[2], a.mNative[3],  b.mNative[2], b.mNative[3] };
#endif
	}

	JPL_INLINE simd combine_lohi(const simd& a, const simd& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_shuffle_ps(a.mNative, b.mNative, _MM_SHUFFLE(3, 2, 1, 0));
#elif defined(JPL_USE_NEON)
		return vcombine_f32(vget_low_f32(a.mNative), vget_high_f32(b.mNative));
#else
		return { a.mNative[0], a.mNative[1],  b.mNative[2], b.mNative[3] };
#endif
	}

	JPL_INLINE simd reverse(const simd& vec) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_shuffle_ps(vec.mNative, vec.mNative, _MM_SHUFFLE(0, 1, 2, 3));
#elif defined(JPL_USE_NEON)
		// { 0, 1, 2, 3 } -> { 1, 0, 3, 2 }
		float32x4_t rev64 = vrev64q_f32(vec.mNative);
		// combine high part { 3, 2 } and low part { 1, 0 } -> {3, 2, 1, 0}
		return vcombine_f32(vget_high_f32(rev64), vget_low_f32(rev64));
#else
		return { vec.mNative[3], vec.mNative[2], vec.mNative[1], vec.mNative[0] };
#endif
	}

	//==========================================================================
	JPL_INLINE simd_mask max(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_max_epu32(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vmaxq_u32(a.mNative, b.mNative);
#else
		return simd_mask(std::max(a.mNative[0], b.mNative[0]),
						 std::max(a.mNative[1], b.mNative[1]),
						 std::max(a.mNative[2], b.mNative[2]),
						 std::max(a.mNative[3], b.mNative[3]));
#endif
	}

	JPL_INLINE simd_mask min(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE)
		return _mm_min_epu32(a.mNative, b.mNative);
#elif defined(JPL_USE_NEON)
		return vminq_u32(a.mNative, b.mNative);
#else
		return simd_mask(std::min(a.mNative[0], b.mNative[0]),
						 std::min(a.mNative[1], b.mNative[1]),
						 std::min(a.mNative[2], b.mNative[2]),
						 std::min(a.mNative[3], b.mNative[3]));
#endif
	}

	JPL_INLINE simd_mask clamp(const simd_mask& value, const simd_mask& minV, const simd_mask& maxV) noexcept
	{
		return max(min(value, maxV), minV);
	}

	JPL_INLINE simd_mask maxs(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE4_1)
		return _mm_max_epi32(a.mNative, b.mNative);
#elif defined(JPL_USE_SSE) // SSE2 fallback
		typename simd_mask::Type gt = _mm_cmpgt_epi32(a.mNative, b.mNative); // a > b ? 0xFFFFFFFF : 0
		// max = a where a > b, else b
		return _mm_or_si128(_mm_and_si128(gt, a.mNative),
							_mm_andnot_si128(gt, b.mNative));
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vmaxq_s32(
			vreinterpretq_s32_u32(a.mNative),
			vreinterpretq_s32_u32(b.mNative)));
#else
		return simd_mask(
			static_cast<uint32>(std::max(static_cast<int32>(a.mNative[0]), static_cast<int32>(b.mNative[0]))),
			static_cast<uint32>(std::max(static_cast<int32>(a.mNative[1]), static_cast<int32>(b.mNative[1]))),
			static_cast<uint32>(std::max(static_cast<int32>(a.mNative[2]), static_cast<int32>(b.mNative[2]))),
			static_cast<uint32>(std::max(static_cast<int32>(a.mNative[3]), static_cast<int32>(b.mNative[3]))));
#endif
	}

	JPL_INLINE simd_mask mins(const simd_mask& a, const simd_mask& b) noexcept
	{
#if defined(JPL_USE_SSE4_1)
		return _mm_min_epi32(a.mNative, b.mNative);
#elif defined(JPL_USE_SSE) // SSE2 fallback: select with cmp
		typename simd_mask::Type gt = _mm_cmpgt_epi32(a.mNative, b.mNative); // a > b ? 0xFFFFFFFF : 0
		// min = b where a > b, else a
		return _mm_or_si128(_mm_and_si128(gt, b.mNative),
							_mm_andnot_si128(gt, a.mNative));
#elif defined(JPL_USE_NEON)
		return vreinterpretq_u32_s32(vminq_s32(
			vreinterpretq_s32_u32(a.mNative),
			vreinterpretq_s32_u32(b.mNative)));
#else
		return simd_mask(
			static_cast<uint32>(std::min(static_cast<int32>(a.mNative[0]), static_cast<int32>(b.mNative[0]))),
			static_cast<uint32>(std::min(static_cast<int32>(a.mNative[1]), static_cast<int32>(b.mNative[1]))),
			static_cast<uint32>(std::min(static_cast<int32>(a.mNative[2]), static_cast<int32>(b.mNative[2]))),
			static_cast<uint32>(std::min(static_cast<int32>(a.mNative[3]), static_cast<int32>(b.mNative[3]))));
#endif
	}

	JPL_INLINE simd_mask clamps(const simd_mask& value, const simd_mask& minV, const simd_mask& maxV) noexcept
	{
		return maxs(mins(value, maxV), minV);
	}

} // namespace JPL