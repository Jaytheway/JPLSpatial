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
#include <JPLSpatial/Math/SIMD.h>
#include <JPLSpatial/Math/Vec3Pack.h>
#include <JPLSpatial/Math/Vec3Traits.h>

#include <concepts>
#include <cstring>
#include <span>

namespace JPL
{
	namespace Impl
	{
		template<class T>
		concept CSIMDorFloat = std::same_as<T, float> || std::same_as<T, simd>;
	}

	//======================================================================
	// Forward declarations
	template<Impl::CSIMDorFloat T>
	struct Vec3BufferView;

	template<Impl::CSIMDorFloat T, std::size_t N>
	struct Vec3Buffer;

	//======================================================================
	/// Vec3-like SoA buffer, holding separate arrays of Vec3 components X, Y, Z as floats
	template<std::size_t N> using Vec3SIMDBuffer = Vec3Buffer<simd, N>;

	/// Vec3-like SoA buffer, holding separate arrays of Vec3 components X, Y, Z as simd
	template<std::size_t N> using Vec3FloatBuffer = Vec3Buffer<float, N>;

	/// View into a Vec3-like SoA buffer, holding separate arrays of Vec3 components X, Y, Z as floats
	using Vec3SIMDBufferView = Vec3BufferView<simd>;

	/// View into a Vec3-like SoA buffer, holding separate arrays of Vec3 components X, Y, Z as simd
	using Vec3FloatBufferView = Vec3BufferView<float>;

	//======================================================================
	/// Vec3-like SoA buffer, holding separate arrays of Vec3 components
	/// X, Y, Z as float or simd
	template<Impl::CSIMDorFloat T, std::size_t N>
	struct Vec3Buffer
	{
		T X[N];
		T Y[N];
		T Z[N];

		[[nodiscard]] JPL_INLINE Vec3BufferView<T> MakeView(std::size_t count) noexcept
		{
			JPL_ASSERT(count <= N);
			return Vec3BufferView<T>{ .X = X, .Y = Y , .Z = Z, .Count = count };
		}

		[[nodiscard]] JPL_INLINE Vec3BufferView<T> MakeView(std::size_t offset, std::size_t count) noexcept
		{
			JPL_ASSERT((offset + count) <= N);
			return Vec3BufferView<T>{ .X = X + offset, .Y = Y + offset, .Z = Z + offset, .Count = count };
		}
	};

	//======================================================================
	/// View into a Vec3-like SoA buffer, holding separate arrays of Vec3
	/// components X, Y, Z as float or simd
	template<Impl::CSIMDorFloat T>
	struct Vec3BufferView
	{
		T* X;
		T* Y;
		T* Z;
		std::size_t Count = 0;

		[[nodiscard]] JPL_INLINE std::size_t size() const noexcept { return Count; }

		template<CVec3 Vec3Type>
		JPL_INLINE void Unpack(std::span<Vec3Type> outVec3s) const
		{
			if constexpr (std::same_as<T, float>)
			{
				JPL_ASSERT(outVec3s.size() >= size());
				for (uint32 i = 0; i < size(); ++i)
				{
					Vec3Type& outVec = outVec3s[i];
					SetX(outVec, X[i]); SetY(outVec, Y[i]); SetZ(outVec, Z[i]);
				}
			}
			else
			{
				JPL_ASSERT(outVec3s.size() >= size() * simd::size());

				for (uint32 i = 0; i < size(); ++i)
				{
					float xs[simd::size()]{}; X[i].store(xs);
					float ys[simd::size()]{}; Y[i].store(ys);
					float zs[simd::size()]{}; Z[i].store(zs);
					for (uint32 ii = 0; ii < simd::size(); ++ii)
					{
						Vec3Type& outVec = outVec3s[i * simd::size() + ii];
						SetX(outVec, xs[ii]); SetY(outVec, ys[ii]); SetZ(outVec, zs[ii]);
					}
				}
			}
		}

		JPL_INLINE void CopyTo(Vec3BufferView& other) const
		{
			JPL_ASSERT(other.size() == size());
			std::memcpy(other.X, X, sizeof(T) * size());
			std::memcpy(other.Y, Y, sizeof(T) * size());
			std::memcpy(other.Z, Z, sizeof(T) * size());
		}
	};
} // namespace JPL
