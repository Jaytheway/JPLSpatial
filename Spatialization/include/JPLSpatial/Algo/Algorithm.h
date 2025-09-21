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
#include "JPLSpatial/Math/Math.h"

#include <type_traits>
#include <utility>
#include <functional>
#include <numeric>
#include <algorithm>

namespace JPL::Algo
{
	namespace Internal
	{
		template<class ContainerType>
		using ElementTypeOf = std::remove_cvref_t<decltype(*std::declval<ContainerType>().begin())>;
	}

	//==========================================================================
	/// Just a wrapper for a range, to not have to type begin and end iterators
	template<class RangeType, class T, class Fn = std::plus<T>>
	[[nodiscard]] JPL_INLINE constexpr T Accumulate(RangeType&& range, T initialValue, Fn reduceOp = {})
	{
		return std::accumulate(range.begin(), range.end(), std::move(initialValue), reduceOp);
	}

	//==========================================================================
	/// functor for std::accumulate (and similar) to accumulate sum of value squares
	template <class T = void>
	struct AccPow2
	{
		[[nodiscard]] JPL_INLINE constexpr T operator()(const T& acc, const T& value) const noexcept
		{
			return acc + value * value;
		}
	};

	/// functor for std::accumulate (and similar) to accumulate absolute values
	template <class T = void>
	struct AccAbs
	{
		[[nodiscard]] JPL_INLINE constexpr T operator()(const T& acc, const T& value) const noexcept
		{
			return acc + Math::Abs(value);
		}
	};

	/// functor for for_each (and similar) to multiply elements by a multiplier
	template <class T = void>
	class Multiply
	{
		T mMultiplier;
	public:
		explicit Multiply(T multiplier) : mMultiplier(multiplier) {}
		
		JPL_INLINE constexpr void operator()(T& value) const noexcept
		{
			value *= mMultiplier;
		}
	};

	//==========================================================================
	/// Normalize so that the sum = 1
	template<class ContainerType>
	JPL_INLINE constexpr void NormalizeL1(ContainerType&& data)
	{
		using ElementType = Internal::ElementTypeOf<ContainerType>;
		const ElementType sum = Accumulate(std::forward<ContainerType>(data), ElementType(0), AccAbs<ElementType>{});
		const float invSum = 1.0f / static_cast<float>(sum);
		std::ranges::for_each(std::forward<ContainerType>(data), Multiply(invSum));
	}

	/// Apply unit vector scaling, so that the magnitude of the vector = 1
	template<class ContainerType>
	JPL_INLINE constexpr void NormalizeL2(ContainerType&& data)
	{
		using ElementType = Internal::ElementTypeOf<ContainerType>;
		const ElementType sum2 = Accumulate(std::forward<ContainerType>(data), ElementType(0), AccPow2<ElementType>{});
		const float invLength = Math::InvSqrt(static_cast<float>(sum2));
		std::ranges::for_each(std::forward<ContainerType>(data), Multiply(invLength));
	}

	template<class ContainerType>
	[[nodiscard]] JPL_INLINE constexpr bool IsNormalizedL1(const ContainerType& data, float tolerance = JPL_FLOAT_EPS)
	{
		using ElementType = Internal::ElementTypeOf<ContainerType>;
		ElementType sum = Accumulate(data, ElementType(0), AccAbs<ElementType>{});
		return Math::IsNearlyEqual(static_cast<float>(sum), 1.0f, tolerance);
	}

	template<class ContainerType>
	[[nodiscard]] JPL_INLINE constexpr bool IsNormalizedL2(const ContainerType& data, float tolerance = JPL_FLOAT_EPS)
	{
		using ElementType = Internal::ElementTypeOf<ContainerType>;
		const ElementType sum2 = Accumulate(data, ElementType(0), AccPow2<ElementType>{});
		return Math::IsNearlyEqual(static_cast<float>(sum2), 1.0f, tolerance);
	}
} // namespace JPL::Algo