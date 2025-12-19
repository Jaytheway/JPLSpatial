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

#include <numbers>
#include <cmath>
#include <limits>
#include <concepts>
#include <type_traits>
#include <bit>
#include <concepts>
#include <utility>

namespace JPL
{
	static constexpr float JPL_HALF_PI = 0.5f * std::numbers::pi_v<float>;
	static constexpr float JPL_PI = std::numbers::pi_v<float>;
	static constexpr float JPL_TWO_PI = 2.0f * std::numbers::pi_v<float>;
	static constexpr float JPL_FOUR_PI = 4.0f * std::numbers::pi_v<float>;

	static constexpr float JPL_INV_PI = std::numbers::inv_pi_v<float>;
	static constexpr float JPL_INV_TWO_PI = std::numbers::inv_pi_v<float> * 0.5f;

	template<std::floating_point T> static constexpr T JPL_TO_RAD_V = std::numbers::pi_v<T> / T(180.0);
	template<std::floating_point T> static constexpr T JPL_TO_DEG_V = T(180.0) / std::numbers::pi_v<T>;

	static constexpr float JPL_TO_RAD = JPL_TO_RAD_V<float>;
	static constexpr float JPL_TO_DEG = JPL_TO_DEG_V<float>;
	
	template<std::floating_point T> struct FloatNubmer;
	template<> struct FloatNubmer<float> { static constexpr float Tolerance = 1e-6f; };
	template<> struct FloatNubmer<double> { static constexpr double Tolerance = 1e-8; };
	
	template<std::floating_point T>
	static constexpr float JPL_FLOAT_EPS_V = FloatNubmer<T>::Tolerance;

	static constexpr float JPL_FLOAT_EPS = JPL_FLOAT_EPS_V<float>;
}

namespace JPL::Math
{
	template<class T> concept CArithmetic = std::is_arithmetic_v<T>;
	template<class T1, class T2> concept COrderedWith = std::totally_ordered_with<T1, T2>;
	template<class T1, class T2> concept CArithmeticAndOrderedWith = CArithmetic<T1> && COrderedWith<T1, T2>;
	
	//======================================================================
	/// Function to convert degrees to radians
	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr T ToRadians(T degrees) noexcept
	{
		return degrees * JPL_TO_RAD_V<T>;
	}
	
	/// Function to convert radians to degrees
	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr T ToDegrees(T radians) noexcept
	{
		return radians * JPL_TO_DEG_V<T>;
	}

	//======================================================================
	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr bool IsNan(T x) noexcept { return x != x; }

	/// Standard `abs` is not constexpr in C++20
	template<CArithmetic T>
	[[nodiscard]] JPL_INLINE constexpr auto Abs(const T& value) noexcept
	{
		return value < 0 ? -value : value;
	}

	//======================================================================
	/// A constexpr implementation of floor.
	/// Note: NaN-s and inf-s are not handled.
	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr T Floor(T value) noexcept
	{
		if (std::is_constant_evaluated())
		{
			const T truncated_f = static_cast<T>(static_cast<int64>(value));
			return truncated_f - static_cast<T>(value < truncated_f);
		}
		else
		{
			return std::floor(value);
		}
	}
#if 0
	static_assert(Floor(3.14) == 3.0);
	static_assert(Floor(3.0) == 3.0);
	static_assert(Floor(-3.14) == -4.0);
	static_assert(Floor(-3.0) == -3.0);
	static_assert(Floor(0.0) == 0.0);
	static_assert(Floor(-0.5) == -1.0);
#endif

	/// A constexpr implementation of floor.
	template<std::integral T>
	[[nodiscard]] JPL_INLINE constexpr T Floor(T value) noexcept { return value; }

	//======================================================================
	/// Sign returns -1 for negative values, 1 for positive, 0 for 0
	template<CArithmetic T>
	[[nodiscard]] JPL_INLINE constexpr T Sign(T value) noexcept
	{
		return (value > T(0)) ? T(1) : ((value < T(0)) ? T(-1) : T(0));
	}

	/// Sign2 returns -1 for negative values, 1 otherwise
	template<CArithmetic T>
	[[nodiscard]] JPL_INLINE constexpr T Sign2(T value) noexcept
	{
		if (std::is_constant_evaluated())
			return value < T(0) ? T(-1) : T(1);
		else
			return std::copysign(T(1), value);
	}

	template<CArithmetic T1, CArithmeticAndOrderedWith<T1> T2>
	[[nodiscard]] JPL_INLINE constexpr bool IsPositiveAndBelow(T1 value, T2 below) noexcept
	{
		return value > T1(0) && value < below;
	}

	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr bool IsNearlyZero(T value, T errorTolerance = JPL_FLOAT_EPS_V<T>) noexcept
	{
		return Abs(value) <= errorTolerance;
	}

	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr bool IsNearlyEqual(T a, T b, T tolerance = JPL_FLOAT_EPS_V<T>) noexcept
	{
		return Abs(a - b) <= tolerance;
	}

	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE std::pair<T, T> SinCos(T value) noexcept
	{
		// One day we may get std::sincos
		return { std::sin(value), std::cos(value) };
	}

	/// Linearly interpolate v0 towards v1 and normalize.
	template<std::floating_point T>
	[[nodiscard]] JPL_INLINE constexpr T Lerp(const T& v0, const T& v1, T t) noexcept
	{
		return t * (v1 - v0) + v0;
	}

	//======================================================================
	/// Square Root

	namespace Detail
	{
		/// constexpr-friendly quiet NaN (IEEE-754 only)
		template <std::floating_point T>
		[[nodiscard]] constexpr T QNan() noexcept
		{
			if constexpr (!std::numeric_limits<T>::is_iec559)
				return std::numeric_limits<T>::quiet_NaN();
			else if constexpr (std::is_same_v<T, float>)
				return std::bit_cast<float>(0x7fc00000u);
			else if constexpr (std::is_same_v<T, double>)
				return std::bit_cast<double>(0x7ff8000000000000ull);
			else
				return std::numeric_limits<T>::quiet_NaN();
		}

		/// Initial guess for invsqrt (constexpr-friendly):
		/// - float: use the classic "Quake" bit trick.
		template <std::floating_point T>
		[[nodiscard]] constexpr T InvSqrtInitialGuess(T x) noexcept
		{
			if constexpr (std::is_same_v<T, float>)
			{
				uint32_t i = std::bit_cast<uint32_t>(x);
				i = 0x5f3759dfu - (i >> 1);
				return std::bit_cast<float>(i);
			}
			else if constexpr (std::is_same_v<T, double>)
			{
				uint64_t i = std::bit_cast<uint64_t>(x);
				i = 0x5fe6eb50c7b537a9ull - (i >> 1);
				return std::bit_cast<double>(i);
			}
			else
			{
				// Safe positive seed; NR converges.
				return T(1);
			}
		}

		/// Scalar NR on inverse sqrt
		template <std::floating_point T, int NR>
		[[nodiscard]] constexpr T InvSqrtNewtonRefine(T x, T y) noexcept
		{
			const T halfx = T(0.5) * x;
			for (int i = 0; i < NR; ++i)
				y = y * (T(1.5) - halfx * y * y);
			return y;
		}

		/// Pure constexpr fallback sqrt using inverse-sqrt NR
		template <std::floating_point T, int NR>
		[[nodiscard]] constexpr T SqrtConstevalFallback(T x) noexcept
		{
			if (x == T(0)) return T(0);
			if (x < T(0)) return QNan<T>();
			if (x == std::numeric_limits<T>::infinity()) return x;
			if (IsNan(x)) return x;

			T y = InvSqrtInitialGuess<T>(x);
			y = InvSqrtNewtonRefine<T, NR>(x, y);
			T r = x * y;

			// Optional polish for float-only constexpr if you need tighter equality on tidy values:
			if constexpr (std::is_same_v<T, float>)
				r = T(0.5) * (r + x / r); // One Heron step (compile-time cost only)

			return r;
		}
	} // namespace Detail

	/// Square root: constexpr at compile time, std::sqrt at runtime
	/// Tempalte param 'NR' (used at for consteval): number of Newton–Raphson refinement steps on 1/sqrt(x).
	/// NR=2 is a decent default for float.
	template<std::floating_point T, int NR = 2>
	[[nodiscard]] JPL_INLINE constexpr T Sqrt(T x) noexcept
	{
		static_assert(NR >= 0, "NR must be >= 0");

		if (std::is_constant_evaluated())
			return Detail::SqrtConstevalFallback<T, NR>(x);
		else
			return std::sqrt(x);
	}

	/// Inverse square root: constexpr at compile time, 1 / std::sqrt at runtime
	/// Tempalte param 'NR' (used at for consteval): number of Newton–Raphson refinement steps on 1/sqrt(x).
	/// NR=2 is a decent default for float.
	template<std::floating_point T, int NR = 2>
	[[nodiscard]] JPL_INLINE constexpr T InvSqrt(T x) noexcept
	{
		if (std::is_constant_evaluated())
			return T(1.0) / Detail::SqrtConstevalFallback<T, NR>(x);
		else
			return T(1.0) / std::sqrt(x); // compiled down to rsqrtss, no division
	}
} // namespace JPL::Math