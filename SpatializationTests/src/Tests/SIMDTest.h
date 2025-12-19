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

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"

#include <gtest/gtest.h>

#include <array>
#include <algorithm>
#include <bit>
#include <cmath>
#include <functional>
#include <ostream>
#include <limits>
#include <ranges>
#include <initializer_list>

namespace JPL
{
	namespace Util
	{
		template<class T>
		auto divide_by(T value) { return [value](float v) { return v / value; }; }
		template<class T>
		auto multiply_by(T value) { return [value](float v) { return v * value; }; }

		// Just to not have to spell out the entire thing
		inline float nan() { return std::numeric_limits<float>::quiet_NaN(); }
		inline float inf() { return std::numeric_limits<float>::infinity(); }

		using simd_ref = std::array<float, 4>;
		struct simd_check
		{
			// Some tests may want to override this
			float Tolerance = 1e-6f;

			simd_check() = default;
			constexpr simd_check(std::initializer_list<float> list)
			{
				std::ranges::copy(list, Ref.begin());
			}
			simd_check(const simd_ref& values, std::function<float(float)> func)
			{
				Ref = values;
				for (float& v : Ref)
					v = func(v);
			}
			simd_check(const simd_ref& valuesA, const simd_ref& valuesB, std::function<float(float, float)> func)
			{
				std::transform(valuesA.begin(), valuesA.end(), valuesB.begin(), Ref.begin(), func);
			}
			simd_check(const simd_ref& valuesA, const std::array<int, 4>& valuesB, std::function<float(float, int)> func)
			{
				std::transform(valuesA.begin(), valuesA.end(), valuesB.begin(), Ref.begin(), func);
			}
			simd_check(const simd_ref& valuesA, const simd_ref& valuesB, const simd_ref& valuesC, std::function<float(float, float, float)> func)
			{
				for (int i = 0; i < 4; ++i)
					Ref[i] = func(valuesA[i], valuesB[i], valuesC[i]);
			}

			simd_check& apply(std::function<float(float)> func)
			{ 
				*this = simd_check(Ref, func);
				return *this;
			}

			simd_check apply(std::function<float(float)> func) const
			{
				return simd_check(*this, func);
			}

			simd_check& apply(const simd_ref& valuesB, std::function<float(float, float)> func)
			{
				*this = simd_check(Ref, valuesB, func);
				return *this;
			}

			simd_check apply(const simd_ref& valuesB, std::function<float(float, float)> func) const
			{
				return simd_check(this->Ref, valuesB, func);
			}

			simd_check& apply(std::function<float(float, float)> func)
			{
				*this = simd_check(Ref, Ref, func);
				return *this;
			}

			simd_check apply(std::function<float(float, float)> func) const
			{
				return simd_check(Ref, Ref, func);
			}

			operator simd_ref() const
			{
				return Ref;
			}

			simd_ref Ref;

			constexpr float* data() { return Ref.data(); }
			constexpr const float* data() const { return Ref.data(); }

			inline float& operator[](uint32 index) { return Ref[index]; }
			inline const float& operator[](uint32 index) const { return Ref[index]; }

			inline bool operator ==(const simd& vec) const
			{
				simd_ref data;
				vec.store(data.data());
				return *this == data;
			}

			inline bool operator ==(const simd_ref& ref) const
			{
				auto sameSigns = [&](uint32 i)
				{
					return std::signbit(Ref[i]) == std::signbit(ref[i]);
				};

				for (uint32 i = 0; i < 4; ++i)
				{
					// For special cases inf, NaN, 0.0: check the signs
					if (std::isinf(Ref[i]))
					{
						if (!std::isinf(ref[i]) || !sameSigns(i))
							return false;
					}
					else if (std::isnan(Ref[i]))
					{
						if (!std::isnan(ref[i]) || !sameSigns(i))
							return false;
					}
					else if (Ref[i] == 0.0f || Ref[i] == -0.0f)
					{
						if (ref[i] != 0.0f || ref[i] != -0.0f || !sameSigns(i))
							return false;
					}
					else if (!Math::IsNearlyEqual(Ref[i], ref[i], Tolerance))
					{
						// Default case: check equality
						return false;
					}
				}
				return true;
			}

			friend std::ostream& operator << (std::ostream& inStream, const simd_check& vec)
			{
				inStream << vec.Ref[0] << ", " << vec.Ref[1] << ", " << vec.Ref[2] << ", " << vec.Ref[3];
				return inStream;
			}
		};

		static constexpr simd_check cTest0123{ 0.0f, 1.0f, 2.0f, 3.0f };
		static constexpr simd_check cTestSplat1{ 1.0f, 1.0f, 1.0f, 1.0f };
		static constexpr simd_check cTestSplat0{ 0.0f, 0.0f, 0.0f, 0.0f };
	}

	TEST(SIMD, LoadStore_CorrectValues)
	{
		simd s;
		s.load(Util::cTest0123.data());

		Util::simd_ref stored;
		s.store(stored.data());

		EXPECT_EQ(stored, Util::cTest0123);
	}

	TEST(SIMD, Constructor_LoadsCorrectValues)
	{
		simd s1(Util::cTest0123.data());
		EXPECT_EQ(s1, Util::cTest0123);

		simd s2(
			Util::cTest0123[0],
			Util::cTest0123[1],
			Util::cTest0123[2],
			Util::cTest0123[3]
		);
		EXPECT_EQ(s2, Util::cTest0123);

		simd s3(1.0f);
		EXPECT_EQ(s3, Util::cTestSplat1);
	}

	TEST(SIMD, IndexOperator_GetsCorrectComponent)
	{
		simd s(Util::cTest0123.data());

		for (uint32 i = 0; i < 4; ++i)
		{
			EXPECT_EQ(s[i], Util::cTest0123[i]);
		}
	}

	TEST(SIMD, MultiplyWithSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		simd result = s1 * s2;
		Util::simd_check expected = Util::cTest0123.apply(std::multiplies<float>());
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, MultiplyWithFloatCorrect)
	{
		simd s1(Util::cTest0123.data());
		float value = 2.0f;
		simd mul = s1 * value;
		simd mul2 = value * s1;
		Util::simd_check expected(Util::cTest0123, Util::multiply_by(value));
		EXPECT_EQ(mul, expected);
		EXPECT_EQ(mul2, expected);
	}

	TEST(SIMD, DivideByFloatCorrect)
	{
		simd s1(Util::cTest0123.data());
		float value = 2.0f;
		simd result = s1 / value;
		Util::simd_check expected(Util::cTest0123, Util::divide_by(value));
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, InPlaceMultiplyWithFloatCorrect)
	{
		simd s1(Util::cTest0123.data());
		float value = 2.0f;
		s1 *= value;
		Util::simd_check expected(Util::cTest0123, Util::multiply_by(value));
		EXPECT_EQ(s1, expected);
	}

	TEST(SIMD, InPlaceMultiplyWithSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		s1 *= s2;
		Util::simd_check expected = Util::cTest0123.apply(std::multiplies<float>());
		EXPECT_EQ(s1, expected);
	}

	TEST(SIMD, InPlaceDivideByFloatCorrect)
	{
		simd s1(Util::cTest0123.data());
		float value = 2.0f;
		s1 /= value;
		Util::simd_check expected(Util::cTest0123, Util::divide_by(value));
		EXPECT_EQ(s1, expected);
	}

	TEST(SIMD, AddSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		simd result = s1 + s2;
		Util::simd_check expected = Util::cTest0123.apply(std::plus<float>());
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, InPlaceAddSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		s1 += s2;
		Util::simd_check expected = Util::cTest0123.apply(std::plus<float>());
		EXPECT_EQ(s1, expected);
	}

	TEST(SIMD, NegateCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2 = -s1;
		Util::simd_check expected =
			Util::cTestSplat0.apply(Util::cTest0123,
									std::minus<float>());
		EXPECT_EQ(s2, expected);
	}

	TEST(SIMD, SubractSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		simd result = s1 - s2;
		Util::simd_check expected = Util::cTest0123.apply(std::minus<float>());
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, InPlaceSubtractSIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd s2(Util::cTest0123.data());
		s1 -= s2;
		Util::simd_check expected = Util::cTest0123.apply(std::minus<float>());
		EXPECT_EQ(s1, expected);
	}

	TEST(SIMD, DivideBySIMDCorrect)
	{
		simd s1(Util::cTest0123.data());
		Util::simd_ref arg2{ 1.0f, 2.0f, 4.0f, 3.0f };
		simd result = s1 / arg2.data();
		Util::simd_check expected =
			Util::cTest0123.apply(arg2,
								  std::divides<float>());
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, ReduceSumCorrect)
	{
		simd s1(Util::cTest0123.data());
		float result = s1.reduce();
		EXPECT_EQ(result, 6.0f);
	}

	TEST(SIMD, ReduceMaxCorrect)
	{
		simd s1(Util::cTest0123.data());
		float result = s1.reduce_max();
		EXPECT_EQ(result, 3.0f);
	}

	TEST(SIMD, ReduceMinCorrect)
	{
		simd s1(Util::cTest0123.data());
		float result = s1.reduce_min();
		EXPECT_EQ(result, 0.0f);
	}

	TEST(SIMD, SqrtCorrect)
	{
		Util::simd_ref arg1{ 4.0f, 9.0f, 16.0f, 25.0f };
		simd result = Math::Sqrt(arg1.data());
		Util::simd_check expected(arg1, &Math::Sqrt<float>);
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, InvSqrtCorrect)
	{
		Util::simd_ref arg1{ 4.0f, 9.0f, 16.0f, 25.0f };
		simd result = Math::InvSqrt(arg1.data());
		Util::simd_check expected(arg1, &Math::InvSqrt<float>);
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, MinCorrect)
	{
		simd s1(Util::cTest0123.data());
		Util::simd_ref arg2{ 1.0f, 0.5f, 3.0f, 2.5f };
		simd result = min(s1, arg2.data());
		Util::simd_check expected{ Util::cTest0123, arg2, fminf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, MaxCorrect)
	{
		simd s1(Util::cTest0123.data());
		Util::simd_ref arg2{ 1.5f, 0.5f, 3.5f, 2.5f };
		simd result = max(s1, arg2.data());
		Util::simd_check expected{ Util::cTest0123, arg2, fmaxf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, ClampCorrect)
	{
		simd s1(Util::cTest0123.data());
		Util::simd_ref minV{ 0.5f, 0.5f, 2.5f, 2.5f };
		Util::simd_ref maxV{ 1.5f, 1.5f, 3.5f, 3.5f };
		simd result = clamp(s1, minV.data(), maxV.data());
		Util::simd_check expected{ Util::cTest0123, minV, maxV, std::clamp<float> };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, AbsCorrect)
	{
		Util::simd_ref arg1{ -1.0f, -2.4f, 2.8f, 3.2f };
		simd result = abs(simd(arg1.data()));
		Util::simd_check expected{ arg1, fabsf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, TrunkCorrect)
	{
		// As per cppreference
		/*
			trunc(+2.7) == +2
			trunc(-2.9) == -2
			trunc(+0.7) == +0
			trunc(-0.9) == -0
			trunc(+0) == +0
			trunc(-0) == -0
			trunc(-inf) == -inf
			trunc(+inf) == +inf
			trunc(-nan) == -nan
			trunc(+nan) == +nan
		*/
		{
			Util::simd_ref arg1{ +2.7f, -2.9f, +0.7f, -0.9f };
			simd result = trunc(simd(arg1.data()));
			Util::simd_check expected{ arg1, truncf };
			EXPECT_EQ(result, expected);
		}
		{
			Util::simd_ref arg1{ +0.0f, -0.0f, -Util::inf(), +Util::inf() };
			simd result = trunc(simd(arg1.data()));
			Util::simd_check expected{ arg1, truncf };
			EXPECT_EQ(result, expected);
		}
		{
			// NaNs are special
			Util::simd_ref arg1{ -Util::nan(), +Util::nan(), 1.0f, 1.0f };
			simd result = trunc(simd(arg1.data()));
			Util::simd_check expected{ arg1, truncf };
			EXPECT_EQ(result, expected);
		}
	}

	TEST(SIMD, FMACorrect)
	{
		simd s1(Util::cTest0123.data());
		Util::simd_ref mul{ 2.0f, 3.0f, 1.0f, 2.0f };
		Util::simd_ref add{ 1.5f, 1.5f, 3.5f, 3.5f };
		simd result = fma(s1, mul.data(), add.data());
		Util::simd_check expected{ Util::cTest0123, mul, add, fmaf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, MaskTrueChecksCorrect)
	{
		static constexpr auto TRUE = simd_mask::cTrueValue;
		EXPECT_TRUE(simd_mask(0, 0, TRUE, 0).any_of());
		EXPECT_FALSE(simd_mask(0, 0, TRUE, 0).all_of());
		EXPECT_FALSE(simd_mask(0, 0, TRUE, 0).none_of());
		EXPECT_EQ(simd_mask(0, 0, TRUE, 0).reduce_count(), 1);

		EXPECT_TRUE(simd_mask(TRUE, TRUE, TRUE, TRUE).any_of());
		EXPECT_TRUE(simd_mask(TRUE, TRUE, TRUE, TRUE).all_of());
		EXPECT_FALSE(simd_mask(TRUE, TRUE, TRUE, TRUE).none_of());
		EXPECT_EQ(simd_mask(TRUE, TRUE, TRUE, TRUE).reduce_count(), 4);

		EXPECT_FALSE(simd_mask(0, 0, 0, 0).any_of());
		EXPECT_FALSE(simd_mask(0, 0, 0, 0).all_of());
		EXPECT_TRUE(simd_mask(0, 0, 0, 0).none_of());
		EXPECT_EQ(simd_mask(0, 0, 0, 0).reduce_count(), 0);

		EXPECT_EQ(simd_mask(0, TRUE, TRUE, 0).reduce_min_index(), 1);
		EXPECT_EQ(simd_mask(0, TRUE, TRUE, 0).reduce_max_index(), 2);
		EXPECT_EQ(simd_mask(0, 0, 0, 0).reduce_min_index(), -1);
		EXPECT_EQ(simd_mask(0, 0, 0, 0).reduce_max_index(), -1);
	}

	TEST(SIMD, MaskSelectCorrect)
	{
		simd a(Util::cTest0123.data());
		simd b(4.0f, 3.0f, 1.0f, 7.0f);
		simd_mask mask(0, simd_mask::cTrueValue, simd_mask::cTrueValue, 0);
		simd result = simd::select(mask, a, b);
		Util::simd_check expected{ 4.0f, 1.0f, 2.0f, 7.0f };
		EXPECT_EQ(result, expected);
	}
		

	TEST(SIMD, NaturalLogarithmCorrect)
	{
		simd s1(Util::cTest0123.data());
		simd result = log(s1);

		// log_ps returns nan for log(0)
		EXPECT_TRUE(std::isnan(result[0]));

		s1 = { 0.5f, 1.0f, 2.0f, 3.0f };
		result = log(s1);
		Util::simd_check expected{{ 0.5f, 1.0f, 2.0f, 3.0f }, logf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, Log10Correct)
	{
		simd s1(Util::cTest0123.data());
		simd result = log10(s1);

		// log_ps returns nan for log(0)
		EXPECT_TRUE(std::isnan(result[0]));

		s1 = { 0.5f, 1.0f, 2.0f, 3.0f };
		result = log10(s1);
		Util::simd_check expected{ { 0.5f, 1.0f, 2.0f, 3.0f }, log10f };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, ExpCorrect)
	{
		simd s1(0.0f, 1.0f, 2.5f, 3.0f);
		simd result = exp(s1);
		Util::simd_check expected{{ 0.0f, 1.0f, 2.5f, 3.0f }, expf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, Exp2Correct)
	{
		simd s1(0.0f, 1.0f, -2.5f, 3.0f);
		simd result = exp2(s1);
		Util::simd_check expected{{ 0.0f, 1.0f, -2.5f, 3.0f }, exp2f };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, LdexpCorrect)
	{
		{
			simd s1(0.0f, 1.0f, -2.5f, 3.0f);
			simd_mask exponent(1, 0, 3, 3);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(0.0f, 1),
				ldexpf(1.0f, 0),
				ldexpf(-2.5f, 3),
				ldexpf(3.0f, 3) };
			EXPECT_EQ(result, expected);
		}
		{
			simd s1(0.0f, 1.0f, -2.5f, 3.0f);
			simd_mask exponent(-1, 0, -2, -3);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(0.0f, -1),
				ldexpf(1.0f, 0),
				ldexpf(-2.5f, -2),
				ldexpf(3.0f, -3) };
			EXPECT_EQ(result, expected);
		}
		{
			simd s1(-3.0f, -3.0f, -2.5f, 3.0f);
			simd_mask exponent(2, 3, 200.0f, -300.0f);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(-3.0f, 2),
				ldexpf(-3.0f, 3),
				ldexpf(-2.5f, 200),
				ldexpf(3.0f, -300) };
			EXPECT_EQ(result, expected);
		}
		{
			simd s1(-0.0f, -0.0f, -0.0f, -0.0f);
			simd_mask exponent(3, 0, 2, -3);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(-0.0f, -1),
				ldexpf(-0.0f, 0),
				ldexpf(-0.f, 2),
				ldexpf(-0.0f, -3) };
			EXPECT_EQ(result, expected);
		}

		{
			simd s1(Util::nan(), -Util::nan(), Util::nan(), -Util::nan());
			simd_mask exponent(2, 3, 0, 0);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(Util::nan(), 2),
				ldexpf(-Util::nan(), 3),
				ldexpf(Util::nan(), 0),
				ldexpf(-Util::nan(), 0) };
			EXPECT_EQ(result, expected);
		}
		{
			simd s1(Util::inf(), -Util::inf(), Util::inf(), -Util::inf());
			simd_mask exponent(2, 3, 0, 0);
			simd result = ldexp(s1, exponent);
			Util::simd_check expected{
				ldexpf(Util::inf(), 2),
				ldexpf(-Util::inf(), 3),
				ldexpf(Util::inf(), 0),
				ldexpf(-Util::inf(), 0) };
			EXPECT_EQ(result, expected);
		}
	}

	TEST(SIMD, PowCorrect)
	{
		static constexpr float toleranceOverride = 5e-5f;

		// All the edge/error-cases as per cppreference.com
		// Note: despite the comments, we don't actually raise the errors
		
		{	// Normal cases
			/*	pow(2, 10) = 1024
				pow(2, 0.5) = 1.41421
				pow(-2, -3) = -0.125		(-3 in exponent is a special valid case)
				pow(-2, 3) = -8
			*/
			Util::simd_ref arg1{ 2.0f, 2.0f, -2.0f, -2.0f };
			Util::simd_ref arg2{ 10.0f, 0.5f, -3.0f, 3.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));
			
			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(+0, exp), where exp is a negative odd integer, returns +∞ and raises FE_DIVBYZERO.
			// pow(-0, exp), where exp is a negative odd integer, returns -∞ and raises FE_DIVBYZERO.
			// pow(+0, exp), where exp is a positive odd integer, returns +0.
			// pow(-0, exp), where exp is a positive odd integer, returns -0.
			Util::simd_ref arg1{ 0.0f, -0.0f, +0.0f, -0.0f };
			Util::simd_ref arg2{ -5.0f, -5.0f, 5.0f, 5.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(±0, exp), where exp is negative, finite, and is an even integer or a non-integer, returns +∞ and raises FE_DIVBYZERO.
			Util::simd_ref arg1{ +0.0f, -0.0f, +0.0f, -0.0f };
			Util::simd_ref arg2{ -2.0f, -2.0f, -2.5f, -2.5f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(±0, -∞) returns +∞ and may raise FE_DIVBYZERO.
			// pow(-1, 1/3) = -nan
			// pow(-1, NAN) = nan
			Util::simd_ref arg1{ +0.0f, -0.0f, -1.0f, -1.0f };
			Util::simd_ref arg2{ -Util::inf(), -Util::inf(), 1.0f / 3.0f, Util::nan()};
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(±0, exp), where exp is positive non-integer or a positive even integer, returns +0.
			Util::simd_ref arg1{ +0.0f, -0.0f, +0.0f, -0.0f };
			Util::simd_ref arg2{ 2.5f, 2.5f, 2.0f, 2.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(+1, exp) returns 1 for any exp, even when exp is NaN.
			Util::simd_ref arg1{ +1.0f, +1.0f, +1.0f, +1.0f };
			Util::simd_ref arg2{ Util::nan(), -Util::inf(), 2.0f, -3.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(-1, ±∞) returns 1.
			// pow(base, exp) returns NaN and raises FE_INVALID if base is finite and negative and exp is finite and non-integer.
			Util::simd_ref arg1{ -1.0f, -1.0f, -2.0f, -2.5f };
			Util::simd_ref arg2{ Util::inf(), -Util::inf(), 2.5f, -2.5f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		
		{	// pow(base, ±0) returns 1 for any base, even when base is NaN.
			Util::simd_ref arg1{ 2.0f, Util::inf(), -Util::nan(), 0.0f};
			Util::simd_ref arg2{ +0.0f, -0.0f, +0.0f, -0.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(base, -∞) returns +∞ for any |base| < 1.
			// pow(base, -∞) returns +0 for any |base| > 1.
			Util::simd_ref arg1{ 0.3f, 0.7f, 1.2f, 12.5f };
			Util::simd_ref arg2{ -Util::inf(), -Util::inf(), -Util::inf(), -Util::inf() };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}

		{	// pow(base, +∞) returns +0 for any |base| < 1.
			// pow(base, +∞) returns +∞ for any |base| > 1.
			Util::simd_ref arg1{ 0.3f, 0.7f, 1.2f, 12.5f };
			Util::simd_ref arg2{ Util::inf(), Util::inf(), Util::inf(), Util::inf() };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}
		
		{	// pow(-∞, exp) returns -0 if exp is a negative odd integer.
			// pow(-∞, exp) returns +0 if exp is a negative non-integer or negative even integer.
			// pow(-∞, exp) returns -∞ if exp is a positive odd integer.
			Util::simd_ref arg1{ -Util::inf(), -Util::inf(), -Util::inf(), -Util::inf() };
			Util::simd_ref arg2{ -5.0f, -2.5f, -2.0f, 5.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}
		
		{	// pow(-∞, exp) returns +∞ if exp is a positive non-integer or positive even integer.
			Util::simd_ref arg1{ -Util::inf(), -Util::inf(), -Util::inf(), -Util::inf() };
			Util::simd_ref arg2{ 2.5f, 2.0f, -2.0f, 5.0f };
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}
		
		{	// pow(+∞, exp) returns +0 for any negative exp.
			// pow(+∞, exp) returns +∞ for any positive exp.
			Util::simd_ref arg1{ Util::inf(), Util::inf(), Util::inf(), Util::inf() };
			Util::simd_ref arg2{ -4.0f, -2.5f, 4.0f, 2.5f};
			simd result = pow(simd(arg1.data()), simd(arg2.data()));

			Util::simd_check expected(arg1, arg2, powf);
			expected.Tolerance = toleranceOverride;
			EXPECT_EQ(result, expected);
		}
	}

	TEST(SIMD, SinCorrect)
	{
		Util::simd_ref arg1{ -2.4f, 1.0f, 0.0f, 2.2f };
		simd result = sin(simd(arg1.data()));
		Util::simd_check expected{ arg1, sinf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, CosCorrect)
	{
		Util::simd_ref arg1{ -2.4f, 1.0f, 0.0f, 2.2f };
		simd result = cos(simd(arg1.data()));
		Util::simd_check expected{ arg1, cosf };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, SinCosCorrect)
	{
		Util::simd_ref arg1{ -2.4f, 1.0f, 0.3f, 2.8f };
		simd resultSin, resultCos;
		SinCos(simd(arg1.data()), resultSin, resultCos);
		Util::simd_check expectedSin{ arg1, sinf };
		Util::simd_check expectedCos{ arg1, cosf };
		EXPECT_EQ(resultSin, expectedSin);
		EXPECT_EQ(resultCos, expectedCos);
	}

	TEST(SIMD, TanCorrect)
	{
		{
			Util::simd_ref arg1{ -2.4f, 1.0f, 0.0f, 2.2f };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}

		static constexpr float pi = std::numbers::pi_v<float>;
		static constexpr float eps = 1e-4f; // small offset for "near-boundary" cases

		{	// Tiny / near zero
			Util::simd_ref arg1{ 1e-7f, -1e-7f, 1e-4f, -1e-4f };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Friendly angles
			Util::simd_ref arg1{ pi / 6,  -pi / 6, pi / 4, -pi / 4 };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Friendly angles
			Util::simd_ref arg1{ pi / 3,  -pi / 3, pi, -pi};
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Friendly angles (and 0, 1)
			Util::simd_ref arg1{ 2 * pi,  -2 * pi, 0.0f, 1.0f };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Near poles (Pi/2 + kPi) — approach from both sides
			Util::simd_ref arg1{ (pi / 2) - eps, (pi / 2) + eps,  (3 * pi / 2) - eps, (3 * pi / 2) + eps };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Near octant boundaries (k*Pi/4): hit just around the reducer’s sweet spots
			Util::simd_ref arg1{ (pi / 4) - eps, (pi / 4) + eps, (3 * pi / 4) - eps, (3 * pi / 4) + eps };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		{	// Large-magnitude inputs (stress argument reduction)
			Util::simd_ref arg1{ 1000.0f, -1000.0f, 1e5f * pi + 0.001f, -1e5f * pi + 0.001f };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
		// Around Pi * 1'000'000 is where our simd tan can't keep up with tanf
#if 0
		{	// Large-magnitude inputs (stress argument reduction)
			Util::simd_ref arg1{ 1e6f * pi + 0.001f, -1e6f * pi + 0.001f, 1e7f * pi + 0.001f, -1e7f * pi + 0.001f };
			simd result = tan(simd(arg1.data()));
			Util::simd_check expected{ arg1, tanf };
			EXPECT_EQ(result, expected) << "Test values: " << simd(arg1.data());
		}
#endif
	}

	TEST(SIMD, InterleaveCorrect)
	{
		simd a(Util::cTest0123.data());
		simd result = interleave_lohi(a);
		Util::simd_check expected{ 0.0f, 2.0f, 1.0f, 3.0f };
		EXPECT_EQ(result, expected);
	}

	TEST(SIMD, CombineLoHiCorrect)
	{
		{
			simd a(Util::cTest0123.data());
			simd b(4.0f, 5.0f, 6.0f, 7.0f);
			simd result = combine_lo(a, b);
			Util::simd_check expected{ 0.0f, 1.0f, 4.0f, 5.0f };
			EXPECT_EQ(result, expected);
		}
		{
			simd a(Util::cTest0123.data());
			simd b(4.0f, 5.0f, 6.0f, 7.0f);
			simd result = combine_hi(a, b);
			Util::simd_check expected{ 2.0f, 3.0f, 6.0f, 7.0f };
			EXPECT_EQ(result, expected);
		}
		{
			simd a(Util::cTest0123.data());
			simd b(4.0f, 5.0f, 6.0f, 7.0f);
			simd result = combine_lohi(a, b);
			Util::simd_check expected{ 0.0f, 1.0f, 6.0f, 7.0f };
			EXPECT_EQ(result, expected);
		}
	}

	TEST(SIMD, ReverseCorrect)
	{
		simd a(Util::cTest0123.data());
		simd result = reverse(a);
		Util::simd_check expected{ 3.0f, 2.0f, 1.0f, 0.0f };
		EXPECT_EQ(result, expected);
	}

} // namespace JPL