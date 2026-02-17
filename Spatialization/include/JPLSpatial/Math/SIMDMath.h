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
#include "JPLSpatial/Math/SIMD.h"

#include <numbers>
#include <concepts>

namespace JPL
{
	static_assert(simd::size() == 4 && "The SIMD counter math below assumes simd::size() is 4.");

	/// Floor `count` to 4-wide simd vector
	[[nodiscard]] JPL_INLINE constexpr auto FloorToSIMDSize(std::unsigned_integral auto count) noexcept
	{
		return count & 0xFFFFFFFC;
	}

	/// Floor `count` to 8-wide simd vector
	[[nodiscard]] JPL_INLINE constexpr auto FloorToSIMDSizeDouble(std::unsigned_integral auto count) noexcept
	{
		return count & 0xFFFFFFF8;
	}

	/// Get the remaining tail from `count` that won't fill a simd vector
	[[nodiscard]] JPL_INLINE constexpr auto GetSIMDTail(std::unsigned_integral auto count) noexcept
	{
		return count & 3;
	}

	/// Get number of SIMD operations that can fit into the `count`
	[[nodiscard]] JPL_INLINE constexpr auto GetNumSIMDOps(std::unsigned_integral auto count) noexcept
	{
		return count >> 2;
	}

	/// Get number of 8-wide simd operations that can fit into the `count`
	[[nodiscard]] JPL_INLINE constexpr auto GetNumSIMDOpsDouble(std::unsigned_integral auto count) noexcept
	{
		return count >> 3;
	}

	/// Get the remaining tail from `count` that won't fill 8-wide simd vector
	[[nodiscard]] JPL_INLINE constexpr auto GetSIMDTailDouble(std::unsigned_integral auto count) noexcept
	{
		return count & 7;
	}

	/// Round `count` up to the next simd width
	[[nodiscard]] JPL_INLINE constexpr auto RoundUpToSIMD(std::unsigned_integral auto count) noexcept
	{
		return (count + 3) >> 2;
	}

	/// Floor `count` to divisible by 2
	[[nodiscard]] JPL_INLINE constexpr auto FloorToDiv2(std::unsigned_integral auto count) noexcept
	{
		return count & (~1llu);
	}

	/// @returns 1 if `count` is uneven, 0 otherwise
	[[nodiscard]] JPL_INLINE constexpr auto GetDiv2Tail(std::unsigned_integral auto count) noexcept
	{
		return count & 1;
	}

	//==========================================================================
	/// Declare some SIMD constants

#define JPL_FL_CONSTANT(Name, Val)\
	JPL_INLINE simd c_##Name() noexcept { return simd(Val);; } 

#define JPL_INT_CONSTANT(Name, Val)\
	JPL_INLINE simd_mask c_i##Name() noexcept { return simd_mask(static_cast<uint32>(Val)); }

#define JPL_MASK_CONSTANT(Name, Val)\
	JPL_INLINE simd_mask c_##Name() noexcept { return simd_mask(static_cast<uint32>(Val)); }

	namespace constant
	{
		// the smallest non denormalized float number
		JPL_MASK_CONSTANT(min_norm_pos, 0x00800000);
		JPL_MASK_CONSTANT(mant_mask, 0x7f800000);
		JPL_MASK_CONSTANT(inv_mant_mask, ~0x7f800000);

		JPL_MASK_CONSTANT(sign_mask, 0x80000000);
		JPL_MASK_CONSTANT(inv_sign_mask, ~0x80000000);

		JPL_INT_CONSTANT(1, 1);
		JPL_INT_CONSTANT(inv1, ~1);
		JPL_INT_CONSTANT(2, 2);
		JPL_INT_CONSTANT(4, 4);
		JPL_INT_CONSTANT(0x7f, 0x7f);

		JPL_FL_CONSTANT(pi, std::numbers::pi_v<float>);
		JPL_FL_CONSTANT(half_pi, std::numbers::pi_v<float> * 0.5f);
	}

	//==========================================================================
	/// Equivalent to std::signbit but for 4-wide 32-bit float vector
	JPL_INLINE simd_mask signbit(const simd& vec) noexcept
	{
		return vec.as_mask() & constant::c_sign_mask();
	}

	/// Determines if the given floating-point number num is a positive or negative infinity
	JPL_INLINE simd_mask isinf(const simd& vec) noexcept
	{
		return abs(vec) == simd::inf();
	}

	/// Determines if the given floating point number `vec` is a not-a-number(NaN) value
	JPL_INLINE simd_mask isnan(const simd& vec) noexcept
	{
		return vec != vec;
	}

	/// Computes the nearest integer not greater in magnitude than `vec`.
	JPL_INLINE simd trunc(const simd& vec) noexcept
	{
		simd_mask sign_bit = signbit(vec);
		// Return original value for any the checks that are true
		simd vec_abs = abs(vec);
		simd_mask is_inf = vec_abs == simd::inf();
		simd_mask is_zero = vec_abs == simd::zero();
		simd_mask is_nan = isnan(vec_abs);
		return simd::select(is_inf | is_zero | is_nan,
							vec,
							vec_abs.to_mask().to_simd() | sign_bit.as_simd());
	}

	//==========================================================================
	/// Log constants
	namespace cephes
	{
		JPL_FL_CONSTANT(SQRTHF, 0.707106781186547524f);
		JPL_FL_CONSTANT(log_p0, 7.0376836292e-2f);
		JPL_FL_CONSTANT(log_p1, -1.1514610310e-1f);
		JPL_FL_CONSTANT(log_p2, 1.1676998740e-1f);
		JPL_FL_CONSTANT(log_p3, -1.2420140846e-1f);
		JPL_FL_CONSTANT(log_p4, +1.4249322787e-1f);
		JPL_FL_CONSTANT(log_p5, -1.6668057665e-1f);
		JPL_FL_CONSTANT(log_p6, +2.0000714765e-1f);
		JPL_FL_CONSTANT(log_p7, -2.4999993993e-1f);
		JPL_FL_CONSTANT(log_p8, +3.3333331174e-1f);
		JPL_FL_CONSTANT(log_q1, -2.12194440e-4f);
		JPL_FL_CONSTANT(log_q2, 0.693359375f);
	}

	namespace logarithm
	{
		// Evaluat log polynomial
		JPL_INLINE simd polynomial(const simd& x) noexcept
		{
			simd y = cephes::c_log_p0();
			y = fma(y, x, cephes::c_log_p1());
			y = fma(y, x, cephes::c_log_p2());
			y = fma(y, x, cephes::c_log_p3());
			y = fma(y, x, cephes::c_log_p4());
			y = fma(y, x, cephes::c_log_p5());
			y = fma(y, x, cephes::c_log_p6());
			y = fma(y, x, cephes::c_log_p7());
			y = fma(y, x, cephes::c_log_p8());
			y *= x;
			return y;
		}
	}

	//==========================================================================
	/// Natural logarithm computed for 4-wide 32-bit float vector
	/// return NaN for x <= 0
	inline simd log(simd x) noexcept
	{
		simd invalid_mask = (x <= simd::zero()).as_simd();

		x = max(x, constant::c_min_norm_pos().as_simd()); // cut off denormalized stuff

		// part 1: x = frexpf(x, &e);
		const simd_mask emm0 = (x.as_mask() >> 23) - constant::c_i0x7f();
		
		// now e contains the really base-2 exponent
		simd e = emm0.to_simd() + simd::c_1();

		// keep only the fractional part
		x &= constant::c_inv_mant_mask().as_simd();
		x |= simd::c_0p5();

		/* part2:
		   if( x < SQRTHF ) {
			 e -= 1;
			 x = x + x - 1.0;
		   } else { x = x - 1.0; }
		*/
		const simd mask = (x < cephes::c_SQRTHF()).as_simd();
		e -= (simd::c_1() & mask);
		x += (x & mask) - simd::c_1();

		// Evaluate polynomial
		simd y = logarithm::polynomial(x);

		const simd x2 = x * x;
		y = fma(e, cephes::c_log_q1(), y * x2) - x2 * simd::c_0p5();
		x = fma(e, cephes::c_log_q2(), x + y);
		return x | invalid_mask; // negative arg will be NAN
	}

	//==========================================================================
	/// Log10 constants
	namespace constant
	{
		JPL_FL_CONSTANT(ln10, 2.3025850930f);
		JPL_FL_CONSTANT(inv_ln10, 0.4342944819f);
		JPL_FL_CONSTANT(ln2, 0.69314718056f);
		JPL_FL_CONSTANT(inv_ln2, 1.4426950216f);

		// Useful for conversion to decibels: pow(10, -energy / 20) == exp(ln10div20, -energy)
		JPL_FL_CONSTANT(ln10div20, 0.11512925465f);
	}

	//==========================================================================
	/// log10 for 4-wide 32-bit float vector
	JPL_INLINE simd log10(const simd& vec) noexcept
	{
		return log(vec) * constant::c_inv_ln10();
	}

	/// log2 for 4-wide 32-bit float vector
	JPL_INLINE simd log2(const simd& vec) noexcept
	{
		return log(vec) * constant::c_inv_ln2();
	}

	//==========================================================================
	/// Exp constants
	namespace constant
	{
		JPL_FL_CONSTANT(exp_hi, 88.3762626647949f);
		JPL_FL_CONSTANT(exp_lo, -88.3762626647949f);
	}
	namespace cephes
	{
		JPL_FL_CONSTANT(LOG2EF, 1.44269504088896341f);
		JPL_FL_CONSTANT(exp_C1, 0.693359375f);
		JPL_FL_CONSTANT(exp_C2, -2.12194440e-4f);

		JPL_FL_CONSTANT(exp_p0, 1.9875691500e-4f);
		JPL_FL_CONSTANT(exp_p1, 1.3981999507e-3f);
		JPL_FL_CONSTANT(exp_p2, 8.3334519073e-3f);
		JPL_FL_CONSTANT(exp_p3, 4.1665795894e-2f);
		JPL_FL_CONSTANT(exp_p4, 1.6666665459e-1f);
		JPL_FL_CONSTANT(exp_p5, 5.0000001201e-1f);
	}

	namespace exponent
	{
		// Exponent polynomial on the small remainder (Cephes form: 1 + x + x^2 * P(x))
		JPL_INLINE simd polynomial(const simd& x) noexcept
		{
			simd x2 = x * x;
			simd y = cephes::c_exp_p0();
			y = fma(y, x, cephes::c_exp_p1());
			y = fma(y, x, cephes::c_exp_p2());
			y = fma(y, x, cephes::c_exp_p3());
			y = fma(y, x, cephes::c_exp_p4());
			y = fma(y, x, cephes::c_exp_p5());
			y = fma(y, x2, x + simd::c_1());
			return y;
		}

		// Build 2^n via exponent bits
		JPL_INLINE simd build2pown(const simd& n)
		{
			return ((n.to_mask() + constant::c_i0x7f()) << 23).as_simd();
		}
	}

	//==========================================================================
	/// Exponent for 4-wide 32-bit float vector
	inline simd exp(simd x) noexcept
	{
		x = clamp(x, constant::c_exp_lo(), constant::c_exp_hi());

		// Express exp(x) as exp(g + n*log(2))

		// n = floor(x * LOG2E + 0.5)  (round to nearest integer)
		simd n = floor(fma(x, cephes::c_LOG2EF(), simd::c_0p5()));

		// Remainder in natural-log space using split ln2 = C1 + C2 (better precision)
		x -= fma(n, cephes::c_exp_C1(), n * cephes::c_exp_C2());

		// Evaluate polynomial
		simd y = exponent::polynomial(x);

		// Build 2^n
		simd pow2n = exponent::build2pown(n);
		return y * pow2n;
	}

	//==========================================================================
	JPL_INLINE simd exp2(simd x) noexcept
	{
		// Range reduction: x = n + r, where r ~= [-0.5, 0.5]
		simd n = floor(x + simd::c_0p5());
		simd r = x - n;

		// Convert the small base-2 remainder to natural-log space:
		// (x): r_ln2 ~= r * ln(2) using split constants for precision
		x = fma(r, cephes::c_exp_C1(), r * cephes::c_exp_C2());

		// Reuse exp polynomial exactly as in exp():
		simd y = exponent::polynomial(x);

		// Build 2^n
		simd pow2n = exponent::build2pown(n);
		return y * pow2n;
	}
	
	/// Multiplies a floating-point value `arg` by the number 2 raised to the `exp` power.
	JPL_INLINE simd ldexp(simd arg, simd_mask exp) noexcept
	{
		// m = sign(q) ? -1 : 0
		simd_mask m = exp.ashr<31>();

		m = (((m.adds(exp)).ashr<6>()).subs(m)) << 4;
		exp = exp.subs(m << 2);

		m = m.adds(127);
		m = clamps(m, simd_mask::zero(), simd_mask::replicate(255));

		simd u = (m << 23).as_simd();
		// x *= u^4
		simd u2 = u * u;
		arg = arg * u2 * u2;
		u = ((exp.adds(0x7f)) << 23).as_simd();
		return arg * u;
	}

	namespace power
	{
		// Flip x's sign bit where y is negative
		JPL_INLINE simd mulsign(const simd& x, const simd& y) noexcept
		{
			return (x.as_mask() ^ (y.as_mask() & constant::c_sign_mask())).to_simd();
		}

		// Return 1.0f with the sign of `x`
		JPL_INLINE simd sign(const simd& x) noexcept
		{
			return mulsign(1.0f, x);
		}
	}

	inline simd pow(const simd& x, const simd& y) noexcept
	{
		simd x_abs = abs(x);

		// Compute pow
		simd result = exp(y * log(x_abs));

		// The rest is error handling...

		// Inline `trunc(y)` to reuse nubmer tests
		simd_mask y_sign_bit = signbit(y);
		simd y_abs = abs(y);
		simd_mask y_is_inf = y_abs == simd::inf();
		simd_mask y_is_zero = y_abs == simd::zero();
		simd_mask y_is_nan = isnan(y_abs);
		simd y_trunc = simd::select(y_is_inf | y_is_zero | y_is_nan,
							y,
							y_abs.to_mask().to_simd() | y_sign_bit.as_simd());

		// y is integer if y == trunc(y) OR |y| >= 2^24 (float can exactly represent all ints below 2^24)
		simd_mask y_is_in_24bit_range = y_abs < simd(static_cast<float>(1 << 24));
		simd_mask y_is_int = (y == y_trunc) | (~y_is_in_24bit_range);

		//int yisodd = (1 & (int)y) != 0 && yisint && fabsfk(y) < (float)(INT64_C(1) << 24);
		simd_mask y_is_odd = ((1 & y_abs.to_mask()) != simd_mask::zero()) & y_is_int & (y_is_in_24bit_range);

		simd_mask x_is_inf = isinf(x);
		simd_mask x_is_nan = isnan(x);
		simd_mask x_is_gte_zero = x >= 0.0f;
		simd_mask x_is_zero = x == simd::c_0();

		// Sleef: turn internal NaN-overflow to +inf
		//result = simd::select(isnan(result), simd::inf(), result);

		// handle x < 0: if y is not integer -> NaN; else apply sign for odd exponents
		simd result_signed = simd::select(y_is_odd, -result, result);
		result = simd::select(x_is_gte_zero,
							  result,
							  simd::select(~y_is_int,
										   simd::nan() | signbit(x).as_simd(),
										   result_signed));

		// efx path for y = +-inf
		simd efx = power::mulsign(x_abs - simd::c_1(), y);
		result = simd::select(y_is_inf,
							  simd::select(efx < simd::c_0(),
										   simd::c_0(),
										   simd::select(efx == simd::c_0(),
														simd::c_1(),
														simd::inf())),
							  result);

		// x is inf or 0
		{
			simd factor = simd::select(y_is_odd,
									   power::sign(x),
									   simd::c_1());

			simd pos0_or_inf = simd::select(simd::select(x_is_zero, -y, y) < simd::c_0(),
											simd::c_0(),
											simd::inf());

			result = simd::select(x_is_inf | x_is_zero, factor * pos0_or_inf, result);
		}

		// NaNs
		result = simd::select(x_is_nan | y_is_nan, simd::nan(), result);

		// y==0 or x==1
		simd_mask x_factor = (x == simd::c_1()) | ((x == -simd::c_1()) & y_is_inf);
		result = simd::select((y == simd::c_0()) | x_factor, simd::c_1(), result);

		return result;
	}

	//==========================================================================
	/// Sin and Cos constants
	namespace cephes
	{
		JPL_FL_CONSTANT(minus_DP1, -0.78515625f);
		JPL_FL_CONSTANT(minus_DP2, -2.4187564849853515625e-4f);
		JPL_FL_CONSTANT(minus_DP3, -3.77489497744594108e-8f);
		JPL_FL_CONSTANT(sincof_p0, -1.9515295891e-4f);
		JPL_FL_CONSTANT(sincof_p1, 8.3321608736e-3f);
		JPL_FL_CONSTANT(sincof_p2, -1.6666654611e-1f);
		JPL_FL_CONSTANT(coscof_p0, 2.443315711809948e-5f);
		JPL_FL_CONSTANT(coscof_p1, -1.388731625493765e-3f);
		JPL_FL_CONSTANT(coscof_p2, 4.166664568298827e-2f);
		JPL_FL_CONSTANT(FOPI, 1.27323954473516f); // 4 / M_PI
	}

	namespace sincos
	{
		// Evaluate sin/cos polynomials
		JPL_INLINE void polynomials(const simd& x, simd& outTaylorCos, simd& outTaylorSin) noexcept
		{
			// Compute x^2
			simd x2 = x * x;

			// Evaluate the first polynom  (0 <= x <= Pi/4)
			// Cos(x) = 1 - x^2/2! + x^4/4! - x^6/6! + x^8/8! + ... = (((x2/8!- 1/6!) * x2 + 1/4!) * x2 - 1/2!) * x2 + 1
			outTaylorCos = fma(cephes::c_coscof_p0(), x2, cephes::c_coscof_p1());
			outTaylorCos = fma(outTaylorCos, x2, cephes::c_coscof_p2());
			outTaylorCos = fma(outTaylorCos, x2 * x2, -(simd::c_0p5() * x2) + simd::c_1());

			// Evaluate the second polynom  (Pi/4 <= x <= 0)
			// Sin(x) = x - x^3/3! + x^5/5! - x^7/7! + ... = ((-x2/7! + 1/5!) * x2 - 1/3!) * x2 * x + x
			outTaylorSin = fma(cephes::c_sincof_p0(), x2, cephes::c_sincof_p1());
			outTaylorSin = fma(outTaylorSin, x2, cephes::c_sincof_p2());
			outTaylorSin = fma(outTaylorSin, x2 * x, x);
		}
	}
	namespace Math
	{
		//==========================================================================
		/// Sin and Cos for 4-wide 32-bit float vector
		inline void SinCos(simd x, simd& outSin, simd& outCos) noexcept
		{
			// Adopted from "Simple SSE and SSE2 (and now NEON) optimized sin, cos, log and exp"
			// by Julien Pommier (http://gruntthepeon.free.fr/ssemath/),
			// and adapted to C++.
			// 
			// Some concepts referenced from JoltPhysics library
			// by Jorrit Rouwe (https://github.com/jrouwe/JoltPhysics)

			// Make argument positive and remember sign for sin only
			// since cos is symmetric around x (highest bit of a float is the sign bit)
			simd_mask sin_sign_bit = signbit(x); // extract the sign bit (upper one)
			x = abs(x); // take the absolute value

			// Store the integer part of y in quadrant
			// x * cephes::c_FOPI(): scale by 4/Pi
			simd_mask quadrant = fma(cephes::c_FOPI(), x, constant::c_i1().to_simd()).to_mask() & constant::c_iinv1();
			simd float_quadrant = quadrant.to_simd();

			// The magic pass: "Extended precision modular arithmetic"
			// x = ((x - y * DP1) - y * DP2) - y * DP3;
			x = fma(float_quadrant, cephes::c_minus_DP1(), x);
			x = fma(float_quadrant, cephes::c_minus_DP2(), x);
			x = fma(float_quadrant, cephes::c_minus_DP3(), x);

			// Evaluate polynomials
			simd taylor_cos, taylor_sin;
			sincos::polynomials(x, taylor_cos, taylor_sin);

			// Get the polynom selection mask for the sine
			simd_mask mask = (quadrant & constant::c_i2()) == simd_mask::zero();

			// Select which one of the results is sin and which one is cos
			simd s = simd::select(mask, taylor_sin, taylor_cos);
			simd c = simd::select(mask, taylor_cos, taylor_sin);

			// Update the signs
			simd_mask bit1 = quadrant << 30;
			simd_mask bit2 = ((quadrant << 29) & constant::c_sign_mask());

			sin_sign_bit = (sin_sign_bit ^ bit2);
			simd_mask sign_bit_cos = bit1 ^ bit2;

			// Correct the signs
			outSin = (s ^ sin_sign_bit.as_simd());
			outCos = (c ^ sign_bit_cos.as_simd());
		}
	} // namespace Math

	//==========================================================================
	/// Sin implementation for 4-wide 32-bit flaot vector
	/// Note: this function islmost identical to SinCos, where you get Cos almost for free.
	inline simd sin(simd x) noexcept
	{
		// Make argument positive and remember sign for sin only
		// since cos is symmetric around x (highest bit of a float is the sign bit)
		simd_mask sin_sign_bit = signbit(x); // extract the sign bit (upper one)
		x = abs(x); // take the absolute value

		// Store the integer part of y in quadrant
		// x * cephes::c_FOPI(): scale by 4/Pi
		simd_mask quadrant = fma(cephes::c_FOPI(), x, constant::c_i1().to_simd()).to_mask() & constant::c_iinv1();
		simd float_quadrant = quadrant.to_simd();

		// The magic pass: "Extended precision modular arithmetic"
		// x = ((x - y * DP1) - y * DP2) - y * DP3;
		x = fma(float_quadrant, cephes::c_minus_DP1(), x);
		x = fma(float_quadrant, cephes::c_minus_DP2(), x);
		x = fma(float_quadrant, cephes::c_minus_DP3(), x);

		// Evaluate polynomials
		simd taylor_cos, taylor_sin;
		sincos::polynomials(x, taylor_cos, taylor_sin);

		// Get the polynom selection mask for the sine
		simd_mask mask = (quadrant & constant::c_i2()) == simd_mask::zero();

		// Select which one of the results is sin and which one is cos
		simd s = simd::select(mask, taylor_sin, taylor_cos);

		// Update the sign
		simd_mask bit2 = ((quadrant << 29) & constant::c_sign_mask());
		sin_sign_bit = (sin_sign_bit ^ bit2);

		// Correct the sign
		return (s ^ sin_sign_bit.as_simd());
	}

	//==========================================================================
	/// Cos implementation for 4-wide 32-bit flaot vector
	/// Note: this function islmost identical to SinCos, where you get Sin almost for free.
	inline simd cos(simd x) noexcept
	{
		// Make argument positive and remember sign for sin only
		// since cos is symmetric around x (highest bit of a float is the sign bit)
		//simd_mask sin_sign_bit = signbit(x); // extract the sign bit (upper one)
		x = abs(x); // take the absolute value

		// Store the integer part of y in quadrant
		// x * cephes::c_FOPI(): scale by 4/Pi
		simd_mask quadrant = fma(cephes::c_FOPI(), x, constant::c_i1().to_simd()).to_mask() & constant::c_iinv1();
		simd float_quadrant = quadrant.to_simd();

		// The magic pass: "Extended precision modular arithmetic"
		// x = ((x - y * DP1) - y * DP2) - y * DP3;
		x = fma(float_quadrant, cephes::c_minus_DP1(), x);
		x = fma(float_quadrant, cephes::c_minus_DP2(), x);
		x = fma(float_quadrant, cephes::c_minus_DP3(), x);

		// Evaluate polynomials
		simd taylor_cos, taylor_sin;
		sincos::polynomials(x, taylor_cos, taylor_sin);

		// Get the polynom selection mask for the sine
		simd_mask mask = (quadrant & constant::c_i2()) == simd_mask::zero();

		// Select which one of the results is sin and which one is cos
		simd c = simd::select(mask, taylor_cos, taylor_sin);

		// Update the sign
		simd_mask bit1 = quadrant << 30;
		simd_mask bit2 = ((quadrant << 29) & constant::c_sign_mask());

		simd_mask sign_bit_cos = bit1 ^ bit2;

		// Correct the sign
		return (c ^ sign_bit_cos.as_simd());
	}

	namespace constant
	{
		JPL_FL_CONSTANT(invPIO2, 0.63661977236758134308f);
	}
	namespace tangent
	{
		// Negated for FMA transposition
		JPL_FL_CONSTANT(minus_DP1, -1.5703125f); // split pi/2
		JPL_FL_CONSTANT(minus_DP2, -0.0004837512969970703125f);
		JPL_FL_CONSTANT(minus_DP3, -7.549789948768648e-8f);

		JPL_INLINE simd polynomial(simd x)
		{
			const simd x2 = x * x;
			simd y = fma(simd(9.38540185543e-3f), x2, simd(3.11992232697e-3f));
			y = fma(y, x2, simd(2.44301354525e-2f));
			y = fma(y, x2, simd(5.34112807005e-2f));
			y = fma(y, x2, simd(1.33387994085e-1f));
			y = fma(y, x2, simd(3.33331568548e-1f));
			y = fma(y, x2 * x, x);
			return y;
		}
	}

	/// Calculate the tangent for each element of the vector (input in radians).
	/// Valid for input values in range +-Pi*100'000
	inline simd tan(simd x) noexcept
	{
		// n = round(x * 2/pi)
		// REQUIRE: round() is round-to-nearest (banker's or away-from-zero both OK here)
		const simd n_f = round(x * constant::c_invPIO2());

		// Reduce to r to [-pi/4, pi/4] with high-precision split
		// r = ((x - y * DP1) - y * DP2) - y * DP3;
		simd r = fma(n_f, tangent::c_minus_DP1(), x);
		r = fma(n_f, tangent::c_minus_DP2(), r);
		r = fma(n_f, tangent::c_minus_DP3(), r);

		// Polynomial on r
		simd tan = tangent::polynomial(r);

		// odd quadrants? tan(x) = -1 / tan(r)
		// Convert n_f (which is an exact integer in float) to int lanes safely
		const simd_mask n = n_f.to_mask(); // float->int32 trunc fine since n_f is already integer
		const simd_mask is_odd = (n & simd_mask(1u)) != simd_mask(0u);

		tan = simd::select(is_odd, -simd::reciprocal(tan), tan);
		return tan;
	}

	namespace asinacos
	{
		JPL_INLINE simd polynomial(const simd& x, const simd& x2)
		{
			simd y = +0.4197454825e-1f;
			y = fma(y, x2, +0.2424046025e-1f);
			y = fma(y, x2, +0.4547423869e-1f);
			y = fma(y, x2, +0.7495029271e-1f);
			y = fma(y, x2, +0.1666677296e+0f);
			y = fma(y, x * x2, x);
			return y;
		}
	}

	/// Calculate the arc sine for each element of this vector (returns value in the range [-PI / 2, PI / 2])
	/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::asin
	inline simd asin(const simd& in)
	{
		const simd_mask asin_sign = signbit(in);
		simd a = abs(in);
		
		// asin is not defined outside the range [-1, 1] but it often happens that a value is slightly above 1 so we just clamp here
		a = min(a, simd::c_1());

		// When |x| < 0.5 we use the asin approximation
		// When |x| >= 0.5 we use the identity asin(x) = PI / 2 - 2 * asin(sqrt((1 - x) / 2))
		const simd_mask o = a < simd::c_0p5();

		const simd x2 = simd::select(o, (a * a), ((simd::c_1() - a) * 0.5f));
		const simd x = simd::select(o, a, Math::Sqrt(x2));

		// Polynomial approximation of asin
		const simd u = asinacos::polynomial(x, x2);
		
		// If |x| >= 0.5 we need to apply the remainder of the identity above
		const simd r = simd::select(o, u, (constant::c_half_pi ()- (u + u)));

		// Put the sign back
		return r ^ asin_sign.as_simd();
	}

	/// Calculate the arc cosine for each element of this vector (returns value in the range [0, PI])
	/// Note that all input values will be clamped to the range [-1, 1] and this function will not return NaNs like std::acos
	inline simd acos(const simd& in)
	{
#if 1
		// Not super accurate, but good enough
		return constant::c_half_pi ()- asin(in);
#else
		// See comments in `asin()`...

		const simd_mask asin_sign = signbit(in);
		simd a = abs(in);
		a = min(a, simd::c_1());
		const simd_mask o = a < simd::c_0p5;

		const simd x2 = simd::select(o, (a * a), ((simd::c_1() - a) * 0.5f));
		const simd x = simd::select(a == simd::c_1(),
									simd::c_0,
									simd::select(o, a, Math::Sqrt(x2)));

		const simd u = asinacos::polynomial(x, x2);
		const simd y = constant::c_half_pi ()- (u ^ asin_sign.as_simd());
		
		simd r = simd::select(o, y, (u + u));
		r = simd::select((~o) & (in < simd::c_0), constant::c_pi ()- r, r);

		return r;
#endif
	}

	// TODO: atan

	namespace arctangent2
	{
		inline simd polynomial(simd x)
		{
			// Store the coefficients
			const simd a1(0.99997726f);
			const simd a3(-0.33262347f);
			const simd a5(0.19354346f);
			const simd a7(-0.11643287f);
			const simd a9(0.05265332f);
			const simd a11(-0.01172120f);

			// Compute the polynomial
			simd x2 = x * x;
			simd result;
			result = a11;
			result = fma(x2, result, a9);
			result = fma(x2, result, a7);
			result = fma(x2, result, a5);
			result = fma(x2, result, a3);
			result = fma(x2, result, a1);
			result *= x;

			return result;
		}
	}

	inline simd atan2(const simd& y, const simd& x)
	{
		// Adopted from AVX implementaion in "Speeding up atan2f by 50x"
		// https://mazzo.li/posts/vectorized-atan2.html

		const simd_mask swap_mask = abs(y) > abs(x);
		
		// Create the atan input by "blending" `y` and `x`, according to the mask computed
		// above. In our case we need the number of larger magnitude to
		// be the denominator.
		const simd atan_input =
			simd::select(swap_mask, x, y) / // pick the lowest between |y| and |x| for each number
			simd::select(swap_mask, y, x);  // and the highest.
			
		// Approximate atan
		simd result = arctangent2::polynomial(atan_input);

		// If swapped, adjust atan output. We use blending again to leave
		// the output unchanged if we didn't swap anything.
		//
		// If we need to adjust it, we simply carry the sign over from the input
		// to `pi_2` by using the `sign_mask`. This avoids a more expensive comparison,
		// and also handles edge cases such as -0 better.
		result = simd::select(swap_mask,
							  (constant::c_half_pi ()| signbit(atan_input).as_simd()) - result,
							  result);

		// Adjust the result depending on the input quadrant.
		//
		// We create a mask for the sign of `x` using an arithmetic right shift:
		// the mask will be all 0s if the sign if positive, and all 1s
		// if the sign is negative. This avoids a further (and slower) comparison
		// with 0.
		const simd x_sign_mask = x.as_mask().ashr<31>().as_simd();
		
		// Then use the mask to perform the adjustment only when the sign
		// is positive, and use the sign bit of `y` to know whether to add
		// `pi` or `-pi`.
		result += ((constant::c_pi ()^ signbit(y).as_simd()) & x_sign_mask);

		return result;
	}

} // namespace JPL

#undef JPL_FL_CONSTANT
#undef JPL_INT_CONSTANT
#undef JPL_MASK_CONSTANT
