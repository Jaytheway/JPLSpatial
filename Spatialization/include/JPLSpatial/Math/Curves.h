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

#include <cmath>
#include <numbers>
#include <algorithm>

namespace JPL
{
	namespace Curve
	{
		struct Point
		{
			float X;
			float Y;
		};

		enum class EType
		{
			Constant,
			Linear,
			SineFadeIn,
			SineFadeOut,
			SCurve,
			//InvertedSCurve,
			LogarithmicBase1_41,
			LogarithmicBase3,
			ExponentialBase1_41,
			ExponentialBase3
		};

		namespace Internal
		{
			static constexpr float halfPi = std::numbers::pi_v<float> *0.5f;
			static const float log1_41 = std::log(1.41f);
			static const float log1_41_inv = 1.0f / log1_41;
			static const float log3 = std::log(3.0f);
			static const float log3_inv = 1.0f / log3;
		}

		JPL_INLINE float EvaluateConstant(float t)				{ return 1.0f; }
		JPL_INLINE float EvaluateLinear(float t)				{ return t; }
		JPL_INLINE float EvaluateSineFadeIn(float t)			{ return std::sin(t * Internal::halfPi); }
		JPL_INLINE float EvaluateSineFadeOut(float t)			{ return 1.0f - std::cos(t * Internal::halfPi); }
		JPL_INLINE float EvaluateSCurve(float t)				{ return t * t * (3 - 2 * t); }
		JPL_INLINE float EvaluateLogarithmicBase1_41(float t)	{ return std::log(1.0f + 0.41f * t) * Internal::log1_41_inv; }
		JPL_INLINE float EvaluateLogarithmicBase3(float t)		{ return std::log(1.0f + 2.0f * t) * Internal::log3_inv; }
		JPL_INLINE float EvaluateExponentialBase1_41(float t)	{ static constexpr float inv_0_41 = 1.0f / 0.41f; return (std::pow(1.41f, t) - 1.0f) * inv_0_41; }
		JPL_INLINE float EvaluateExponentialBase3(float t)		{ return(std::pow(3.0f, t) - 1.0f) * 0.5f; }

		JPL_INLINE float EvaluateCurve(EType type, float t)
		{
			switch (type)
			{
			case EType::Constant:				return EvaluateConstant(t);
			case EType::Linear:					return EvaluateLinear(t);
			case EType::SineFadeIn:				return EvaluateSineFadeIn(t);
			case EType::SineFadeOut:			return EvaluateSineFadeOut(t);
			case EType::SCurve:					return EvaluateSCurve(t);
			case EType::LogarithmicBase1_41:	return EvaluateLogarithmicBase1_41(t);
			case EType::LogarithmicBase3:		return EvaluateLogarithmicBase3(t);
			case EType::ExponentialBase1_41:	return EvaluateExponentialBase1_41(t);
			case EType::ExponentialBase3:		return EvaluateExponentialBase3(t);
			default:
				return t;
			}
		}
	} // namespace Curve
} // namespace JPL