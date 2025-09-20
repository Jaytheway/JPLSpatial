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

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <cmath>
#include <numbers>
#include <limits>

namespace JPL::Internal
{
	template<CVec3Accessible Vec3>
	using FloatOf = std::remove_cvref_t<decltype(GetX(std::declval<Vec3>()))>;
}

namespace JPL::Math
{
	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE constexpr bool IsNearlyEqual(const Vec3& a,
														  const Vec3& b,
														  Internal::FloatOf<Vec3> tolerance = JPL_FLOAT_EPS_V<Internal::FloatOf<Vec3>>) noexcept
	{
		return IsNearlyEqual(GetX(a), GetX(b), tolerance)
			&& IsNearlyEqual(GetY(a), GetY(b), tolerance)
			&& IsNearlyEqual(GetZ(a), GetZ(b), tolerance);
	}

	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE constexpr bool IsNearlyZero(const Vec3& a,
														 Internal::FloatOf<Vec3> tolerance = JPL_FLOAT_EPS_V<Internal::FloatOf<Vec3>>) noexcept
	{
		return IsNearlyEqual(a, Vec3(0, 0, 0));
	}

	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE bool HasNans(const Vec3& vec)
	{
		return std::isnan(GetX(vec))
			|| std::isnan(GetY(vec))
			|| std::isnan(GetZ(vec));
	}

	template<CVec3Accessible Vec3>
	[[nodiscard]] inline Vec3 GetNormalizedPerpendicular(const Vec3& vec) noexcept
	{
		using FloatType = Internal::FloatOf<Vec3>;

		if (Abs(GetX(vec)) > Abs(GetY(vec)))
		{
			const FloatType len = Math::Sqrt(GetX(vec) * GetX(vec) + GetZ(vec) * GetZ(vec));
			return Vec3(GetZ(vec), FloatType(0.0), -GetX(vec)) / len;
		}
		else
		{
			const FloatType len = Math::Sqrt(GetY(vec) * GetY(vec) + GetZ(vec) * GetZ(vec));
			return Vec3(FloatType(0.0), GetZ(vec), -GetY(vec)) / len;
		}
	};

	template<CVec3Accessible Vec3>
	inline void CreateOrthonormalBasis(const Vec3& normal, Vec3& tangent, Vec3& bitangent) noexcept
	{
		using FloatType = Internal::FloatOf<Vec3>;

		if (Abs(GetX(normal)) > Abs(GetY(normal)))
		{
			// normal.X is largest component
			tangent = Vec3(-GetZ(normal), FloatType(0.0), GetX(normal));
		}
		else
		{
			// normal.Y is largest component
			tangent = Vec3(FloatType(0.0), GetZ(normal), -GetY(normal));
		}
		Normalize(tangent);
		bitangent = CrossProduct(normal, tangent); // right-handed basis
	}

	/// Rotate a 3D vector using Rodrigues' formula
	/// @param v : vector to rotate
	/// @param k : axis to rotate 'v' around
	/// @param theta : angle to rotate 'v' around 'k' axis
	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE Vec3 RotateVector(const Vec3& vector, const Vec3& rotationAxis, float angleRad) noexcept
	{
		const auto [sinTheta, cosTheta] = Math::SinCos(angleRad);
		return vector * cosTheta
			+ CrossProduct(rotationAxis, vector) * sinTheta
			+ rotationAxis * DotProduct(rotationAxis, vector) * (1.0f - cosTheta);
	}

	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE Vec3 Lerp(const Vec3& v0, const Vec3& v1, float t) noexcept
	{
		return v0 + t * (v1 - v0);
	}

	/// Linearly interpolate v0 towards v1 and normalize.
	/// Input vectors must be normalized
	template<CVec3Accessible Vec3>
	[[nodiscard]] JPL_INLINE Vec3 Nlerp(const Vec3& v0, const Vec3& v1, float t) noexcept
	{
		return Normalized(Lerp(v0, v1, t));
	}

	/// Input vectors must be normalized
	template<CVec3Accessible Vec3>
	[[nodiscard]] inline Vec3 Slerp(const Vec3& v0, const Vec3& v1, float t) noexcept
	{
		using FloatType = Internal::FloatOf<Vec3>;
		FloatType dot = DotProduct(v0, v1);

		// Clamp dot product to handle floating point inaccuracies
		// (values slightly outside [-1, 1] can cause acos to return NaN)
		dot = std::fmax(FloatType(-1.0), std::fmin(FloatType(1.0), dot));
		
		static constexpr FloatType EPSILON =
			std::numeric_limits<FloatType>::epsilon() * FloatType(100.0);

		// Case 1: Vectors are very close (parallel or nearly parallel)
		// This provides good enough results for nearly parallel vectors.
		if (dot > FloatType(1.0) - EPSILON)
		{
			// Use linear interpolation and re-normalize (NLERP)
			//return (v0 * (FloatType(1.0) - t) + v1 * t).Normalize();
			return Nlerp(v0, v1, t);
		}

		// Case 2: Vectors are opposite (dot product approx -1)
		if (dot < FloatType(-1.0) + EPSILON)
		{
			// When vectors are opposite, SLERP is ambiguous.
			// We pick an arbitrary axis perpendicular to v0 to rotate around.
			Vec3 axis = Vec3(1.0, 0.0, 0.0);
			if (Math::Abs(DotProduct(v0, axis)) > FloatType(1.0) - EPSILON)
			{
				// If v0 is too close to X-axis, try Y-axis
				axis = Vec3(0.0, 1.0, 0.0);
			}

			// The rotation axis is perpendicular to v0.
			axis = CrossProduct(v0, axis);

			// Rotate v0 by t * 180 degrees around the chosen axis.
			// This is equivalent to a half-rotation of a quaternion.
			const FloatType angle = t * std::numbers::pi_v<FloatType>;
			const auto [sinAngle, cosAngle] = Math::SinCos(angle);

			// Optimized for unit vectors and axis perpendicular to v:
			// (axis . v) = 0
			// So: v_rot = v * cos(a) + (axis x v) * sin(a)
			return Normalized(v0 * cosAngle + (CrossProduct(axis, v0)) * sinAngle);
		}

		// Case 3: General case (vectors are neither very close nor opposite)
		const FloatType theta = std::acos(dot); // Angle between vectors
		const FloatType invSinTheta = 1.0f / std::sin(theta); // Will not be close to zero here

		const FloatType w0 = std::sin((FloatType(1.0) - t) * theta) * invSinTheta;
		const FloatType w1 = std::sin(t * theta) * invSinTheta;

		return Normalized(v0 * w0 + v1 * w1);
	}

} // namespace JPL::Math