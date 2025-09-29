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
#include "JPLSpatial/Math/Vec3Math.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <cmath>

namespace JPL
{
	/// Orthonormal basis (column-major)
	template<CVec3 Vec3>
	struct Basis
	{
		using Float = Internal::FloatOf<Vec3>;

		Vec3 X, Y, Z;

		[[nodiscard]] static constexpr const Basis& Identity() noexcept
		{
			static constexpr Basis sIdentity(
				Vec3{ Float(1), Float(0), Float(0) },
				Vec3{ Float(0), Float(1), Float(0) },
				Vec3{ Float(0), Float(0), Float(1) }
			);
			return sIdentity;
		}

		/// Creates a basis from the given forward (normal vector).
		/// @param n : normal vector, must be normalized
		[[nodiscard]] JPL_INLINE static Basis FromForward(const Vec3& forward) noexcept
		{
			Basis basis{ .Z = forward };
			CreateOrthonormalBasis(forward, basis.X, basis.Y);
			return basis;
		}

		/// Creates a basis from the given up and forward normalized unit vectors.
		[[nodiscard]] inline static Basis FromUpAndForward(const Vec3& up, const Vec3& forward) noexcept
		{
			Vec3 Y = up;
			const Vec3 Z = forward;
			if (Math::IsNearlyEqual(Math::Abs(DotProduct(Y, Z)), Float(1.0)))
				Y = (Math::Abs(GetZ(Z)) < Float(0.999) ? Vec3(0, 0, 1) : Vec3(1, 0, 0));

			const Vec3 X = Normalized(CrossProduct(Y, Z));
			Y = CrossProduct(Z, X);

			return Basis{ X, Y, Z };
		}

		/// Build a rotation "matrix" axis–angle.
		/// Columns are R*ex, R*ey, R*ez (column-major).
		/// (this is cheaper than Quat if > 2 vectors need the same rotation)
		/// @param axis : must be a normalized direction vector
		[[nodiscard]] inline static Basis Rotation(const Vec3& axis, Float angleRad) noexcept
		{
			const auto [s, c] = Math::SinCos(angleRad);
			const Float t = Float(1) - c;

			const Vec3 taxyz = t * axis;
			const Vec3 sAxis = s * axis;

			// Outer-product columns: (a ⊗ a) columns are ax*a, ay*a, az*a
			const Vec3 t_ax_a = axis * taxyz.X;
			const Vec3 t_ay_a = axis * taxyz.Y;
			const Vec3 t_az_a = axis * taxyz.Z;

			return Basis{
				.X = t_ax_a + Vec3{ c, sAxis.Z, -sAxis.Y },
				.Y = t_ay_a + Vec3{ -sAxis.Z, c, sAxis.X },
				.Z = t_az_a + Vec3{ sAxis.Y, -sAxis.X, c }
			};
		}

		/// Apply rotation world -> local
		[[nodiscard]] JPL_INLINE Vec3 InverseTransform(const Vec3& pWorld) const noexcept
		{
			return Vec3{
				DotProduct(pWorld, X),	// component along local X
				DotProduct(pWorld, Y),	// along local Y
				DotProduct(pWorld, Z)	// along local Z
			};
		}

		/// Apply rotation local -> world
		[[nodiscard]] JPL_INLINE Vec3 Transform(const Vec3& vector) const noexcept
		{
			// 3 FMAs, or 9 mul + 6 add
			return GetX(vector) * X + GetY(vector) * Y + GetZ(vector) * Z;
		}

		/// Compose orientations: apply 'this' basis mapping to the other's axes.
		[[nodiscard]] JPL_INLINE Basis Transform(const Basis& otherBasis) const noexcept
		{
			return Basis{ // R * R_other
				.X = Transform(otherBasis.X),
				.Y = Transform(otherBasis.Y),
				.Z = Transform(otherBasis.Z)
			};
		}

		/// Re-express 'other' in 'this' local frame.
		[[nodiscard]] JPL_INLINE Basis InverseTransform(const Basis& otherBasis) const noexcept
		{
			return Basis{ // RT * R_other
				.X = InverseTransform(otherBasis.X),
				.Y = InverseTransform(otherBasis.Y),
				.Z = InverseTransform(otherBasis.Z)
			};
		}

	};

	namespace Math
	{
		/// Creates a basis from the given forward (normal vector).
		/// @param n : normal vector, must be normalized
		template<CVec3 Vec3>
		[[nodiscard]] JPL_INLINE static Basis<Vec3> MakeBasis(const Vec3& forward) noexcept
		{
			return Basis<Vec3>::MakeFrom(forward);
		}

		/// Creates a basis from the given forward (normal vector).
		/// @param n : normal vector, must be normalized
		template<CVec3 Vec3>
		[[nodiscard]] JPL_INLINE static Basis<Vec3> MakeBasis(const Vec3& forward, const Vec3& up) noexcept
		{
			return Basis<Vec3>::FromUpAndForward(up, forward);
		}
	}
} // namespace JPL