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
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/Vec3Math.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <cmath>

namespace JPL::Math
{
    /// Minimal 2x2 matrix interface
    template<class Vec2>
    struct Mat2
    {
        using Float = std::remove_cvref_t<decltype(std::declval<Vec2>().X)>;

        // Columns: [ l1 | l2 ]
        Vec2 C0, C1;

        [[nodiscard]] static JPL_INLINE constexpr Mat2 Identity() noexcept { return { Vec2{ 1, 0 }, Vec2{ 0, 1 } }; }
        [[nodiscard]] static JPL_INLINE constexpr Mat2 FromColumns(const Vec2& l1, const Vec2& l2) noexcept { return { l1, l2 }; }

        // p' = M * p  (column-major)
        [[nodiscard]] JPL_INLINE constexpr Vec2 Transform(const Vec2& p) const noexcept { return C0 * p.X + C1 * p.Y; }

        [[nodiscard]] JPL_INLINE constexpr Float Det() const noexcept{ return CrossProduct(C0, C1); }
        [[nodiscard]] JPL_INLINE constexpr bool IsInvertible(Float eps = Float(1e-6)) const noexcept{ return Math::Abs(Det()) >= eps; }

        // Build inverse into 'out', returns success
        [[nodiscard]] inline constexpr bool TryInverse(Mat2& out, Float eps = Float(1e-6)) const
        {
            const Float det = Det();
            if (!JPL_ENSURE(Math::Abs(det) >= eps))
            {
                out = Identity();
                return false;
            }
            const Float inv = Float(1.0) / det;

            // Rows of L^{-1}
            const Vec2 r0{ C1.Y, -C1.X }; // PerpCW(C1)
            const Vec2 r1{ -C0.Y, C0.X }; // PerpCCW(C0)
            const Vec2 R0 = r0 * inv, R1 = r1 * inv;

            // Convert rows -> columns for column-major storage
            out.C0 = Vec2{ R0.X, R1.X };
            out.C1 = Vec2{ R0.Y, R1.Y };
            return true;
        }

        [[nodiscard]] JPL_INLINE constexpr Mat2 Inversed(Float eps = Float(1e-6)) const
        {
            Mat2 m;
            TryInverse(m, eps);
            return m;
        }
    };

    /// Minimal 3x3 matrix interface
    template<CVec3 Vec3>
    struct Mat3
    {
        using Float =  Internal::FloatOf<Vec3>;

        // Columns: [ l1 | l2 | l3 ]
        Vec3 C0, C1, C2;

        [[nodiscard]] static JPL_INLINE constexpr Mat3 Identity() noexcept { return { Vec3{1, 0, 0}, Vec3{0, 1, 0}, Vec3{0, 0, 1} }; }
        [[nodiscard]] static JPL_INLINE constexpr Mat3 FromColumns(const Vec3& l1, const Vec3& l2, const Vec3& l3) noexcept { return { l1, l2, l3 }; }

        [[nodiscard]] JPL_INLINE constexpr Vec3 Transform(const Vec3& v) const noexcept { return C0 * GetX(v) + C1 * GetY(v) + C2 * GetZ(v); }

        [[nodiscard]] JPL_INLINE constexpr Float Det() const { return DotProduct(C0, CrossProduct(C1, C2)); }
        [[nodiscard]] JPL_INLINE constexpr bool IsInvertible(Float eps = Float(1e-6)) const noexcept { return Math::Abs(Det()) >= eps; }

        [[nodiscard]] inline constexpr bool TryInverse(Mat3& out, Float eps = Float(1e-6)) const
        {
            const Vec3 v1 = CrossProduct(C1, C2); // cofactor row 0^T
            const Vec3 v2 = CrossProduct(C2, C0); // cofactor row 1^T
            const Vec3 v3 = CrossProduct(C0, C1); // cofactor row 2^T
            const Float det = DotProduct(C0, v1);
            if (!JPL_ENSURE(Math::Abs(det) >= eps))
            {
                out = Identity();
                return false;
            }
            const Float inv = 1.0f / det;

            const Vec3 R0 = v1 * inv;  // row 0 of L^{-1}
            const Vec3 R1 = v2 * inv;  // row 1 of L^{-1}
            const Vec3 R2 = v3 * inv;  // row 2 of L^{-1}

            // rows -> columns (column-major)
            out.C0 = Vec3{ GetX(R0), GetX(R1), GetX(R2) };
            out.C1 = Vec3{ GetY(R0), GetY(R1), GetY(R2) };
            out.C2 = Vec3{ GetZ(R0), GetZ(R1), GetZ(R2) };
            return true;
        }

        [[nodiscard]] JPL_INLINE constexpr Mat3 Inversed(Float eps = Float(1e-6)) const
        {
            Mat3 m;
            TryInverse(m, eps);
            return m;
        }
    };
} // namespace JPL