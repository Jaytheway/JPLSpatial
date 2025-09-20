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
#include "JPLSpatial/Math/MinimalBasis.h"

#include <cmath>
#include <ostream>

namespace JPL
{
    //======================================================================
    ///  Minimal quaternion  (w + xi + yj + zk)
    template<CVec3Accessible Vec3>
    struct Quat
    {
        using Float = Internal::FloatOf<Vec3>;
        Vec3 V; // v = {x,y,z}
        Float W;

        /// Check if two quaternions are exactly equal
        inline bool operator == (const Quat& rhs) const noexcept { return V == rhs.V && W == rhs.W; }
        inline bool operator != (const Quat& rhs) const noexcept { return V != rhs.V || W != rhs.W; }

        [[nodiscard]] JPL_INLINE static Quat Identity() noexcept { return Quat({ 0, 0, 0, }, 1); }

        [[nodiscard]] JPL_INLINE Quat Conjugated() const noexcept { return Quat{ -V, W }; }

        [[nodiscard]] JPL_INLINE Quat Inversed() const noexcept
        {
            const Float n2Inv = 1.0f / (DotProduct(V, V) + W * W);
            return Quat{ -V * n2Inv, W * n2Inv };
        }

        /// Construct quaternion from the a rotation around the given axis.
        /// @param axis : must be a normalized direction vector
        [[nodiscard]] static JPL_INLINE Quat Rotation(const Vec3& axis, Float angleRad) noexcept
        {
            const Float halfAngle = Float(0.5) * angleRad;
            const auto [s, c] = Math::SinCos(halfAngle);
            return Quat(axis * s, c);
        }

        /// Find quaternian rotation between two unit vectors
        /// Input vectors must be normalized
        [[nodiscard]] inline static Quat FromTo(const Vec3& from, const Vec3& to) noexcept
        {
            const Float dot = DotProduct(from, to);

            if (dot > Float(0.999999)) // directions already equal
                return Identity(); // no rotation

            if (dot < Float(-0.999999)) // opposite directions
                return Quat(Math::GetNormalizedPerpendicular(from), Float(0));

            // General case
            return Quat(CrossProduct(from, to), Float(1) + dot).Normalized();
        }

        /// Construct quaternian from basis columns X, Y, Z
        [[nodiscard]] inline static Quat FromBasis(const Basis<Vec3>& basis) noexcept
        {
            // axes-as-columns rotation matrix:
            const Float r00 = basis.X.X, r01 = basis.Y.X, r02 = basis.Z.X;
            const Float r10 = basis.X.Y, r11 = basis.Y.Y, r12 = basis.Z.Y;
            const Float r20 = basis.X.Z, r21 = basis.Y.Z, r22 = basis.Z.Z;

            Float tr = r00 + r11 + r22;
            Float qw, qx, qy, qz;

            if (tr > 0.0f)
            {
                const Float s = Math::Sqrt(tr + Float(1.0)) * Float(2.0);
                const Float invS = Float(1.0) / s;
                qw = Float(0.25) * s;
                qx = (r21 - r12) * invS;
                qy = (r02 - r20) * invS;
                qz = (r10 - r01) * invS;
            }
            else if (r00 > r11 && r00 > r22)
            {
                const Float s = Math::Sqrt(Float(1.0) + r00 - r11 - r22) * Float(2.0);
                const Float invS = Float(1.0) / s;
                qw = (r21 - r12) / s;
                qx = Float(0.25) * s;
                qy = (r01 + r10) / s;
                qz = (r02 + r20) / s;
            }
            else if (r11 > r22)
            {
                const Float s = Math::Sqrt(Float(1.0) + r11 - r00 - r22) * Float(2.0);
                const Float invS = Float(1.0) / s;
                qw = (r02 - r20) * invS;
                qx = (r01 + r10) * invS;
                qy = Float(0.25) * s;
                qz = (r12 + r21) * invS;
            }
            else
            {
                const Float s = Math::Sqrt(Float(1.0) + r22 - r00 - r11) * Float(2.0);
                const Float invS = Float(1.0) / s;
                qw = (r10 - r01) * invS;
                qx = (r02 + r20) * invS;
                qy = (r12 + r21) * invS;
                qz = Float(0.25) * s;
            }

            return Quat { Vec3{ qx, qy, qz }, qw };
        }

        /// Construct rotation quaternion from up and forward axis
        /// Input parameters must be normalized.
        [[nodiscard]] static JPL_INLINE Quat FromUpAndForward(const Vec3& up, const Vec3& forward) noexcept
        {
            return FromBasis(Basis<Vec3>::FromUpAndForward(up, forward));
        }

        [[nodiscard]] static JPL_INLINE Quat LookAt(const Vec3& direction, const Vec3& up) noexcept
        {
            return FromUpAndForward(up, direction);
        }

        /// Quaternion multiplication (rotation composition)
        [[nodiscard]] JPL_INLINE friend Quat operator*(const Quat& a, const Quat& b) noexcept
        {
#if 1
            return Quat{
                .V = a.W * b.V + b.W * a.V + CrossProduct(a.V, b.V),
                .W = a.W * b.W - DotProduct(a.V, b.V)
            };
#else
            // Let the compiler figure out optimization
            Float lx = a.V.X;
            Float ly = a.V.Y;
            Float lz = a.V.Z;
            Float lw = a.W;

            Float rx = b.V.X;
            Float ry = b.V.Y;
            Float rz = b.V.Z;
            Float rw = b.W;

            Float x = lw * rx + lx * rw + ly * rz - lz * ry;
            Float y = lw * ry - lx * rz + ly * rw + lz * rx;
            Float z = lw * rz + lx * ry - ly * rx + lz * rw;
            Float w = lw * rw - lx * rx - ly * ry - lz * rz;

            return Quat({ x, y, z }, w);
#endif
        }

        /// Create quaternian to slerp direction vector based on 'from' and 'to'
        [[nodiscard]] JPL_INLINE static Quat MakeSlerp(const Vec3& from, const Vec3& to, Float t) noexcept
        {
            const Quat qFromTo = FromTo(from, to);
            return Slerp(Identity(), qFromTo, t); // id -> qFromTo
        }

        /// Rotate a vector by this quaternion
        [[nodiscard]] JPL_INLINE Vec3 Rotate(const Vec3& vector) const noexcept
        {
            //  q p q*  (H. Hamilton trick, 18 mul + 12 add)
            const Vec3 t = Float(2) * CrossProduct(V, vector);
            return vector + W * t + CrossProduct(V, t);
        }

        [[nodiscard]] JPL_INLINE Float LengthSquared() const noexcept { return DotProduct(V, V) + W * W; }

        [[nodiscard]] JPL_INLINE Float Length() const noexcept { return Math::Sqrt(LengthSquared()); }

        [[nodiscard]] JPL_INLINE Quat Normalized() const noexcept
        {
            const Float invLen = Float(1) / Length();
            return Quat{ V * invLen, W * invLen };
        }

        [[nodiscard]] JPL_INLINE bool IsNormalized(Float tolerance = Float(1.0e-5)) const noexcept
        {
            return Math::Abs(LengthSquared() - Float(1.0)) <= tolerance;
        }

        /// Get rotation angle around axis
        [[nodiscard]] JPL_INLINE Float GetRotationAngle(const Vec3& axis) const noexcept
        {
            return W == Float(0.0) ? JPL_PI : Float(2.0) * std::atan(DotProduct(V, axis) / W);
        }

        /// Convert to an orthonormal Basis3 (column-major)
        [[nodiscard]] inline Basis<Vec3> ToBasis() const noexcept
        {
            const Float xx = V.X * V.X, yy = V.Y * V.Y, zz = V.Z * V.Z;
            const Float xy = V.X * V.Y, xz = V.X * V.Z, yz = V.Y * V.Z;
            const Float wx = W * V.X,   wy = W * V.Y,   wz = W * V.Z;
            return Basis<Vec3>{
                .X = Vec3{ Float(1) - Float(2) * (yy + zz), Float(2) * (xy + wz),               Float(2) * (xz - wy) },
                .Y = Vec3{ Float(2) * (xy - wz),            Float(1) - Float(2) * (xx + zz),    Float(2) * (yz + wx) },
                .Z = Vec3{ Float(2) * (xz + wy),            Float(2) * (yz - wx),               Float(1) - Float(2) * (xx + yy) }
            };
        }

    };
    
    template<CVec3Accessible Vec3>
    std::ostream& operator<<(std::ostream& os, const Quat<Vec3>& quat) { os << "V ={ " << quat.V.X << ", " << quat.V.Y << ", " << quat.V.Z << " }, W = " << quat.W; return os; }

    /// Slerp two Quats based on `t`
    template<CVec3Accessible Vec3>
    [[nodiscard]] static inline Quat<Vec3> Slerp(const Quat<Vec3>& a, const Quat<Vec3>& b, typename Quat<Vec3>::Float t) noexcept
    {
        using F = typename Quat<Vec3>::Float;
        F dot = a.W * b.W + DotProduct(a.V, b.V);
        Quat<Vec3> b1 = (dot < 0) ? Quat<Vec3>{-b.W, -b.V} : b; // enforce short arc
        
        if (Math::Abs(dot) > F(0.9995))
        {
            // near 0 degrees -> nlerp
            Vec3 v = Math::Lerp(a.V, b1.V, t);
            F w = Math::Lerp(a.W, b1.W, t);
            return Quat<Vec3>{ v, w }.Normalized();
        }

        F phi = std::acos(dot);
        F invSinPhi = F(1.0) / std::sin(phi);
        F wA = std::sin((1 - t) * phi) * invSinPhi;
        F wB = std::sin(t * phi) * invSinPhi;
        return Quat<Vec3>{
            .V = wA * a.V + wB * b1.V,
            .W = wA * a.W + wB * b1.W
        };
    }

    namespace Math
    {
        // Mainly just some helpers for template argument deduction

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> GetDeltaQuat(const Quat<Vec3>& a, const Quat<Vec3> b) noexcept
        {
            return a * b.Conjugated();
        }

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> QuatRotation(const Vec3& axis, Internal::FloatOf<Vec3> angleRad) noexcept
        {
            return Quat<Vec3>::Rotation(axis, angleRad);
        }

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> QuatFromTo(const Vec3& from, const Vec3& to) noexcept
        {
            return Quat<Vec3>::FromTo(from, to);
        }

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> QuatFromBasis(const Basis<Vec3>& basis) noexcept
        {
            return Quat<Vec3>::FromBasis(basis);
        }

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> QuatFromUpAndForward(const Vec3& up, const Vec3& forward) noexcept
        {
            return Quat<Vec3>::FromUpAndForward(up, forward);
        }

        template<CVec3Accessible Vec3>
        [[nodiscard]] static JPL_INLINE Quat<Vec3> QuatLookAt(const Vec3& direction, const Vec3& up) noexcept
        {
            return Quat<Vec3>::LookAt(direction, up);
        }

    } // namespace Math
} // namespace JPL