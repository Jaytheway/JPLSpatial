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

#include "JPLSpatial/Core.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <type_traits>
#include <ostream>
#include <span>

namespace JPL
{
    /// Structure to pass around per axis mask parameters
    struct Vec3MaskPack
    {
        simd_mask X, Y, Z;
    };

    //======================================================================
    /// Minimal Vec3 class for SIMD operations on 4 Vec3 at a time.
    struct Vec3Pack
    {
        simd X, Y, Z;

        Vec3Pack() = default;
        JPL_INLINE Vec3Pack(float x, float y, float z) noexcept : X(x), Y(y), Z(z) {}
        JPL_INLINE Vec3Pack(const float* x, const float* y, const float* z) noexcept : X(x), Y(y), Z(z) {}
        JPL_INLINE Vec3Pack(simd x, simd y, simd z) noexcept : X(x), Y(y), Z(z) {}
        
        template<CVec3 Vec3Type>
        explicit JPL_INLINE Vec3Pack(const Vec3Type& vec) noexcept : X(GetX(vec)), Y(GetY(vec)), Z(GetZ(vec)) {}

        explicit JPL_INLINE Vec3Pack(const simd& axis) noexcept : X(axis), Y(axis), Z(axis) {}

        template<CVec3 Vec3Type>
		JPL_INLINE Vec3Pack(const Vec3Type& vec1,
							const Vec3Type& vec2,
							const Vec3Type& vec3,
							const Vec3Type& vec4)
            : X(GetX(vec1), GetX(vec2), GetX(vec3), GetX(vec4))
            , Y(GetY(vec1), GetY(vec2), GetY(vec3), GetY(vec4))
            , Z(GetZ(vec1), GetZ(vec2), GetZ(vec3), GetZ(vec4))
        {
        }

        template<CVec3 Vec3Type>
        JPL_INLINE void load(std::span<const Vec3Type> inVecs)
        {
            JPL_ASSERT(inVecs.size() >= simd::size());

            *this = Vec3Pack(inVecs[0], inVecs[1], inVecs[2], inVecs[3]);
        }

        /// Pointers must point to storate allocated for at least simd::size() num of floats each
        JPL_INLINE void store(float* outX, float* outY, float* outZ) const
        {
            X.store(outX); Y.store(outY); Z.store(outZ);
        }

        template<CVec3 Vec3Type>
        JPL_INLINE void store(std::span<Vec3Type> outVecs) const
        {
            JPL_ASSERT(outVecs.size() >= simd::size());

            float xs[simd::size()]{}; X.store(xs);
            float ys[simd::size()]{}; Y.store(ys);
            float zs[simd::size()]{}; Z.store(zs);

            for (uint32 i = 0; i < simd::size(); ++i)
            {
                Vec3Type& outVec = outVecs[i];
                SetX(outVec, xs[i]); SetY(outVec, ys[i]); SetZ(outVec, zs[i]);
            }
        }

        /// Component-wise select, returns 'ifTrue' if mask is true, 'ifFalse' otherwise
        /// Uses the same mask for all components.
        [[nodiscard]] static JPL_INLINE Vec3Pack select(const simd_mask& conditionMask, const Vec3Pack& ifTrue, const Vec3Pack& ifFalse)
        {
            return Vec3Pack{
                simd::select(conditionMask, ifTrue.X, ifFalse.X),
                simd::select(conditionMask, ifTrue.Y, ifFalse.Y),
                simd::select(conditionMask, ifTrue.Z, ifFalse.Z)
            };
        }

        /// Component-wise select, returns 'ifTrue' if mask is true, 'ifFalse' otherwise
        /// Uses individual masks per component.
        [[nodiscard]] static JPL_INLINE Vec3Pack select(const Vec3MaskPack& conditionMasks, const Vec3Pack& ifTrue, const Vec3Pack& ifFalse)
        {
            return Vec3Pack{
                simd::select(conditionMasks.X, ifTrue.X, ifFalse.X),
                simd::select(conditionMasks.Y, ifTrue.Y, ifFalse.Y),
                simd::select(conditionMasks.Z, ifTrue.Z, ifFalse.Z)
            };
        }

        [[nodiscard]] JPL_INLINE static Vec3Pack Zero() noexcept { return Vec3Pack{ 0.0f, 0.0f, 0.0f }; }

        [[nodiscard]] JPL_INLINE simd LengthSquared() const noexcept { return X * X + Y * Y + Z * Z; }
        [[nodiscard]] JPL_INLINE simd Length() const noexcept { return Math::Sqrt(LengthSquared()); }

        JPL_INLINE Vec3Pack& Normalize() noexcept
        {
            const simd invLength = Math::InvSqrtFast(LengthSquared());
            X *= invLength;
            Y *= invLength;
            Z *= invLength;
            return *this;
        }

        /// Check if all components of all lanes match
        [[nodiscard]] JPL_INLINE bool IsEqual(const Vec3Pack& other) const;

        [[nodiscard]] JPL_INLINE Vec3Pack operator-() const { return Vec3Pack(-X, -Y, -Z); }
        [[nodiscard]] JPL_INLINE Vec3Pack operator-(const Vec3Pack& V) const { return Vec3Pack(X - V.X, Y - V.Y, Z - V.Z); }
        [[nodiscard]] JPL_INLINE Vec3Pack operator+(const Vec3Pack& V) const { return Vec3Pack(X + V.X, Y + V.Y, Z + V.Z); }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE Vec3Pack operator*(Arg Scale) const noexcept { return Vec3Pack(X * (float)Scale, Y * (float)Scale, Z * (float)Scale); }
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE Vec3Pack operator/(Arg Scale) const noexcept { return Vec3Pack(X / (float)Scale, Y / (float)Scale, Z / (float)Scale); }

        [[nodiscard]] JPL_INLINE Vec3Pack operator*(simd Scale) const noexcept { return Vec3Pack(X * Scale, Y * Scale, Z * Scale); }
        [[nodiscard]] JPL_INLINE Vec3Pack operator/(simd Scale) const noexcept { return Vec3Pack(X / Scale, Y / Scale, Z / Scale); }
        
        
        JPL_INLINE void operator+=(const Vec3Pack& V) noexcept { X += V.X; Y += V.Y; Z += V.Z; }
        JPL_INLINE Vec3Pack& operator*=(const Vec3Pack Other) { X *= Other.X; Y *= Other.Y; Z *= Other.Z; return *this; }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE Vec3Pack& operator/=(Arg Scale) noexcept { X /= (float)Scale; Y /= (float)Scale; Z /= (float)Scale; return *this; }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE Vec3Pack& operator*=(Arg Scale) noexcept { X *= (float)Scale; Y *= (float)Scale; Z *= (float)Scale; return *this; }

        JPL_INLINE Vec3Pack& operator/=(simd Scale) noexcept { X /= Scale; Y /= Scale; Z /= Scale; return *this; }
        JPL_INLINE Vec3Pack& operator*=(simd Scale) noexcept { X *= Scale; Y *= Scale; Z *= Scale; return *this; }

    };

    [[nodiscard]] JPL_INLINE simd_mask operator==(const Vec3Pack& A, const Vec3Pack& B) noexcept { return (A.X == B.X) & (A.Y == B.Y) & (A.Z == B.Z); }
    [[nodiscard]] JPL_INLINE bool Vec3Pack::IsEqual(const Vec3Pack& other) const { return (*this == other).all_of(); }

    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE Vec3Pack operator*(T Scale, const JPL::Vec3Pack& V) noexcept { return V.operator*(Scale); }
    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE Vec3Pack operator/(T Scale, const JPL::Vec3Pack& V) noexcept { return V.operator/(Scale); }

    [[nodiscard]] JPL_INLINE Vec3Pack operator*(simd Scale, const JPL::Vec3Pack& V) noexcept { return V.operator*(Scale); }
    [[nodiscard]] JPL_INLINE Vec3Pack operator/(simd Scale, const JPL::Vec3Pack& V) noexcept { return V.operator/(Scale); }

    
    [[nodiscard]] JPL_INLINE Vec3Pack operator*(const JPL::Vec3Pack& A, const JPL::Vec3Pack& B) noexcept { return { A.X * B.X, A.Y * B.Y, A.Z * B.Z }; }
    [[nodiscard]] JPL_INLINE Vec3Pack operator/(const JPL::Vec3Pack& A, const JPL::Vec3Pack& B) noexcept { return { A.X / B.X, A.Y / B.Y, A.Z / B.Z }; }

    [[nodiscard]] static JPL_INLINE simd LengthSquared(const Vec3Pack& V) noexcept { return V.LengthSquared(); }
    [[nodiscard]] static JPL_INLINE simd Length(const Vec3Pack& V) noexcept { return V.Length(); }
    static JPL_INLINE Vec3Pack& Normalize(Vec3Pack& V) noexcept { return V.Normalize(); }
    [[nodiscard]] static JPL_INLINE Vec3Pack Normalized(const Vec3Pack& V) noexcept { return Vec3Pack(V).Normalize(); }
    [[nodiscard]] static JPL_INLINE simd DotProduct(const Vec3Pack& A, const Vec3Pack& B) noexcept { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
    [[nodiscard]] static JPL_INLINE Vec3Pack CrossProduct(const Vec3Pack& A, const Vec3Pack& B) noexcept
    {
        return Vec3Pack{
            A.Y * B.Z - A.Z * B.Y,
            A.Z * B.X - A.X * B.Z,
            A.X * B.Y - A.Y * B.X
        };
    }

    [[nodiscard]] static JPL_INLINE Vec3Pack Abs(const Vec3Pack& V) noexcept { return { abs(V.X), abs(V.Y), abs(V.Z) }; }

    inline std::ostream& operator<<(std::ostream& os, const Vec3Pack& v)
    {
        os << "{ " << v.X << ", " << v.Y << ", " << v.Z << " }"; return os;
    }

    namespace Math
    {
        [[nodiscard]] static JPL_INLINE Vec3Pack FMA(const Vec3Pack& mulA, const Vec3Pack& mulB, const Vec3Pack& addC) noexcept
        {
            return Vec3Pack{
                fma(mulA.X, mulB.X, addC.X),
                fma(mulA.Y, mulB.Y, addC.Y),
                fma(mulA.Z, mulB.Z, addC.Z),
            };
        }

        [[nodiscard]] static JPL_INLINE Vec3Pack FMA(const Vec3Pack& mulA, const simd& mulB, const Vec3Pack& addC) noexcept
        {
            return Vec3Pack{
                fma(mulA.X, mulB, addC.X),
                fma(mulA.Y, mulB, addC.Y),
                fma(mulA.Z, mulB, addC.Z),
            };
        }

        [[nodiscard]] static JPL_INLINE Vec3Pack FMA(const simd& mulA, const Vec3Pack& mulB, const Vec3Pack& addC) noexcept
        {
            return Vec3Pack{
                fma(mulA, mulB.X, addC.X),
                fma(mulA, mulB.Y, addC.Y),
                fma(mulA, mulB.Z, addC.Z),
            };
        }
    }
} // namespace JPL
