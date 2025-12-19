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
#include "JPLSpatial/Math/Vec3Traits.h"

#include <array>
#include <cmath>
#include <type_traits>
#include <ostream>

namespace JPL
{
    //======================================================================
    /// Minimal vector class to plug into BDPT interface
    /// if user doesn't provide their own.
    struct MinimalVec3
    {
        union
        {
            std::array<float, 3> mVF;
            struct
            {
                float X, Y, Z;
            };
        };

        MinimalVec3() = default;
        JPL_INLINE constexpr MinimalVec3(float x, float y, float z) noexcept : X(x), Y(y), Z(z) {}

        [[nodiscard]] JPL_INLINE static constexpr MinimalVec3 Zero() noexcept { return MinimalVec3{ 0.0f, 0.0f, 0.0f }; }

        [[nodiscard]] JPL_INLINE constexpr float operator [](uint32 coordinate) const { return mVF[coordinate]; }

        [[nodiscard]] JPL_INLINE constexpr float LengthSquared() const noexcept { return X * X + Y * Y + Z * Z; }
        [[nodiscard]] JPL_INLINE constexpr float Length() const noexcept { return Math::Sqrt(LengthSquared()); }

        JPL_INLINE constexpr MinimalVec3& Normalize() noexcept
        {
            const float invLength = Math::InvSqrt(LengthSquared());
            X *= invLength;
            Y *= invLength;
            Z *= invLength;

            return *this;
        }

        [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator-() const { return MinimalVec3(-X, -Y, -Z); }
        [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator-(const MinimalVec3& V) const { return MinimalVec3(X - V.X, Y - V.Y, Z - V.Z); }
        [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator+(const MinimalVec3& V) const { return MinimalVec3(X + V.X, Y + V.Y, Z + V.Z); }
        
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator*(Arg Scale) const noexcept { return MinimalVec3(X * (float)Scale, Y * (float)Scale, Z * (float)Scale); }
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator/(Arg Scale) const noexcept { return MinimalVec3(X / (float)Scale, Y / (float)Scale, Z / (float)Scale); }
        
        JPL_INLINE constexpr void operator+=(const MinimalVec3& V) noexcept { X += V.X; Y += V.Y; Z += V.Z; }
        JPL_INLINE constexpr MinimalVec3& operator*=(const MinimalVec3 Other) { X *= Other.X; Y *= Other.Y; Z *= Other.Z; return *this; }
        
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE constexpr MinimalVec3& operator/=(Arg Scale) noexcept { X /= (float)Scale; Y /= (float)Scale; Z /= (float)Scale; return *this; }
        
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE constexpr MinimalVec3& operator*=(Arg Scale) noexcept { X *= (float)Scale; Y *= (float)Scale; Z *= (float)Scale; return *this; }

    };
    
    [[nodiscard]] JPL_INLINE constexpr bool operator==(const MinimalVec3& A, const MinimalVec3& B) noexcept{ return A.X == B.X && A.Y == B.Y && A.Z == B.Z; }

    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator*(T Scale, const JPL::MinimalVec3& V) noexcept { return V.operator*(Scale); }
    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator/(T Scale, const JPL::MinimalVec3& V) noexcept { return V.operator/(Scale); }

    [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator*(const JPL::MinimalVec3& A, const JPL::MinimalVec3& B) noexcept { return { A.X * B.X, A.Y * B.Y, A.Z * B.Z }; }
    [[nodiscard]] JPL_INLINE constexpr MinimalVec3 operator/(const JPL::MinimalVec3& A, const JPL::MinimalVec3& B) noexcept { return { A.X / B.X, A.Y / B.Y, A.Z / B.Z }; }

    [[nodiscard]] static JPL_INLINE constexpr float LengthSquared(const MinimalVec3& V) noexcept { return V.LengthSquared(); }
    [[nodiscard]] static JPL_INLINE constexpr float Length(const MinimalVec3& V) noexcept { return V.Length(); }
    static JPL_INLINE constexpr MinimalVec3& Normalize(MinimalVec3& V) noexcept { return V.Normalize(); }
    [[nodiscard]] static JPL_INLINE constexpr MinimalVec3 Normalized(const MinimalVec3& V) noexcept { return MinimalVec3(V).Normalize(); }
    [[nodiscard]] static JPL_INLINE constexpr float DotProduct(const MinimalVec3& A, const MinimalVec3& B) noexcept{ return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
    [[nodiscard]] static JPL_INLINE constexpr  MinimalVec3 CrossProduct(const MinimalVec3& A, const MinimalVec3& B) noexcept
    {
        return MinimalVec3{
            A.Y * B.Z - A.Z * B.Y,
            A.Z * B.X - A.X * B.Z,
            A.X * B.Y - A.Y * B.X
        };
    }

    [[nodiscard]] static JPL_INLINE constexpr MinimalVec3 Abs(const MinimalVec3& V) noexcept { return { Math::Abs(V.X), Math::Abs(V.Y), Math::Abs(V.Z) }; }

    // CVec3 interface
    template<>
    struct Vec3Access<MinimalVec3>
    {
        [[nodiscard]] static JPL_INLINE float GetX(const MinimalVec3& v) noexcept { return v.X; }
        [[nodiscard]] static JPL_INLINE float GetY(const MinimalVec3& v) noexcept { return v.Y; }
        [[nodiscard]] static JPL_INLINE float GetZ(const MinimalVec3& v) noexcept { return v.Z; }

        static JPL_INLINE void SetX(MinimalVec3& v, float value) noexcept { v.X = value; }
        static JPL_INLINE void SetY(MinimalVec3& v, float value) noexcept { v.Y = value; }
        static JPL_INLINE void SetZ(MinimalVec3& v, float value) noexcept { v.Z = value; }
    };

    inline std::ostream& operator<<(std::ostream& os, const MinimalVec3& v) { os << "{ " << v.X << ", " << v.Y << ", " << v.Z << " }"; return os; }
} // namespace JPL