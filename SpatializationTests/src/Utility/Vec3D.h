//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2026 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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
    /// Minimal vector class with double precision, mainly for tests
    struct Vec3D
    {
        union
        {
            std::array<double, 3> mVF;
            struct
            {
                double X, Y, Z;
            };
        };

        Vec3D() = default;
        JPL_INLINE constexpr Vec3D(double x, double y, double z) noexcept : X(x), Y(y), Z(z) {}

        [[nodiscard]] JPL_INLINE static constexpr Vec3D Zero() noexcept { return Vec3D{ 0.0f, 0.0f, 0.0f }; }

        [[nodiscard]] JPL_INLINE constexpr double operator [](uint32 coordinate) const { return mVF[coordinate]; }

        [[nodiscard]] JPL_INLINE constexpr double LengthSquared() const noexcept { return X * X + Y * Y + Z * Z; }
        [[nodiscard]] JPL_INLINE constexpr double Length() const noexcept { return Math::Sqrt(LengthSquared()); }

        JPL_INLINE constexpr Vec3D& Normalize() noexcept
        {
            const double invLength = Math::InvSqrt(LengthSquared());
            X *= invLength;
            Y *= invLength;
            Z *= invLength;

            return *this;
        }

        [[nodiscard]] JPL_INLINE constexpr Vec3D operator-() const { return Vec3D(-X, -Y, -Z); }
        [[nodiscard]] JPL_INLINE constexpr Vec3D operator-(const Vec3D& V) const { return Vec3D(X - V.X, Y - V.Y, Z - V.Z); }
        [[nodiscard]] JPL_INLINE constexpr Vec3D operator+(const Vec3D& V) const { return Vec3D(X + V.X, Y + V.Y, Z + V.Z); }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
            [[nodiscard]] JPL_INLINE constexpr Vec3D operator*(Arg Scale) const noexcept { return Vec3D(X * (double)Scale, Y * (double)Scale, Z * (double)Scale); }
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
            [[nodiscard]] JPL_INLINE constexpr Vec3D operator/(Arg Scale) const noexcept { return Vec3D(X / (double)Scale, Y / (double)Scale, Z / (double)Scale); }

        JPL_INLINE constexpr void operator+=(const Vec3D& V) noexcept { X += V.X; Y += V.Y; Z += V.Z; }
        JPL_INLINE constexpr Vec3D& operator*=(const Vec3D Other) { X *= Other.X; Y *= Other.Y; Z *= Other.Z; return *this; }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE constexpr Vec3D& operator/=(Arg Scale) noexcept { X /= (double)Scale; Y /= (double)Scale; Z /= (double)Scale; return *this; }

        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        JPL_INLINE constexpr Vec3D& operator*=(Arg Scale) noexcept { X *= (double)Scale; Y *= (double)Scale; Z *= (double)Scale; return *this; }

    };

    [[nodiscard]] JPL_INLINE constexpr bool operator==(const Vec3D& A, const Vec3D& B) noexcept { return A.X == B.X && A.Y == B.Y && A.Z == B.Z; }

    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr Vec3D operator*(T Scale, const JPL::Vec3D& V) noexcept { return V.operator*(Scale); }
    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr Vec3D operator/(T Scale, const JPL::Vec3D& V) noexcept { return V.operator/(Scale); }

    [[nodiscard]] JPL_INLINE constexpr Vec3D operator*(const JPL::Vec3D& A, const JPL::Vec3D& B) noexcept { return { A.X * B.X, A.Y * B.Y, A.Z * B.Z }; }
    [[nodiscard]] JPL_INLINE constexpr Vec3D operator/(const JPL::Vec3D& A, const JPL::Vec3D& B) noexcept { return { A.X / B.X, A.Y / B.Y, A.Z / B.Z }; }

    [[nodiscard]] static JPL_INLINE constexpr double LengthSquared(const Vec3D& V) noexcept { return V.LengthSquared(); }
    [[nodiscard]] static JPL_INLINE constexpr double Length(const Vec3D& V) noexcept { return V.Length(); }
    static JPL_INLINE constexpr Vec3D& Normalize(Vec3D& V) noexcept { return V.Normalize(); }
    [[nodiscard]] static JPL_INLINE constexpr Vec3D Normalized(const Vec3D& V) noexcept { return Vec3D(V).Normalize(); }
    [[nodiscard]] static JPL_INLINE constexpr double DotProduct(const Vec3D& A, const Vec3D& B) noexcept { return A.X * B.X + A.Y * B.Y + A.Z * B.Z; }
    [[nodiscard]] static JPL_INLINE constexpr  Vec3D CrossProduct(const Vec3D& A, const Vec3D& B) noexcept
    {
        return Vec3D{
            A.Y * B.Z - A.Z * B.Y,
            A.Z * B.X - A.X * B.Z,
            A.X * B.Y - A.Y * B.X
        };
    }

    [[nodiscard]] static JPL_INLINE constexpr Vec3D Abs(const Vec3D& V) noexcept { return { Math::Abs(V.X), Math::Abs(V.Y), Math::Abs(V.Z) }; }

    // CVec3 interface
    template<>
    struct Vec3Access<Vec3D>
    {
        [[nodiscard]] static JPL_INLINE double GetX(const Vec3D& v) noexcept { return v.X; }
        [[nodiscard]] static JPL_INLINE double GetY(const Vec3D& v) noexcept { return v.Y; }
        [[nodiscard]] static JPL_INLINE double GetZ(const Vec3D& v) noexcept { return v.Z; }

        static JPL_INLINE void SetX(Vec3D& v, double value) noexcept { v.X = value; }
        static JPL_INLINE void SetY(Vec3D& v, double value) noexcept { v.Y = value; }
        static JPL_INLINE void SetZ(Vec3D& v, double value) noexcept { v.Z = value; }
    };

    inline std::ostream& operator<<(std::ostream& os, const Vec3D& v) { os << "{ " << v.X << ", " << v.Y << ", " << v.Z << " }"; return os; }
} // namespace JPL
