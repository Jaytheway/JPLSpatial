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

#include <ostream>

namespace JPL
{
    struct Vec2
    {
        float X;
        float Y;

        [[nodiscard]] static JPL_INLINE constexpr Vec2 Zero() noexcept { return Vec2{ 0.0f, 0.0f }; }

        [[nodiscard]] JPL_INLINE constexpr float LengthSquared() const noexcept { return X * X + Y * Y; }
        [[nodiscard]] JPL_INLINE constexpr float Length() const noexcept { return Math::Sqrt(LengthSquared()); }
        JPL_INLINE constexpr Vec2& Normalize() noexcept
        {
            const float invLength = Math::InvSqrt(LengthSquared());
            X *= invLength;
            Y *= invLength;
            return *this;
        }

        [[nodiscard]] JPL_INLINE constexpr Vec2 operator-() const noexcept { return Vec2{ -X, -Y }; }
        [[nodiscard]] JPL_INLINE constexpr Vec2 operator-(const Vec2& V) const noexcept { return Vec2{ X - V.X, Y - V.Y }; }
        [[nodiscard]] JPL_INLINE constexpr Vec2 operator+(const Vec2& V) const noexcept { return Vec2{ X + V.X, Y + V.Y }; }
        
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE constexpr Vec2 operator*(Arg Scale) const noexcept { return Vec2{ X * (float)Scale, Y * (float)Scale }; }
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE constexpr Vec2 operator/(Arg Scale) const noexcept { const float invScale = 1.0f / (float)Scale; return Vec2{ X * invScale, Y * invScale }; }
        template<typename Arg> requires(std::is_arithmetic_v<Arg>)
        [[nodiscard]] JPL_INLINE constexpr Vec2& operator/=(Arg Scale) noexcept { const float invScale = 1.0f / (float)Scale; X* invScale; Y* invScale; return *this; }
        
        JPL_INLINE constexpr void operator+=(const Vec2& V) noexcept { X += V.X; Y += V.Y; }
    };

    [[nodiscard]] JPL_INLINE constexpr bool operator==(const Vec2& A, const Vec2& B) noexcept { return A.X == B.X && A.Y == B.Y; }
    
    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr Vec2 operator*(T Scale, const JPL::Vec2& V) noexcept { return V.operator*(Scale); }
    template<typename T> requires(std::is_arithmetic_v<T>)
    [[nodiscard]] JPL_INLINE constexpr Vec2 operator/(T Scale, const JPL::Vec2& V) noexcept { return V.operator/(Scale); }

    [[nodiscard]] JPL_INLINE constexpr Vec2 operator*(const JPL::Vec2& A, const JPL::Vec2& B) noexcept { return { A.X * B.X, A.Y * B.Y}; }
    [[nodiscard]] JPL_INLINE constexpr Vec2 operator/(const JPL::Vec2& A, const JPL::Vec2& B) noexcept { return { A.X / B.X, A.Y / B.Y}; }

    [[nodiscard]] static JPL_INLINE constexpr float CrossProduct(const Vec2& a, const Vec2& b) noexcept { return a.X * b.Y - a.Y * b.X; }
    [[nodiscard]] static JPL_INLINE constexpr float DotProduct(const Vec2& a, const Vec2& b) noexcept { return a.X * b.X + a.Y * b.Y; }
    [[nodiscard]] static JPL_INLINE constexpr Vec2 Normalized(const Vec2& V) noexcept { return Vec2(V).Normalize(); }

    static JPL_INLINE constexpr Vec2& Normalize(Vec2& V) noexcept { return V.Normalize(); }
    
    [[nodiscard]] static JPL_INLINE constexpr Vec2 Abs(const Vec2& V) noexcept { return { Math::Abs(V.X), Math::Abs(V.Y) }; }

    inline std::ostream& operator<<(std::ostream& os, const Vec2& v) { os << "{ " << v.X << ", " << v.Y << " }"; return os; }

} // namespace JPL