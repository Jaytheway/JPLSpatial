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

#include <JPLSpatial/Core.h>
#include <JPLSpatial/ErrorReporting.h>
#include <JPLSpatial/Math/Math.h>
#include <JPLSpatial/Math/Vec3Math.h>
#include <JPLSpatial/Math/Vec3Traits.h>
#include <JPLSpatial/Math/MinimalVec2.h>
#include <JPLSpatial/Memory/Bits.h>

#include <cmath>
#include <numbers>
#include <concepts>
#include <algorithm>

namespace JPL
{
    //==========================================================================
    /// Type traits and utilities for octahedron encoder
    namespace Octahedron
    {
        namespace Internal
        {
            template <class T, class... Types>
            concept AnyOf = (std::same_as<T, Types> || ...);

            template<class EncodedType_>
            struct PrecisionType
            {
                using EncodedType = EncodedType_;
                static constexpr auto cBitsPerAxis = static_cast<EncodedType>(BitWidthOf<EncodedType>() / 2);
            };
        }

        //======================================================================
        /// Available precision types
        using Precision4bits = Internal::PrecisionType<uint8>;
        using Precision8bits = Internal::PrecisionType<uint16>;
        using Precision16bits = Internal::PrecisionType<uint32>;
        using Precision32bits = Internal::PrecisionType<uint64>;

        //======================================================================
        /// Constraint for valid precision types
        template<class T>
        concept CPrecision = Internal::AnyOf<T, Precision4bits, Precision8bits, Precision16bits, Precision32bits>;

        //======================================================================
        /// Maximum "per-component" error after Encode -> Decode
        /// for an octahedral map with B bits per axis.
        /// Valid for any vector on the unit sphere.
        template <size_t BitsPerAxis>
        constexpr double cMaxComponentError = 4.0 / (static_cast<double>((1ULL << BitsPerAxis)) - 1.0);

        template <size_t BitsPerAxis>
        constexpr double cMaxVectorError = cMaxComponentError<BitsPerAxis> * std::numbers::sqrt3;
        
        // Sanity check
        static_assert(cMaxComponentError<8> >= 0.013);      // ~ 0.0157
        static_assert(cMaxComponentError<16> >= 2.5e-5);    // ~ 6.1e-5

        //======================================================================
        /// Run-time variant of 'cMaxComponentError'
        [[nodiscard]] JPL_INLINE constexpr double MaxComponentError(size_t bitsPerAxis) noexcept
        {
            return 4.0 / (static_cast<double>((1ULL << bitsPerAxis)) - 1.0);
        }

        /// Run-time variant of 'cMaxVectorError'
        [[nodiscard]] JPL_INLINE constexpr double MaxVectorError(size_t bitsPerAxis) noexcept
        {
            return MaxComponentError(bitsPerAxis) * std::numbers::sqrt3;
        }
    }

    //==========================================================================
    /// Forward declaration
    template<Octahedron::CPrecision Precision>
    class OctahedronEncoding;

    //==========================================================================
    /// Encoder aliases for available encoding precisions
    using Octahedron8Bit = OctahedronEncoding<Octahedron::Precision4bits>;
    using Octahedron16Bit = OctahedronEncoding<Octahedron::Precision8bits>;
    using Octahedron32Bit = OctahedronEncoding<Octahedron::Precision16bits>;
    using Octahedron64Bit = OctahedronEncoding<Octahedron::Precision32bits>;

    //==========================================================================
    /// Static utility class for octahedral encoding of a direction vector in 3D space.
    template<Octahedron::CPrecision Precision>
    class OctahedronEncoding
    {
    public:
        using EncodedType = typename Precision::EncodedType;
        static constexpr auto cBitsPerAxis = Precision::cBitsPerAxis;
        
        // To ensure correctness, math is done on this float type regardless of the compoennt type of user Vec3
        using FloatType = std::conditional_t<(Precision::cBitsPerAxis > 23), double, float>;

        static constexpr EncodedType cAxisMask = static_cast<EncodedType>((1llu << cBitsPerAxis) - 1u);

        // Actual number of valid codes is 'cAxisRange - 2'
        static constexpr size_t cAxisRange = cAxisMask + 1; // Note: this may overflow for 64 bit encoder

        static constexpr auto cMaxComponentError = Octahedron::cMaxComponentError<cBitsPerAxis>;
        static constexpr auto cMaxVectorError = Octahedron::cMaxVectorError<cBitsPerAxis>;

    private:
        static constexpr EncodedType cCenter = cAxisMask >> 1;
        static constexpr EncodedType cPolePosZ = (cCenter << cBitsPerAxis) | cCenter;   // (0, 0, 1)
        static constexpr EncodedType cPoleNegZ = (cCenter << cBitsPerAxis) | 1;         // (0, 0, -1)

        static constexpr EncodedType cMaxComponentValue = cAxisMask - 1;

    public:

        /// Encodes a direction vector into octahedron.
        /// 
        /// @param direction    : direction vector to be encoded, must be normalized and non-zero
        /// @returns            : integer encoded representation of the input direction vector
        template<CVec3 Vec3>
        static constexpr EncodedType Encode(const Vec3& direction);

        /// Decodes octahedron encoded direction back to a vector.
        /// This reverses the operation performed by `Encode`.
        ///
        /// @param encodedDirection : the octahedron integer to be decoded back to a direction vector.
        /// @returns : direction vector represented by the input octahedron.
        template<CVec3 Vec3>
        static constexpr Vec3 Decode(EncodedType encodedDirection);

        /// Check if 'code' falls within valid range of the encoder's precision,
        /// taking into acount padding rim.
        template<std::integral T>
        static JPL_INLINE constexpr bool IsValidCode(T code) noexcept;

        /// Check if 'x' and 'y' components' can produce a valid code,
        /// taking into acount padding rim.
        template<std::integral T>
        static JPL_INLINE constexpr bool AreValidComponents(T x, T y) noexcept;

        /// Combine encoded 'x' and 'y' components into a valid code,
        /// sanitizing if necessary.
        template<std::integral T>
        static JPL_INLINE constexpr EncodedType CombineComponents(T x, T y) noexcept;

        /// Alias the padding rim to the last real texel
        template<std::integral T>
        static JPL_INLINE constexpr auto SanitizeCode(T code) noexcept;
    };

    //==========================================================================
    /// Encode a direction vector into octahedron 32 bit integer at 16 bits precision.
    /// 
    /// @param direction : direction vector to be encoded, must be normalized and non-zero
    /// @returns : encoded representation of the input direction vector
    template<CVec3 Vec3>
    JPL_INLINE constexpr uint32 ToOctahedron32(const Vec3& direction) { return Octahedron32Bit::Encode(direction); }

    /// Decode octahedron encoded direction back to a vector.
    /// This reverses the operation performed by `ToOctahedron32`.
    ///
    /// @param encodedDirection : the octahedron integer to be decoded back to a direction vector.
    /// @returns : direction vector represented by the input octahedron.
    template<CVec3 Vec3>
    JPL_INLINE constexpr Vec3 FromOctahedron32(uint32 encodedDirection) { return Octahedron32Bit::Decode<Vec3>(encodedDirection); }

    //==========================================================================
    /// "Diamond Encoding" of a 2D unit vector as per:
    // https://www.freesteel.co.uk/wpblog/2009/06/05/encoding-2d-angles-without-trigonometry/
    // https://www.jeremyong.com/graphics/2023/01/09/tangent-spaces-and-diamond-encoding/

    /// Encode a 2D unit vector to scalar float [0, 1]
    inline constexpr float ToDiamond(Vec2 dir) noexcept
    {
        // Project to the unit diamond, then to the x-axis.
        dir.X /= (Math::Abs(dir.X) + Math::Abs(dir.Y));

        // Contract the x coordinate by a factor of 4 to represent all 4 quadrants in
        // the unit range and remap
        const float pySign = Math::Sign2(dir.Y);
        return -pySign * 0.25f * dir.X + 0.5f + pySign * 0.25f;
    }

    /// Decode scalar [0, 1] to a 2D unit vector
    inline constexpr Vec2 FromDiamond(float p) noexcept
    {
        Vec2 v;

        // Remap p to the appropriate segment on the diamond
        const float pSign = Math::Sign2(p - 0.5f);
        v.X = -pSign * 4.0f * p + 1.0f + pSign * 2.0f;
        v.Y= pSign * (1.0f - Math::Abs(v.X));

        // Normalization extends the point on the diamond back to the unit circle
        return v.Normalize();
    }
} // namespace JPL


//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

// (enable in case of issues with (0,0,-1) direction)
#define JPL_OCTAHEDRON_SPECIAL_CASE_HANDLING 0

namespace JPL
{
    template<Octahedron::CPrecision Precision>
    template<std::integral T>
    JPL_INLINE constexpr bool OctahedronEncoding<Precision>::AreValidComponents(T x, T y) noexcept
    {
        return x != cAxisMask && y != cAxisMask;
    }

    template<Octahedron::CPrecision Precision>
    template<std::integral T>
    JPL_INLINE constexpr bool OctahedronEncoding<Precision>::IsValidCode(T code) noexcept
    {
        const auto x = static_cast<EncodedType>(code) & cAxisMask;
        const auto y = (static_cast<EncodedType>(code) >> cBitsPerAxis) & cAxisMask;
        return AreValidComponents(x, y);
    }

    template<Octahedron::CPrecision Precision>
    template<std::integral T>
    JPL_INLINE constexpr OctahedronEncoding<Precision>::EncodedType OctahedronEncoding<Precision>::CombineComponents(T x, T y) noexcept
    {
        return (std::min(EncodedType(y), cMaxComponentValue) << cBitsPerAxis) | std::min(EncodedType(x), cMaxComponentValue);
    }

    template<Octahedron::CPrecision Precision>
    template<std::integral T>
    JPL_INLINE constexpr auto OctahedronEncoding<Precision>::SanitizeCode(T code) noexcept
    {
        const EncodedType x = static_cast<EncodedType>(code) & cAxisMask;
        const EncodedType y = (static_cast<EncodedType>(code) >> cBitsPerAxis) & cAxisMask;
        return static_cast<T>((std::min(y, cMaxComponentValue) << cBitsPerAxis) | std::min(x, cMaxComponentValue));
    }

    template<Octahedron::CPrecision Precision>
    template<CVec3 Vec3>
    constexpr OctahedronEncoding<Precision>::EncodedType OctahedronEncoding<Precision>::Encode(const Vec3& direction)
    {
        const Vec3 dirAbs = Abs(direction);
        const FloatType L1Norm = FloatType(GetX(dirAbs) + GetY(dirAbs) + GetZ(dirAbs));

        // It should be up to the user to ensure valid direction vector
        JPL_ASSERT(!Math::IsNearlyZero(L1Norm) && "direction must be normalised & non-zero");
#if 0
        // Handle zero vector (map to positive Z pole or some other neutral value)
        if (Math::IsNearlyZero(L1Norm))
        {
            // return (static_cast<EncodedType>(center_val) << cBitsPerAxis) | center_val;
            return cCenter; // return forward vector (we may or may not want different fallback)
        }
#endif

#if JPL_OCTAHEDRON_SPECIAL_CASE_HANDLING
        // --- Special Handling for Poles
        // If it's a pole (only Z component is non-`zero)
        constexpr auto cPoleEps = FloatType(1e-6);
        const bool isPole = (dirAbs.X <= cPoleEps) & (dirAbs.Y <= cPoleEps);
        if (isPole)
            return (direction.Z < FloatType(0.0)) ? cPoleNegZ : cPolePosZ;
        // ---------------------------
#endif

        const FloatType invL1Norm = FloatType(1.0) / L1Norm;
        FloatType px = FloatType(GetX(direction)) * invL1Norm;
        FloatType py = FloatType(GetY(direction)) * invL1Norm;

        // Standard folding for Z < 0
        if (GetZ(direction) < 0.0f)
        {
            const FloatType tempPx = px; // Store original px for py calculation
            px = (FloatType(1.0) - Math::Abs(py)) * Math::Sign2(tempPx);
            py = (FloatType(1.0) - Math::Abs(tempPx)) * Math::Sign2(py);
        }

        constexpr FloatType cHalfTexel = FloatType(0.5) / FloatType(cAxisMask);
        px = std::clamp(px, FloatType(-1.0) + cHalfTexel, FloatType(1.0) - cHalfTexel);
        py = std::clamp(py, FloatType(-1.0) + cHalfTexel, FloatType(1.0) - cHalfTexel);

        // Map to [0,1)
        const FloatType v0 = px * FloatType(0.5) + FloatType(0.5);
        const FloatType v1 = py * FloatType(0.5) + FloatType(0.5);

        // Truncate to requested precision
        const auto dx = static_cast<EncodedType>(Math::Floor(v0 * FloatType(cAxisMask)));
        const auto dy = static_cast<EncodedType>(Math::Floor(v1 * FloatType(cAxisMask)));

        return (dy << cBitsPerAxis) | dx;
    }

    template<Octahedron::CPrecision Precision>
    template<CVec3 Vec3>
    constexpr Vec3 JPL::OctahedronEncoding<Precision>::Decode(EncodedType encodedDirection)
    {
        constexpr FloatType muInv = FloatType(1.0) / static_cast<FloatType>(cAxisMask);

        EncodedType dx = encodedDirection & cAxisMask;                      // Low bits for dx
        EncodedType dy = (encodedDirection >> cBitsPerAxis) & cAxisMask;    // High bits for dy

        // alias the padding rim to the last real texel
        dx = std::min(dx, cMaxComponentValue);
        dy = std::min(dy, cMaxComponentValue);

#if JPL_OCTAHEDRON_SPECIAL_CASE_HANDLING
        // --- Special Handling for Poles
        // Check if it matches our special encoding for (0,0,-1)
        if (encodedDirection == cPoleNegZ)
        {
            return Vec3(Internal::FloatOf<Vec3>(0.0),
                        Internal::FloatOf<Vec3>(0.0),
                        Internal::FloatOf<Vec3>(-1.0));
        }
        // ---------------------------
#endif

        // Map from [0, 1] to [-1, 1]
        // (the added 0.5 shifts the sample from the lower-left vertex to the texel’s centre)
        const FloatType pxDecoded = FloatType(-1.0) + FloatType(2.0) * (FloatType(dx) + FloatType(0.5)) * muInv;
        const FloatType pyDecoded = FloatType(-1.0) + FloatType(2.0) * (FloatType(dy) + FloatType(0.5)) * muInv;

        // Keep as array in case FloatType != VectorFloatType
        FloatType direction[3]{
            pxDecoded,
            pyDecoded,
            FloatType(1.0) - Math::Abs(pxDecoded) - Math::Abs(pyDecoded)
        };

        // "Unfold" the negative Z hemisphere
        const FloatType t = std::max(FloatType(-direction[2]), FloatType(0.0));
        direction[0] += (FloatType(direction[0]) > FloatType(0.0)) ? -t : t;
        direction[1] += (FloatType(direction[1]) > FloatType(0.0)) ? -t : t;

        return Normalized(Vec3{
                Internal::FloatOf<Vec3>(direction[0]),
                Internal::FloatOf<Vec3>(direction[1]),
                Internal::FloatOf<Vec3>(direction[2])
            });
    }

    //==========================================================================
    namespace UnitTests
    {
#if 0
        // Encoder to test
        using EncoderType = Octahedron16Bit;

        namespace // (for validating actual values)
        {
            static constexpr MinimalVec3 OriginalVec = Normalized(MinimalVec3(-1.0f, -1.0f, 1.0f));
            static constexpr MinimalVec3 Decoded = EncoderType::Decode<MinimalVec3>(EncoderType::Encode(OriginalVec));
            static constexpr MinimalVec3 Delta = Abs(OriginalVec - Decoded);
            static_assert(Delta.X < EncoderType::cMaxComponentError &&
                          Delta.Y < EncoderType::cMaxComponentError &&
                          Delta.Z < EncoderType::cMaxComponentError);
        }

        // Helper function to perform and assert a test case
        template<class VectorType>
        constexpr bool TestOctahedron(const VectorType& original)
        {
            const VectorType normalizedOriginal = Normalized(original);
            const auto encoded = EncoderType::Encode(normalizedOriginal);
            const VectorType decoded = EncoderType::Decode<VectorType>(encoded);

			return Math::Abs(normalizedOriginal.X - decoded.X) < EncoderType::cMaxComponentError
				&& Math::Abs(normalizedOriginal.Y - decoded.Y) < EncoderType::cMaxComponentError
				&& Math::Abs(normalizedOriginal.Z - decoded.Z) < EncoderType::cMaxComponentError;
		}

        // --- 0. Zero Vector ---
        // Note: zero input should be handled by the caller
        //static_assert(TestOctahedron(VecMock(0.0f, 0.0f, 0.0f), cOctahedronEncodingTolerance));

        // --- 1. Cardinal Axes ---
        // Positive Z
        static_assert(TestOctahedron(MinimalVec3(0.0f, 0.0f, 1.0f)));
        // Negative Z
        static_assert(TestOctahedron(MinimalVec3(0.0f, 0.0f, -1.0f)));

        // Positive X (on Z>=0 plane)
        static_assert(TestOctahedron(MinimalVec3(1.0f, 0.0f, 0.0f)));
        // Negative X (on Z>=0 plane)
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 0.0f, 0.0f)));

        // Positive Y (on Z>=0 plane)
        static_assert(TestOctahedron(MinimalVec3(0.0f, 1.0f, 0.0f)));
        // Negative Y (on Z>=0 plane)
        static_assert(TestOctahedron(MinimalVec3(0.0f, -1.0f, 0.0f)));


        // --- 2. Face Centers / Diagonals (Z >= 0 hemisphere) ---
        // Your initial test case: (1,1,0) normalized is (0.707, 0.707, 0.0)
        static_assert(TestOctahedron(MinimalVec3(1.0f, 1.0f, 0.0f)));
        // Other quadrants
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 1.0f, 0.0f)));
        static_assert(TestOctahedron(MinimalVec3(1.0f, -1.0f, 0.0f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, -1.0f, 0.0f)));

        // Vectors purely in XY plane, not on axes
        static_assert(TestOctahedron(MinimalVec3(1.0f, 0.5f, 0.0f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 0.5f, 0.0f)));
        static_assert(TestOctahedron(MinimalVec3(1.0f, -0.5f, 0.0f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, -0.5f, 0.0f)));

        // Vectors with positive Z components (e.g., Octahedron face centers in positive Z)
        static_assert(TestOctahedron(MinimalVec3(1.0f, 1.0f, 1.0f))); // (1/sqrt(3), 1/sqrt(3), 1/sqrt(3))
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 1.0f, 1.0f)));
        static_assert(TestOctahedron(MinimalVec3(1.0f, -1.0f, 1.0f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, -1.0f, 1.0f)));
        static_assert(TestOctahedron(MinimalVec3(0.5f, 0.5f, 1.0f)));


        // --- 3. Face Centers / Diagonals (Z < 0 hemisphere) ---
        // Negative Z analogs of the above
        static_assert(TestOctahedron(MinimalVec3(1.0f, 1.0f, -1.0f))); // (1/sqrt(3), 1/sqrt(3), -1/sqrt(3))
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 1.0f, -1.0f)));
        static_assert(TestOctahedron(MinimalVec3(1.0f, -1.0f, -1.0f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, -1.0f, -1.0f)));
        static_assert(TestOctahedron(MinimalVec3(0.5f, 0.5f, -1.0f)));


        // --- 4. Near-Zero (But not exactly zero) ---
        // These test the L1 norm division and the handling of very small values.
        static_assert(TestOctahedron(MinimalVec3(1e-6f, 1e-6f, 1.0f)));
        static_assert(TestOctahedron(MinimalVec3(1e-6f, 1.0f, 1e-6f)));
        static_assert(TestOctahedron(MinimalVec3(1.0f, 1e-6f, 1e-6f)));

        static_assert(TestOctahedron(MinimalVec3(1e-6f, 1e-6f, -1.0f)));
        static_assert(TestOctahedron(MinimalVec3(1e-6f, -1.0f, 1e-6f)));
        static_assert(TestOctahedron(MinimalVec3(-1.0f, 1e-6f, 1e-6f)));


        // --- 5. Known Problematic Quantization Values (if any appear during development) ---
        // Add any specific values that caused issues during interactive testing or debugging.
        static_assert(TestOctahedron(MinimalVec3(0.0f, 0.0995037183f, -0.995037138f)));


        // Diamond encoding
        static constexpr Vec2 dir2D = Vec2(-0.25534f, 4.5433f).Normalize();
        static constexpr float dir2DEncoded = ToDiamond(dir2D);
        static constexpr Vec2 dir2DDecoded = FromDiamond(dir2DEncoded);
        static_assert(Math::IsNearlyZero(dir2D.X - dir2DDecoded.X) && Math::IsNearlyZero(dir2D.Y - dir2DDecoded.Y));
#endif
    } // namespace UnitTests

} // namespace JPL