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
#if JPL_HAS_PATH_TRACING
#include <span>
#include <cassert>
#include <concepts>

namespace JPL
{
    /// Converts spherical harmonic order to number of spherical harmonic components
    template<std::integral T>
    constexpr inline T OrderToNumSH(T order) { return (order + 1) * (order + 1); }

	/// Static wrapper for a subset of Google's
    /// spherical harmonics library functions
    /// https://github.com/google/spherical-harmonics
    class SphericalHarmonics
    {
    public:
        static constexpr int DegreeLimit = 4;

		/// Simple wrapper to access the x, y, z components
        /// of a vector representation
        template<std::floating_point T>
        struct VecParam
        {
            std::span<const T, 3> data;
            
            constexpr VecParam(std::span<const T, 3> inVecData) : data(inVecData) {}
            
            constexpr inline float x() const { return data[0]; }
            constexpr inline float y() const { return data[1]; }
            constexpr inline float z() const { return data[2]; }
        };

        /// Evaluate the spherical harmonic basis function of degree @l and order @m
        /// for the given direction vector, @dir.
        /// Assert will fail if @dir is not unit, or if l is not in [0, DegreeLimit],
        /// or if m is not in [-l, l].
		/// @param inDir must be a contiguous 3 floats representing a unit vector
        template<std::floating_point T>
        static constexpr T Evaluate(int l, int m, const T inDir[3])
        {
			return Evaluate(l, m, std::span<const T, 3>(inDir, 3));
        }

        /// Evaluate the spherical harmonic basis function of degree @l and order @m
        /// for the given direction vector, @dir.
        /// Assert will fail if @dir is not unit, or if l is not in [0, DegreeLimit],
		/// or if m is not in [-l, l].
        template<std::floating_point T>
        static constexpr T Evaluate(int l, int m, std::span<const T, 3> inDir)
        {
            VecParam<T> dir(inDir);

            constexpr auto squaredNorm = [](VecParam<T> vec) { return vec.x() * vec.x() + vec.y() * vec.y() + vec.z() * vec.z(); };
            constexpr auto nearByMargin = [](T a, T b) { return std::abs(a - b) < T(1e-6); };
            
            assert(l >= 0 && l <= DegreeLimit);             // l must be between 0 and OrderLimit
            assert(-l <= m && m <= l);                      // m must be between -l and l
            assert(nearByMargin(squaredNorm(dir), T(1.0))); // dir is not unit

            switch (l)
            {
            case 0:
                return HardcodedSH00(dir);
            case 1:
                switch (m)
                {
                case -1:
                    return HardcodedSH1n1(dir);
                case 0:
                    return HardcodedSH10(dir);
                case 1:
                    return HardcodedSH1p1(dir);
                }
            case 2:
                switch (m)
                {
                case -2:
                    return HardcodedSH2n2(dir);
                case -1:
                    return HardcodedSH2n1(dir);
                case 0:
                    return HardcodedSH20(dir);
                case 1:
                    return HardcodedSH2p1(dir);
                case 2:
                    return HardcodedSH2p2(dir);
                }
            case 3:
                switch (m)
                {
                case -3:
                    return HardcodedSH3n3(dir);
                case -2:
                    return HardcodedSH3n2(dir);
                case -1:
                    return HardcodedSH3n1(dir);
                case 0:
                    return HardcodedSH30(dir);
                case 1:
                    return HardcodedSH3p1(dir);
                case 2:
                    return HardcodedSH3p2(dir);
                case 3:
                    return HardcodedSH3p3(dir);
                }
            case 4:
                switch (m)
                {
                case -4:
                    return HardcodedSH4n4(dir);
                case -3:
                    return HardcodedSH4n3(dir);
                case -2:
                    return HardcodedSH4n2(dir);
                case -1:
                    return HardcodedSH4n1(dir);
                case 0:
                    return HardcodedSH40(dir);
                case 1:
                    return HardcodedSH4p1(dir);
                case 2:
                    return HardcodedSH4p2(dir);
                case 3:
                    return HardcodedSH4p3(dir);
                case 4:
                    return HardcodedSH4p4(dir);
                }
            }

            // This is unreachable given the assert's above but the compiler can't tell.
            return T(0.0);
        }

    private:
        // Hardcoded spherical harmonic functions for low orders (l is first number
        // and m is second number (sign encoded as preceeding 'p' or 'n')).
        //
        // As polynomials they are evaluated more efficiently in cartesian coordinates,
        // assuming that @d is unit. This is not verified for efficiency.
        template<std::floating_point T>
        static constexpr T HardcodedSH00(VecParam<T> d)
        {
            // 0.5 * sqrt(1/pi)
            return T(0.282095);
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH1n1(VecParam<T> d)
        {
            // -sqrt(3/(4pi)) * y
            return T(-0.488603) * d.y();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH10(VecParam<T> d)
        {
            // sqrt(3/(4pi)) * z
            return T(0.488603) * d.z();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH1p1(VecParam<T> d)
        {
            // -sqrt(3/(4pi)) * x
            return T(-0.488603) * d.x();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH2n2(VecParam<T> d)
        {
            // 0.5 * sqrt(15/pi) * x * y
            return T(1.092548) * d.x() * d.y();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH2n1(VecParam<T> d)
        {
            // -0.5 * sqrt(15/pi) * y * z
            return T(-1.092548) * d.y() * d.z();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH20(VecParam<T> d)
        {
            // 0.25 * sqrt(5/pi) * (-x^2-y^2+2z^2)
            return T(0.315392) * (-d.x() * d.x() - d.y() * d.y() + T(2.0) * d.z() * d.z());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH2p1(VecParam<T> d)
        {
            // -0.5 * sqrt(15/pi) * x * z
            return T(-1.092548) * d.x() * d.z();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH2p2(VecParam<T> d)
        {
            // 0.25 * sqrt(15/pi) * (x^2 - y^2)
            return T(0.546274) * (d.x() * d.x() - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3n3(VecParam<T> d)
        {
            // -0.25 * sqrt(35/(2pi)) * y * (3x^2 - y^2)
            return T(-0.590044) * d.y() * (T(3.0) * d.x() * d.x() - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3n2(VecParam<T> d)
        {
            // 0.5 * sqrt(105/pi) * x * y * z
            return T(2.890611) * d.x() * d.y() * d.z();
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3n1(VecParam<T> d)
        {
            // -0.25 * sqrt(21/(2pi)) * y * (4z^2-x^2-y^2)
            return T(-0.457046) * d.y() * (T(4.0) * d.z() * d.z() - d.x() * d.x()
                                                   - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH30(VecParam<T> d)
        {
            // 0.25 * sqrt(7/pi) * z * (2z^2 - 3x^2 - 3y^2)
            return T(0.373176) * d.z() * (T(2.0) * d.z() * d.z() - T(3.0) * d.x() * d.x()
                                                  - T(3.0) * d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3p1(VecParam<T> d)
        {
            // -0.25 * sqrt(21/(2pi)) * x * (4z^2-x^2-y^2)
            return T(-0.457046) * d.x() * (T(4.0) * d.z() * d.z() - d.x() * d.x()
                                                   - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3p2(VecParam<T> d)
        {
            // 0.25 * sqrt(105/pi) * z * (x^2 - y^2)
            return T(1.445306) * d.z() * (d.x() * d.x() - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH3p3(VecParam<T> d)
        {
            // -0.25 * sqrt(35/(2pi)) * x * (x^2-3y^2)
            return T(-0.590044) * d.x() * (d.x() * d.x() - T(3.0) * d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4n4(VecParam<T> d)
        {
            // 0.75 * sqrt(35/pi) * x * y * (x^2-y^2)
            return T(2.503343) * d.x() * d.y() * (d.x() * d.x() - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4n3(VecParam<T> d)
        {
            // -0.75 * sqrt(35/(2pi)) * y * z * (3x^2-y^2)
            return T(-1.770131) * d.y() * d.z() * (T(3.0) * d.x() * d.x() - d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4n2(VecParam<T> d)
        {
            // 0.75 * sqrt(5/pi) * x * y * (7z^2-1)
            return T(0.946175) * d.x() * d.y() * (T(7.0) * d.z() * d.z() - T(1.0));
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4n1(VecParam<T> d)
        {
            // -0.75 * sqrt(5/(2pi)) * y * z * (7z^2-3)
            return T(-0.669047) * d.y() * d.z() * (T(7.0) * d.z() * d.z() - T(3.0));
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH40(VecParam<T> d)
        {
            // 3/16 * sqrt(1/pi) * (35z^4-30z^2+3)
            T z2 = d.z() * d.z();
            return T(0.105786) * (T(35.0) * z2 * z2 - T(30.0) * z2 + T(3.0));
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4p1(VecParam<T> d)
        {
            // -0.75 * sqrt(5/(2pi)) * x * z * (7z^2-3)
            return T(-0.669047) * d.x() * d.z() * (T(7.0) * d.z() * d.z() - T(3.0));
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4p2(VecParam<T> d)
        {
            // 3/8 * sqrt(5/pi) * (x^2 - y^2) * (7z^2 - 1)
            return T(0.473087) * (d.x() * d.x() - d.y() * d.y())
                * (T(7.0) * d.z() * d.z() - T(1.0));
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4p3(VecParam<T> d)
        {
            // -0.75 * sqrt(35/(2pi)) * x * z * (x^2 - 3y^2)
            return T(-1.770131) * d.x() * d.z() * (d.x() * d.x() - T(3.0) * d.y() * d.y());
        }

        template<std::floating_point T>
        static constexpr T HardcodedSH4p4(VecParam<T> d)
        {
            // 3/16*sqrt(35/pi) * (x^2 * (x^2 - 3y^2) - y^2 * (3x^2 - y^2))
            T x2 = d.x() * d.x();
            T y2 = d.y() * d.y();
            return T(0.625836) * (x2 * (x2 - T(3.0) * y2) - y2 * (T(3.0) * x2 - y2));
        }
    };
} // namespace JPL
#endif