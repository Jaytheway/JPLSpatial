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
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/Vec3Math.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <functional>
#include <random>
#include <numbers>
#include <cmath>
#include <utility>
#include <concepts>

namespace JPL
{
	static constexpr float JPL_SPEED_OF_SOUND = 343.0f;
	static constexpr float JPL_INV_SPEAD_OF_SOUND = 1.0f / JPL_SPEED_OF_SOUND;
} // namespace JPL

namespace JPL::Math
{
	namespace InternalUtils
	{
		template<std::floating_point FloatType>
		static constexpr auto four_pi = FloatType(4.0) * std::numbers::pi_v<FloatType>;
		template<std::floating_point FloatType>
		static constexpr FloatType two_pi = FloatType(2.0) * std::numbers::pi_v<FloatType>;

		template<std::floating_point FloatType>
		static inline FloatType RandFloat()
		{
			static constexpr auto bits = std::same_as<FloatType, double> ? 64u : 32u;
			static std::mt19937 mt;
			return std::generate_canonical<FloatType, bits>(mt);
		}

		template<class Vec3>
		static inline Vec3 RandDirection()
		{
			return Normalized(Vec3{
				RandFloat<Internal::FloatOf<Vec3>>() * 2.0f - 1.0f,
				RandFloat<Internal::FloatOf<Vec3>>() * 2.0f - 1.0f,
				RandFloat<Internal::FloatOf<Vec3>>() * 2.0f - 1.0f
			});
		}
	}

	template<class Vec3>
	static inline Vec3 GetImageSource(const Vec3& point, const Vec3& planeNormal, float planeConstant)
	{
		const float signedDistance = DotProduct(point, planeNormal) + planeConstant;
		return point - (planeNormal * (2.0f * signedDistance));
	}

	template<class Vec3>
	static inline Vec3 GetImageSource(const Vec3& point, const Vec3& planeNormal, const Vec3& pointOnPlane)
	{
		const float planeConstant = DotProduct(-planeNormal, pointOnPlane);
		return GetImageSource(point, planeNormal, planeConstant);
	}

	// Geometry term represents energy dispersion and occlusion during the propagation,
	// combined with propagation operator (inverse square law + optional air absorption factor)
	template<class Vec3>
	static auto GeometryTerm(const Vec3& x, const Vec3& xNorm, const Vec3& y, const Vec3& yNorm) -> Internal::FloatOf<Vec3>
	{
		using FloatType = Internal::FloatOf<Vec3>;

		Vec3 dir = y - x;
		const FloatType dist2 = dir.LengthSquared();
		if (dist2 < FloatType(1e-6))
			return FloatType(0.0);

		dir /= std::sqrt(dist2);
		const FloatType cosThetaX = std::abs(DotProduct(xNorm, dir));
		const FloatType cosThetaY = std::abs(DotProduct(yNorm, -dir));

		// (optionally) here we can multiply by air absorption factor between x and y
		return (cosThetaX * cosThetaY) / dist2; // * airAbsorption;
	}

	// Geometry term between two points (without surface normal information)
	// is propagation operator (inverse square law + optional air absorption factor)
	template<class Vec3>
	static auto GeometryTerm(const Vec3& x, const Vec3& y) -> Internal::FloatOf<Vec3>
	{
		using FloatType = Internal::FloatOf<Vec3>;

		const FloatType dist2 = (y - x).LengthSquared();
		if (dist2 < FloatType(1e-6))
			return FloatType(0.0);

		// (optionally) here we can multiply by air absorption factor between x and y
		return 1.0f / dist2; // * airAbsorption;
	}

	// Calculates the specular reflection of an incident vector relative to a surface normal.
	// @param incident        Incident vector (any length).
	// @param normal          Surface normal (must be normalized).
	// @returns               Reflected vector in the specular direction, with the same length as the incident vector.
	//
	// Note: If 'incident' is also normalized, the result will be normalized.
	template<class Vec3>
	static Vec3 SpecularReflection(const Vec3& incident, const Vec3& normal)
	{
		return incident - Internal::FloatOf<Vec3>(2.0) * DotProduct(incident, normal) * normal;
	}

	// Cosine-weighted scattered direction in the same hemisphere (Lambert)
	template<class Vec3, class RandomFloatFunc>
	static Vec3 SampleHemisphereCosine(const Vec3& normal, RandomFloatFunc getRandomFloat = InternalUtils::RandFloat<Internal::FloatOf<Vec3>>)
	{
		using FloatType = Internal::FloatOf<Vec3>;

		// 1. Sample in tangent space
		const FloatType u1 = getRandomFloat();	// [0,1)
		const FloatType u2 = getRandomFloat();	// [0,1)
		const FloatType r = std::sqrt(u1);			// radius in the disk
		const FloatType phi = InternalUtils::two_pi<FloatType> *u2;

		const FloatType x = r * std::cos(phi);
		const FloatType z = r * std::sin(phi);
		const FloatType y = std::sqrt(std::max(FloatType(0.0), FloatType(1.0) - u1)); // Y is up (height)

		// 2. Orthonormal basis for the normal
		Vec3 tangent, bitangent;
		CreateOrthonormalBasis(normal, tangent, bitangent);

		// 3. Transform to world space: [t n b] * local coords
		return Normalized(Vec3(
			x * tangent +
			y * normal +
			z * bitangent
		));
	}

	// Vector Based Scattering (Christensen 2005)
	// Produces a continuous distribution between specular and diffused vectors.
	// @param diffusion must be in range [0, 1]
	template<class Vec3, class RandomFloatFunc>
	static Vec3 VectorBasedScatter2(const Vec3& specular, const Vec3& normal, float diffusion, RandomFloatFunc getRandomFloat = InternalUtils::RandFloat<Internal::FloatOf<Vec3>>)
	{
		using FloatType = Internal::FloatOf<Vec3>;
		const Vec3 randomDirection = SampleHemisphereCosine(normal, getRandomFloat);
		return diffusion * randomDirection + (FloatType(1.0) - diffusion) * specular;
	}

	// Vector Based Scattering (Christensen 2005)
	// Produces a continuous distribution between specular and diffused vectors.
	// @param diffusion must be in range [0, 1]
	template<class Vec3, class RandomFloatFunc>
	static Vec3 VectorBasedScatter(const Vec3& incident, const Vec3& normal, float diffusion, RandomFloatFunc getRandomFloat = InternalUtils::RandFloat<Internal::FloatOf<Vec3>>)
	{
		const Vec3 specular = SpecularReflection(incident, normal);
		return VectorBasedScatter2(specular, normal, diffusion, getRandomFloat);
	}

	template<class Vec3>
	struct VBSSample
	{
		Vec3 OutDirection;
		float PDF;
		bool bIsSpecular;
	};

	// Vector Based Scattering (Christensen 2005)
	// Produces a discrete distribution between specular and diffused vectors,
	// which allows to also extract PDF (Probability Density Function)
	// suitable for BDTP.
	// 
	// @param diffusion (d) must be in range [0, 1]
	// 
	// @returns with probability (1-d): deterministic specular direction;
	// 
	// with probability d: a cosine-weighted sample
	template<class Vec3, class RandomFloatFunc>
	static VBSSample<Vec3> VectorBasedScatterAndPDF2(const Vec3& specular,
													 const Vec3& normal,
													 float diffusion,
													 RandomFloatFunc getRandomFloat = InternalUtils::RandFloat<Internal::FloatOf<Vec3>>)
	{
		using FloatType = Internal::FloatOf<Vec3>;

		if (getRandomFloat() < FloatType(1.0) - diffusion) // --- specular branch
		{
			return {
				.OutDirection = specular,
				.PDF = FloatType(1.0) - diffusion, // dirac weight
				.bIsSpecular = true
			};
		}
		else // --- diffuse branch
		{
			const Vec3 outDirection = SampleHemisphereCosine(normal);
			return {
				.OutDirection = outDirection,
				.PDF = diffusion *
					std::max(FloatType(0.0), DotProduct(normal, outDirection)) *
					std::numbers::inv_pi_v<FloatType>,
				.bIsSpecular = false
			};
		}
	}

	// Vector Based Scattering (Christensen 2005)
	// Produces a discrete distribution between specular and diffused vectors,
	// which allows to also extract PDF (Probability Density Function)
	// suitable for BDTP.
	// 
	// @param diffusion (d) must be in range [0, 1]
	// 
	// @returns with probability (1-d): deterministic specular direction;
	// 
	// with probability d: a cosine-weighted sample
	template<class Vec3, class RandomFloatFunc>
	static VBSSample<Vec3> VectorBasedScatterAndPDF(const Vec3& incident,
													const Vec3& normal,
													float diffusion,
													RandomFloatFunc getRandomFloat = InternalUtils::RandFloat<Internal::FloatOf<Vec3>>)
	{
		const Vec3 specular = SpecularReflection(incident, normal);
		return VectorBasedScatterAndPDF2(specular, normal, diffusion, getRandomFloat);
	}


	// Acoustic Biderectional Reflectance Distribution Function (Durany et. al, 2015)
	// @param specular	specular direction vector, must be normalized
	// @param outgoing	outgoing angle, must be normalized
	// @param diffusion	diffusion or scattering coefficient must be in range [0, 1]
	// 
	// @returns probability of reflection in the outgoing direction
	template<class Vec3>
	static auto ABRDF(const Vec3& specular, const Vec3& outgoing, float diffusion) -> Internal::FloatOf<Vec3>
	{
		using FloatType = Internal::FloatOf<Vec3>;

		// Compute gamma (cosine between specular and outgoing)
		const FloatType gamma = DotProduct(specular, outgoing);
		
		static constexpr auto epsilon =
			std::numeric_limits<FloatType>::epsilon() * FloatType(2.0);

		if (diffusion <= epsilon)
		{
			// When diffusion -> 0, only outgoing directions
			// close to specularity are attainable.
			return FloatType(std::fabs(gamma - FloatType(1.0)) <= epsilon);
		}

		// Evaluate A-BRDF according to diffusion
		const FloatType d = diffusion;
		const FloatType oneMinusD = FloatType(1.0) - d;

		// Common sub-expressions
		const FloatType a2 = oneMinusD * oneMinusD;				// (1-d)^2
		const FloatType gamma2 = gamma * gamma;					// gamma^2

		// Domain test Eq. (20)
		const FloatType rootArg = a2 * gamma2 + FloatType(2.0) * d - FloatType(1.0);

		// ..if it's negative, no solution exists
		// (also guard against tiny negative round-off that would blow off sqrt())
		if (rootArg <= epsilon)
			return 0.0f;

		const FloatType sqrtTerm = std::sqrt(rootArg);
		const FloatType numer = FloatType(2.0) * rootArg;

		if (d >= FloatType(0.5)) // Eq. (21)
		{
			const FloatType denom1 = InternalUtils::four_pi<FloatType> * d * sqrtTerm;
			const FloatType term1 = numer / denom1;

			const FloatType term2 = (oneMinusD * gamma) / (InternalUtils::two_pi<FloatType> * d);

			return term1 + term2;
		}
		else // Eq. (22)
		{
			const FloatType denom = InternalUtils::two_pi<FloatType> * d * sqrtTerm;
			return numer / denom;
		}
	}

	// Acoustic Biderectional Reflectance Distribution Function (Durany et. al, 2015)
	// @param incident	incident vector, must be normalized
	// @param normal		surface normal vector, must be normalized
	// @param outgoing	outgoing angle, must be normalized
	// @param diffusion	diffusion or scattering coefficient must be in range [0, 1]
	// 
	// @returns probability of reflection in the outgoing direction
	template<class Vec3>
	static auto ABRDF(const Vec3& incident, const Vec3& normal, const Vec3& outgoing, float diffusion) -> Internal::FloatOf<Vec3>
	{
		// Compute specular direction (assume incident, normal, outgoing are normalized)
		const Vec3 specular = SpecularReflection(incident, normal);
		return ABRDF(specular, outgoing, diffusion);
	}

} // namespace JPL::Math
