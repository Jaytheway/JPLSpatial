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

#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/AcousticMaterial.h"

#include "../Utility/BoxTest.h"

#include <array>
#include <functional>
#include <random>
#include <iterator>

namespace JPL::TestUtil
{
	using Vec3 = MinimalVec3;

	static float RandFloat()
	{
		// seeding for reproducibility
		static std::mt19937 mt{ 345345809 };
		return std::generate_canonical<float, 32>(mt);
	}

	static Vec3 RandVec()
	{
		return Vec3{
			RandFloat() * 2.0f - 1.0f,
			RandFloat() * 2.0f - 1.0f,
			RandFloat() * 2.0f - 1.0f
		};
	}

	static Vec3 RandUnitVec()
	{
		return RandVec().Normalize();
	}

	struct Dir
	{
		static inline const Vec3 Forward{ 0.0f, 0.0f, -1.0f };
		static inline const Vec3 Backward{ 0.0f, 0.0f, 1.0f };
		static inline const Vec3 Left{ -1.0f, 0.0f, 0.0f };
		static inline const Vec3 Right{ 1.0f, 0.0f, 0.0f };
		static inline const Vec3 Up{ 0.0f, 1.0f, 0.0f };
		static inline const Vec3 Down{ 0.0f, -1.0f, 0.0f };

		static inline Vec3 Rand() { return RandUnitVec(); }
	};

	struct SceneInterfaceMock
	{
		using Vec3 = MinimalVec3;

		Box<Vec3> box{ Vec3::Zero(), Vec3{ 50.0f, 50.0f, 50.0f } };

		struct Ray
		{
			Vec3 Origin;
			Vec3 Direction;
		};

		struct Intersection
		{
			Vec3 Normal;
			Vec3 Position;
			int Material;
			int SurfaceID;
		};

		struct Source
		{
			Vec3 Position; //? temp for testing
			uint32 Id;
		};

		struct Listener
		{
			Vec3 Position; //? temp for testing
			uint32 Id;
		};

		std::function<bool(const Ray&, Intersection&)> RayIntersect =
			[](const Ray&, Intersection&)
		{
			return false;
		};

		std::function<bool(const Vec3&, const Vec3&, Intersection&)> LineIntersect =
			[](const Vec3&, const Vec3&, Intersection&)
		{
			return false;
		};

		inline bool Intersect(const Ray& ray, Intersection& outIntersection) const
		{
			return RayIntersect(ray, outIntersection);
		}

		inline bool Intersect(const Vec3& posA, const Vec3& posB, Intersection& outIntersection) const
		{
			return LineIntersect(posA, posB, outIntersection);
		}

		inline bool IsOccluded(const Vec3&, const Vec3&) const
		{
			return false;
		}

		inline float GetMaterialFactor(int) const
		{
			return 0.6f; // 0 results in fully specular reflections
		}

		inline bool GetMaterialAbsorption(int surfaceId, EnergyBands& outAbsorption) const
		{
			//! for now just tesing something reasonably absorptive
			static const auto material = AcousticMaterial::Get("ConcreteBlockRough");
			outAbsorption = material->Coeffs;
			return true;
		}

		std::function<Vec3(const Vec3&, const Intersection&)> SampleTraceDirection =
			[](const Vec3& incomingDirection, const Intersection& intersection)
		{
			return Dir::Forward;
		};
	};

	struct CastRayFuncs
	{
		using Vec3 = MinimalVec3;

		using RayCastFunc = std::function<bool(const SceneInterfaceMock::Ray&, SceneInterfaceMock::Intersection&)>;

		static bool BoxCast(const Box<Vec3>& box,
							const SceneInterfaceMock::Ray& ray,
							SceneInterfaceMock::Intersection& outIntersection,
							float maxDistance = std::numeric_limits<float>::infinity())
		{
			const auto hit = box.CastRay(ray.Origin, ray.Direction, maxDistance);
			if (hit.bHit)
			{
				outIntersection.Normal = DotProduct(ray.Direction, hit.Normal) > 0.0f ? -hit.Normal : hit.Normal;
				outIntersection.Position = hit.Position;
				outIntersection.Material = 1;

				static const std::array<Vec3, 6> surfaces{
					Vec3{ -1, 0, 0 },
					Vec3{ 1, 0, 0 },
					Vec3{ 0,-1, 0 },
					Vec3{ 0, 1, 0 },
					Vec3{ 0, 0,-1 },
					Vec3{ 0, 0, 1 }
				};

				// Set SurfaceID to the index of the box face that was hit
				outIntersection.SurfaceID =
					static_cast<int>(std::distance(surfaces.begin(),
												   std::find(surfaces.begin(), surfaces.end(), hit.Normal)));

				return true;
			}
			return false;
		}

		static RayCastFunc Box(const Box<Vec3>& box)
		{
			using namespace std::placeholders;
			return std::bind(BoxCast, box, _1, _2, std::numeric_limits<float>::infinity());
		}

		/*static RayCastFunc Box(const Box<Vec3>& box)
		{
			return std::bind_front(BoxCast, box);
		}*/
	};

	struct CastLineFuncs
	{
		using Vec3 = MinimalVec3;
		using Intersection = typename SceneInterfaceMock::Intersection;
		using Ray = typename SceneInterfaceMock::Ray;

		using LineCastFunc = std::function<bool(const Vec3&, const Vec3&, Intersection&)>;

		static LineCastFunc Box(const Box<Vec3>& box)
		{
			return [=](const Vec3& pointA, const Vec3& pointB, Intersection& outIntersection)
			{
				const Vec3 line = pointB - pointA;
				const float distance = line.Length();
				return CastRayFuncs::BoxCast(
					box,
					Ray{ .Origin = pointA, .Direction = line / distance},
					outIntersection,
					distance);
			};
		}
	};


	class GetPositionFuncs
	{
	public:
		using PositionFunc = std::function<Vec3(const SceneInterfaceMock::Ray&)>;

		//! This distance parameter will determine sample bin distribution based on propagation time
		static PositionFunc FixedDistance(float distance)
		{
			return [distance](const SceneInterfaceMock::Ray& ray)
			{
				return ray.Origin + ray.Direction * distance;
			};
		};

		static PositionFunc RandomDistance(float min, float max)
		{
			return [min, max](const SceneInterfaceMock::Ray& ray)
			{
				return ray.Origin + ray.Direction * (min + RandFloat() * (max - min));
			};
		};

		static PositionFunc TimeDistance(float time)
		{
			return [time](const SceneInterfaceMock::Ray& ray)
			{
				return ray.Origin + ray.Direction * (time * JPL_SPEED_OF_SOUND);
			};
		};
	};

	class GetNormalFuncs
	{
	public:

		static Vec3 RandomDir(const SceneInterfaceMock::Ray& ray) { return RandUnitVec(); };
		static Vec3 ReflectBack(const SceneInterfaceMock::Ray& ray) { return -ray.Direction; };
		static Vec3 ReflectBackRnd(const SceneInterfaceMock::Ray& ray)
		{
			// TODO: here we could "bounce" based on surface normal (reflection), or shadow region (diffraction)
			Vec3 normal = RandUnitVec();
			// e.g. this is roughly mimicking surface reflection
			while (DotProduct(normal, ray.Direction) > 0.0f)
				normal = RandUnitVec();
			return normal;
		};
	};

	struct IntersectStub
	{
		std::function<Vec3(const SceneInterfaceMock::Ray&)> GetNormal = GetNormalFuncs::ReflectBack;
		std::function<Vec3(const SceneInterfaceMock::Ray&)> GetPosition = GetPositionFuncs::FixedDistance(10.0f);
		std::function<bool(const SceneInterfaceMock::Ray&)> IsIntersected = [](const SceneInterfaceMock::Ray& ray)
		{
			return true;
		};

		// Can be overriden by the test
		std::function<bool(const SceneInterfaceMock::Ray&, SceneInterfaceMock::Intersection&)> CastRay;

		bool operator()(const SceneInterfaceMock::Ray& ray,
						SceneInterfaceMock::Intersection& outIntersection,
						float maxDistance = std::numeric_limits<float>::infinity())
		{
			if (CastRay)
				return CastRay(ray, outIntersection);

			if (!IsIntersected(ray))
				return false;

			// just use byte hash of Normal for surface ID, if not handled in CastRay()
			auto hashBytes = [](const void* inData, uint32_t inSize, uint64 inSeed = 0xcbf29ce484222325UL) -> uint32_t
			{
				uint64_t hash = inSeed;
				for (const uint8_t* data = reinterpret_cast<const uint8*>(inData); data < reinterpret_cast<const uint8_t*>(inData) + inSize; ++data)
				{
					hash = hash ^ uint64_t(*data);
					hash = hash * 0x100000001b3UL;
				}

				return 0x811C9DC5u ^ (uint32_t(hash) ^ uint32_t(hash >> 32))
					+ 0x9e3779b9u + (0x811C9DC5u << 6) + (0x811C9DC5u >> 2);
			};

			outIntersection.Position = GetPosition(ray);
			outIntersection.Normal = GetNormal(ray);
			outIntersection.Material = 1;
			outIntersection.SurfaceID = hashBytes(&outIntersection.Normal, static_cast<uint32_t>(sizeof(Vec3)));
			return true;
		}
	};

	static float GetMaterialFactor(int)
	{
		return 0.0f; // 0 results in fully specular reflections
	}

	struct NextDirectionFunc
	{
		static Vec3 HemisphereCosine(const Vec3& incomingDirection, const SceneInterfaceMock::Intersection& intersection)
		{
			return Math::SampleHemisphereCosine(intersection.Normal, TestUtil::RandFloat);
		}

		static Vec3 ReflectBack(const Vec3& incomingDirection, const SceneInterfaceMock::Intersection& intersection)
		{
			return -incomingDirection;
		}

		static Vec3 ReflectOfSurface(const Vec3& incomingDirection, const SceneInterfaceMock::Intersection& intersection)
		{
			return Math::SpecularReflection(incomingDirection, intersection.Normal);
		}

		static Vec3 VectorBasedScattering(const Vec3& incomingDirection, const SceneInterfaceMock::Intersection& intersection)
		{
			// TODO: maybe parametrize scattering coefficient
			return Math::VectorBasedScatter(incomingDirection, intersection.Normal, GetMaterialFactor(intersection.Material), TestUtil::RandFloat);
		}
	};
} // namespace JPL::Test