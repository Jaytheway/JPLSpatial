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

#include "JPLSpatial/Core.h"

#include "JPLSpatial/AcousticMaterial.h"
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Math/DecibelsAndGain.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"
#include "JPLSpatial/PathTracing/SceneInterface.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"
#include "JPLSpatial/PathTracing/SpecularPathCache.h"
#include "JPLSpatial/PathTracing/SpecularRayTracing.h"

#include "../Utility/BoxTest.h"
#include "../Utility/SceneInterfaceUtils.h"

#include <gtest/gtest.h>

#include <cmath>
#include <vector>

namespace JPL
{
	class SpecularRayTracingTest : public testing::Test
	{
	protected:
		using Vec3 = MinimalVec3;
		using Scene = TestUtil::SceneInterfaceMock;
		using Ray = typename Scene::Ray;
		using Intersection = typename Scene::Intersection;
		using Source = typename Scene::Source;
		using Listener = typename Scene::Listener;
	protected:
		SpecularRayTracingTest() = default;

		static Scene GetBoxScene()
		{
			const auto room = Box{ Vec3::Zero(), Vec3{ 50.0f, 50.0f, 50.0f } };
			return Scene {
				.RayIntersect = TestUtil::IntersectStub{
					//! To make the multi-trace test fair for the algorithm, we need to simulate the same geometry, e.g. a box room
					.CastRay = TestUtil::CastRayFuncs::Box(room)
				},
				.LineIntersect = TestUtil::CastLineFuncs::Box(room)
			};
		}
	};

	TEST_F(SpecularRayTracingTest, SpecularPathCache_ContainsWorks)
	{
		SpecularPathId id;
		id.AddVertex(1);
		id.AddVertex(2);
		id.AddVertex(3);

		const std::vector<int> nodes{ 1, 2, 3 };

		Vec3 imageSource(0, 0, 0);
		EnergyBands energy;

		SpecularPathCache<Vec3> spc;
		spc.Add(id, nodes, imageSource, energy, true);

		EXPECT_TRUE(spc.Contains(id));
	}

	TEST_F(SpecularRayTracingTest, LowOrderAllBoxSurfacesAreFound)
	{
		static constexpr uint32 numPrimaryRays = 1000;

		Scene scene = GetBoxScene();

		SpecularRayTracing tracer;

		const Source source{ .Position = Vec3(0, 0, -10), .Id = 1 };
		const std::vector<Listener> listeners
		{
			Listener{ .Position = Vec3(0, 0, 0), .Id = 2}
		};

		for (const uint32 reflectionOrder : { 1, 2 })
		{
			SpecularPathCache<Vec3> spc;
			tracer.Trace(
				scene,
				numPrimaryRays,
				source,
				listeners,
				reflectionOrder,
				spc
			);

			EXPECT_EQ(spc.GetNumPaths(), std::pow(6, reflectionOrder))
				<< "Reflection order: " << reflectionOrder;
		}
	}
} // namespace JPL