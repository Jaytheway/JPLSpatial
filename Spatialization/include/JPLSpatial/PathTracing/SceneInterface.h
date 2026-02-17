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
#include "JPLSpatial/PathTracing/SpecularPath.h"

#include <span>

namespace JPL
{
	/// A very experimental scene interface for specular ray tracing
	class SceneInterface
	{
	public:
		// Validate visibility of the reflected image source from the reciever
		template<class Vec3, class SceneType>
		static inline bool ValidatePath(const SceneType& scene,
										std::span<const int> triCache,
										std::span<const Vec3> imageSources,
										const Vec3& receiver)
		{
			using Intersection = typename SceneType::Intersection;

			Vec3 R = receiver;

			// Check that we do intersect the correct surface sequance,
			// and nothing is obstructing visibility of the image source
			for (auto i = imageSources.size() - 1; i >= 1; --i)
			{
				Intersection hit;
				if (not scene.Intersect(R, imageSources[i], hit) || hit.SurfaceID != triCache[i])
					return false;

				// Small offset to avoide self intersection
				static constexpr float offset = 0.001f;

				R = hit.Position + hit.Normal * offset;
			}

			// Lastly check visibility between
			// source and first refleciton point
			return not scene.IsOccluded(R, imageSources[0]);

			// TODO: optimizations
			return true;
		}

		// TODO: this should really be material + ABRDF, or just ABRDF
		template<class SceneType>
		static void AccumulateMaterialAbsorption(const SceneType& scene, std::span<const int> surfaces, EnergyBands& outEnergyLoss)
		{
			for (int surfaceId : surfaces)
			{
				EnergyBands materialAbsorption;
				if (scene.GetMaterialAbsorption(surfaceId, materialAbsorption))
					outEnergyLoss += materialAbsorption;
			}
		};
	};
	
} // namespace JPL