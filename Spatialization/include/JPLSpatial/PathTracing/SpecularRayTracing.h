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
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/PathTracing/SpecularPathCache.h"
#include "JPLSpatial/PathTracing/SceneInterface.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"
#include "JPLSpatial/PathTracing/Math.h"

#include <span>

namespace JPL
{
	/// A very experimental specular ray tracing algorithm
	class SpecularRayTracing
	{
	public:
		static constexpr std::size_t cMaxOrder = 16;

		// TODO: in practice it will a set of listeners per source
		// TODO: make this agnostic of tracing direction
		template<class SceneType>
		inline void Trace(const SceneType& scene,
						  uint32 numberOfPrimaryRays,
						  const typename SceneType::Source& source,
						  std::span<const typename SceneType::Listener> listeners,
						  uint32 maxOrder,
						  SpecularPathCache<typename SceneType::Vec3>& pathCache)
		{
			using Source = typename SceneType::Source;
			using Listener = typename SceneType::Listener;
			using Ray = typename SceneType::Ray;
			using Intersection = typename SceneType::Intersection;
			using Vec3 = typename SceneType::Vec3;

			JPL_ASSERT(maxOrder <= cMaxOrder);

			// Path node ids
			StaticArray<int, cMaxOrder + 2> nodeCache;
			
			// Sequence of Image Sources
			StaticArray<Vec3, cMaxOrder + 1> IS;

			// Path hash starts with the source id
			SpecularPathId Ks;
			Ks.AddVertex(source.Id);

			for (uint32 i = 0; i < numberOfPrimaryRays; ++i)
			{
				// Take a copy for iterative hashing
				SpecularPathId K = Ks;

				// Sample primary outgoing ray
				Ray ray{
					.Origin = source.Position,
					.Direction = Math::InternalUtils::RandDirection<Vec3>()
				};

				// Add source as the first vertex
				nodeCache.clear();
				nodeCache.push_back(source.Id);

				// Add source position as the first image source
				IS.clear();
				IS.push_back(source.Position);

				// Trace rays up to 'maxOrder'
				for (uint32 d = 0; d < maxOrder; ++d)
				{
					Intersection hit;
					if (not scene.Intersect(ray, hit))
						break;

					// TODO: we may want to take into account or take a record
					//		of the materials here

					const int Tjd = hit.SurfaceID; // intersected surface
						
					// Add surface to the path
					nodeCache.push_back(Tjd);

					// Add surface to the path key hash
					K.AddVertex(static_cast<uint32>(Tjd));

					// Reflect the previous image source across the surface's plane
					IS.push_back(Math::GetImageSource(IS.back(), hit.Normal, hit.Position));

					// Small offset to avoide self intersection
					static constexpr float offset = 0.001f;

					// Generate next outgoing ray
					ray = {
						.Origin = hit.Position + hit.Normal * offset,
						.Direction = Math::SpecularReflection(ray.Direction, hit.Normal)
					};

					// Create space for a listener
					nodeCache.emplace_back();

					// Validate subpath for each listener
					for (const Listener& l : listeners)
					{
						// Add listener id to the path key hash
						SpecularPathId Kl = K;
						Kl.AddVertex(l.Id);

						// Assign listener as the last vertex
						nodeCache.back() = l.Id;

						// Add path to the cache if it hasn't been previously validated
						if (not pathCache.Contains(Kl))
						{
							// TODO: can we incrementally validate path "up to listener" and here just validate the listener endpoint?
							//		We'd lose some of the benefints of cache, but can it still be faster?

							// TODO: we need to either compute energy here while tracing, or store necessary data in cache to do it later

							/* TODO
								Params:
								- directional pattern (angle of emmision of the source)
								- geometry term (reflected energy loss)
								- air term (distance, HF attenuation)
							*/
							const bool bIsPathValid = SceneInterface::ValidatePath<Vec3>(scene, nodeCache, IS, l.Position);

							EnergyBands energyLoss(0.0f);
							Vec3 imageSourceVector = IS.back() - l.Position;

							if (bIsPathValid)
							{
								// Note: Distance-based attenuation has to be added as its own factor,
								// it doesn't work well as just a multiplier for our 4-band filter
								
								const float pathLength = Length(imageSourceVector);
								
								// this overrides `energyLoss` param, don't put it at the end
								AirAbsorption::ComputeForDistance(pathLength, energyLoss);
								
								SceneInterface::AccumulateMaterialAbsorption(scene, nodeCache, energyLoss);

								// TODO: here we might want to keep energy loss as dBs or convert to gains:
								//		- leaving as dBs makes it easy to then sort and use the most relevant paths for ER rendering
								//		- converting to gains makes it possible to apply distance-based attenuation here once,
								//			instead of later potentially every query even for unchanged paths
							}

							pathCache.Add(Kl, nodeCache, imageSourceVector, energyLoss, bIsPathValid);
						}
					}

					// Pop the listener to reuse same node list
					// for the next segment to be traced
					nodeCache.pop_back();
				}
			}
		}
	};
} // namespace JPL