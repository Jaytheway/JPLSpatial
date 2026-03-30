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
#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"
#include "JPLSpatial/PathTracing/Math.h"

#include <span>
#include <ranges>
#include <optional>

namespace JPL
{
	/// A very experimental scene interface for specular ray tracing
	class SceneInterface
	{
	public:
		static constexpr std::size_t cMaxOrder = 16; // TODO: make sure to use the same value across specular path classes

		template<class SceneType, class Vec3>
		[[nodiscard]] inline static std::optional<Vec3> ReflectPoint(const SceneType& scene,
																	 const Vec3& point,
																	 int surfaceId);

		template<class Vec3, class SceneType>
		static inline bool ValidatePath(const SceneType& scene,
										std::span<const int> triCache,
										std::span<const Vec3> imageSources,
										const Vec3& receiver);

		// Validate visibility of the reflected image source from the reciever
		template<class Vec3, class SceneType>
		static inline bool ValidatePath(const SceneType& scene,
										const SpecularPathData<Vec3>& path);

		template<class SceneType>
		static void AccumulateMaterialAbsorption(const SceneType& scene, std::span<const int> surfaces, EnergyBands& outEnergyLoss);

		template<class SceneType, class OnISsReadyCallback>
		static void ReconstructImageSources(const SceneType& scene, std::span<const int> pathNodes, const OnISsReadyCallback& callback);
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	template<class SceneType, class Vec3>
	inline std::optional<Vec3> SceneInterface::ReflectPoint(const SceneType& scene, const Vec3& point, int surfaceId)
	{
		Vec3 planeNormal, planePoint;
		if (scene.GetSurfacePlane(surfaceId, planeNormal, planePoint))
			return Math::GetImageSource(point, planeNormal, planePoint);

		return std::nullopt;
	}

	template<class Vec3, class SceneType>
	inline bool SceneInterface::ValidatePath(const SceneType& scene, std::span<const int> triCache, std::span<const Vec3> imageSources, const Vec3& receiver)
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

			// Small offset to avoid self intersection
			static constexpr float offset = 0.001f;

			R = hit.Position + hit.Normal * offset;
		}

		// Lastly check visibility between
		// source and first refleciton point
		return not scene.IsOccluded(R, imageSources[0]);
	}

	template<class Vec3, class SceneType>
	inline bool SceneInterface::ValidatePath(const SceneType& scene, const SpecularPathData<Vec3>& path)
	{
		bool bPathValid = false;

		ReconstructImageSources(scene, path.Nodes, [&](std::span<const Vec3> newImageSources, const Vec3& receiver)
		{
			if (path.Nodes.size() - 1 != newImageSources.size())
			{
				// Reflection order changed, the path is not valid
				bPathValid = false;
				return;
			}

			Vec3 R = receiver;

			// Check that we do intersect the correct surface sequance,
			// and nothing is obstructing visibility of the image source
			for (auto i = newImageSources.size() - 1; i >= 1; --i)
			{
				using Intersection = typename SceneType::Intersection;

				Intersection hit;
				if (not scene.Intersect(R, newImageSources[i], hit) || hit.SurfaceID != path.Nodes[i])
				{
					bPathValid = false;
					return;
				}

				// Small offset to avoid self intersection
				static constexpr float offset = 0.001f;

				R = hit.Position + hit.Normal * offset;
			}

			// Lastly check visibility between
			// source and first refleciton point
			bPathValid = not scene.IsOccluded(R, newImageSources[0]);
		});

		return bPathValid;
		}

		return true;
	}

	template<class SceneType>
	inline void SceneInterface::AccumulateMaterialAbsorption(const SceneType& scene, std::span<const int> surfaces, EnergyBands& outEnergyLoss)
	{
		// TODO: this should really be material + ABRDF, or just ABRDF
		for (int surfaceId : surfaces)
		{
			EnergyBands materialAbsorption;
			if (scene.GetMaterialAbsorption(surfaceId, materialAbsorption))
				outEnergyLoss += materialAbsorption;
		}
	}

	template<class SceneType, class OnISsReadyCallback>
	inline void SceneInterface::ReconstructImageSources(const SceneType& scene, std::span<const int> pathNodes, const OnISsReadyCallback& callback)
	{
		using Vec3 = typename SceneType::Vec3;

		const auto surfaces = pathNodes.subspan(0, pathNodes.size() - 1);	// -1 removes receiver
		const Vec3 source = scene.GetSourcePosition(pathNodes.front());
		const Vec3 receiver = scene.GetReceiverPosition(pathNodes.back());

		StaticArray<Vec3, cMaxOrder + 2> IS;
		IS.push_back(source);

		JPL_ASSERT(pathNodes.size() <= IS.size());

		for (const int Tjd : surfaces | std::views::drop(1)) // drop the source
		{
			auto imageSource = ReflectPoint(scene, IS.back(), Tjd);

			//if (JPL_ENSURE(imageSource.has_value(), "Invalid surface id in specular path."))
			if (imageSource.has_value())
			{
				IS.push_back(*imageSource);
			}
			else
			{
				// TODO: this technically could happen if the surface has been deleted,
				//		in which case we should just quietly invalidate the path
				JPL_ASSERT(false); //? temp assert. testing
				break;
			}
		}

		callback(std::span<const Vec3>(IS), receiver);
	}
} // namespace JPL
