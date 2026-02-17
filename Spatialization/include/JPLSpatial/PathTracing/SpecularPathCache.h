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
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/PathTracing/Math.h"
#include "JPLSpatial/PathTracing/SceneInterface.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"

#include <algorithm>
#include <concepts>
#include <memory_resource>
#include <optional>
#include <span>
#include <unordered_map>

namespace JPL
{
	//======================================================================
	/// A very experimental specular path cache.
	// Specular path cache is used to cache specular reflection paths from
	// previous frames, thereby reducing the cost involved in finding paths
	// on future frames, allowing fewer rays to be traced,
	// and improving performance
	template<class Vec3>
	class SpecularPathCache // TODO: make it generic
	{
	public:
		using Scene = SceneInterface;
		using SPMap = std::pmr::unordered_map<SpecularPathId, SpecularPathData<Vec3>>;

		static_assert(sizeof(SpecularPathData<Vec3>) == JPL_CACHE_LINE_SIZE); // for no current reason

	public:
		SpecularPathCache() = default;
		~SpecularPathCache() noexcept
		{
			// TODO: we might want to use frame allocator (just let the user to provide one)
			auto allocator = GetDefaultPmrAllocator<int>();
			for (auto&& [id, data] : mPaths)
				allocator.deallocate(data.Nodes.data(), data.Nodes.size());
		}

		template<class SceneType>
		[[nodiscard]] JPL_INLINE static std::optional<Vec3> ReflectPoint(const SceneType& scene,
																		 const Vec3& point,
																		 int surfaceId)
		{
			Vec3 planeNormal, planePoint;
			if (scene.GetSurfacePlane(surfaceId, planeNormal, planePoint))
				return Math::GetImageSource(point, planeNormal, planePoint);
			
			return std::nullopt;
		}

		[[nodiscard]] JPL_INLINE std::size_t GetNumPaths() const { return mPaths.size(); }

		[[nodiscard]] JPL_INLINE std::size_t GetNumValidPaths() const
		{
			std::size_t count = 0;
			for (const auto& [pathId, pathData] : mPaths)
				count += pathData.bValid;
			return count;
		}

		[[nodiscard]] JPL_INLINE bool Contains(SpecularPathId pathId) const
		{
			return mPaths.contains(pathId);
		}

		// Add path to cache
		JPL_INLINE void Add(SpecularPathId path,
							std::span<const int> nodes,
							const Vec3& imageSource,
							const EnergyBands& energy,
							bool isPathValid)
		{
			// TODO: do we want to compute path energy here, or during tracing to reduce the number of paths?

			// TODO: frame allocator (?)
			int* nodeData = GetDefaultPmrAllocator<int>().allocate(nodes.size());
			std::copy(nodes.begin(), nodes.end(), nodeData);

			mPaths.emplace(path,
						   SpecularPathData<Vec3>{
								.Nodes = std::span(nodeData, nodes.size()),
								.Energy = energy,
								.ImageSource = imageSource,
								.bValid = isPathValid
						   });
		}

		// TODO: take into account that Reciever's movement requires only revalidation, while Source's movement requires ISs regeneration

		// Paths should be revalidated at the beginning of each simulation frame
		// in case any source, listener, or scene geometry moved during the time interval.
		// 
		// The paths that were invalid on the previous frame are removed from the cache,
		// while the valid paths are revalidated and stored again in the cache.
		template<class SceneType>
		inline void Validate(const SceneType& scene)
		{
			// TODO:
			//	if we could store paths as a tree,
			//	then if first node is invalidated,
			//	we could eliminate a whole branch

			// TODO: we might want to take a copy of previously validated paths first

			static constexpr std::size_t cMaxOrder = 16; // TODO: make sure to use the same value across specular path classes
			
			StaticArray<Vec3, cMaxOrder * 2> IS;

			for (auto it = mPaths.begin(); it != mPaths.end();) // TODO: flat map (?)
			{
				SpecularPathData<typename SceneType::Vec3>& path = it->second;

				if (path.bValid)
				{
					JPL_ASSERT(path.Nodes.size() > 2,
							   "Path contains only source and listener verices, no reflections.");

					JPL_ASSERT(path.Nodes.size() <= cMaxOrder + 2,
							   "Path is deeper than max order allowed.");

					Vec3 Sk = scene.GetSourcePosition(path.Nodes.front()); // TODO: this semantics assumes forward tracing, no bueno
					Vec3 Ll = scene.GetListenerPosition(path.Nodes.back());

					IS.clear();
					IS.push_back(Sk);

					std::span<const int> surfaces(
						path.Nodes.data() + 1,
						path.Nodes.size() - 2
					);

					bool bAllSurfacesValid = true;

					for (const int Tjd : surfaces)
					{
						auto imageSource = ReflectPoint(scene, IS.back(), Tjd);

						//if (JPL_ENSURE(imageSource.has_value(), "Invalid surface id in specular path."))
						if (imageSource.has_value())
						{
							IS.push_back(*imageSource);
							bAllSurfacesValid &= true;
						}
						else
						{
							// TODO: this technically could happen if the surface has been deleted,
							//		in which case we should just quietly invalidate the path
							JPL_ASSERT(false); //? temp assert. testing
							bAllSurfacesValid = false;
						}
					}

					// TODO: should we just use two separate containres for valid vs invalid paths?
					path.bValid = bAllSurfacesValid && SceneType::ValidatePath(scene, surfaces, IS, Ll);

					++it;
				}
				else
				{
					GetDefaultPmrAllocator<int>().deallocate(it->second.Nodes.data(), it->second.Nodes.size());
					it = mPaths.erase(it);
				}
			}
		}

		template<class Predicate>
		void ForEachValidPath(Predicate predicate)
		{
			for (auto&& [pathId, pathData] : mPaths)
			{
				if (pathData.bValid)
				{
					if constexpr (std::invocable<Predicate, const SpecularPathId&, SpecularPathData<Vec3>>)
						std::invoke(predicate, pathId, pathData);
					else
						std::invoke(predicate, pathData);
				}
			}
		}

		template<class Predicate>
		void ForEachValidPath(Predicate predicate) const
		{
			for (const auto& [pathId, pathData] : mPaths)
			{
				if (pathData.bValid)
				{
					if constexpr (std::invocable<Predicate, const SpecularPathId&, SpecularPathData<Vec3>>)
						std::invoke(predicate, pathId, pathData);
					else
						std::invoke(predicate, pathData);
				}
			}
		}

	private:
		// TODO: we need specular path cache per source
		// (technically per source-listner pair, to compute IR later)
		SPMap mPaths{ GetDefaultMemoryResource() };
	};
} // namespace JPL
