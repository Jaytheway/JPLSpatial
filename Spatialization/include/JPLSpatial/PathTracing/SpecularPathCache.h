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
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/PathTracing/Math.h"
#include "JPLSpatial/PathTracing/SceneInterface.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"

#include <algorithm>
#include <functional>
#include <memory_resource>
#include <span>
#include <ranges>
#include <unordered_map>
#include <utility>
#include <vector>

namespace JPL
{
	//======================================================================
	/// A very experimental specular path cache.
	// Specular path cache is used to cache specular reflection paths from
	// previous frames, thereby reducing the cost involved in finding paths
	// on future frames, allowing fewer rays to be traced,
	// and improving performance
	template<class Vec3>
	class SpecularPathCache
	{
	public:
		using Scene = SceneInterface;
		using SPMap = std::pmr::unordered_map<SpecularPathId, SpecularPathData<Vec3>>;
		using SPEntry = std::pair<SpecularPathId, SpecularPathData<Vec3>>;
		using SPArray = std::pmr::vector<SPEntry>;

		static constexpr std::size_t cMaxOrder = 16; // TODO: make sure to use the same value across specular path classes

		//static_assert(sizeof(SpecularPathData<Vec3>) == JPL_CACHE_LINE_SIZE); // for no current reason

	public:
		SpecularPathCache() = default;
		~SpecularPathCache() noexcept;

		[[nodiscard]] JPL_INLINE std::size_t GetNumPaths() const { return mValidPaths.size() + mInvalidPaths.size(); }
		[[nodiscard]] JPL_INLINE std::size_t GetNumValidPaths() const { return mValidPaths.size(); }
		[[nodiscard]] JPL_INLINE std::size_t GetNumInvalidPaths() const { return mInvalidPaths.size(); }

		// TODO: we might want to switch to some kind of sparse array with HashTable
		//		because this Contains check is quite hot
		[[nodiscard]] JPL_INLINE bool Contains(SpecularPathId pathId) const;

		// Add path to cache
		inline void Add(SpecularPathId path,
						std::span<const int> nodes,
						const Vec3& imageSource, // last image source vector relative to receiver
						const EnergyBands& energy,
						bool isPathValid);

		// TODO: take into account that Reciever's movement requires only revalidation, while Source's movement requires ISs regeneration

		// Paths should be revalidated at the beginning of each simulation frame
		// in case any source, listener, or scene geometry moved during the time interval.
		// 
		// The paths that were invalid on the previous frame are removed from the cache,
		// while the valid paths are revalidated and stored again in the cache.
		template<class SceneType>
		void Validate(const SceneType& scene);

		JPL_INLINE std::span<const SPEntry> GetValidPaths() const { return mValidPaths; }
		JPL_INLINE std::span<const SPEntry> GetInvalidPaths() const { return mInvalidPaths; }

		JPL_INLINE void Clear()
		{
			ClearInvalidPaths();
			ClearValidPaths();
		}

	private:
		// Delete previously invalidated paths
		JPL_INLINE void ClearInvalidPaths()
		{
			for (auto&& [pathKey, path] : mInvalidPaths)
			{
				DeallocatePathData(path);
			}
			mInvalidPaths.clear();
		}

		// Delete valid paths
		JPL_INLINE void ClearValidPaths()
		{
			for (auto&& [pathKey, path] : mValidPaths)
			{
				DeallocatePathData(path);
			}
			mValidPaths.clear();
		}

		JPL_INLINE void DeallocatePathData(SpecularPathData<Vec3>& path)
		{
			GetDefaultPmrAllocator<int>().deallocate(path.Nodes.data(), path.Nodes.size());
		}

	private:
		// Note: we need specular path cache per source
		// (technically per source-listner pair, to compute IR later)
		
		// TODO: maybe use hash-table to speed up `Contains` queries that we do a lot
		SPArray mValidPaths{ GetDefaultMemoryResource() };
		SPArray mInvalidPaths{ GetDefaultMemoryResource() };
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	template<class Vec3>
	inline SpecularPathCache<Vec3>::~SpecularPathCache() noexcept
	{
		// TODO: we might want to use frame allocator (just let the user to provide one)
		auto allocator = GetDefaultPmrAllocator<int>();
		for (SPArray& pathsArray : { std::ref(mValidPaths), std::ref(mInvalidPaths) })
		{
			for (auto&& [id, data] : pathsArray)
			{
				allocator.deallocate(data.Nodes.data(), data.Nodes.size());
			}
		}
	}

	template<class Vec3>
	inline JPL_INLINE bool SpecularPathCache<Vec3>::Contains(SpecularPathId pathId) const
	{
		auto hasId = [pathId](const SPEntry& entry) { return entry.first == pathId; };
		return std::ranges::find_if(mValidPaths, hasId) != std::ranges::end(mValidPaths)
			|| std::ranges::find_if(mInvalidPaths, hasId) != std::ranges::end(mInvalidPaths);
	}

	template<class Vec3>
	inline void SpecularPathCache<Vec3>::Add(SpecularPathId path, std::span<const int> nodes, const Vec3& imageSource, const EnergyBands& energy, bool isPathValid)
	{
		JPL_ASSERT(not Contains(path));

		// TODO: frame allocator (?)
		int* nodeData = GetDefaultPmrAllocator<int>().allocate(nodes.size());
		std::copy(nodes.begin(), nodes.end(), nodeData);

		auto& pathsArray = isPathValid ? mValidPaths : mInvalidPaths;

		pathsArray.emplace_back(path,
								SpecularPathData<Vec3>{
			.Nodes = std::span(nodeData, nodes.size()),
				.Energy = energy,
				.ImageSource = imageSource,
				.bValid = isPathValid
		});
	}

	template<class Vec3>
	template<class SceneType>
	inline void SpecularPathCache<Vec3>::Validate(const SceneType& scene)
	{
		// TODO:
		//	if we could store paths as a tree,
		//	then if first node is invalidated,
		//	we could eliminate a whole branch

		// Delete previously invalidated paths
		ClearInvalidPaths();

		// Validated previously valid paths
		for (auto&& [pathKey, path] : mValidPaths)
		{
			JPL_ASSERT(path.Nodes.size() > 2,
					   "Path contains only source and listener verices, no reflections.");

			JPL_ASSERT(path.Nodes.size() <= cMaxOrder + 2,
					   "Path is deeper than max order allowed.");

			// The position of both, source or receiver may change,
			// in case of backtracing, ImageSource[0] is the listener, so we need to use the updated
			// positions of listener and sources and revalidate image sources here via the same surfaces
			path.bValid = SceneInterface::ValidatePath(scene, path);
		}

		// Move invalidated paths to invalid array
		{
			auto isValid = [](const SPEntry& entry)
			{
				return entry.second.bValid;
			};

			auto invalidRange = std::ranges::partition(mValidPaths, isValid);

#if 0
			//? do we need this, or should we just immediately delete the invalid paths
			mInvalidPaths.reserve(mInvalidPaths.size() + invalidRange.size());
			std::ranges::move(invalidRange, std::back_inserter(mInvalidPaths));
#else
			for (auto&& [pathKey, path] : invalidRange)
			{
				DeallocatePathData(path);
			}
#endif
			mValidPaths.erase(invalidRange.begin(), mValidPaths.end());
		}
	}

} // namespace JPL