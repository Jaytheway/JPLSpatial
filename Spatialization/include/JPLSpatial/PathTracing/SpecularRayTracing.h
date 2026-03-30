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
#include "JPLSpatial/Algo/Algorithm.h"
#include "JPLSpatial/Containers/StaticArray.h"
#include "JPLSpatial/Containers/ScratchHashSet.h"
#include "JPLSpatial/PathTracing/SpecularPathCache.h"
#include "JPLSpatial/PathTracing/SceneInterface.h"
#include "JPLSpatial/PathTracing/SpecularPath.h"
#include "JPLSpatial/PathTracing/Math.h"

#include <concepts>
#include <memory_resource>
#include <span>
#include <ranges>
#include <vector>
#include <utility>

//? temp. move somewhere reasonable
#ifndef JPL_PROFILE
	#define JPL_PROFILE(...)
#endif // !JPL_PROFILE

#ifndef JPL_PROFILE_SET_INT
	#define JPL_PROFILE_SET_INT(Stat, Value)
#endif // !JPL_PROFILE_SET_INT

#ifndef JPL_PROFILE_SET_FLOAT
	#define JPL_PROFILE_SET_FLOAT(Stat, Value)
#endif // !JPL_PROFILE_SET_FLOAT


#ifndef JPL_HAS_FUNCTION
#define JPL_HAS_FUNCTION(Type, MemberFunctionCallPrototype)\
requires(Type Object){ Object.MemberFunctionCallPrototype; }
#endif

#ifndef JPL_HAS_FUNCTION_R
#define JPL_HAS_FUNCTION_R(Type, MemberFunctionCallPrototype, ReturnType)\
requires(Type Object) { { Object.MemberFunctionCallPrototype } -> std::same_as<ReturnType>; }
#endif

namespace JPL
{
	//==========================================================================
	/// Generic parameters used in different kinds of traces
	struct TraceParameters
	{
		uint32 NumPrimaryRays;
		uint32 MaxTraceOrder;
		float MaxRayLength;
	};

	/// Node of a traced paths
	template<class IntersectionType>
	struct alignas(JPL_CACHE_LINE_SIZE) TraceNode
	{
		// Trace hit data for this node
		IntersectionType Hit;

		// Surface hash up to this node, or just this node (depends on context)
		uint32 Hash;
	};

	/// Sequence of traced path intersections
	template<class IntersectionType>
	struct alignas(JPL_CACHE_LINE_SIZE) TracedPath
	{
		std::pmr::vector<TraceNode<IntersectionType>> Nodes;
	};

	/// Result of tracing paths
	template<class IntersectionType>
	struct TraceResults
	{
		std::pmr::vector<TracedPath<IntersectionType>> Paths;
	};

	//==========================================================================
	/// A very experimental specular ray tracing algorithm
	class SpecularRayTracing
	{
	public:
		static constexpr std::size_t cMaxOrder = 16;

		/// Trace paths from source position based on TraceParameters
		/// If template parameter `bPathHashCombine` set to `true`,
		/// hash of each node is a combination "up to" this node,
		/// otherwise it's a hash of this particular intersection.
		template<class SceneType, bool bPathHashCombine = true>
		static void Trace(const SceneType& sceneInterface,
						  const typename SceneType::Vec3& sourcePosition,
						  const TraceParameters& parameters,
						  TraceResults<typename SceneType::Intersection>& outTraceResults);

		/// Process traces from source direction
		template<class SceneType, class SpecularPathCacheContainer> // SpecularPathCacheContainer must have operator[] overloaded to access cache for a receiver index
		static void ProcessTraces(SceneType& sceneInterface,
								  const typename SceneType::SourceData& sourceData,
								  TraceResults<typename SceneType::Intersection>& traces,
								  std::span<const typename SceneType::ReceiverData> receiverData,
								  SpecularPathCacheContainer& caches);

		/// Process bidirectional traces
		template<class SceneType, class SpecularPathCacheContainer> // SpecularPathCacheContainer must have operator[] overloaded to access cache for a receiver index
		static void ProcessTraces(SceneType& sceneInterface,
								  const typename SceneType::SourceData& sourceData,
								  TraceResults<typename SceneType::Intersection>& sourceTraces,
								  std::span<const typename SceneType::ReceiverData> receiverData,
								  std::span<TraceResults<typename SceneType::Intersection>> receiverTraces,
								  SpecularPathCacheContainer& caches);

	private:
		/// Validate unidirectional paths 
		template<class SceneType>
		static bool ValidatePathForReceiver(const SceneType& sceneInterface,
											std::span<const TraceNode<typename SceneType::Intersection>> nodes, // does not include source/receiver
											std::span<const typename SceneType::Vec3> imageSources,
											const typename SceneType::Vec3& receiver);

		/// Validate bidirectional paths
		template<class SceneType>
		static bool ValidatePathForReceiver(const SceneType& sceneInterface,
											std::span<const TraceNode<typename SceneType::Intersection>> fwdNodes,
											std::span<const TraceNode<typename SceneType::Intersection>> bwdNodes,
											std::span<const typename SceneType::Vec3> imageSources,
											const typename SceneType::Vec3& receiver);

		/// Accumulate material absorption for a set of surfaces
		template<class SceneType>
		static void AccumulateMaterialAbsorption(const SceneType& sceneInterface,
												 std::span<const TraceNode<typename SceneType::Intersection>> surfaces,
												 EnergyBands& outEnergyLoss);

		/// Construct image sources from the source psition through a sequence of forward traced surfaces
		template<class SceneType>
		static void ConstructImageSources(const SceneType& sceneInterface,
										  const typename SceneType::Vec3& sourcePosition,
										  std::span<const TraceNode<typename SceneType::Intersection>> path,
										  std::span<typename SceneType::Vec3> outImageSources);

		/// Construct image sources from the source psition through a sequence of forard and backward traced surfaces
		template<class SceneType>
		static void ConstructImageSources(const SceneType& sceneInterface,
										  const typename SceneType::Vec3& sourcePosition,
										  std::span<const TraceNode<typename SceneType::Intersection>> fwdPath,
										  std::span<const TraceNode<typename SceneType::Intersection>> bwdPath,
										  std::span<typename SceneType::Vec3> outImageSources);
		
		struct TraceInfo
		{
			uint32 PathCount;
			uint32 TotalNumSubpaths;
			uint32 MaxOrder;
			uint32 TotalNumImageSources;

			template<class IntersectionType>
			static TraceInfo Parse(const TraceResults<IntersectionType>& traces);
		};

		/// Remove invalid/empty paths, parse TraceInfo about TraceResults
		template<class IntersectionType>
		static TraceInfo PreprocessTraces(TraceResults<IntersectionType>& traces);

	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	template<class SceneType>
	inline bool SpecularRayTracing::ValidatePathForReceiver(const SceneType& sceneInterface,
															std::span<const TraceNode<typename SceneType::Intersection>> nodes,
															std::span<const typename SceneType::Vec3> imageSources,
															const typename SceneType::Vec3& receiver)
	{
		typename SceneType::Vec3 R = receiver;

		// Check that we do intersect the correct surface sequance,
		// and nothing is obstructing visibility of the image source
		for (auto i = imageSources.size() - 1; i >= 1; --i)
		{
			typename SceneType::Intersection hit;

			if (not sceneInterface.Intersect(R, imageSources[i], hit) || not sceneInterface.IsSameSurface(hit, nodes[i - 1].Hit))
				return false;

			// Small offset to avoid self intersection
			static constexpr float offset = 0.001f;

			R = sceneInterface.GetPosition(hit) + sceneInterface.GetNormal(hit) * offset;
		}

		// Lastly check visibility between
		// source and first refleciton point
		return not sceneInterface.IsOccluded(R, imageSources[0]);
	}

	template<class SceneType>
	inline bool SpecularRayTracing::ValidatePathForReceiver(const SceneType& sceneInterface,
															std::span<const TraceNode<typename SceneType::Intersection>> fwdNodes,
															std::span<const TraceNode<typename SceneType::Intersection>> bwdNodes,
															std::span<const typename SceneType::Vec3> imageSources,
															const typename SceneType::Vec3& receiver)
	{
		typename SceneType::Vec3 R = receiver;

		// Check that we do intersect the correct surface sequance,
		// and nothing is obstructing visibility of the image source

		// Going backwards, first check backward subpath
		int32 imageSouceIdx = imageSources.size() - 1;
		for (uint32 bwdIdx = 0; imageSouceIdx >= (1 + fwdNodes.size()); --imageSouceIdx, ++bwdIdx)
		{
			typename SceneType::Intersection hit;

			if (not sceneInterface.Intersect(R, imageSources[imageSouceIdx], hit) ||
				not sceneInterface.IsSameSurface(hit, bwdNodes[bwdIdx].Hit))
				return false;

			// Small offset to avoid self intersection
			static constexpr float offset = 0.001f;

			R = sceneInterface.GetPosition(hit) + sceneInterface.GetNormal(hit) * offset;
		}

		// ...then check forward subpath
		for (int32 fwdIdx = (fwdNodes.size() - 1); imageSouceIdx >= 1; --imageSouceIdx, --fwdIdx)
		{
			typename SceneType::Intersection hit;

			if (not sceneInterface.Intersect(R, imageSources[imageSouceIdx], hit) ||
				not sceneInterface.IsSameSurface(hit, fwdNodes[fwdIdx].Hit))
				return false;

			// Small offset to avoid self intersection
			static constexpr float offset = 0.001f;

			R = sceneInterface.GetPosition(hit) + sceneInterface.GetNormal(hit) * offset;
		}

		// Lastly check visibility between
		// source and first refleciton point
		return not sceneInterface.IsOccluded(R, imageSources[0]);
	}

	template<class SceneType>
	inline void SpecularRayTracing::AccumulateMaterialAbsorption(const SceneType& sceneInterface,
																 std::span<const TraceNode<typename SceneType::Intersection>> surfaces,
																 EnergyBands& outEnergyLoss)
	{
		for (const auto& surfaceHit : surfaces)
		{
			EnergyBands materialAbsorption;
			if (sceneInterface.GetMaterialAbsorption(surfaceHit, materialAbsorption))
				outEnergyLoss += materialAbsorption;
		}
	}

	template<class SceneType>
	inline void SpecularRayTracing::ConstructImageSources(const SceneType& sceneInterface,
														  const typename SceneType::Vec3& sourcePosition,
														  std::span<const TraceNode<typename SceneType::Intersection>> path,
														  std::span<typename SceneType::Vec3> outImageSources)
	{
		JPL_ASSERT(outImageSources.size() >= path.size() + 1);

		outImageSources[0] = sourcePosition;
		for (int32 i = 0; i < path.size(); ++i)
		{
			const TraceNode<typename SceneType::Intersection>& node = path[i];
			const typename SceneType::Vec3 imageSource = Math::GetImageSource(outImageSources[i],
																			  sceneInterface.GetNormal(node.Hit),
																			  sceneInterface.GetPosition(node.Hit));
			outImageSources[i + 1] = imageSource;
		}
	}

	template<class SceneType>
	inline void SpecularRayTracing::ConstructImageSources(const SceneType& sceneInterface,
														  const typename SceneType::Vec3& sourcePosition,
														  std::span<const TraceNode<typename SceneType::Intersection>> fwdPath,
														  std::span<const TraceNode<typename SceneType::Intersection>> bwdPath,
														  std::span<typename SceneType::Vec3> outImageSources)

	{
		JPL_ASSERT(outImageSources.size() >= fwdPath.size() + bwdPath.size() + 1);

		outImageSources[0] = sourcePosition;

		uint32 fwdIdx = 0;
		for (; fwdIdx < fwdPath.size(); ++fwdIdx)
		{
			const TraceNode<typename SceneType::Intersection>& node = fwdPath[fwdIdx];
			const typename SceneType::Vec3 imageSource = Math::GetImageSource(outImageSources[fwdIdx],
																			  sceneInterface.GetNormal(node.Hit),
																			  sceneInterface.GetPosition(node.Hit));
			outImageSources[fwdIdx + 1] = imageSource;
		}

		// Back-traced path we iterate backwards
		for (int32 bwdIdx = bwdPath.size() - 1; bwdIdx >= 0; --bwdIdx, ++fwdIdx)
		{
			const TraceNode<typename SceneType::Intersection>& node = bwdPath[bwdIdx];
			const typename SceneType::Vec3 imageSource = Math::GetImageSource(outImageSources[fwdIdx],
																			  sceneInterface.GetNormal(node.Hit),
																			  sceneInterface.GetPosition(node.Hit));
			outImageSources[fwdIdx + 1] = imageSource;
		}
	}

	//==========================================================================
	template<class SceneType, bool bPathHashCombine>
	inline void SpecularRayTracing::Trace(const SceneType& sceneInterface,
										  const typename SceneType::Vec3& sourcePosition,
										  const TraceParameters& parameters,
										  TraceResults<typename SceneType::Intersection>& outTraceResults)
	{
		using Ray = typename SceneType::Ray;
		using Vec3 = typename SceneType::Vec3;
		using Intersection = typename SceneType::Intersection;
		using PathType = TracedPath<Intersection>;

		outTraceResults.Paths.resize(parameters.NumPrimaryRays);
		for (PathType& path : outTraceResults.Paths) //? not ideal, we still make 100 allocations here
		{
			path.Nodes.reserve(parameters.MaxTraceOrder); // The max order may be large when late reverberation is implemented
		}

		auto traceRay = [&](int32 index)
		{
			PathType& path = outTraceResults.Paths[index];

			// Sample primary outgoing ray
			Ray ray(sourcePosition, Math::InternalUtils::RandDirection<Vec3>());
			uint32 hash = Hash::cStartSeed;

			// Trace rays up to 'MaxOrder'
			for (uint32 d = 0; d < parameters.MaxTraceOrder; ++d)
			{
				Intersection hit;
				if (not sceneInterface.Intersect(ray, parameters.MaxRayLength, hit))
				{
					break;
				}

				// Reset hash for each node, if not combining
				if constexpr (not bPathHashCombine)
				{
					hash = Hash::cStartSeed;
				}

				HashCombine32(hash, sceneInterface.GetHash(hit));

				path.Nodes.push_back({ .Hit = hit, .Hash = hash });

				// Small offset to avoide self intersection
				static constexpr float offset = 0.001f;

				// Generate next outgoing ray
				ray.Origin = sceneInterface.GetPosition(hit) + sceneInterface.GetNormal(hit) * offset;
				ray.Direction = Math::SpecularReflection(ray.Direction, sceneInterface.GetNormal(hit));
			}
		};

		// Do the scene trace
		{
			// Give the caller a chance to prepare for a potentially concurrent set of traces
			// (e.g. lock physics scene)
			if constexpr (JPL_HAS_FUNCTION(SceneType, PreTrace()))
			{
				sceneInterface.PreTrace();
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, ParallelFor(uint32(42u), traceRay)))
			{
				sceneInterface.ParallelFor(parameters.NumPrimaryRays, traceRay);
			}
			else
			{
				for (uint32 i = 0; i < parameters.NumPrimaryRays; ++i)
				{
					traceRay(i);
				}
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, PostTrace()))
			{
				sceneInterface.PostTrace();
			}
		}
	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessTraces(SceneType& sceneInterface,
												  const typename SceneType::SourceData& sourceData,
												  TraceResults<typename SceneType::Intersection>& traces,
												  std::span<const typename SceneType::ReceiverData> receiverData,
												  SpecularPathCacheContainer& caches)
	{
		JPL_PROFILE(SpecularRayTracing_ProcessTraces);

		using Vec3 = typename SceneType::Vec3;
		using Intersection = typename SceneType::Intersection;
		using PathType = TracedPath<Intersection>;
		using PathNodeType = TraceNode<Intersection>;
		using ReceiverData = typename SceneType::ReceiverData;
		using SourceData = typename SceneType::SourceData;

		const TraceInfo traceInfo = [&]
		{
			JPL_PROFILE(SpecularRayTracing_PreprocessTraces);
			return PreprocessTraces(traces);
		}();

		auto& paths = traces.Paths;
		
		std::pmr::vector<uint32> ISCacheTable(JPL::GetDefaultMemoryResource());
		ISCacheTable.reserve(paths.size());

		std::pmr::vector<Vec3> imageSourceCache(traceInfo.TotalNumImageSources, JPL::GetDefaultMemoryResource());
		{
			int32 ISCacheOffset = 0;
			Vec3* ISCacheData = imageSourceCache.data();

			// Construct Image Sources
			for (uint32 i = 0; i < paths.size(); ++i)
			{
				const PathType& path = paths[i];

				const uint32 imageSourceSubpathSize = path.Nodes.size() + 1; // +1 for source
				std::span<Vec3> imageSourceSubpath(&ISCacheData[ISCacheOffset], imageSourceSubpathSize);

				ConstructImageSources(sceneInterface, sourceData.Position, path.Nodes, imageSourceSubpath); //! This is not touching physics data at all (no need for a lock)

				ISCacheTable.push_back(ISCacheOffset);
				ISCacheOffset += imageSourceSubpathSize;
			}
		}

		// Subpaths that don't exist yet in the Specular Cache
		// and need to be validated and added to it.
		struct alignas(JPL_CACHE_LINE_SIZE) NewSubpathEntry
		{
			// Set in in validation step
			JPL::EnergyBands EnergyLoss;
			bool bIsValid;

			std::span<const PathNodeType> Subpath;
			std::span<const Vec3> SubpathIS;
			uint32 ReceiverIdx;
			JPL::SpecularPathId PathId;
		};
		std::pmr::vector<NewSubpathEntry> newSubpaths(JPL::GetDefaultMemoryResource());
		newSubpaths.reserve(traceInfo.TotalNumSubpaths * receiverData.size()); // TODO: this could potentially be huge (50 rays * 3 depth * 10 sources * 64 bytes = 96k bytes)
		//! if this does turn out to be huge, we could do our validation below per receiver and move this array to per receiver loop as well

		// Check for duplicate/existing subpaths per receiver
		{
			JPL_PROFILE(SpecularRayTracing_CreateUniquePaths);

			const std::size_t bufferSize = ScratchHashSetIdentity::GetRequiredMemorySize(traceInfo.TotalNumSubpaths);
			std::pmr::monotonic_buffer_resource resource(bufferSize, JPL::GetDefaultMemoryResource());

			// Allocate temp scratch buffer
			ScratchHashSetIdentity subpathsChecked(resource.allocate(bufferSize), bufferSize, traceInfo.TotalNumSubpaths);

			for (uint32 i = 0; i < paths.size(); ++i) // For each Path
			{
				const PathType& path = paths[i];
				std::span<const Vec3> imageSources(&imageSourceCache[ISCacheTable[i]], path.Nodes.size() + 1);

				// Validate subpath for receiver
				for (uint32 n = 0; n < path.Nodes.size(); ++n) // For each Subpath in Path
				{
					std::span<const PathNodeType> subpath(path.Nodes.data(), n + 1);
					const uint32 subpathHash = subpath.back().Hash;

					// First, resolve local duplicates
					if (not subpathsChecked.Insert(subpathHash))
					{
						continue;
					}

					JPL::SpecularPathId pathPartialId{ .Id = sourceData.Id };
					pathPartialId.AddVertex(subpathHash);

					// For each receiver, craete new subpath entry to validate
					for (uint32 r = 0; r < receiverData.size(); ++r)
					{
						const ReceiverData& receiver = receiverData[r];
						JPL::SpecularPathCache<Vec3>* pathCache = caches[r];
						JPL_ASSERT(pathCache);

						// Add receiver id to the path key hash
						JPL::SpecularPathId pathId = pathPartialId;
						pathId.AddVertex(receiver.Id);

						// Second, see if PathCach already contains this subpath
						if (not pathCache->Contains(pathId))
						{
							std::span<const Vec3> subpathIS = imageSources.subspan(0, subpath.size() + 1);

							newSubpaths.emplace_back(
								NewSubpathEntry{
									.Subpath = subpath,
									.SubpathIS = subpathIS,
									.ReceiverIdx = r,
									.PathId = pathId
								});
						}
					}
				}
			}
		}

		// Validate individual subpaths
		{
			JPL_PROFILE(SpecularRayTracing_ValidateConnectedPaths);
			JPL_PROFILE_SET_INT(NumPathsValidated, static_cast<int32>(newSubpaths.size()))

			auto validateSubpath = [&](int32 ei)
			{
				NewSubpathEntry& entry = newSubpaths[ei];
				const ReceiverData& receiver = receiverData[entry.ReceiverIdx];

				// This is touching physics scene (potentially needs a lock)
				entry.bIsValid = ValidatePathForReceiver(sceneInterface, entry.Subpath, entry.SubpathIS, receiver.Position);

				if (entry.bIsValid)
				{
					AccumulateMaterialAbsorption(sceneInterface, entry.Subpath, entry.EnergyLoss);
				}
			};

			// Give the caller a chance to prepare for a potentially concurrent set of traces
			// (e.g. lock physics scene)
			if constexpr (JPL_HAS_FUNCTION(SceneType, PreTrace()))
			{
				sceneInterface.PreTrace();
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, ParallelFor(uint32(newSubpaths.size()), validateSubpath)))
			{
				sceneInterface.ParallelFor(static_cast<uint32>(newSubpaths.size()), validateSubpath);
			}
			else
			{
				for (uint32 ei = 0; ei < newSubpaths.size(); ++ei)
				{
					validateSubpath(ei);
				}
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, PostTrace()))
			{
				sceneInterface.PostTrace();
			}
		}

		{
			JPL_PROFILE(SpecularRayTracing_CacheValidPaths);

			// TODO: we could avoid this allocation if we reuse one of the scratch buffers from above
			std::pmr::vector<int32> nodeCache(JPL::GetDefaultMemoryResource());
			nodeCache.reserve(traceInfo.MaxOrder + 2);

			// Assign source as the first vertex
			nodeCache.push_back(sourceData.Id);

			[[maybe_unused]] uint32 numValidPathsFound = 0;

			// Add new subpaths to Path Caches
			for (const NewSubpathEntry& entry : newSubpaths) //! This should be done sequentially, unlees we group entries by receiver
			{
				// Note: this is where we need simple integer IDs for the path nodes
				//		that we resolve using Geometry Cache

				// Remove all but the first source node
				nodeCache.resize(1 + entry.Subpath.size());

				// ...essentially asking the caller to convert trace path to a set of surface/node identifiers
				sceneInterface.CacheSubpath(entry.Subpath, std::span(&nodeCache[1], entry.Subpath.size()));

				// Assign receiver as the last vertex
				nodeCache.push_back(static_cast<int32>(receiverData[entry.ReceiverIdx].Id));

				// Add set of Geometry Cache handles to Path Cache
				JPL::SpecularPathCache<Vec3>* pathCache = caches[entry.ReceiverIdx];
				JPL_ASSERT(pathCache);

				// Add new entry to the receiver's path cache
				// (this cannot be called concurrently)
				pathCache->Add(entry.PathId, nodeCache, entry.SubpathIS.back(), entry.EnergyLoss, entry.bIsValid);

				numValidPathsFound += entry.bIsValid;
			}

			JPL_PROFILE_SET_INT(NumValidPathsFound, static_cast<int32>(numValidPathsFound));
			JPL_PROFILE_SET_FLOAT(TraceQuality, newSubpaths.empty() ? 0.0f : 100.0f * numValidPathsFound / float(newSubpaths.size()));
		}
	}

	//==========================================================================
	template<class IntersectionType>
	auto SpecularRayTracing::TraceInfo::Parse(const TraceResults<IntersectionType>& traces) -> TraceInfo
	{
		TraceInfo info;
		info.PathCount = static_cast<uint32>(traces.Paths.size());

		info.TotalNumSubpaths = 0;
		info.MaxOrder = 0;
		for (const auto& path : traces.Paths)
		{
			info.TotalNumSubpaths += path.Nodes.size();
			info.MaxOrder = std::max(info.MaxOrder, static_cast<uint32>(path.Nodes.size()));
		}

		info.TotalNumImageSources = info.TotalNumSubpaths + info.PathCount; // +1 for source per path

		return info;
	}

	//==========================================================================
	template<class IntersectionType>
	inline auto SpecularRayTracing::PreprocessTraces(TraceResults<IntersectionType>& traces) -> TraceInfo
	{
		// Remove empty paths
		Algo::ErasePartitionUnordered(traces.Paths, [](const auto& path) { return path.Nodes.empty(); });

#if 0	//! not worth extra allocations to remove 1 in a million duplicate (we cull subpath duplicates later anyway)
		// Second, remove duplicate surface paths
		{
			// Use preallocated growing buffer to avoid micro-allocations
			const std::size_t memSize = ScratchHashSetIdentity::GetRequiredMemorySize(traces.Paths.size());
			void* buffer = JPL::GetDefaultMemoryResource()->allocate(memSize);

			auto eraseDuplicatePaths = [&](auto& paths)
			{
				ScratchHashSetIdentity uniqueKeys(buffer, memSize, paths.size());

				//! Note: this only removes the full paths, not the subpaths
				//! (subpath is a path up to Nth node/order within a full path)
				Algo::ErasePartitionUnordered(paths, [&uniqueKeys](const auto& path)
				{
					return not uniqueKeys.Insert(path.Nodes.back().Hash);
				});
			};

			eraseDuplicatePaths(traces.Paths);

			JPL::GetDefaultMemoryResource()->deallocate(buffer, memSize);
		}
#endif

		// Parse trace info
		return TraceInfo::Parse(traces);
	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessTraces(SceneType& sceneInterface,
												  const typename SceneType::SourceData& sourceData,
												  TraceResults<typename SceneType::Intersection>& sourceTraces,
												  std::span<const typename SceneType::ReceiverData> receiverData,
												  std::span<TraceResults<typename SceneType::Intersection>> receiverTraces,
												  SpecularPathCacheContainer& caches)
	{
		JPL_PROFILE(SpecularRayTracing_ProcessTraces);

		using Vec3 = typename SceneType::Vec3;
		using Intersection = typename SceneType::Intersection;
		using PathType = TracedPath<Intersection>;
		using PathNodeType = TraceNode<Intersection>;
		using ReceiverData = typename SceneType::ReceiverData;
		using SourceData = typename SceneType::SourceData;

		// 1. Preprocess traces
		std::pmr::vector<TraceInfo> traceInfos(1 + receiverData.size(), JPL::GetDefaultMemoryResource());

		{
			JPL_PROFILE(SpecularRayTracing_PreprocessTraces);

			for (uint32 i = 0; i < receiverData.size(); ++i)
			{
				traceInfos[i] = PreprocessTraces(sceneInterface, receiverData[i].Position, receiverTraces[i]);
			}

			traceInfos.back() = PreprocessTraces(sceneInterface, sourceData.Position, sourceTraces);
		}

		const auto [potentialTotalNumConnectedPaths, maxSubpathCount, maxPathOrder] = [&]()
		{
			uint32 totalSubpaths = 0;
			uint32 maxSubpathCount = 0;
			uint32 maxPathOrder = 0;
			for (const TraceInfo& info : traceInfos)
			{
				totalSubpaths += info.TotalNumSubpaths;
				maxSubpathCount = std::max(maxSubpathCount, info.TotalNumSubpaths);
				maxPathOrder = std::max(maxPathOrder, info.MaxOrder);
			}
			return std::tuple(totalSubpaths, maxSubpathCount, maxPathOrder);
		}();

		// Paths that don't exist yet in the Specular Cache
		// and need to be validated and added to it.
		struct alignas(JPL_CACHE_LINE_SIZE) ConnectedPathEntry
		{
			// Set in in validation step
			JPL::EnergyBands EnergyLoss;
			bool bIsValid;

			std::span<const PathNodeType> FwdPath;
			std::span<const PathNodeType> BwdPath;
			uint32 ReceiverIdx;
			JPL::SpecularPathId PathId;
			Vec3 LastImageSource;
		};
		std::pmr::vector<ConnectedPathEntry> newConnectedPaths(JPL::GetDefaultMemoryResource());
		newConnectedPaths.reserve(potentialTotalNumConnectedPaths);

		// 2. Try connecting subpaths
		{
			JPL_PROFILE(SpecularRayTracing_CreateUniquePaths);

			const TraceInfo& sourceTraceInfo = traceInfos.back();

			const std::size_t bufferSize = ScratchHashSetIdentity::GetRequiredMemorySize(potentialTotalNumConnectedPaths);

			// Using memory resource as a simple RAII
			std::pmr::monotonic_buffer_resource resource(bufferSize, JPL::GetDefaultMemoryResource());
			void* dedupSubpathMem = resource.allocate(bufferSize);
			ScratchHashSetIdentity subpathsChecked(dedupSubpathMem, bufferSize, potentialTotalNumConnectedPaths);
	
			{
				// Try connect forward paths directly to the receivers
				for (uint32 pathIforward = 0; pathIforward < sourceTraceInfo.PathCount; ++pathIforward)
				{
					const auto& forwardPath = sourceTraces.Paths[pathIforward].Nodes;

					for (uint32 nf = 0; nf < forwardPath.size(); ++nf) // For each Subpath in Path
					{
						std::span<const PathNodeType> forwardSubpath(forwardPath.data(), nf + 1);

						JPL::SpecularPathId pathPartialId{ .Id = sourceData.Id };
						pathPartialId.AddVertex(forwardSubpath.back().Hash);

						for (uint32 receiverIdx = 0; receiverIdx < receiverData.size(); ++receiverIdx)
						{
							const ReceiverData& receiver = receiverData[receiverIdx];
							JPL::SpecularPathCache<Vec3>* pathCache = caches[receiverIdx];
							JPL_ASSERT(pathCache);

							// Connected path ID
							JPL::SpecularPathId pathId = pathPartialId;
							pathId.AddVertex(receiver.Id);

							if (not subpathsChecked.Insert(pathId.Id))
							{
								continue; // Cull duplicate subpaths
							}

							// See if PathCach already contains this subpath
							if (not pathCache->Contains(pathId))
							{
								newConnectedPaths.emplace_back(
									ConnectedPathEntry{
										.FwdPath = forwardSubpath,
										//.BwdPath = backwardSubpath,
										.ReceiverIdx = receiverIdx,
										.PathId = pathId
									});
							}
						}
					}
				}
			}

			// Try connect back paths directly to the source
			{
				JPL::SpecularPathId pathPartialId{ .Id = sourceData.Id };

				std::pmr::vector<uint32> hashSequence(JPL::GetDefaultMemoryResource());
				hashSequence.reserve(maxPathOrder);

				auto computeHashSequence = [](std::span<const PathNodeType> path, std::pmr::vector<uint32>& outContaienr)
				{
					outContaienr.clear();
					
					uint32 hash = path.back().Hash;
					outContaienr.push_back(hash);

					for (auto const& node : path | std::views::reverse | std::views::drop(1))
					{
						HashCombine32(hash, node.Hash);
						outContaienr.push_back(hash);
					}
				};

				for (uint32 receiverIdx = 0; receiverIdx < receiverData.size(); ++receiverIdx)
				{
					const TraceResults<Intersection>& traceResults = receiverTraces[receiverIdx];
					const ReceiverData& receiver = receiverData[receiverIdx];
					JPL::SpecularPathCache<Vec3>* pathCache = caches[receiverIdx];
					JPL_ASSERT(pathCache);

					for (uint32 pathIback = 0; pathIback < traceResults.Paths.size(); ++pathIback)
					{
						const auto& backwardPath = traceResults.Paths[pathIback].Nodes;

						computeHashSequence(backwardPath, hashSequence);

						// Construct backward path in reverse order (from receiver to soruce)
						// This will ensure consistent hash/id order with forward paths
						for (int32 nb = backwardPath.size() - 1, size = 1; nb >= 0; --nb, ++size)
						{
							std::span<const PathNodeType> backwardSubpath(backwardPath.data() + nb, size);

							// Connected path ID
							JPL::SpecularPathId pathId = pathPartialId;
							pathId.AddVertex(hashSequence[nb]);
							pathId.AddVertex(receiver.Id);

							if (not subpathsChecked.Insert(receiver.Id))
							{
								continue; // Cull duplicate subpaths
							}

							// See if PathCach already contains this subpath
							if (not pathCache->Contains(pathId))
							{
								newConnectedPaths.emplace_back(
									ConnectedPathEntry{
										//.FwdPath = forwardSubpath,
										.BwdPath = backwardSubpath,
										.ReceiverIdx = receiverIdx,
										.PathId = pathId
									});
							}
						}
					}
				}
			}
		}

		// Validate individual paths
		{
			//! This takes all of the function processing time
			JPL_PROFILE(SpecularRayTracing_ValidateConnectedPaths);
			JPL_PROFILE_SET_INT(NumPathsValidated, static_cast<int32>(newConnectedPaths.size()))

			// Construct Image Source table
			const uint32 totalNumberOfImageSources = Algo::Accumulate(newConnectedPaths, 0u, [](uint32 acc, const ConnectedPathEntry& entry)
			{
				return acc + static_cast<uint32>(entry.FwdPath.size() + entry.BwdPath.size() + 1); // +1 for source
			});
			std::pmr::vector<Vec3> imageSourceCache(totalNumberOfImageSources, JPL::GetDefaultMemoryResource());
			std::pmr::vector<uint32> imageSourceCacheTable(newConnectedPaths.size(), JPL::GetDefaultMemoryResource());
		
			// TODO: we may or may not want to also construct images sources here
			for (uint32 entryIndex = 0, ISCacheOffset = 0; entryIndex < newConnectedPaths.size(); ++entryIndex)
			{
				imageSourceCacheTable[entryIndex] = ISCacheOffset;
				const ConnectedPathEntry& entry = newConnectedPaths[entryIndex];
				const uint32 imageSourcePathSize = entry.FwdPath.size() + entry.BwdPath.size() + 1; // +1 for source
				ISCacheOffset += imageSourcePathSize;

				JPL_ASSERT(ISCacheOffset <= imageSourceCache.size());
			}

			auto validateConnectedPath = [&](int32 ei)
			{
				ConnectedPathEntry& entry = newConnectedPaths[ei];
				const ReceiverData& receiver = receiverData[entry.ReceiverIdx];

				const uint32 imageSourcePathSize = entry.FwdPath.size() + entry.BwdPath.size() + 1; // +1 for source
				
				std::span<Vec3> imageSources(&imageSourceCache[imageSourceCacheTable[ei]], imageSourcePathSize);
				ConstructImageSources(sceneInterface, sourceData.Position, entry.FwdPath, entry.BwdPath, imageSources);
				entry.LastImageSource = imageSources.back();

				// This is touching physics scene (potentially needs a lock)
				entry.bIsValid = ValidatePathForReceiver(sceneInterface, entry.FwdPath, entry.BwdPath, imageSources, receiver.Position);

				if (entry.bIsValid)
				{
					AccumulateMaterialAbsorption(sceneInterface, entry.FwdPath, entry.EnergyLoss);
					AccumulateMaterialAbsorption(sceneInterface, entry.BwdPath, entry.EnergyLoss);
				}
			};

			// Give the caller a chance to prepare for a potentially concurrent set of traces
			// (e.g. lock physics scene)
			if constexpr (JPL_HAS_FUNCTION(SceneType, PreTrace()))
			{
				sceneInterface.PreTrace();
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, ParallelFor(uint32(newConnectedPaths.size()), validateConnectedPath)))
			{
				sceneInterface.ParallelFor(static_cast<uint32>(newConnectedPaths.size()), validateConnectedPath);
			}
			else
			{
				for (uint32 ei = 0; ei < newConnectedPaths.size(); ++ei)
				{
					validateConnectedPath(ei);
				}
			}

			if constexpr (JPL_HAS_FUNCTION(SceneType, PostTrace()))
			{
				sceneInterface.PostTrace();
			}
		}

		{
			JPL_PROFILE(SpecularRayTracing_CacheValidPaths);

			// TODO: we could avoid this allocation if we reuse one of the scratch buffers from above
			std::pmr::vector<int32> nodeCache(JPL::GetDefaultMemoryResource());
			nodeCache.reserve(maxPathOrder + 2);
			// Assign source as the first vertex
			nodeCache.push_back(sourceData.Id);

			std::pmr::vector<PathNodeType> connectedPath(JPL::GetDefaultMemoryResource());
			connectedPath.reserve(maxPathOrder);

			[[maybe_unused]] uint32 numValidPathsFound = 0;

			// Add new subpaths to Path Caches
			for (const ConnectedPathEntry& entry : newConnectedPaths) //! This should be done sequentially, unlees we group entries by receiver
			{
				// Note: this is where we need simple integer IDs for the path nodes
				//		that we resolve using Geometry Cache

				// Get a contiguous surface path range
				connectedPath.resize(entry.FwdPath.size() + entry.BwdPath.size());
				std::ranges::copy(entry.FwdPath, connectedPath.begin());

				// Backward traced paths has to be reversed
				std::ranges::copy(std::views::reverse(entry.BwdPath), connectedPath.begin() + entry.FwdPath.size());

				// Remove all but the first source node
				nodeCache.resize(1 + connectedPath.size());

				// ...essentially asking the caller to convert trace path to a set of surface/node identifiers
				sceneInterface.CacheSubpath(connectedPath, std::span(&nodeCache[1], connectedPath.size()));

				// Assign receiver as the last vertex
				nodeCache.push_back(static_cast<int32>(receiverData[entry.ReceiverIdx].Id));

				// Add set of Geometry Cache handles to Path Cache
				JPL::SpecularPathCache<Vec3>* pathCache = caches[entry.ReceiverIdx];
				JPL_ASSERT(pathCache);

				// Add new entry to the receiver's path cache
				// (this cannot be called concurrently)
				pathCache->Add(entry.PathId, nodeCache, entry.LastImageSource, entry.EnergyLoss, entry.bIsValid);

				numValidPathsFound += entry.bIsValid;
			}

			JPL_PROFILE_SET_INT(NumValidPathsFound, static_cast<int32>(numValidPathsFound));
			JPL_PROFILE_SET_FLOAT(TraceQuality, newConnectedPaths.empty() ? 0.0f : 100.0f * numValidPathsFound / float(newConnectedPaths.size()));
		}
	}
} // namespace JPL

