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

#include <algorithm>
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
	enum class ETraceDirection
	{
		Forward,
		Backward
	};

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
	/// Specular Ray Tracing algorithm.
	/// Supporting backtracing - tracing from the listener.
	/// And bidirectional tracing - from the listener and sources.
	/// 
	/// Example backtracing from the listener:
	/// 
	///		// Step 1. Trace paths from the listener's postiion
	/// 
	///		MySceneType scenInterface;
	///		Vec3 listenerPosition = ...;
	///		TraceParameters parameters = ...;
	///		TraceResults outTraceResults;
	/// 
	///		SpecularRayTracing::Trace(scenInterface, listenerPosition, parameters, outTraceResults)
	/// 
	///		// Step 2. Process traces
	/// 
	///		SourceData sourceData{ listenerPosition, listenerId };
	///		std::span<ReceiverData> receiverData = ... // per sound source { position, Id }
	///		std::span<SpecularPathCache*> specularPathCache = ... // per sound source SPC
	///		
	///		SpecularRayTracing::ProcessTraces(scenInterface, sourceData, outTraceResults, receiversData, specularPathCache);
	/// 
	///		// Step 3. Make early reflection taps from SpecularPathCache
	/// 
	///		...
	/// 
	/// For bidirectional traces:
	///		1) Do Step 1 from above for listener and for each source.
	///			- Important: for traces from sources - set bPathHashCombine template for SpecularRayTracing::Trace to `false` 
	///		2) Use second SpecularRayTracing::ProcessTraces overload that takes span of receiver traces.
	/// 
	class SpecularRayTracing
	{
	public:
		static constexpr std::size_t cMaxOrder = 16; // arbitrary number

		//==========================================================================
		/// Trace paths from `origin` based on TraceParameters.
		/// This can be used to trace from the listener and sources.
		/// 
		/// @tparam bPathHashCombine	if set to `true`, hash of each node is
		///							a combination "up to" this node, otherwise
		///							it's a hash of this particular intersection.
		template<class SceneType, bool bPathHashCombine = true>
		static void Trace(const SceneType& sceneInterface,
						  const typename SceneType::Vec3& origin,
						  const TraceParameters& parameters,
						  TraceResults<typename SceneType::Intersection>& outTraceResults);

		//==========================================================================
		/// Process traces from source direction, connect paths to receivers.
		/// 
		/// Note: here source is assumed to be listener and thus the Image Sources
		/// are constructed from the receivers.
		/// 
		/// @param sourceData		data for the source of the traces
		/// @param receiverData	data for the receiver of the traces
		/// @param traces			traces done from the source position
		/// 
		/// @tparam SpecularPathCacheContainer	must have operator[] overloaded
		///									to access cache for a receiver index
		template<class SceneType, class SpecularPathCacheContainer> 
		static void ProcessTraces(SceneType& sceneInterface,
								  const typename SceneType::SourceData& sourceData,
								  TraceResults<typename SceneType::Intersection>& traces,
								  std::span<const typename SceneType::ReceiverData> receiverData,
								  SpecularPathCacheContainer& caches);

		/// Process bidirectional traces, connect source paths with receivers
		/// and receiver paths with the source.
		/// 
		/// Note: here source is assumed to be listener and thus the Image Sources
		/// are constructed from the receivers.
		/// 
		/// @param sourceData		data for the source of the traces
		/// @param receiverData	data for the receivers of the traces
		/// @param sourceTraces	traces done from the source position
		/// @param receiverTraces	traces done from the positions of receivers
		/// 
		/// @tparam SpecularPathCacheContainer	must have operator[] overloaded
		///									to access cache for a receiver index
		template<class SceneType, class SpecularPathCacheContainer>
		static void ProcessTraces(SceneType& sceneInterface,
								  const typename SceneType::SourceData& sourceData,
								  TraceResults<typename SceneType::Intersection>& sourceTraces,
								  std::span<const typename SceneType::ReceiverData> receiverData,
								  std::span<TraceResults<typename SceneType::Intersection>> receiverTraces,
								  SpecularPathCacheContainer& caches);

	private:
		//==========================================================================
		struct TraceInfo
		{
			uint32 PathCount;
			uint32 TotalNumSubpaths;
			uint32 MaxOrder;
			uint32 TotalNumImageSources;

			template<class IntersectionType>
			static TraceInfo Parse(const TraceResults<IntersectionType>& traces);
		};

		template<class Vec3>
		struct ImageSourceBuffer
		{
			std::pmr::vector<Vec3> ImageSources;
			std::pmr::vector<uint32> IndexTable;

			static ImageSourceBuffer MakeFor(const auto& subpathsList);
			std::span<Vec3> GetImageSourcesFor(const auto& subpath, uint32 subpathIndex);
		};

		template<class PathNodeType, class Vec3>
		struct alignas(JPL_CACHE_LINE_SIZE) NewSubpath
		{
			// Set in in validation step
			JPL::EnergyBands EnergyLoss;
			bool bIsValid;

			std::span<const PathNodeType> Subpath;
			ETraceDirection Direction;
			uint32 ReceiverIdx;
			JPL::SpecularPathId PathId;
			Vec3 LastImageSource;
		};

		//======================================================================
		/// Wrapper class to hold parameters used in various stages
		/// of the trace processing routine
		template<class SceneType, class SpecularPathCacheContainer>
		class ProcessRoutine
		{
			using Vec3 = typename SceneType::Vec3;
			using Intersection = typename SceneType::Intersection;
			using PathNodeType = TraceNode<Intersection>;
			using NewSubpathEntry = NewSubpath<PathNodeType, Vec3>;
			using SourceData = typename SceneType::SourceData;
			using ReceiverData = typename SceneType::ReceiverData;

			SceneType& mSceneInterface;
			const SourceData& mSourceData;
			TraceResults<Intersection>& mSourceTraces;
			std::span<const ReceiverData> mReceiverData;
			std::span<TraceResults<Intersection>> mReceiverTraces;
			SpecularPathCacheContainer& mCaches;

		public:
			ProcessRoutine(SceneType& sceneInterface,
						   const SourceData& sourceData,
						   TraceResults<Intersection>& sourceTraces,
						   std::span<const ReceiverData> receiverData,
						   std::span<TraceResults<Intersection>> receiverTraces,
						   SpecularPathCacheContainer& caches);

			/// Run the trace processing routine
			void Process();

			std::pair<uint32, uint32> GetSubpathCountAndMaxOrder() const;

			/// Remove invalid/empty paths, parse TraceInfo about TraceResults
			static TraceInfo PreprocessTraces(TraceResults<Intersection>& traces);

			void CreateForwardSubpathsEntries(TraceResults<Intersection>& traces,
											  ScratchHashSetIdentity& uniqueCheckSet,
											  std::pmr::vector<NewSubpathEntry>& outNewSubpaths) const;

			void CreateBackwardSubpathsEntries(std::span<TraceResults<Intersection>> traces,
											   ScratchHashSetIdentity& uniqueCheckSet,
											   std::pmr::vector<NewSubpathEntry>& outNewSubpaths) const;

			/// Construct image sources from the source psition through a sequence of forward traced surfaces
			template<ETraceDirection PathDirection>
			void ConstructImageSources(const Vec3& sourcePosition,
									   std::span<const TraceNode<Intersection>> path,
									   std::span<Vec3> outImageSources) const;

			/// Construct image sources from the source psition through a sequence of forward or backward traced surfaces
			template<ETraceDirection TraceDirection>
			void ConstructImageSources(const Vec3& sourcePosition,
									   std::span<const TraceNode<Intersection>> path,
									   ETraceDirection pathDirection,
									   std::span<Vec3> outImageSources) const;

			/// Validate unidirectional paths
			template<ETraceDirection PathDirection>
			bool ValidatePathForListener(std::span<const TraceNode<Intersection>> nodes, // does not include source/receiver
										 std::span<const Vec3> imageSources,
										 const Vec3& listenerPosition) const;

			/// Validate unidirectional paths
			template<ETraceDirection TraceDirection>
			bool ValidatePathForListener(std::span<const TraceNode<Intersection>> nodes,
										 ETraceDirection pathDirection,
										 std::span<const Vec3> imageSources,
										 const Vec3& listenerPosition) const;

			/// Validate visibility of specular reflections between sources and listener
			void ValidateNewSubpaths(std::span<NewSubpathEntry> newSubpaths,
									 const Vec3 listenerPosition) const;


			/// Accumulate material absorption for a set of surfaces
			void AccumulateMaterialAbsorption(std::span<const TraceNode<Intersection>> surfaces,
											  EnergyBands& outEnergyLoss) const;

			/// Write validated specular paths to receivers' cache
			void CacheValidatedSubpaths(std::span<NewSubpathEntry> validatedSubpaths,
										uint32 maxPathOrderHint) const;
		};
	};
	
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

// TODO: maybe refactor the interface to use source-listener pairs (?)
//		however, multiple listeners for acoustics simulation should be discouraged.

// TODO: this whole Forward/Backward semantics is a bit messy

namespace JPL
{
	//==========================================================================
	template<class SceneType, bool bPathHashCombine>
	inline void SpecularRayTracing::Trace(const SceneType& sceneInterface,
										  const typename SceneType::Vec3& origin,
										  const TraceParameters& parameters,
										  TraceResults<typename SceneType::Intersection>& outTraceResults)
	{
		using Vec3 = typename SceneType::Vec3;
		using Ray = typename SceneType::Ray;
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
			Ray ray(origin, Math::InternalUtils::RandDirection<Vec3>());
			uint32 hash = Hash::cStartSeed;

			// Trace rays up to 'MaxOrder'
			for (uint32 d = 0; d < parameters.MaxTraceOrder; ++d)
			{
				Intersection hit;
				if (not sceneInterface.Intersect(ray, parameters.MaxRayLength, hit))
				{
					break;
				}

				if constexpr (bPathHashCombine)
				{
					HashCombine32(hash, sceneInterface.GetHash(hit));
				}
				else
				{
					// If not combining, just use the user-provided surface hash
					hash = sceneInterface.GetHash(hit);
				}


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
	template<class Vec3>
	inline auto SpecularRayTracing::ImageSourceBuffer<Vec3>::MakeFor(const auto& subpathsList) -> ImageSourceBuffer<Vec3>
	{
		// Construct Image Source table
		const uint32 totalNumberOfImageSources = Algo::Accumulate(subpathsList, 0u, [](uint32 acc, const auto& entry)
		{
			return acc + static_cast<uint32>(entry.Subpath.size() + 1); // +1 for source
		});

		ImageSourceBuffer ISBuffer
		{
			std::pmr::vector<Vec3>(totalNumberOfImageSources, JPL::GetDefaultMemoryResource()),
			std::pmr::vector<uint32>(subpathsList.size(), JPL::GetDefaultMemoryResource())
		};

		for (uint32 entryIndex = 0, ISCacheOffset = 0; entryIndex < subpathsList.size(); ++entryIndex)
		{
			ISBuffer.IndexTable[entryIndex] = ISCacheOffset;
			const auto& entry = subpathsList[entryIndex];
			const uint32 imageSourcePathSize = entry.Subpath.size() + 1; // +1 for source
			ISCacheOffset += imageSourcePathSize;

			JPL_ASSERT(ISCacheOffset <= ISBuffer.ImageSources.size());
		}

		return ISBuffer;
	}

	template<class Vec3>
	inline std::span<Vec3> SpecularRayTracing::ImageSourceBuffer<Vec3>::GetImageSourcesFor(const auto& subpath, uint32 subpathIndex)
	{
		const uint32 imageSourcePathSize = subpath.size() + 1; // +1 for source
		return std::span<Vec3>(&ImageSources[IndexTable[subpathIndex]], imageSourcePathSize);
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
	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessTraces(SceneType& sceneInterface,
												  const typename SceneType::SourceData& sourceData,
												  TraceResults<typename SceneType::Intersection>& traces,
												  std::span<const typename SceneType::ReceiverData> receiverData,
												  SpecularPathCacheContainer& caches)
	{
		JPL_PROFILE(SpecularRayTracing_ProcessTraces);

		ProcessRoutine(sceneInterface, sourceData, traces, receiverData, {}, caches).Process();
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

		ProcessRoutine(sceneInterface, sourceData, sourceTraces, receiverData, receiverTraces, caches).Process();
	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	inline SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::
		ProcessRoutine(SceneType& sceneInterface,
					   const SourceData& sourceData,
					   TraceResults<Intersection>& sourceTraces,
					   std::span<const ReceiverData> receiverData,
					   std::span<TraceResults<Intersection>> receiverTraces,
					   SpecularPathCacheContainer& caches)
		: mSceneInterface(sceneInterface)
		, mSourceData(sourceData)
		, mSourceTraces(sourceTraces)
		, mReceiverData(receiverData)
		, mReceiverTraces(receiverTraces)
		, mCaches(caches)
	{
		// TODO: stack allocator to reuse memory throughout the routine stages (?)
	}

	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::Process()
	{
		// 1. Preprocess traces
		const auto [totalSubpathCount, maxPathOrder] = GetSubpathCountAndMaxOrder();
		
		// Paths that don't exist yet in the Specular Cache
		// and need to be validated and added to it.
		std::pmr::vector<NewSubpathEntry> newSubpaths(JPL::GetDefaultMemoryResource());
		newSubpaths.reserve(totalSubpathCount);
		// TODO: this could potentially be huge (50 rays * 3 depth * 10 sources * 64 bytes = 96k bytes)
		// if this does turn out to be huge, we could do our validation below per receiver and move this array to per receiver loop as well

		// 2. Create unique subpaths
		{
			JPL_PROFILE(SpecularRayTracing_CreateUniquePaths);

			// Using memory resource as a simple RAII
			const std::size_t bufferSize = ScratchHashSetIdentity::GetRequiredMemorySize(totalSubpathCount);
			std::pmr::monotonic_buffer_resource resource(bufferSize, JPL::GetDefaultMemoryResource());
			ScratchHashSetIdentity subpathsChecked(resource.allocate(bufferSize), bufferSize, totalSubpathCount);

			CreateForwardSubpathsEntries(mSourceTraces, subpathsChecked, newSubpaths);

			if (not mReceiverTraces.empty())
			{
				CreateBackwardSubpathsEntries(mReceiverTraces, subpathsChecked, newSubpaths);
			}
		}

		// 3. Validate specular reflections for the unique subpaths
		ValidateNewSubpaths(std::span(newSubpaths), mSourceData.Position); //? allocating

		// 4. Write validated specular reflection paths to cache
		CacheValidatedSubpaths(std::span(newSubpaths), maxPathOrder); //? allocating
	}

	template<class SceneType, class SpecularPathCacheContainer>
	inline std::pair<uint32, uint32> SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::GetSubpathCountAndMaxOrder() const
	{
		JPL_PROFILE(SpecularRayTracing_PreprocessTraces);

		const TraceInfo sourceTraceInfo = PreprocessTraces(mSourceTraces);
		const uint32 listenerToReceiverSubpathCount = sourceTraceInfo.TotalNumSubpaths * mReceiverData.size();
		
		uint32 totalSubpaths = listenerToReceiverSubpathCount;
		uint32 maxPathOrder = sourceTraceInfo.MaxOrder;
		
		for (auto& receiverTraceResults : mReceiverTraces)
		{
			const TraceInfo receiverTraceInfo = PreprocessTraces(receiverTraceResults);
			totalSubpaths += receiverTraceInfo.TotalNumSubpaths;
			maxPathOrder = std::max(maxPathOrder, receiverTraceInfo.MaxOrder);
		}

		return std::pair(totalSubpaths, maxPathOrder);

	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	inline auto SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::PreprocessTraces(TraceResults<Intersection>& traces) -> TraceInfo
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
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::CreateForwardSubpathsEntries(TraceResults<Intersection>& traces,
																														ScratchHashSetIdentity& uniqueCheckSet,
																														std::pmr::vector<NewSubpathEntry>& outNewSubpaths) const
	{
		using NewSubpathEntry = NewSubpath<PathNodeType, Vec3>;

		for (uint32 pathIdx = 0; pathIdx < traces.Paths.size(); ++pathIdx)
		{
			const auto& path = traces.Paths[pathIdx].Nodes;

			for (uint32 nf = 1; nf <= path.size(); ++nf) // For each Subpath in Path
			{
				std::span<const PathNodeType> subpath(path.data(), nf);

				JPL::SpecularPathId pathPartialId{ .Id = mSourceData.Id };
				pathPartialId.AddVertex(subpath.back().Hash);

				for (uint32 receiverIdx = 0; receiverIdx < mReceiverData.size(); ++receiverIdx)
				{
					const ReceiverData& receiver = mReceiverData[receiverIdx];
					JPL::SpecularPathCache<Vec3>* pathCache = mCaches[receiverIdx];
					JPL_ASSERT(pathCache);

					// Connected path ID
					JPL::SpecularPathId pathId = pathPartialId;
					pathId.AddVertex(receiver.Id);

					// We need to check including receiver Id
					// to be able to match duplicates agains backward subpaths
					if (not uniqueCheckSet.Insert(pathId.Id))
					{
						continue; // Cull duplicate subpaths
					}

					// See if PathCach already contains this subpath
					if (not pathCache->Contains(pathId))
					{
						outNewSubpaths.emplace_back(
							NewSubpathEntry{
								.Subpath = subpath,
								.Direction = ETraceDirection::Forward,
								.ReceiverIdx = receiverIdx,
								.PathId = pathId
							});
					}
				}
			}
		}
	}

	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::CreateBackwardSubpathsEntries(std::span<TraceResults<Intersection>> traces,
																														 ScratchHashSetIdentity& uniqueCheckSet,
																														 std::pmr::vector<NewSubpathEntry>& outNewSubpaths) const
	{
		JPL::SpecularPathId pathPartialId{ .Id = mSourceData.Id };

		// We have to rehash subpath that is backwards, accumulating from the head
		auto hashSequence = [](std::span<const PathNodeType> subpath)
		{
			Hash hash;
			for (auto const& node : subpath | std::views::reverse)
			{
				hash.Combine(node.Hash);
			}
			return hash.GetCurrent();
		};

		for (uint32 receiverIdx = 0; receiverIdx < mReceiverData.size(); ++receiverIdx)
		{
			const TraceResults<Intersection>& traceResults = traces[receiverIdx];
			const ReceiverData& receiver = mReceiverData[receiverIdx];
			JPL::SpecularPathCache<Vec3>* pathCache = mCaches[receiverIdx];
			JPL_ASSERT(pathCache);

			for (uint32 pathIdx = 0; pathIdx < traceResults.Paths.size(); ++pathIdx)
			{
				const auto& path = traceResults.Paths[pathIdx].Nodes;

				// Construct backward path in reverse order (from receiver to soruce)
				// This will ensure consistent hash/id order with forward paths
				for (int32 nb = 1; nb <= path.size(); ++nb)
				{
					std::span<const PathNodeType> subpath(path.data(), nb);

					// Connected path ID
					JPL::SpecularPathId pathId = pathPartialId;
					pathId.AddVertex(hashSequence(subpath));
					pathId.AddVertex(receiver.Id);

					if (not uniqueCheckSet.Insert(pathId.Id))
					{
						continue; // Cull duplicate subpaths
					}

					// See if PathCach already contains this subpath
					if (not pathCache->Contains(pathId))
					{
						outNewSubpaths.emplace_back(
							NewSubpathEntry{
								.Subpath = subpath,
								.Direction = ETraceDirection::Backward,
								.ReceiverIdx = receiverIdx,
								.PathId = pathId
							});
					}
				}
			}
		}
	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	template<ETraceDirection PathDirection>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::ConstructImageSources(const Vec3& sourcePosition,
																												 std::span<const TraceNode<Intersection>> path,
																												 std::span<Vec3> outImageSources) const
	{
		JPL_ASSERT(outImageSources.size() >= path.size() + 1);

		outImageSources[0] = sourcePosition;

		if constexpr (PathDirection == ETraceDirection::Forward)
		{
			for (int32 i = 0; i < path.size(); ++i)
			{
				const TraceNode<Intersection>& node = path[i];
				outImageSources[i + 1] = Math::GetImageSource(outImageSources[i],
															  mSceneInterface.GetNormal(node.Hit),
															  mSceneInterface.GetPosition(node.Hit));
			}
		}
		else // PathDirection == ETraceDirection::Backward
		{
			for (int32 i = path.size() - 1, ISIndex = 0; i >= 0; --i, ++ISIndex)
			{
				const TraceNode<Intersection>& node = path[i];
				outImageSources[ISIndex + 1] = Math::GetImageSource(outImageSources[ISIndex],
																	mSceneInterface.GetNormal(node.Hit),
																	mSceneInterface.GetPosition(node.Hit));
			}
		}
	}

	template<class SceneType, class SpecularPathCacheContainer>
	template<ETraceDirection TraceDirection>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::ConstructImageSources(const Vec3& sourcePosition,
																												 std::span<const TraceNode<Intersection>> path,
																												 ETraceDirection pathDirection,
																												 std::span<Vec3> outImageSources) const
	{
		if (pathDirection == ETraceDirection::Forward)
		{
			static constexpr auto cISTraceDirection = TraceDirection;
			return ConstructImageSources<cISTraceDirection>(sourcePosition, path, outImageSources);
		}
		else
		{
			static constexpr auto cISTraceDirection = TraceDirection == ETraceDirection::Backward ? ETraceDirection::Forward : ETraceDirection::Backward;
			return ConstructImageSources<cISTraceDirection>(sourcePosition, path, outImageSources);
		}
	}

	template<class SceneType, class SpecularPathCacheContainer>
	template<ETraceDirection PathDirection>
	inline bool SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::ValidatePathForListener(std::span<const TraceNode<Intersection>> nodes,
																												   std::span<const Vec3> imageSources,
																												   const Vec3& listenerPosition) const
	{
		// Check that we do intersect the correct surface sequance,
		// and nothing is obstructing visibility of the image source

		Vec3 R = listenerPosition;

		if constexpr (PathDirection == ETraceDirection::Backward)
		{
			for (uint32 i = imageSources.size() - 1, ni = 0; i >= 1; --i, ++ni)
			{
				Intersection hit;

				if (not mSceneInterface.Intersect(R, imageSources[i], hit) ||
					not mSceneInterface.IsSameSurface(hit, nodes[ni].Hit))
					return false;

				// Small offset to avoid self intersection
				static constexpr float offset = 0.001f;

				R = mSceneInterface.GetPosition(hit) + mSceneInterface.GetNormal(hit) * offset;
			}
		}
		else
		{
			for (auto i = imageSources.size() - 1; i >= 1; --i)
			{
				Intersection hit;

				if (not mSceneInterface.Intersect(R, imageSources[i], hit) ||
					not mSceneInterface.IsSameSurface(hit, nodes[i - 1].Hit))
					return false;

				// Small offset to avoid self intersection
				static constexpr float offset = 0.001f;

				R = mSceneInterface.GetPosition(hit) + mSceneInterface.GetNormal(hit) * offset;
			}
		}

		// Lastly check visibility between
		// source and first refleciton point
		return not mSceneInterface.IsOccluded(R, imageSources[0]);
	}

	template<class SceneType, class SpecularPathCacheContainer>
	template<ETraceDirection TraceDirection>
	inline bool SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::ValidatePathForListener(std::span<const TraceNode<Intersection>> nodes,
																												   ETraceDirection pathDirection,
																												   std::span<const Vec3> imageSources,
																												   const Vec3& listenerPosition) const
	{
		if (pathDirection == ETraceDirection::Forward)
		{
			static constexpr auto cISDirection = TraceDirection;
			return ValidatePathForListener<cISDirection>(nodes, imageSources, listenerPosition);
		}
		else
		{
			static constexpr auto cISDirection = TraceDirection == ETraceDirection::Backward ? ETraceDirection::Forward : ETraceDirection::Backward;
			return ValidatePathForListener<cISDirection>(nodes, imageSources, listenerPosition);
		}
	}

	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::ValidateNewSubpaths(std::span<NewSubpathEntry> newSubpaths,
																											   const Vec3 listenerPosition) const
	{
		//! This takes all of the function processing time
		JPL_PROFILE(SpecularRayTracing_ValidateNewSubpaths);
		JPL_PROFILE_SET_INT(NumPathsValidated, static_cast<int32>(newSubpaths.size()));


		// Construct Image Source table
		auto imageSourceBuffer = ImageSourceBuffer<Vec3>::MakeFor(newSubpaths); //? allocating

		auto validateSubpath = [&](int32 ei)
		{
			NewSubpathEntry& entry = newSubpaths[ei];
			const ReceiverData& receiver = mReceiverData[entry.ReceiverIdx];
			std::span<Vec3> imageSources = imageSourceBuffer.GetImageSourcesFor(entry.Subpath, ei);

			ConstructImageSources<ETraceDirection::Backward>(receiver.Position,
															 entry.Subpath,
															 entry.Direction,
															 imageSources);

			entry.LastImageSource = imageSources.back();

			// This is touching physics scene (potentially needs a lock)
			entry.bIsValid = ValidatePathForListener<ETraceDirection::Backward>(entry.Subpath,
																				entry.Direction,
																				imageSources,
																				listenerPosition);

			if (entry.bIsValid)
			{
				AccumulateMaterialAbsorption(entry.Subpath, entry.EnergyLoss);
			}
		};

		// Give the caller a chance to prepare for a potentially concurrent set of traces
		// (e.g. lock physics scene)
		if constexpr (JPL_HAS_FUNCTION(SceneType, PreTrace()))
		{
			mSceneInterface.PreTrace();
		}

		if constexpr (JPL_HAS_FUNCTION(SceneType, ParallelFor(uint32(newSubpaths.size()), validateSubpath)))
		{
			mSceneInterface.ParallelFor(static_cast<uint32>(newSubpaths.size()), validateSubpath);
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
			mSceneInterface.PostTrace();
		}
	}

	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::AccumulateMaterialAbsorption(std::span<const TraceNode<Intersection>> surfaces,
																														EnergyBands& outEnergyLoss) const
	{
		for (const auto& surfaceHit : surfaces)
		{
			EnergyBands materialAbsorption;
			if (mSceneInterface.GetMaterialAbsorption(surfaceHit, materialAbsorption))
			{
				outEnergyLoss += materialAbsorption;
			}
		}
	}

	//==========================================================================
	template<class SceneType, class SpecularPathCacheContainer>
	inline void SpecularRayTracing::ProcessRoutine<SceneType, SpecularPathCacheContainer>::CacheValidatedSubpaths(std::span<NewSubpathEntry> validatedSubpaths,
																												  uint32 maxPathOrderHint) const
	{
		JPL_PROFILE(SpecularRayTracing_CacheValidPaths);

		std::pmr::vector<int32> nodeCache(JPL::GetDefaultMemoryResource());
		nodeCache.reserve(maxPathOrderHint + 2);

		// Assign source as the first vertex
		nodeCache.push_back(mSourceData.Id);

		std::pmr::vector<PathNodeType> subpathCopy(JPL::GetDefaultMemoryResource());
		subpathCopy.reserve(maxPathOrderHint);

		[[maybe_unused]] uint32 numValidPathsFound = 0;

		// Add new subpaths to Path Caches
		for (const NewSubpathEntry& entry : validatedSubpaths) //! This should be done sequentially, unlees we group entries by receiver
		{
			// Note: this is where we need simple integer IDs for the path nodes
			//		that we resolve using Geometry Cache

			// Get a contiguous surface path range
			subpathCopy.resize(entry.Subpath.size());
			std::ranges::copy(entry.Subpath, subpathCopy.begin());

			// Backward traced paths has to be reversed
			if (entry.Direction == ETraceDirection::Backward)
			{
				std::ranges::reverse(subpathCopy);
			}

			// Remove all but the first source node
			nodeCache.resize(1 + subpathCopy.size());

			// ...essentially asking the caller to convert trace path to a set of surface/node identifiers
			mSceneInterface.CacheSubpath(subpathCopy, std::span(&nodeCache[1], subpathCopy.size()));

			// Assign receiver as the last vertex
			nodeCache.push_back(static_cast<int32>(mReceiverData[entry.ReceiverIdx].Id));

			// Add set of Geometry Cache handles to Path Cache
			JPL::SpecularPathCache<Vec3>* pathCache = mCaches[entry.ReceiverIdx];
			JPL_ASSERT(pathCache);

			// Add new entry to the receiver's path cache
			// (this cannot be called concurrently)
			pathCache->Add(entry.PathId, nodeCache, entry.LastImageSource, entry.EnergyLoss, entry.bIsValid);

			numValidPathsFound += entry.bIsValid;
		}

		JPL_PROFILE_SET_INT(NumValidPathsFound, static_cast<int32>(numValidPathsFound));
		JPL_PROFILE_SET_FLOAT(TraceQuality, validatedSubpaths.empty() ? 0.0f : 100.0f * numValidPathsFound / float(validatedSubpaths.size()));
	}
} // namespace JPL

