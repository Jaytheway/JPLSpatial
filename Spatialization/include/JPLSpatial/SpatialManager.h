//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Containers/FlatMap.h"
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalQuat.h"
#include "JPLSpatial/Math/Position.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Services/DirectPathService.h"
#include "JPLSpatial/Services/PanningService.h"
#if JPL_HAS_ENV_PROPAGATION
#include "JPLSpatial/Services/EnvironmentService.h"
#include "JPLSpatial/Environment.h"
#endif
#include "JPLSpatial/Utilities/IDType.h"


#include <algorithm>
#include <cmath>
#include <functional>
#include <optional>
#include <memory>
#include <ranges>
#include <span>
#include <vector>
#include <unordered_set>
#include <memory_resource>

#if 1
namespace JPL
{
#ifdef JPL_TEST
	class SpatializationTest_CreateDeleteSource_Test;
#endif
} // namespace JPL

namespace JPL::Spatial
{
	struct SpatialManagerIDTag {};
	using SourceId = IDType<SpatialManagerIDTag>;

	struct ListenerMockIDTag {};
	using ListenerId = IDType<ListenerMockIDTag>;

	struct ObjectInSpaceIDTag {};
	using PositionId = IDType<ObjectInSpaceIDTag>;

	struct SourceInitParameters
	{
		// TODO: decide if we want to cache some of the parameters (occlusion, etc.) per source, or per object

		/// Number of source channels. If 0, panning is not initialized for this source.
		uint32 NumChannels = 0;

		/// Number of target channels for panning. Must be greater than 0.
		uint32 NumTargetChannels = 2;

		/// Optional listener to assign this source to. If not set, default listener is used.
		JPL::Spatial::ListenerId ListenerId;

		/// Determines spatialized source channel oritentation
		ESpatializationType SpatializationType = ESpatializationType::Position;

		//? not used yet
		// Object this source should be associated to,
		// or 0, to associate this source to a new object.
		uint64 ObjectId;

		/// Pan parameters used if panning effect is initialized for the source
		PanEffectParameters PanParameters{ .Focus = 0.0f, .Spread = 1.0f };

		/// If set, Spread value is calculated from ObjectSize based on distance to the listener
		std::optional<float> ObjectSize;
		
		/// Base distance attenuation curve
		AttenuationCurveRef DistanceAttenuationCurve = make_pmr_shared<AttenuationCurve>(); 

		/// Source directivity cone
		JPL::AttenuationCone AttenuationCone;
	};

	template<CVec3 Vec3Type>
	struct ListenerMock
	{
		ListenerId Id;
#if JPL_HAS_ENV_PROPAGATION
		RoomId Room;
#endif
		JPL::Position<Vec3Type> Position;
	};

	// At each position in space relative to listener we have:
	struct PositionData
	{
		float Distance;
		float DirectionDot;

		float Obstruction;
		float Occlusion;

		//float AirAbsorptionFilterValue; //? probably this doesn't belong her, since it is next level. i.e. computed from Distance
	};

	//? not used yet
	template<CVec3 Vec3Type>
	struct ObjectInSpace
	{
		PositionId Id;
		JPL::Position<Vec3Type> Position;

		// Data computed for the object position
		PositionData Data;
	};

	template<CVec3 Vec3Type>
	struct SourceData
	{
		SourceId Id;
		ListenerId Listener;

		JPL::Position<Vec3Type> Position;

#if JPL_HAS_ENV_PROPAGATION
		RoomId Room; //< Room this source is currently in
#endif
		JPL::DirectEffectHandle DirectEffectHandle;
		JPL::PanEffectHandle PanEffectHandle;

		std::optional<float> ObjectSize;

		// std::vector<ReflectionId> ReflectionImageSources;
		// std::vector<DifractionId> DifractionSources;

		// TODO: this could be treated as main distance volume attenuation curve to use for indirect paths
		AttenuationCurveRef AttenuationCurve;
		
		ESpatializationType SpatializationType;
	};

	//==========================================================================
	/// High level interface to manage spatialized sources.
	/// It holds an instance of each spatialization service.
	template<CVec3 Vec3Type, class VBAPTraits = VBAPBaseTraits<Vec3Type>>
	class SpatialManager
	{
	private:
		/*static */JPL_INLINE SourceId NewSourceId() { return SourceId::New(); }

	public:
		// Alias to override allocator for the internal FlatMap we use
		template<class Key, class T>
		using FlatMapType = FlatMapWithAllocator<Key, T, std::pmr::polymorphic_allocator>;

		template<class Key>
		using SetType = std::pmr::unordered_set<Key, std::hash<Key>, std::equal_to<Key>>;

	public:
		SpatialManager() = default;
		~SpatialManager() = default;

		SpatialManager(const SpatialManager&) = delete;
		SpatialManager& operator=(const SpatialManager&) = delete;

		// Evaluate all the updated parametesr to all the spatial effects.
		// Process sound propagation simulation.

		JPL_INLINE const DirectPathService& GetDirectPathService() const { return mDirectPathService; }
		JPL_INLINE DirectPathService& GetDirectPathService() { return mDirectPathService; }
		JPL_INLINE const PanningService<VBAPTraits>& GetPanningService() const { return mPanningService; }
		JPL_INLINE PanningService<VBAPTraits>& GetPanningService() { return mPanningService; }

		/// Process all the parameters of the modified sources.
		/// E.g. this can be called at the end of an update frame
		/// to process changed positions of all the sources.
		JPL_INLINE void AdvanceSimulation();

		JPL_INLINE std::span<const SourceId> GetLastUpdatedSource() const;

		JPL_INLINE ListenerId CreateListener();
		JPL_INLINE bool DeleteListener(ListenerId listener);

		JPL_INLINE SourceId CreateSource(const SourceInitParameters& options);
		JPL_INLINE bool DeleteSource(SourceId source);

		JPL_INLINE bool SetListener(SourceId source, ListenerId listener);
		JPL_INLINE ListenerId GetDefaultListener() const { return mDefaultListener; }

		/// Set new position for the listener. This will flag all sources associated
		/// with this listener for update if the position is different than the current one.
		/// @returns		'true' if the listener is valid, 'false' otherwise
		JPL_INLINE bool SetListenerPosition(ListenerId listener, const Position<Vec3Type>& newPosition);
		JPL_INLINE std::optional<Position<Vec3Type>> GetListenerPosition(ListenerId listener) const;

		/// Updating any data for sources can be done many times,
		/// the data is only processed when calling AdvanceSimulation()
		JPL_INLINE bool SetSourcePosition(SourceId source, const Position<Vec3Type>& newPosition);

		/// Set focus and spread parameters for the source.
		/// @returns		'true' if the source is valid and has PanEffect initialized, 'fasle' otherwise
		JPL_INLINE bool SetSourceFocusAndSpread(SourceId source, PanEffectParameters focusAndSpread);

		static JPL_INLINE float GetSpreadFromSourceSize(float sourceSize, float distance);

		JPL_INLINE DirectEffectHandle GetDirectEffectHandle(SourceId source) const { return mSourceStuff.at(source).DirectEffectHandle; }
		JPL_INLINE PanEffectHandle GetPanEffectHandle(SourceId source) const { return mSourceStuff.at(source).PanEffectHandle; }

		/// @returns		distance attenuation value evaluated for the 'source' and 'curve'
		///				the last time AdvanceSimulation() was called
		JPL_INLINE float GetDistanceAttenuation(SourceId source, const AttenuationCurveRef& curve) const;

		JPL_INLINE float GetConeAttenuation(SourceId source) const;

		/// Get channel gains of the source for the ChannelMap based on the number of values/channels in `outGains`
		/// The returned channel gains are based on the data computed during the last call to AdvanceSimulation()
		/// @returns		span of ChannelGains per source channel, if the 'source' is valid and
		///				the `targetChannelMap` was initialized for that source, empty span otherwise
		JPL_INLINE std::span<const float> GetChannelGains(SourceId source, ChannelMap targetChannelMap) const;
		// TODO: alternatively, we could employ SteamAudio's approach of feeding in buffer and applying all the processing internally

		// Process audio for a source based on data computed during last call to AdvanceSimulation()
		//! This may or may not be safe to call from audio thread (TBD)
		//void ProcessSourceBuffer(SourceId source, std::span<const float> inAudio, std::span<float> outAudio);

	private:
#ifdef JPL_TEST
		friend class JPL::SpatializationTest_CreateDeleteSource_Test;
#endif

	private:
		// Services
		DirectPathService mDirectPathService;
		PanningService<VBAPTraits> mPanningService;
#if JPL_HAS_ENV_PROPAGATION
		EnvironmentService mEnvironmentService;
#endif
		// TODO: we may or may not whant flat set instead of std::unordered_set

		// Sources to be updated at next call to AdvanceSimulation()
		SetType<SourceId> mDirtySources{ GetDefaultMemoryResource() };		///< Sources to update paths for
#if JPL_HAS_ENV_PROPAGATION
		SetType<SourceId> mSourcesMoved{ GetDefaultMemoryResource() };		///< Sources to update room containment for
		SetType<ListenerId> mListenersMoved{ GetDefaultMemoryResource() };	///< Listeners to update room containment for
#endif
		// Object that have changed during last update.
		// User or mixing engine may need this information.
		std::pmr::vector<SourceId> mLastUpdatedSources{ GetDefaultMemoryResource() };
#if JPL_HAS_ENV_PROPAGATION
		std::pmr::vector<SourceId> mLastSourceRoomsChanged{ GetDefaultMemoryResource() };
		std::pmr::vector<ListenerId> mLastListenerRoomsChanged{ GetDefaultMemoryResource() };
#endif

		// Per source data
		FlatMapType<SourceId, SourceData<Vec3Type>> mSourceStuff{ GetDefaultMemoryResource() };

		// Intermediate data processed for the position the sources were at
		// at the last call to AdvanceSimulation()
		FlatMapType<SourceId, PositionData> mPositionData{ GetDefaultMemoryResource() }; // TODO: this is weird, it assumes single listener per source

		// Default listener
		const ListenerId mDefaultListener = ListenerId::New();

		// Map of listeners
		FlatMapType<ListenerId, ListenerMock<Vec3Type>> mListeners
		{ {
			{ mDefaultListener, ListenerMock<Vec3Type>{
				.Id = mDefaultListener,
				.Position = {
					.Location = Vec3Type(0, 0, 0),
					.Orientation = OrientationData<Vec3Type>::IdentityForward()
				}}
			}
		}, GetDefaultMemoryResource() };
	};

} // namespace JPL::Spatialziation

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL::Spatial
{
	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE void SpatialManager<Vec3Type, VBAPTraits>::AdvanceSimulation()
	{
		// TODO: before processing distance attenuation we need to process propagation paths,
		//		since sources may not have direct path, also we need to process distance attenuation
		//		for indirect paths and reflections

		// Remove dirty sources that don't have listeners from the path update list
		std::erase_if(mDirtySources, [this](SourceId source)
		{
			return !mListeners.contains(mSourceStuff[source].Listener);
		});

		//? we do still want to know which room each source is in,
		// in case it gets a new listener
		//! TBD
		/*std::erase_if(mSourcesMoved, [this](SourceId source)
		{
			return !mListeners.contains(mSourceStuff[source].Listener);
		});*/


#if JPL_HAS_ENV_PROPAGATION
		// Update listener room containment
		{
			mLastListenerRoomsChanged.clear();
			mLastListenerRoomsChanged.reserve(mListenersMoved.size());

			for (ListenerId listener : mListenersMoved)
			{
				ListenerMock<Vec3Type>& listenerData = mListeners[listener];

				const RoomId room = mEnvironmentService.GetRoomAt(listenerData.Position.Location);
				if (listenerData.Room != room)
				{
					listenerData.Room = room;

					// TODO: notify user/mixing engine?

					mLastListenerRoomsChanged.push_back(listener);
				}
			}
		}

		// Update source room containment
		{
			mLastSourceRoomsChanged.clear();
			mLastSourceRoomsChanged.reserve(mSourcesMoved.size());

			for (SourceId source : mSourcesMoved)
			{
				SourceData<Vec3Type>& sourceData = mSourceStuff[source];

				const RoomId room = mEnvironmentService.GetRoomAt(sourceData.Position.Location);
				if (sourceData.Room != room)
				{
					sourceData.Room = room;
					// TODO: notify user/mixing engine?

					// Store sources that moved roomes during this iteration to let user
					// update audio bus routing and mixing
					mLastSourceRoomsChanged.push_back(source);
				}
			}
		}
#endif

		// Update source positional data
		for (SourceId source : mDirtySources)
		{
			const SourceData<Vec3Type>& sourceData = mSourceStuff[source];

			const auto& sourcePosition = sourceData.Position;
			const auto& listenerPosition = mListeners[sourceData.Listener].Position;

			// TODO: process rooms & portals, and indirect paths,
			//		update transmission loss and diffractoin path
			/*
				1. Get all portals of the room
				2. Get the ones between source and listener
				3. Process all diffraction paths
			*/

			const DirectPathResult<Vec3Type> directPathResult =
				mDirectPathService.ProcessDirectPath(
					sourcePosition,
					listenerPosition
				);

			// Cache direct path data
			auto& positionData = mPositionData[source];
			positionData.Distance = directPathResult.Distance;
			positionData.DirectionDot = directPathResult.DirectionDot;

			// Attenuation is a higher level API, depends on AttenuationCurve,
			// we can't assume what curve user wants to use.
			// But we can preprocess all curves associated with this source.
			// TODO: mabye we could batch process multiple distances/sources per curve instead, if they are sharing curves?
			mDirectPathService.EvaluateDistance(sourceData.DirectEffectHandle, directPathResult.Distance);

			// Calculate cone attenuatoin factor
			mDirectPathService.EvaluateDirection(sourceData.DirectEffectHandle, directPathResult.InvDirectionDot); // TODO: why are we evaluating source cone attenuation, but not listener's?

			// TODO: can we eliminate this branch?
			if (sourceData.ObjectSize)
			{
				const float spread = GetSpreadFromSourceSize(*sourceData.ObjectSize, directPathResult.Distance);
				mPanningService.SetPanningEffectSpread(sourceData.PanEffectHandle, spread);
			}

			// TODO: this is not very useful for indirect paths that don't use spread and focus
			//		- in such case a different panning method needed

			mPanningService.EvaluateDirection(sourceData.PanEffectHandle, directPathResult.Position, sourceData.SpatializationType);
		}

		// Store sources updated during this iteration to let user check
		// which sourced were actually changed
		mLastUpdatedSources.resize(mDirtySources.size());
		std::ranges::copy(mDirtySources, mLastUpdatedSources.begin());

		mDirtySources.clear();
#if JPL_HAS_ENV_PROPAGATION
		mSourcesMoved.clear();
#endif
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE std::span<const SourceId> SpatialManager<Vec3Type, VBAPTraits>::GetLastUpdatedSource() const
	{
		return mLastUpdatedSources;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE ListenerId SpatialManager<Vec3Type, VBAPTraits>::CreateListener()
	{
		const auto listenerId = ListenerId::New();
		mListeners.emplace(listenerId,
						   ListenerMock<Vec3Type>{
			.Id = listenerId,
				.Position = {
					.Location = Vec3Type(0, 0, 0),
					.Orientation = OrientationData<Vec3Type>::IdentityForward()
			}
		});
		return listenerId;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::DeleteListener(ListenerId listener)
	{
		if (!mListeners.contains(listener) || listener == mDefaultListener)
			return false;

#if JPL_HAS_ENV_PROPAGATION
		mListenersMoved.erase(listener);
#endif
		mListeners.erase(listener);

		// TODO: we may or may not want to assign default listener to orphan sources
		return true;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE SourceId SpatialManager<Vec3Type, VBAPTraits>::CreateSource(const SourceInitParameters& options)
	{
		// Source has to be initialized with target channel count
		JPL_ASSERT(options.NumTargetChannels > 0);

		const SourceId newId = NewSourceId();

		const ChannelMap targetChannelMap = ChannelMap::FromNumChannels(options.NumTargetChannels);
		std::span<const ChannelMap> targetChannelMaps = targetChannelMap.IsValid()
			? std::span<const ChannelMap>(&targetChannelMap, 1)
			: std::span<const ChannelMap>{};

		const DirectEffectInitParameters directEffectParameters
		{
			.BaseCurve = options.DistanceAttenuationCurve,
			.AttenuationCone = options.AttenuationCone
		};

		const PanEffectInitParameters panEffectParameters
		{
			.SourceChannelMap = ChannelMap::FromNumChannels(options.NumChannels),
			.TargetChannelMaps = targetChannelMaps,
			.EffectParameters = options.PanParameters
		};

		mSourceStuff.emplace(newId, SourceData<Vec3Type>{
			.Id = newId,
				.Listener = options.ListenerId.IsValid() ? options.ListenerId : mDefaultListener,
				.DirectEffectHandle = mDirectPathService.InitializeDirrectEffect(std::move(directEffectParameters)),
				.PanEffectHandle = mPanningService.InitializePanningEffect(std::move(panEffectParameters)),
				.ObjectSize = options.ObjectSize,
				.AttenuationCurve = options.DistanceAttenuationCurve,
				.SpatializationType = options.SpatializationType
		});

		mPositionData.emplace(newId,
							  PositionData{
								  .Distance = 0.0f,
								  .DirectionDot = 1.0f,
								  .Obstruction = 0.0f,
								  .Occlusion = 0.0f
							  });

		// If number of source channel specified, initialize panning data
		// TODO: make panning initialization optional, as per user flags?

		return newId;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::DeleteSource(SourceId source)
	{
		if (!source.IsValid())
			return false;

		if (mSourceStuff.contains(source))
		{
			const auto& sourceData = mSourceStuff[source];
			mDirectPathService.ReleaseEffectData(sourceData.DirectEffectHandle);
			mPanningService.ReleasePanningEffect(sourceData.PanEffectHandle);
		}

		mSourceStuff.erase(source);
		mPositionData.erase(source);
		mDirtySources.erase(source);
#if JPL_HAS_ENV_PROPAGATION
		mSourcesMoved.erase(source);
#endif
		return true;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::SetListener(SourceId source, ListenerId listener)
	{
		if (!source || !listener)
			return false;

		if (!mListeners.contains(listener))
			return false;

		mSourceStuff.at(source).Listener = listener;
		return true;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::SetListenerPosition(ListenerId listener, const Position<Vec3Type>& newPosition)
	{
		auto it = mListeners.find(listener);
		if (it == mListeners.end())
			return false;

		auto& foundListener = it->second;
		if (foundListener.Position != newPosition)
		{
			foundListener.Position = newPosition;

			// Flag relevant sourced for the update
			for (const auto& [sourceId, sourceData] : mSourceStuff)
			{
				if (sourceData.Listener == listener)
					mDirtySources.insert(sourceId);
			}
#if JPL_HAS_ENV_PROPAGATION
			mListenersMoved.insert(listener);
#endif
		}

		return true;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE std::optional<Position<Vec3Type>> SpatialManager<Vec3Type, VBAPTraits>::GetListenerPosition(ListenerId listener) const
	{
		auto it = mListeners.find(listener);
		if (it != mListeners.end())
			return it->second.Position;
		return {};
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::SetSourcePosition(SourceId source, const Position<Vec3Type>& newPosition)
	{
		auto it = mSourceStuff.find(source);
		if (it == mSourceStuff.end())
			return false;

		auto& foundSource = it->second;
		if (foundSource.Position != newPosition)
		{
			foundSource.Position = newPosition;
			mDirtySources.insert(source);
#if JPL_HAS_ENV_PROPAGATION
			mSourcesMoved.insert(source);
#endif
		}

		return true;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE bool SpatialManager<Vec3Type, VBAPTraits>::SetSourceFocusAndSpread(SourceId source, PanEffectParameters focusAndSpread)
	{
		return mPanningService.SetPanningEffectParameters(
			GetPanEffectHandle(source),
			focusAndSpread);
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE float SpatialManager<Vec3Type, VBAPTraits>::GetSpreadFromSourceSize(float sourceSize, float distance)
	{
		if (distance <= 0.0f)
			return 1.0f;

		return std::atan((0.5f * sourceSize) / distance) * JPL_INV_PI;
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE float SpatialManager<Vec3Type, VBAPTraits>::GetDistanceAttenuation(SourceId source, const AttenuationCurveRef& curve) const
	{
		return mDirectPathService.GetDistanceAttenuation(mSourceStuff.at(source).DirectEffectHandle, curve);
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE float SpatialManager<Vec3Type, VBAPTraits>::GetConeAttenuation(SourceId source) const
	{
		return mDirectPathService.GetDirectionAttenuation(mSourceStuff.at(source).DirectEffectHandle);
	}

	template<CVec3 Vec3Type, class VBAPTraits>
	JPL_INLINE std::span<const float> SpatialManager<Vec3Type, VBAPTraits>::GetChannelGains(SourceId source, ChannelMap targetChannelMap) const
	{
		return mPanningService.GetChannelGainsFor(mSourceStuff.at(source).PanEffectHandle, targetChannelMap);
	}

} // JPL::Spatial

#endif