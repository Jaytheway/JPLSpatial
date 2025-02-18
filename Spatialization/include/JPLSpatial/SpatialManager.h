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

#include "Core.h"
#include "IDType.h"
#include "DistanceAttenuation.h"
#include "Services/DirectPathService.h"
#include "Services/PanningService.h"

#include <Jolt/Jolt.h>

#include <unordered_map>
#include <unordered_set>
#include <memory>
#include <ranges>
#include <optional>

namespace JPL
{
#ifdef JPL_TEST
	class SpatializationTest_CreateDeleteSource_Test;
#endif
} // namespace JPL

namespace JPL::Spatial
{
	// TODO: CUSTOMIZATION TRAITS

	class SpatialManager;
	using SourceId = IDType<SpatialManager>;

	struct ListenerMock;
	using ListenerId = IDType<ListenerMock>;

	struct ObjectInSpace;
	using PositionId = IDType<ObjectInSpace>;

	struct SourceInitParameters
	{
		// TODO: decide out if we want to cache some of the parameters (occlusion, etc.) per source, or per object

		/// Number of source channels. If 0, panning is not initialized for this source.
		uint32 NumChannels = 0;

		/// Number of target channels for panning. If 0, panning target channel maps are not initialzied
		uint32 NumTargetChannels = 0;

		/// Optional listener to assign this source to. If not set, default listener is used.
		ListenerId ListenerId;

		//? not used yet
		// Object this source should be associated to,
		// or 0, to associate this source to a new object.
		uint64 ObjectId;

		/// Pan parameters used if panning effect is initialized for the source
		PanEffectParameters PanParameters;

		/// If set, Spread value is calculated from ObjectSize based on distance to the listener
		std::optional<float> ObjectSize;
		
		/// Base distance attenuation curve
		AttenuationCurveRef DistanceAttenuationCurve; 

		/// Source directivity cone
		AttenuationCone AttenuationCone;
	};

	struct ListenerMock
	{
		ListenerId Id;
		JPH::Mat44 Position;
	};

	// At each position in space relative to listener we have:
	struct PositionData
	{
		float Distance;
		float Azimuth;
		float Altitude;

		float Obstruction;
		float Occlusion;

		//float AirAbsorptionFilterValue; //? probably this doesn't belong her, since it is next level. i.e. computed from Distance
	};

	//? not used yet
	struct ObjectInSpace
	{
		PositionId Id;
		JPH::Mat44 Position;

		// Data computed for the object position
		PositionData Data;
	};

	struct SourceData
	{
		SourceId Id;
		ListenerId Listener;

		// TODO: maybe just associate Source to ObjectInSpace, or treat source as object in space?
		JPH::Mat44 Position;

		DirectEffectHandle DirectEffectHandle;
		PanEffectHandle PanEffectHandle;

		std::optional<float> ObjectSize;

		// std::vector<ReflectionId> ReflectionImageSources;
		// std::vector<DifractionId> DifractionSources;

		// TODO: this could be treated as main distance volume attenuation curve to use for indirect paths
		AttenuationCurveRef AttenuationCurve;
	};

	//==========================================================================
	/// High level interface to manage spatialized sources.
	/// It holds an instance of each spatialization service.
	class SpatialManager
	{
	private:
		/*static */JPL_INLINE SourceId NewSourceId() { return SourceId::New(); }

	public:
		SpatialManager() = default;
		~SpatialManager() = default;

		SpatialManager(const SpatialManager&) = delete;
		SpatialManager& operator=(const SpatialManager&) = delete;

		// Evaluate all the updated parametesr to all the spatial effects.
		// Process sound propagation simulation.

		JPL_INLINE const DirectPathService& GetDirectPathService() const { return mDirectPathService; }
		JPL_INLINE DirectPathService& GetDirectPathService() { return mDirectPathService; }
		JPL_INLINE const PanningService& GetPanningService() const { return mPanningService; }
		JPL_INLINE PanningService& GetPanningService() { return mPanningService; }

		/// Process all the parameters of the modified sources.
		/// E.g. this can be called at the end of an update frame
		/// to process changed positions of all the sources.
		JPL_INLINE void AdvanceSimulation();

		JPL_INLINE const std::vector<SourceId>& GetLastUpdatedSource() const;

		JPL_INLINE ListenerId CreateListener();
		JPL_INLINE bool DeleteListener(ListenerId listener);

		JPL_INLINE SourceId CreateSource(const SourceInitParameters& options);
		JPL_INLINE bool DeleteSource(SourceId source);

		JPL_INLINE bool SetListener(SourceId source, ListenerId listener);
		JPL_INLINE ListenerId GetDefaultListener() const { return mDefaultListener; }

		/// Set new position for the listener. This will flag all sources associated
		/// with this listener for update if the position is different than the current one.
		/// @returns		'true' if the listener is valid, 'false' otherwise
		JPL_INLINE bool SetListenerPosition(ListenerId listener, const JPH::Mat44& newPosition);
		JPL_INLINE std::optional<JPH::Mat44> GetListenerPosition(ListenerId listener) const;

		/// Updating any data for sources can be done many times,
		/// the data is only processed when calling AdvanceSimulation()
		JPL_INLINE bool SetSourcePosition(SourceId source, const JPH::Mat44& newPosition);

		/// Set focus and spread parameters for the source.
		/// @returns		'true' if the source is valid and has PanEffect initialized, 'fasle' otherwise
		JPL_INLINE bool SetSourceFocusAndSpread(SourceId source, PanParameters focusAndSpread);

		static JPL_INLINE float GetSpreadFromSourceSize(float sourceSize, float distance);

		JPL_INLINE DirectEffectHandle GetDirectEffectHandle(SourceId source) const { return mSourceStuff.at(source).DirectEffectHandle; }
		JPL_INLINE PanEffectHandle GetPanEffectHandle(SourceId source) const { return mSourceStuff.at(source).PanEffectHandle; }

		/// @returns		distance attenuation value evaluated for the 'source' and 'curve'
		///				the last time AdvanceSimulation() was called
		JPL_INLINE float GetDistanceAttenuation(SourceId source, const AttenuationCurveRef& curve) const;

		JPL_INLINE float GetConeAttenuation(SourceId source) const;

		/// Get channel gains of the source for the ChannelMap based on the number of values/channels in `outGains`
		/// The returned channel gains are based on the data computed during the last call to AdvanceSimulation()
		/// @returns		pointer to vector of ChannelGains per source channel, if the 'source' is valid and
		///				the `targetChannelMap` was initialized for that source
		JPL_INLINE const std::vector<StandartChannelGains>* GetChannelGains(SourceId source, ChannelMap targetChannelMap) const;
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
		PanningService mPanningService;

		// Sources to be updated at next call to AdvanceSimulation()
		std::unordered_set<SourceId> mDirtySources;		///< Sources to update paths for

		// Object that have changed during last update.
		// User or mixing engine may need this information.
		std::vector<SourceId> mLastUpdatedSources;
		std::vector<SourceId> mLastSourceRoomsChanged;
		std::vector<ListenerId> mLastListenerRoomsChanged;

		// Per source data
		std::unordered_map<SourceId, SourceData> mSourceStuff;

		// Intermediate data processed for the position the sources were at
		// at the last call to AdvanceSimulation()
		std::unordered_map<SourceId, PositionData> mPositionData;

		// Default listener
		const ListenerId mDefaultListener = ListenerId::New();

		// Map of listeners
		std::unordered_map<ListenerId, ListenerMock> mListeners
		{
			{ mDefaultListener, ListenerMock{
				.Id = mDefaultListener,
				.Position = JPH::Mat44::sLookAt(JPH::Vec3::sZero(),
												-JPH::Vec3::sAxisZ(),
												JPH::Vec3::sAxisY()) }}
		};
	};

} // namespace JPL::Spatialziation

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL::Spatial
{
	JPL_INLINE void SpatialManager::AdvanceSimulation()
	{
		// TODO: before processing distance attenuation we need to process propagation paths,
		//		since sources may not have direct path, also we need to process distance attenuation
		//		for indirect paths and reflections

		// Remove dirty sources that don't have listeners
		std::erase_if(mDirtySources, [this](SourceId source)
		{
			const auto& sourceStuff = mSourceStuff[source];
			return !mListeners.contains(sourceStuff.Listener);
		});

		for (SourceId source : mDirtySources)
		{
			const SourceData& sourceData = mSourceStuff[source];

			const auto& sourcePosition = sourceData.Position;
			const auto& listenerPosition = mListeners[sourceData.Listener].Position;

			const DirectPathResult directPathResult =
				mDirectPathService.ProcessDirectPath(
					sourcePosition,
					listenerPosition
				);

			// Cache direct path data
			auto& positionData = mPositionData[source];
			positionData.Distance = directPathResult.Distance;
			positionData.Azimuth = directPathResult.Azimuth;
			positionData.Altitude = directPathResult.Altitude;

			// Attenuation is a higher level API, depends on AttenuationCurve,
			// we can't assume what curve user wants to use.
			// But we can preprocess all curves associated with this source.
			// TODO: mabye we could batch process multiple distances/sources per curve instead, if they are sharing curves?
			mDirectPathService.EvaluateDistance(sourceData.DirectEffectHandle, directPathResult.Distance);

			// Calculate cone attenuatoin factor
			mDirectPathService.EvaluateDirection(sourceData.DirectEffectHandle, directPathResult.InvAzimuth);


			// TODO: can we eliminate this branch?
			if (sourceData.ObjectSize)
			{
				const float spread = GetSpreadFromSourceSize(*sourceData.ObjectSize, directPathResult.Distance);
				mPanningService.SetPanningEffectSpread(sourceData.PanEffectHandle, spread);
			}

			// TODO: this is not very useful for indirect paths that don't use spread and focus
			mPanningService.EvaluateDirection(sourceData.PanEffectHandle, positionData.Azimuth, positionData.Altitude);
		}

		// Store sources updated during this iteration to let user check
		// which sourced were actually changed
		mLastUpdatedSources.resize(mDirtySources.size());
		std::ranges::copy(mDirtySources, mLastUpdatedSources.begin());

		mDirtySources.clear();
	}

	JPL_INLINE const std::vector<SourceId>& SpatialManager::GetLastUpdatedSource() const
	{
		return mLastUpdatedSources;
	}

	JPL_INLINE ListenerId SpatialManager::CreateListener()
	{
		const auto listenerId = ListenerId::New();
		mListeners.emplace(listenerId,
						   ListenerMock{
							   .Id = listenerId,
							   .Position = JPH::Mat44::sLookAt(JPH::Vec3::sZero(),
															   -JPH::Vec3::sAxisZ(),
															   JPH::Vec3::sAxisY())
						   });
		return listenerId;
	}

	JPL_INLINE bool SpatialManager::DeleteListener(ListenerId listener)
	{
		if (!mListeners.contains(listener) || listener == mDefaultListener)
			return false;

		mListeners.erase(listener);

		// TODO: we may or may not want to assign default listener to orphan sources
		return true;
	}

	JPL_INLINE SourceId SpatialManager::CreateSource(const SourceInitParameters& options)
	{
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

		mSourceStuff.emplace(newId,
							 SourceData{
								 .Id = newId,
								 .Listener = options.ListenerId.IsValid() ? options.ListenerId : mDefaultListener,
								 .DirectEffectHandle = mDirectPathService.InitializeDirrectEffect(std::move(directEffectParameters)),
								 .PanEffectHandle = mPanningService.InitializePanningEffect(std::move(panEffectParameters)),
								 .ObjectSize = options.ObjectSize,
								 .AttenuationCurve = options.DistanceAttenuationCurve
							 });

		mPositionData.emplace(newId,
							  PositionData{
								  .Distance = 0.0f,
								  .Azimuth = 0.0f,
								  .Altitude = 0.0f
							  });

		// If number of source channel specified, initialize panning data
		// TODO: make panning initialization optional, as per user flags?

		return newId;
	}

	JPL_INLINE bool SpatialManager::DeleteSource(SourceId source)
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

		return true;
	}

	JPL_INLINE bool SpatialManager::SetListener(SourceId source, ListenerId listener)
	{
		if (!source || !listener)
			return false;

		if (!mListeners.contains(listener))
			return false;

		mSourceStuff.at(source).Listener = listener;
		return true;
	}

	JPL_INLINE bool SpatialManager::SetListenerPosition(ListenerId listener, const JPH::Mat44& newPosition)
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
		}

		return true;
	}

	JPL_INLINE std::optional<JPH::Mat44> SpatialManager::GetListenerPosition(ListenerId listener) const
	{
		auto it = mListeners.find(listener);
		if (it != mListeners.end())
			return it->second.Position;
		return {};
	}

	JPL_INLINE bool SpatialManager::SetSourcePosition(SourceId source, const JPH::Mat44& newPosition)
	{
		auto it = mSourceStuff.find(source);
		if (it == mSourceStuff.end())
			return false;

		auto& foundSource = it->second;
		if (foundSource.Position != newPosition)
		{
			foundSource.Position = newPosition;
			mDirtySources.insert(source);
		}

		return true;
	}

	JPL_INLINE bool SpatialManager::SetSourceFocusAndSpread(SourceId source, PanParameters focusAndSpread)
	{
		return mPanningService.SetPanningEffectParameters(
			GetPanEffectHandle(source),
			{
				.Focus = focusAndSpread.Focus,
				.Spread = focusAndSpread.Spread
			});
	}

	JPL_INLINE float SpatialManager::GetSpreadFromSourceSize(float sourceSize, float distance)
	{
		if (distance <= 0.0f)
			return 1.0f;

		static constexpr auto inv_pi = 1.0f / std::numbers::pi_v<float>;
		return JPH::ATan((0.5f * sourceSize) / distance) * inv_pi;
	}

	JPL_INLINE float SpatialManager::GetDistanceAttenuation(SourceId source, const AttenuationCurveRef& curve) const
	{
		return mDirectPathService.GetDistanceAttenuation(mSourceStuff.at(source).DirectEffectHandle, curve);
	}

	JPL_INLINE float SpatialManager::GetConeAttenuation(SourceId source) const
	{
		return mDirectPathService.GetDirectionAttenuation(mSourceStuff.at(source).DirectEffectHandle);
	}

	JPL_INLINE const std::vector<StandartChannelGains>* SpatialManager::GetChannelGains(SourceId source, ChannelMap targetChannelMap) const
	{
		return mPanningService.GetChannelGainsFor(mSourceStuff.at(source).PanEffectHandle, targetChannelMap);
	}

} // JPL::Spatial
