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

#include "../VBAP.h"
#include "../IDType.h"

#include "Jolt/Core/Core.h"
#include "Jolt/Core/HashCombine.h"

#include "unordered_map"

namespace JPL
{
	using StandartPanner = VBAPanner<VBAPStandartTraits>;
	using StandartVBAPData = typename StandartPanner::VBAPData;
	using StandartChannelGains = typename VBAPStandartTraits::ChannelGains;
	using StandartPanData = PanUpdateData<VBAPStandartTraits>;

	class PanningService;
	using PanEffectHandle = IDType<PanningService>;

	struct PanningCacheKey
	{
		PanEffectHandle Handle;
		ChannelMap TargetChannelMap;

		constexpr bool operator==(const PanningCacheKey& other) const
		{
			return Handle == other.Handle && TargetChannelMap == other.TargetChannelMap;
		}
	};
} // namespace JPL
JPH_MAKE_HASHABLE(JPL::PanningCacheKey, t.Handle, t.TargetChannelMap)

namespace JPL
{
	//==========================================================================
	struct PanEffectParameters
	{
		float Focus;
		float Spread;
	};

	struct PanEffectInitParameters
	{
		ChannelMap SourceChannelMap;					//< ChannelMap to initialize panning effect for
		std::span<const ChannelMap> TargetChannelMaps;	//< (optional) Target ChannelMaps for the panning effect
		
		PanEffectParameters EffectParameters			//< Parameters controlling panning behavior
		{
			.Focus = 1.0f,
			.Spread = 1.0f
		};
	};

	//==========================================================================
	class PanningService
	{
	public:
		PanningService() = default;

		PanningService(const PanningService&) = delete;
		PanningService& operator=(const PanningService&) = delete;

		// High level API to:
		// - create panners for different kinds of channel maps and panning settings
		// - get channel gains for sources
		// ...

		/// Create panner for the target ChannelMap if it doesn't exist yet
		/// @return panner for the requrested channel map, or nullptr if target channel map is invalid
		JPL_INLINE const StandartPanner* CreatePannerFor(ChannelMap targetChannelMap);
	
		/// @returns panner for the target ChannelMap if it exists, `nullptr` otherwise
		JPL_INLINE const StandartPanner* GetPannerFor(ChannelMap targetChannelMap) const;

		//==========================================================================
		/// High level API

		/// Initialize panning effect with a source channel map and optional target channel maps.
		/// Target channel maps can be added later by calling AddSourceTargets, but without them
		/// nothing will be processed for the source.
		/// 
		/// @returns		handle for the panning effect interface, which may be invalid if failed
		///				to initialize data, e.g. if the source channel map is not valid.
		JPL_INLINE PanEffectHandle InitializePanningEffect(const PanEffectInitParameters& initParameters);
		
		/// Release any data associated with the source handle.
		/// @returns true if anything was cleared, false if no data was associated with the handle
		JPL_INLINE bool ReleasePanningEffect(PanEffectHandle source);

		/// Add target maps to be processed for the source when calling EvaluateDirection
		/// @returns		true if targes were added, false if source handle is invalid, was
		///				never initialized, or has been released
		JPL_INLINE bool AddSourceTargets(PanEffectHandle source, std::span<const ChannelMap> targetChannelMaps);
		JPL_INLINE bool AddSourceTargets(PanEffectHandle source, std::initializer_list<const ChannelMap> targetChannelMaps);

		/// Set parameters for the panning effect to be used when EvaluateDirection is called
		/// @returns		true if effect handle is valid and the parameters were set, false otherwise
		JPL_INLINE bool SetPanningEffectParameters(PanEffectHandle effect, const PanEffectParameters& parameters);

		/// Set spread parameter for the panning effect to be used when EvaluateDirection is called
		/// @returns		true if effect handle is valid and the spead value was set, false otherwise
		JPL_INLINE bool SetPanningEffectSpread(PanEffectHandle effect, float spread);

		/// Process channel gains for the source for all target channel maps initialized for it
		/// @returns		false if the source hanlde is invalid, true otherwise
		JPL_INLINE bool EvaluateDirection(PanEffectHandle source, float azimuth, float /*altitude*/);
		
		/// @returns		pointer to channel gains array for the source and target channel map,
		///				`nullptr` if source targets have never been added to the source.
		///				The array contains the gains evaluated at the last call of EvaluateDirection
		JPL_INLINE const std::vector<StandartChannelGains>* GetChannelGainsFor(PanEffectHandle source, ChannelMap targetChannelMap) const;

		//==========================================================================
		/// Low level API
		/// Mainly internal functions used by high level API.
		/// May be used at user's discression to bypass high level API.

		// Create panning data for channel map and optionally associate source with it.
		// This can be used to initialize panning data for known channel maps,
		// as well as to associate that data with sources, since the data only created if
		// it doesn't exist yet in the cache.
		// @returns panning data created for the channel map, which can be nullptr if failed to initialize
		JPL_INLINE std::shared_ptr<const StandartVBAPData> CreatePanningDataFor(ChannelMap sourceChannelMap, PanEffectHandle source = {});
		
		// @returns panning data for requrested channel map if it was previously created
		// by calling CreatePanningDataFor
		JPL_INLINE std::shared_ptr<const StandartVBAPData> GetPanningDataFor(ChannelMap sourceChannelMap) const;

		// Initialize channel gains for source channel map.
		// Returned gains must be managed by the user.
		// This overload is meant for the case when using Panners directly.
		JPL_INLINE bool CreateChannelGainsFor(ChannelMap targetChannelMap, ChannelMap sourceChannelMap, std::vector<StandartChannelGains>& outChannelGains) const;
		
		// Initialize channel gains for source. This is called internally in AddSourceTargets function.
		// The gains are manaded by PanningService and used to cache results of the last call to EvaluateDirection
		JPL_INLINE bool CreateChannelGainsFor(ChannelMap targetChannelMap, PanEffectHandle source);

		// @returns VBAPData associated with the source if exists
		JPL_INLINE std::shared_ptr<const StandartVBAPData> GetVBAPDataFor(PanEffectHandle source) const;

	private:
		// TODO: do we need to cleare these at any point?

		/// Standard panners and VBAPs, shared between sources,
		/// only created for the requested ChannelMaps
		std::unordered_map<ChannelMap, StandartPanner> mPanners;
		std::unordered_map<ChannelMap, std::shared_ptr<StandartVBAPData>> mVBAPs;
		// TODO: maybe we could use number of channels everywhere instead of ChannelMap?
		//		In that case we could use simple static arrays here instead of maps

		// Static association VBAPs with handles
		std::unordered_map<PanEffectHandle, std::shared_ptr<StandartVBAPData>> mSourceVBAPs;
		
		// Dynamic parameters that can be changed at runtime
		std::unordered_map<PanEffectHandle, PanEffectParameters> mPanningParams;

		// Per target map - store ChannelGains of each source channel
		// vector size = number of source channels
		std::unordered_map<PanningCacheKey, std::vector<StandartChannelGains>> mPanningCache;
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL
{
	JPL_INLINE const StandartPanner* PanningService::CreatePannerFor(ChannelMap targetChannelMap)
	{
		if (!targetChannelMap.IsValid())
			return nullptr;

		auto& panner = mPanners[targetChannelMap];
		if (!panner.IsInitialized())
		{
			if (!panner.InitializeLUT(targetChannelMap))
			{
				mPanners.erase(targetChannelMap);
				return nullptr;
			}
		}
		return &panner;
	}

	JPL_INLINE const StandartPanner* PanningService::GetPannerFor(ChannelMap targetChannelMap) const
	{
		auto it = mPanners.find(targetChannelMap);
		return it != mPanners.end() ? &(it->second) : nullptr;
	}

	JPL_INLINE std::shared_ptr<const StandartVBAPData> PanningService::CreatePanningDataFor(ChannelMap sourceChannelMap, PanEffectHandle source /*= {}*/)
	{
		if (!sourceChannelMap.IsValid())
			return nullptr;

		// TODO: do we need customization for this value, or is it too much micro?
		static constexpr auto numVirtualSourcePerChannel = 3;

		auto& vbapData = mVBAPs[sourceChannelMap];

		if (!vbapData)
		{
			vbapData = std::make_shared<StandartVBAPData>();
			if (!vbapData->Initialize(sourceChannelMap, numVirtualSourcePerChannel))
			{
				JPL_ERROR_TAG("PanningService", "Failed to initialize VBAPData");
				vbapData.reset();
				mVBAPs.erase(sourceChannelMap);
			}
		}

		if (source.IsValid() && vbapData)
			mSourceVBAPs[source] = vbapData;

		return vbapData;
	}

	JPL_INLINE std::shared_ptr<const StandartVBAPData> PanningService::GetPanningDataFor(ChannelMap sourceChannelMap) const
	{
		auto vbapData = mVBAPs.find(sourceChannelMap);
		if (vbapData != mVBAPs.end())
			return vbapData->second;

		return nullptr;
	}

	JPL_INLINE bool PanningService::CreateChannelGainsFor(ChannelMap targetChannelMap,
														  ChannelMap sourceChannelMap,
														  std::vector<StandartChannelGains>& outChannelGains) const
	{
		if (!targetChannelMap.IsValid() || !sourceChannelMap.IsValid())
			return false;

		outChannelGains.resize(sourceChannelMap.GetNumChannels(), { 0 });
		return true;
	}

	JPL_INLINE bool PanningService::CreateChannelGainsFor(ChannelMap targetChannelMap, PanEffectHandle source)
	{
		if (!targetChannelMap.IsValid() || !source.IsValid() || !mSourceVBAPs.contains(source))
			return false;

		// TODO: do we want to ensure existance of VBAP association for this source?
		//		We could also try decoupling source channel map from source, and allow multiple source channel maps per source (?)
			// Ensure we have gain arrays for each channel of the source
		mPanningCache[
			PanningCacheKey{
				.Handle = source,
				.TargetChannelMap = targetChannelMap
			}]
			.resize(mSourceVBAPs[source]->ChannelGroups.size(), { 0 });

		return true;
	}

	JPL_INLINE const std::vector<StandartChannelGains>* PanningService::GetChannelGainsFor(PanEffectHandle source, ChannelMap targetChannelMap) const
	{
		auto cache = mPanningCache.find(
			PanningCacheKey{
				.Handle = source,
				.TargetChannelMap = targetChannelMap
			});

		if (cache != mPanningCache.end())
			return &cache->second;

		return nullptr;
	}

	JPL_INLINE PanEffectHandle PanningService::InitializePanningEffect(const PanEffectInitParameters& initParameters)
	{
		if (!initParameters.SourceChannelMap.IsValid())
			return {};

		const auto newHandle = PanEffectHandle::New();

		if (std::shared_ptr<const StandartVBAPData> panningData = CreatePanningDataFor(initParameters.SourceChannelMap, newHandle))
		{
			if (AddSourceTargets(newHandle, initParameters.TargetChannelMaps))
			{
				mPanningParams[newHandle] = initParameters.EffectParameters;
				
				return newHandle;
			}
			else
			{
				// If we failed to initialize taret data,
				// we need to clear other data that might have been initialized
				ReleasePanningEffect(newHandle);
			}
		}

		return {};
	}

	JPL_INLINE bool PanningService::ReleasePanningEffect(PanEffectHandle source)
	{
		return mSourceVBAPs.erase(source)
			+ mPanningParams.erase(source)
			+ std::erase_if(mPanningCache, [source](const std::pair<const PanningCacheKey, std::vector<StandartChannelGains>>& pair)
				{
					return pair.first.Handle == source;
				});
	}

	JPL_INLINE bool PanningService::AddSourceTargets(PanEffectHandle source, std::span<const ChannelMap> targetChannelMaps)
	{
		if (!source.IsValid())
			return false;

		if (!mSourceVBAPs.contains(source))
		{
			JPL_ERROR_TAG("PanningService", "AddSoruceTargets failed because source handle wasn't initialized before adding source targets. "
						  "This is likely because the provided source handle was released previously and is no longer valid.");
			return false;
		}

		if (targetChannelMaps.empty())
			return true;

		bool hasAnyTargetInitialized = false;

		for (ChannelMap channelMap : targetChannelMaps)
		{
			hasAnyTargetInitialized |= CreateChannelGainsFor(channelMap, source);
		}

		return hasAnyTargetInitialized;
	}

	JPL_INLINE bool PanningService::AddSourceTargets(PanEffectHandle source, std::initializer_list<const ChannelMap> targetChannelMaps)
	{
		return AddSourceTargets(source, std::span(targetChannelMaps));
	}

	JPL_INLINE bool PanningService::EvaluateDirection(PanEffectHandle source, float azimuth, float /*altitude*/)
	{
		if (!source.IsValid())
			return false;

		for (auto& [key, gains] : mPanningCache)
		{
			if (key.Handle == source && !gains.empty())
			{
				JPL_ASSERT(mSourceVBAPs.contains(key.Handle));
				JPL_ASSERT(mPanners.contains(key.TargetChannelMap));

				const auto& vbapData = mSourceVBAPs[key.Handle];
				const auto& params = mPanningParams[key.Handle];

				JPL_ASSERT(vbapData);

				const StandartPanData updateData
				{
					.PanAngle = azimuth,
					.Focus = params.Focus,
					.Spread = params.Spread
				};

				auto getOutGains = [&gains](uint32 channel) -> auto& { return gains[channel]; };

				mPanners[key.TargetChannelMap].ProcessVBAPData(*vbapData, updateData, getOutGains);
			}
		}

		return true;
	}

	JPL_INLINE std::shared_ptr<const StandartVBAPData> PanningService::GetVBAPDataFor(PanEffectHandle source) const
	{
		auto data = mSourceVBAPs.find(source);
		return data == mSourceVBAPs.end() ? nullptr : data->second;
	}

	JPL_INLINE bool PanningService::SetPanningEffectParameters(PanEffectHandle effect, const PanEffectParameters& parameters)
	{
		if (!effect.IsValid())
			return false;
		
		mPanningParams[effect] = parameters;
		return true;
	}

	JPL_INLINE bool JPL::PanningService::SetPanningEffectSpread(PanEffectHandle effect, float spread)
	{
		if (!effect.IsValid())
			return false;

		mPanningParams[effect].Spread = spread;
		return true;
	}

} // namespace JPL