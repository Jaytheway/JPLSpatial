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

#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Containers/FlatMap.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Panning/PannerBase.h"
#include "JPLSpatial/Panning/VBAPanning2D.h"
#include "JPLSpatial/Utilities/IDType.h"

#include <concepts>
#include <span>
#include <memory>
#include <vector>
#include <utility>

namespace JPL
{
	using StandartPanner = VBAPanner2D<VBAPStandartTraits>;
	using StandartSourceLayout = typename StandartPanner::SourceLayoutType;
	using StandartChannelGains = typename VBAPStandartTraits::ChannelGains;
	using StandartPanData = typename StandartPanner::PanUpdateData;
	using StandartPanDataWithOrientation = typename StandartPanner::PanUpdateDataWithOrientation;

	struct PanningServiceIDTag {};
	using PanEffectHandle = IDType<PanningServiceIDTag>;

	struct PanningCacheKey
	{
		PanEffectHandle Handle;
		ChannelMap TargetChannelMap;

		[[nodiscard]] JPL_INLINE constexpr bool operator==(const PanningCacheKey& other) const
		{
			return Handle == other.Handle && TargetChannelMap == other.TargetChannelMap;
		}
	};

	struct SourceLayoutKey
	{
		ChannelMap SourceMap;
		ChannelMap TargetMap;

	};

	[[nodiscard]] JPL_INLINE constexpr bool operator==(const SourceLayoutKey& lhs, const SourceLayoutKey& rhs)
	{
		return lhs.SourceMap == rhs.SourceMap && lhs.TargetMap == rhs.TargetMap;
	}


} // namespace JPL

namespace std
{
	static_assert(std::same_as<size_t, uint64_t>);

	template <>
	struct hash<JPL::SourceLayoutKey>
	{
		size_t operator()(const JPL::SourceLayoutKey& key) const
		{
			return (static_cast<uint64_t>(key.SourceMap.GetChannelMask()) << 32) | key.TargetMap.GetChannelMask();
		}
	};

	template <>
	struct hash<JPL::PanningCacheKey>
	{
		size_t operator()(const JPL::PanningCacheKey& key) const
		{
			// Sanity check that handle is some kind of 32-bit int that we can just bit-shift pack
			static_assert(sizeof(key.Handle) == 4);
			return (static_cast<uint64_t>(std::hash<JPL::PanEffectHandle>{}(key.Handle)) << 32) | key.TargetChannelMap.GetChannelMask();
		}
	};
}

namespace JPL
{
	//==========================================================================
	/// Determines spatialized source channel oritentation
	enum class ESpatializationType
	{
		None,
		Position,
		PositionAndOrientation
	};
	// TODO: should we store spatialization type with the pan effect parameters,
	// or keep it elsewhere since it shouldn't be dynamic?

	struct PanEffectParameters
	{
		float Focus;
		float Spread;
	};

	struct PanEffectInitParameters
	{
		ChannelMap SourceChannelMap;					//< ChannelMap to initialize panning effect for
		std::span<const ChannelMap> TargetChannelMaps;	//< (required) Target ChannelMaps for the panning effect
		
		PanEffectParameters EffectParameters			//< Parameters controlling panning behavior
		{
			.Focus = 1.0f,
			.Spread = 1.0f
		};
	};

	//==========================================================================
	template<CVec3Accessible Vec3Type, template<class> class Allocator = std::allocator>
	class PanningService
	{
		template<class T>
		using AllocatorType = Allocator<T>;
	public:
		// Alias to override allocator for the internal FlatMap we use
		template<class Key, class T>
		using FlatMapType = FlatMapWithAllocator<Key, T, AllocatorType>;
		
		using PannerTraits = VBAPBaseTraits<Vec3Type, AllocatorType>;

		using PannerType = VBAPanner2D<PannerTraits>;
		using SourceLayout = typename PannerType::SourceLayoutType;
		using ChannelGains = typename PannerTraits::ChannelGains;
		using PanData = typename PannerType::PanUpdateData;
		using PanDataWithOrientation = typename PannerType::PanUpdateDataWithOrientation;

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
		JPL_INLINE const PannerType* CreatePannerFor(ChannelMap targetChannelMap);

		/// @returns panner for the target ChannelMap if it exists, `nullptr` otherwise
		JPL_INLINE const PannerType* GetPannerFor(ChannelMap targetChannelMap) const;

		//==========================================================================
		/// High level API

		/// Initialize panning effect with a source channel map and optional target channel maps.
		/// This can also be called to assign different target channel maps to the source.
		/// 
		/// @returns		handle for the panning effect interface, which may be invalid if failed
		///				to initialize data, e.g. if the source channel map is not valid.
		JPL_INLINE PanEffectHandle InitializePanningEffect(const PanEffectInitParameters& initParameters);

		/// Release any data associated with the source handle.
		/// @returns true if anything was cleared, false if no data was associated with the handle
		JPL_INLINE bool ReleasePanningEffect(PanEffectHandle source);

	private:
		/// Add target maps to be processed for the source when calling EvaluateDirection
		/// @returns		true if targes were added, false if source handle is invalid, was
		///				never initialized, or has been released
		JPL_INLINE bool AddSourceTargets(PanEffectHandle source, std::span<const ChannelMap> targetChannelMaps);
		JPL_INLINE bool AddSourceTargets(PanEffectHandle source, std::initializer_list<const ChannelMap> targetChannelMaps);
	public:

		/// Set parameters for the panning effect to be used when EvaluateDirection is called
		/// @returns		true if effect handle is valid and the parameters were set, false otherwise
		JPL_INLINE bool SetPanningEffectParameters(PanEffectHandle effect, const PanEffectParameters& parameters);

		/// Set spread parameter for the panning effect to be used when EvaluateDirection is called
		/// @returns		true if effect handle is valid and the spead value was set, false otherwise
		JPL_INLINE bool SetPanningEffectSpread(PanEffectHandle effect, float spread);

		/// Process channel gains for the source for all target channel maps initialized for it.
		/// @param position		position of the source relative to listener
		/// @returns				false if the source hanlde is invalid, true otherwise
		JPL_INLINE bool EvaluateDirection(PanEffectHandle source, const Position<Vec3Type>& position, ESpatializationType spatialziationType);

		/// @returns		span of channel gains for the source and target channel map,
		///				empty span if source targets have never been added to the source.
		///				The span contains the gains evaluated at the last call of EvaluateDirection
		JPL_INLINE std::span<const ChannelGains> GetChannelGainsFor(PanEffectHandle source, ChannelMap targetChannelMap) const;

		//==========================================================================
		/// Low level API
		/// Mainly internal functions used by high level API.
		/// May be used at user's discression to bypass high level API.

		// Create panning data for channel map and optionally associate source with it.
		// This can be used to initialize panning data for known channel maps,
		// as well as to associate that data with sources, since the data only created if
		// it doesn't exist yet in the cache.
		// @returns panning data created for the channel map, which can be nullptr if failed to initialize
		JPL_INLINE std::shared_ptr<const SourceLayout> CreatePanningDataFor(SourceLayoutKey layout, PanEffectHandle source = {});

		// @returns panning data for requrested channel map if it was previously created
		// by calling CreatePanningDataFor
		JPL_INLINE std::shared_ptr<const SourceLayout> GetPanningDataFor(SourceLayoutKey layout) const;

		// Initialize channel gains for source channel map.
		// Returned gains must be managed by the user.
		// This overload is meant for the case when using Panners directly.
		JPL_INLINE bool CreateChannelGainsFor(ChannelMap targetChannelMap, ChannelMap sourceChannelMap, std::vector<ChannelGains>& outChannelGains) const;

		// Initialize channel gains for source. This is called internally in AddSourceTargets function.
		// The gains are manaded by PanningService and used to cache results of the last call to EvaluateDirection
		JPL_INLINE bool CreateChannelGainsFor(ChannelMap targetChannelMap, PanEffectHandle source);

		// @returns SourceLayout associated with the source if exists
		JPL_INLINE std::shared_ptr<const SourceLayout> GetSourceLayoutFor(PanEffectHandle source) const;

	private:
		// TODO: do we need to clear these at any point?

		/// Standard panners and VBAPs, shared between sources,
		/// only created for the requested ChannelMaps
		FlatMapType<ChannelMap, PannerType> mPanners;

		//std::unordered_map<ChannelMap, std::shared_ptr<SourceLayout>> mInitializedSourceLayouts;
		// TODO: maybe we could use number of channels everywhere instead of ChannelMap?
		//		In that case we could use simple static arrays here instead of maps
		FlatMapType<SourceLayoutKey, std::shared_ptr<SourceLayout>> mInitializedSourceLayouts;
		// TODO: we should probably delete unused/unreferenced source layouts at some point, right?

		// Static association VBAPs with handles
		FlatMapType<PanEffectHandle, std::shared_ptr<SourceLayout>> mSourceLayouts;

		// Dynamic parameters that can be changed at runtime
		FlatMapType<PanEffectHandle, PanEffectParameters> mPanningParams;

		// Per target map - store ChannelGains of each source channel
		// vector size = number of source channels
		FlatMapType<PanningCacheKey, std::vector<ChannelGains, AllocatorType<ChannelGains>>> mPanningCache;
	};
} // namespace JPL



//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL
{
	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::CreatePannerFor(ChannelMap targetChannelMap) -> const PannerType*
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

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::GetPannerFor(ChannelMap targetChannelMap) const -> const PannerType*
	{
		auto it = mPanners.find(targetChannelMap);
		return it != mPanners.end() ? &(it->second) : nullptr;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::CreatePanningDataFor(SourceLayoutKey layout, PanEffectHandle source /*= {}*/)
		-> std::shared_ptr<const SourceLayout>
	{
		if (!layout.SourceMap.IsValid())
			return nullptr;

		auto& sourceLayout = mInitializedSourceLayouts[layout];

		if (!sourceLayout)
		{
			sourceLayout = std::make_shared<SourceLayout>();

			// We have to initialize source for a specific target output channel layout
			const auto* panner = CreatePannerFor(layout.TargetMap);

			if (!panner || !panner->InitializeSourceLayout(layout.SourceMap, *sourceLayout))
			{
				JPL_ERROR_TAG("PanningService", "Failed to initialize SourceLayout");
				sourceLayout.reset();
				mInitializedSourceLayouts.erase(layout);
			}
		}

		if (source.IsValid() && sourceLayout)
			mSourceLayouts[source] = sourceLayout;

		return sourceLayout;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::GetPanningDataFor(SourceLayoutKey layout) const -> std::shared_ptr<const SourceLayout>
	{
		auto sourceLayout = mInitializedSourceLayouts.find(layout);
		if (sourceLayout != mInitializedSourceLayouts.end())
			return sourceLayout->second;

		return nullptr;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::CreateChannelGainsFor(ChannelMap targetChannelMap,
																				   ChannelMap sourceChannelMap,
																				   std::vector<ChannelGains>& outChannelGains) const
	{
		if (!targetChannelMap.IsValid() || !sourceChannelMap.IsValid())
			return false;

		outChannelGains.resize(sourceChannelMap.GetNumChannels());
		for (auto& gains : outChannelGains)
			gains.fill(0.0f);

		return true;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::CreateChannelGainsFor(ChannelMap targetChannelMap, PanEffectHandle source)
	{
		if (!targetChannelMap.IsValid() || !source.IsValid() || !mSourceLayouts.contains(source))
			return false;

		// TODO: do we want to ensure existance of VBAP association for this source?
		//		We could also try decoupling source channel map from source, and allow multiple source channel maps per source (?)

		// Ensure we have gain arrays for each channel of the source
		auto& channelGains = mPanningCache[
			PanningCacheKey{
				.Handle = source,
				.TargetChannelMap = targetChannelMap
			}];

			channelGains.resize(mSourceLayouts[source]->ChannelGroups.size());
			for (auto& gains : channelGains)
				gains.fill(0.0f);

			return true;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::GetChannelGainsFor(PanEffectHandle source, ChannelMap targetChannelMap) const
		-> std::span<const ChannelGains>
	{
		auto cache = mPanningCache.find(
			PanningCacheKey{
				.Handle = source,
				.TargetChannelMap = targetChannelMap
			});

		if (cache != mPanningCache.end())
			return cache->second;

		return {};
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE PanEffectHandle PanningService<Vec3Type, AllocatorType>::InitializePanningEffect(const PanEffectInitParameters& initParameters)
	{
		if (!initParameters.SourceChannelMap.IsValid())
			return {};

		if (initParameters.TargetChannelMaps.empty())
			return {};

		const auto newHandle = PanEffectHandle::New();

		// Create panning data for each source->target pair requirested for this source
		for (const ChannelMap& targetMap : initParameters.TargetChannelMaps)
		{
			std::shared_ptr<const SourceLayout> panningData =
				CreatePanningDataFor(
					SourceLayoutKey{
						.SourceMap = initParameters.SourceChannelMap,
						.TargetMap = targetMap
					}, newHandle);

			if (!panningData)
			{
				ReleasePanningEffect(newHandle);
				return {};
			}
		}

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

		return {};
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::ReleasePanningEffect(PanEffectHandle source)
	{
		return mSourceLayouts.erase(source)
			+ mPanningParams.erase(source)
			+ mPanningCache.erase_if([source](const std::pair<const PanningCacheKey, std::vector<ChannelGains>>& pair)
		{
			return pair.first.Handle == source;
		});
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::AddSourceTargets(PanEffectHandle source, std::span<const ChannelMap> targetChannelMaps)
	{
		if (!source.IsValid())
			return false;

		if (!mSourceLayouts.contains(source))
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

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::AddSourceTargets(PanEffectHandle source, std::initializer_list<const ChannelMap> targetChannelMaps)
	{
		return AddSourceTargets(source, std::span(targetChannelMaps));
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::EvaluateDirection(PanEffectHandle source, const Position<Vec3Type>& position, ESpatializationType spatialziationType)
	{
		if (!source.IsValid())
			return false;

		// Assert Location is a normalized relative unit length direction vector
		JPL_ASSERT(Math::IsNearlyEqual(LengthSquared(position.Location), 1.0f));

		for (auto&& [key, gains] : mPanningCache)
		{
			if (key.Handle == source && !gains.empty())
			{
				JPL_ASSERT(mSourceLayouts.contains(key.Handle));
				JPL_ASSERT(mPanners.contains(key.TargetChannelMap));

				const auto& sourceLayout = mSourceLayouts[key.Handle];
				const auto& params = mPanningParams[key.Handle];

				JPL_ASSERT(sourceLayout);

				auto getOutGains = [&gains](uint32 channel) -> auto& { return gains[channel]; };

				// TODO: move this switch statement outside of the loop
				switch (spatialziationType)
				{
				case ESpatializationType::None:
				{
					// TODO: we need to still do basic direct gain assignment
					JPL_ASSERT(false, "Not implemented.");
				}
				break;
				case ESpatializationType::Position:
				{
					const PanData updateData
					{
						.SourceDirection = position.Location, // location is direction
						.Focus = params.Focus,
						.Spread = params.Spread,
					};
					mPanners[key.TargetChannelMap].ProcessVBAPData(*sourceLayout, updateData, getOutGains);
				}
				break;
				case ESpatializationType::PositionAndOrientation:
				{
					const PanDataWithOrientation updateData
					{
						.Pan = {
							.SourceDirection = position.Location,
							.Focus = params.Focus,
							.Spread = params.Spread
						},
						.Orientation = position.Orientation // TODO: orientation is actually up and forward
					};
					mPanners[key.TargetChannelMap].ProcessVBAPData(*sourceLayout, updateData, getOutGains);
				}
				break;
				default:
					JPL_ASSERT(false, "Should be unreachable");
					break;
				}
			}
		}

		return true;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE auto PanningService<Vec3Type, AllocatorType>::GetSourceLayoutFor(PanEffectHandle source) const -> std::shared_ptr<const SourceLayout>
	{
		auto data = mSourceLayouts.find(source);
		return data == mSourceLayouts.end() ? nullptr : data->second;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool PanningService<Vec3Type, AllocatorType>::SetPanningEffectParameters(PanEffectHandle effect, const PanEffectParameters& parameters)
	{
		if (!effect.IsValid())
			return false;

		mPanningParams[effect] = parameters;
		return true;
	}

	template<CVec3Accessible Vec3Type, template<class> class AllocatorType>
	JPL_INLINE bool JPL::PanningService<Vec3Type, AllocatorType>::SetPanningEffectSpread(PanEffectHandle effect, float spread)
	{
		if (!effect.IsValid())
			return false;

		mPanningParams[effect].Spread = spread;
		return true;
	}

} // namespace JPL
