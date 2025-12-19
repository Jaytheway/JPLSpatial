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
#include "JPLSpatial/ErrorReporting.h"

#include "JPLSpatial/ChannelMap.h"

#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Math/Vec3Math.h"
#include "JPLSpatial/Math/MinimalQuat.h"
#include "JPLSpatial/Math/MinimalBasis.h"
#include "JPLSpatial/Math/Position.h"
#include "JPLSpatial/Memory/Memory.h"

#include "JPLSpatial/Algo/Algorithm.h"

#include "JPLSpatial/Panning/VBAPEx.h"

#include <array>
#include <optional>
#include <vector>
#include <limits>
#include <map>
#include <span>
#include <ranges>
#include <algorithm>
#include <numeric>
#include <memory>

#define JPL_VALIDATE_VBAP_LUT 0

namespace JPL
{
	/// Forward declaration
	template<CVec3 Vec3>
	struct VBAPBaseTraits;

	//======================================================================
	using VBAPStandartTraits = VBAPBaseTraits<MinimalVec3>;

	//======================================================================
	/// Customization. You can either inherit from this and shadow some types
	/// and functions, or use a completely separate traits type with VBAP API.
	template<CVec3 Vec3>
	struct VBAPBaseTraits
	{
		using Vec3Type = Vec3;

		/// Can be useful to override to save a tiny bit of memory if max number
		/// of channels ever used is known at compile time. Or to handle larger
		/// channel maps.
		/// (though currently ChannelMap interface doesn't support 64-bit masks
		/// and is not very customizable, e.g. storing in an int mask, not array)
		static constexpr auto MAX_CHANNELS = 32;

		using ChannelGains = std::array<float, MAX_CHANNELS>;

		// TODO: customizable EChannel type and ChannelMap type as well?
		static float GetChannelAngle(EChannel channel)
		{
			/// Standard channel vectors converted to angles
			static constexpr auto gChannelAngles
				= std::to_array({
					std::pair<EChannel, float>{ FrontLeft,       -0.785398f },
					std::pair<EChannel, float>{ FrontRight,      0.785398f },
					std::pair<EChannel, float>{ FrontCenter,     0.0f },
					std::pair<EChannel, float>{ LFE,             0.0f },
					std::pair<EChannel, float>{ BackLeft,        -2.35619f },
					std::pair<EChannel, float>{ BackRight,       2.35619f },
					std::pair<EChannel, float>{ FrontLeftCenter, -0.321719f },
					std::pair<EChannel, float>{ FrontRightCenter,0.321719f },
					std::pair<EChannel, float>{ BackCenter,      3.14159f },
					std::pair<EChannel, float>{ SideLeft,        -1.5708f },
					std::pair<EChannel, float>{ SideRight,       1.5708f },

					std::pair<EChannel, float>{ TopCenter,       3.14159f },
					std::pair<EChannel, float>{ TopFrontLeft,    -0.785398f },
					std::pair<EChannel, float>{ TopFrontCenter,  0.0f },
					std::pair<EChannel, float>{ TopFrontRight,   0.785398f },

					std::pair<EChannel, float>{ TopSideLeft,      -1.5708f },
					std::pair<EChannel, float>{ TopSideRight,     1.5708f },

					std::pair<EChannel, float>{ TopBackLeft,     -2.35619f },
					std::pair<EChannel, float>{ TopBackCenter,   3.14159f },
					std::pair<EChannel, float>{ TopBackRight,    2.35619f },

					std::pair<EChannel, float>{ WideLeft,       -1.0472f },
					std::pair<EChannel, float>{ WideRight,       1.0472f }
				});

			return std::find_if(gChannelAngles.begin(), gChannelAngles.end(),
								[channel](const std::pair<EChannel, float>& p)
								{
									return p.first == channel;
								})->second;
		}

		static Vec3Type GetChannelVector(EChannel channel)
		{
			/// Standard channel vectors converted to angles
			static constexpr auto gChannelVectors =
				std::to_array({
					std::pair<EChannel, Vec3Type>{ FrontLeft,        { -0.7071f, 0.0f,       -0.7071f } },
					std::pair<EChannel, Vec3Type>{ FrontRight,       { 0.7071f,  0.0f,       -0.7071f } },
					std::pair<EChannel, Vec3Type>{ FrontCenter,      { 0.0f,     0.0f,       -1.0f } },
					std::pair<EChannel, Vec3Type>{ LFE,              { 0.0f,     0.0f,       -1.0f } },
					std::pair<EChannel, Vec3Type>{ BackLeft,         { -0.7071f, 0.0f,       0.7071f } },
					std::pair<EChannel, Vec3Type>{ BackRight,        { 0.7071f,  0.0f,       0.7071f } },
					std::pair<EChannel, Vec3Type>{ FrontLeftCenter,  { -0.3162f, 0.0f,       -0.9487f } },
					std::pair<EChannel, Vec3Type>{ FrontRightCenter, { 0.3162f,  0.0f,       -0.9487f } },
					std::pair<EChannel, Vec3Type>{ BackCenter,       { 0.0f,     0.0f,       1.0f } },
					std::pair<EChannel, Vec3Type>{ SideLeft,         { -1.0f,    0.0f,       0.0f } },
					std::pair<EChannel, Vec3Type>{ SideRight,        { 1.0f,     0.0f,       0.0f } },

					std::pair<EChannel, Vec3Type>{ TopCenter,        { 0.0f,     1.0f,       0.0f } },
					std::pair<EChannel, Vec3Type>{ TopFrontLeft,     { -0.5774f, 0.5774f,    -0.5774f } },
					std::pair<EChannel, Vec3Type>{ TopFrontCenter,   { 0.0f,     0.7071f,    -0.7071f } },
					std::pair<EChannel, Vec3Type>{ TopFrontRight,    { 0.5774f,  0.5774f,    -0.5774f } },

					std::pair<EChannel, Vec3Type>{ TopSideLeft,      { -0.7071f, 0.7071f,    0.0f } },
					std::pair<EChannel, Vec3Type>{ TopSideRight,     { 0.7071f,  0.7071f,    0.0f } },

					std::pair<EChannel, Vec3Type>{ TopBackLeft,      { -0.5774f, 0.5774f,    0.5774f } },
					std::pair<EChannel, Vec3Type>{ TopBackCenter,    { 0.0f,     0.7071f,    0.7071f } },
					std::pair<EChannel, Vec3Type>{ TopBackRight,     { 0.5774f,  0.5774f,    0.5774f } },

					std::pair<EChannel, Vec3Type>{ WideLeft,         { -0.866f,  0.0f,       -0.5f } },
					std::pair<EChannel, Vec3Type>{ WideRight,        { 0.866f,   0.0f,       -0.5f } },

				});

			return std::find_if(gChannelVectors.begin(), gChannelVectors.end(),
								[channel](const std::pair<EChannel, Vec3Type>& p)
								{
									return p.first == channel;
								})->second;
		}
	};

	//======================================================================
	/// @returns error message if `channelMap` is not a valid channel map
	/// for a VBAP panning source; nullopt otherwise
	[[nodiscard]] JPL_INLINE std::optional<const char*> IsValidSourceChannelMap(ChannelMap channelMap)
	{
		// TODO: handle top channels, maybe we can just ignore them for now, instead of failing to initialize?
		if (!JPL_ENSURE(channelMap.IsValid()))
			return "Failed to initialize SourceLayout, provided invalid channel map.";

		// TODO: handle this more gracefully
		if (!JPL_ENSURE(!channelMap.HasTopChannels()))
			return "Failed to initialize SourceLayout for VBAPPanner, provided channel map has top channels that don't participate in panning.";

		return std::nullopt;
	}

	//======================================================================
	/// Vector Based Amplitude Panner that can handle panning from any
	/// source channel map to any target/output channel map.
	/// 
	/// Internally uses MDAP (Multiple Directions Amplitude Panning)
	/// with virtual sources (aka virtual positions).
	/// 
	/// SourceLayout and ProcessVBAPData function can be used for a more conventional
	/// channel map -> channel map panning, based on parameters like spread and
	/// focus, which can be driven by, for example, distance from the soruce
	/// to the listener (on the client side).
	/// 
	/// While VirtualSource(s) can be used directly with ProcessVirtualSources
	/// function for a more customizable spatialization, e.g. for volumetric
	/// sound sources, or any other cases where position and spatial
	/// extend of each channel needed to be manually cotrolled.
	/// 
	/// Instead of using VBAPannerBase direction, it is adviced to use
	/// 'VBAPanner2D' and 'VBAPanner3D' aliases from the similarly named files.
	template<class PannerType, class Traits>
	class VBAPannerBase
	{
	public:
		/// Aliases to avoid typing wordy templates
		using Vec3Type = typename Traits::Vec3Type;
		template<class T> using Array = std::pmr::vector<T>;
		using ChannelGainsRef = typename Traits::ChannelGains&;

		//======================================================================
		template<class T> static constexpr bool CGetSpeakerGainsFunction = std::is_invocable_r_v<ChannelGainsRef, T, uint32>;

		// TODO: maybe move these params outside of the template, otherwise we have different types for 2D and 3D traits
		//======================================================================
		/// Parameters to update VBAP data for a source
		struct PanUpdateData
		{
			Vec3Type SourceDirection;
			float Focus;                // [0, 1]
			float Spread;               // [0, 1]
		};

		using PanSourceOrientation = OrientationData<Vec3Type>;

		struct PanUpdateDataWithOrientation
		{
			PanUpdateData Pan;
			PanSourceOrientation Orientation;
		};

		//======================================================================
		/// Represents one of potentially many dirctions taken into account
		/// when calculating resulting panning for a source layout
		struct VirtualSource
		{
			Vec3Type Direction; // Direction relative to listener
			float Weight;       // Contribution to the resulting pan image
		};

		/// Represents source channel rotation.
		/// Initializing SourceLayout for a ChannelMap creates a ChannelGroup per.
		/// channel of the source ChannelMap.
		struct ChannelGroup
		{
			/// Channel group rotation offset as per the channel this group is associated to
			Quat<Vec3Type> Rotation;

			/// Index or Id of the channel this group is associated to
			uint32 Channel;
		};

		//======================================================================
		/// Represents channel layout of a a source as groups of Virtual Sources
		/// per source channel.
		/// SourceLayout can be initialized per ChannelMap and reused for soruces
		/// with the same channel count routing to the output of the same target
		/// channel count.
		/// (initialized by the panner of the target output channel map)
		struct VBAPLayoutBase
		{
			/// Groups of virtual sources associated with source channels
			Array<ChannelGroup> ChannelGroups{ GetDefaultMemoryResource()};

			[[nodiscard]] JPL_INLINE bool IsInitialized() const noexcept { return !ChannelGroups.empty(); }
			[[nodiscard]] JPL_INLINE ChannelMap GetTargetChannelMap() const noexcept { return mTargetChannelMap; }

		protected:
			[[nodiscard]] JPL_INLINE float GetDefaultVSWeight() const noexcept { return mDefaultVSWeight; }
			JPL_INLINE void SetTargetChannelMap(ChannelMap targetChannelMap) noexcept { mTargetChannelMap = targetChannelMap; }

			bool InitializeBase(ChannelMap channelMap, ChannelMap targetMap, uint32 numVirtualSourcesPerChannel);

		protected:
			ChannelMap mTargetChannelMap;       // Output channel map this source layout is made for
			float mDefaultVSWeight;             // Cached virtual source default weight based on number of virtual sources per channel
		};

		//======================================================================
		using PanType = PannerType;
		using LUTType = typename PanType::LUTType;
		using LUTInterface = typename PanType::LUTInterface;
		using SourceLayoutType = typename PanType::SourceLayout;
		//..we have to declare PanType at the bottom to give it visibility
		// to the types defiend in VBAPannerBase

		// Sanity check
		static_assert(std::same_as<typename LUTInterface::Vec3Type, Vec3Type>);

	public:

		//======================================================================
		/// Initialize LUT for a specific `channelMap`.
		/// @param channelMap : target channel map to create LUT for
		inline bool InitializeLUT(ChannelMap channelMap);

		[[nodiscard]] JPL_INLINE bool IsLUTInitialized() const noexcept { return mLUT != nullptr; }

		/// Get the look-up table. Can be nullptr, if hasn't been initialized yet.
		[[nodiscard]] JPL_INLINE const LUTType* GetLUT() const noexcept { return mLUT.get(); }
		
		//======================================================================
		/// Create/initialize SourceLayout for given source `channelMap`
		JPL_INLINE bool InitializeSourceLayout(ChannelMap channelMap, SourceLayoutType& outLayout) const;

		/// Get speaker gains from LUT based on 'direction' vector.
		/// @param outGains : must be of size equals to number of target channels of this panner
		JPL_INLINE void GetSpeakerGains(const Vec3Type& direction, std::span<float> outGains) const;


		/// @returns true if LUT has been initialized and the panner can be used
		[[nodiscard]] JPL_INLINE bool IsInitialized() const noexcept { return mChannelMap.IsValid() && IsLUTInitialized(); }

		/// Get number of channels the LUT is initialized for.
		/// Effectively this is the channel count of the channel map the panner
		/// is initialized for, including LFE (though LFE gain is always 0.0).
		[[nodiscard]] JPL_INLINE uint32 GetNumChannels() const noexcept { return mNumChannels; } // TODO: maybe move common members to base class as well?

		/// Get the channel map the panner is initialized to.
		[[nodiscard]] JPL_INLINE ChannelMap GetChannelMap() const noexcept { return mChannelMap; }

		//======================================================================
		/// This function should be called whenever pan data changes to update 
		/// set gains based on PanUpdateData for each source channel retrieved by `getOutGains` callback.
		/// SourceLayout must be initialized beforehand.
		/// E.g. this would be called for each source that has unique channel layout or panning state
		/// @param getOutGains :    A function to retrieve array of gains to fill for the provided
		///                         source channel index, the gains written are accumulated and
		///                         normalized gains of virtual sources associated to this channel group.
#if JPL_TEST
		template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback = std::identity>
		JPL_INLINE void ProcessVBAPData(const SourceLayoutType& sourceLayout,
										const PanUpdateData& updateData,
										ChannelGroupGainsGetter&& getOutGains,
										OnChannelGeneratedCallback&& callback = {}) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#else
		template<class ChannelGroupGainsGetter>
		JPL_INLINE void ProcessVBAPData(const SourceLayoutType& data,
										const PanUpdateData& updateData,
										ChannelGroupGainsGetter&& getOutGains) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#endif

#if JPL_TEST
		template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback = std::identity>
		JPL_INLINE void ProcessVBAPData(const SourceLayoutType& sourceLayout,
										const PanUpdateDataWithOrientation& updateData,
										ChannelGroupGainsGetter&& getOutGains,
										OnChannelGeneratedCallback&& callback = {}) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#else
		template<class ChannelGroupGainsGetter>
		JPL_INLINE void ProcessVBAPData(const SourceLayoutType& sourceLayout,
										const PanUpdateDataWithOrientation& updateData,
										ChannelGroupGainsGetter&& getOutGains) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#endif

		//======================================================================
		/// Get and accumulate speaker gains for all virtual soruces into @outGains
		/// Weights of the virtual sources must be L1 normalized
		JPL_INLINE void ProcessVirtualSources(std::span<const VirtualSource> virtualSources,
											  std::span<float> outGains) const;

		/// Normalize weights of the virtual sources to ensure consistent energy
		static JPL_INLINE void NormalizeWeights(std::span<VirtualSource> virtualSources);

	private:
#if JPL_TEST
		template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback>
		inline void ProcessVBAPDataImpl(const SourceLayoutType& sourceLayout,
										const PanUpdateData& updateData,
										const Quat<Vec3Type>& panRotation,
										ChannelGroupGainsGetter&& getOutGains,
										OnChannelGeneratedCallback&& callback) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#else
		template<class ChannelGroupGainsGetter>
		inline void ProcessVBAPDataImpl(const SourceLayoutType& sourceLayout,
										const PanUpdateData& updateData,
										const Quat<Vec3Type>& panRotation,
										ChannelGroupGainsGetter&& getOutGains) const
			requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>;
#endif

		/// Get and accumulate speaker gains for all virtual soruces into @outGains
		template<bool bAccumulatePow>
		inline void ProcessVirtualSourcesImpl(std::span<const VirtualSource> virtualSources,
											  std::span<float> outGains) const;

		static JPL_INLINE void RotateChannelCap(std::span<Vec3Type> virtualSources, const Basis<Vec3Type>& rotation);
		static JPL_INLINE void SlerpChannelCap(std::span<Vec3Type> virtualSources, const Vec3Type& targetDirection, float t);

	private:
		pmr_unique_ptr<LUTType> mLUT;

		ChannelMap mChannelMap;     // Target channel map
		uint32 mNumChannels = 0;  // Number of channels in target channel map

		// Shortest aperture between two speakers (dot product for 3D, angle for 2D panner)
		// Used to determine number of virtual sources required to leave no gaps in the target channel layout
		float mShortestEdgeAperture = std::numeric_limits<float>::max();
	};


	//==============================================================================
	//
	//   Code beyond this point is implementation detail...
	//
	//==============================================================================

	// This is similar to Wwise, where each vs is slerped towards
	// forward (target pan). Which is more expansive, but produces
	// different distribution, which may or may not sound more natural.
#define JPL_SLERP_SPREAD 1

	template<class PannerType, class Traits>
	JPL_INLINE bool VBAPannerBase<PannerType, Traits>::InitializeSourceLayout(ChannelMap channelMap, SourceLayoutType& outLayout) const
	{
		return outLayout.Initialize(channelMap, mChannelMap, mShortestEdgeAperture);
	}

	template<class PannerType, class Traits>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::GetSpeakerGains(const Vec3Type& direction, std::span<float> outGains) const
	{
		JPL_ASSERT(mLUT != nullptr);
		LUTInterface::Query(*mLUT).GainsFor(direction, outGains);
	}

	template<class PannerType, class Traits>
	inline bool VBAPannerBase<PannerType, Traits>::InitializeLUT(ChannelMap channelMap)
	{
		if (auto error = PanType::IsValidTargetChannelMap(channelMap))
		{
			JPL_ERROR_TAG("VBAPannerBase", error.value());
			return false;
		}

		// Create LUT
		mLUT = JPL::make_pmr_unique<LUTType>();

		// Construct LUT builder
		auto lutBuilder = LUTInterface::MakeBuilder(channelMap, *mLUT);

		// Find the shortest edge to later use for SourceLayout VSs initialization
		mShortestEdgeAperture = lutBuilder.FindShortestAperture();
		mChannelMap = channelMap; // target channel map
		// TODO: we may or may not want include LFE here        
		mNumChannels = channelMap.GetNumChannels();// -channelMap.HasLFE();

		const bool bLUTBuildSuccessful = lutBuilder.BuildForAllDirections();

#if JPL_VALIDATE_VBAP_LUT
		lutBuilder.ValidateLUT();
#endif

		return bLUTBuildSuccessful;
	}

#if JPL_TEST
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::ProcessVBAPData(const SourceLayoutType& sourceLayout,
																	   const PanUpdateData& updateData,
																	   ChannelGroupGainsGetter&& getOutGains,
																	   OnChannelGeneratedCallback&& callback) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
	{
		JPL_ASSERT(sourceLayout.GetTargetChannelMap() == mChannelMap, "VBAP Panner can only work with VBAP source layout created for that panner.");

		static const Vec3Type cWorldUp(0, 1, 0); // TODO: Listener UP

		// For panning based only on emitter Direction
		// - Direction defines sound-field/ring forward vector
		const auto qPan = Math::QuatLookAt(updateData.SourceDirection, cWorldUp);

		ProcessVBAPDataImpl(sourceLayout, updateData, qPan, getOutGains, callback);
	}
#else
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::ProcessVBAPData(const SourceLayoutType& sourceLayout,
																	   const PanUpdateData& updateData,
																	   ChannelGroupGainsGetter&& getOutGains) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
	{
		JPL_ASSERT(sourceLayout.GetTargetChannelMap() == mChannelMap, "VBAP Panner can only work with VBAP source layout created for that panner.");

		static const Vec3Type cWorldUp(0, 1, 0); // TODO: Listener UP

		// For panning based only on emitter Direction
		// - Direction defines sound-field/ring forward vector
		const auto qPan = Math::QuatLookAt(updateData.SourceDirection, cWorldUp);

		ProcessVBAPDataImpl(sourceLayout, updateData, qPan, getOutGains);
	}
#endif

#if JPL_TEST
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::ProcessVBAPData(const SourceLayoutType& sourceLayout,
																	   const PanUpdateDataWithOrientation& updateData,
																	   ChannelGroupGainsGetter&& getOutGains,
																	   OnChannelGeneratedCallback&& callback) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
	{
		JPL_ASSERT(sourceLayout.GetTargetChannelMap() == mChannelMap, "VBAP Panner can only work with VBAP source layout created for that panner.");
		// For panning based on Direction + Orientation
		// - Orientation (emitter relative to listeners) defines sound-field/ring forward vector
		// - Direction defines the target of spread-based contraction
		const auto qPan = Math::QuatFromUpAndForward(updateData.Orientation.Up, updateData.Orientation.Forward);

		ProcessVBAPDataImpl(sourceLayout, updateData.Pan, qPan, getOutGains, callback);
	}
#else
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::ProcessVBAPData(const SourceLayoutType& sourceLayout,
																	   const PanUpdateDataWithOrientation& updateData,
																	   ChannelGroupGainsGetter&& getOutGains) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
	{
		JPL_ASSERT(sourceLayout.GetTargetChannelMap() == mChannelMap, "VBAP Panner can only work with VBAP source layout created for that panner.");

		// For panning based on Direction + Orientation
		// - Orientation (emitter relative to listeners) defines sound-field/ring forward vector
		// - Direction defines the target of spread-based contraction
		const auto qPan = Math::QuatFromUpAndForward(updateData.Orientation.Up, updateData.Orientation.Forward);

		ProcessVBAPDataImpl(sourceLayout, updateData.Pan, qPan, getOutGains);
	}
#endif

#if JPL_TEST
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter, class OnChannelGeneratedCallback>
	inline void VBAPannerBase<PannerType, Traits>::ProcessVBAPDataImpl(const SourceLayoutType& sourceLayout,
																	   const PanUpdateData& updateData,
																	   const Quat<Vec3Type>& panRotation,
																	   ChannelGroupGainsGetter&& getOutGains,
																	   OnChannelGeneratedCallback&& callback) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
#else
	template<class PannerType, class Traits>
	template<class ChannelGroupGainsGetter>
	inline void VBAPannerBase<PannerType, Traits>::ProcessVBAPDataImpl(const SourceLayoutType& sourceLayout,
																	   const PanUpdateData& updateData,
																	   const Quat<Vec3Type>& panRotation,
																	   ChannelGroupGainsGetter&& getOutGains) const
		requires CGetSpeakerGainsFunction<ChannelGroupGainsGetter>
#endif
	{
		// The gains are normalized for the number of input channels to ensure stable power
		/*
			- For each output channel, the square of the gains of all contributing input channels are summed together.
			- They are then divided by the number of input channels.
			- And then the square root of the quotient is taken.
		*/
		/*
			// Effectively normalization is doing this:

			std::vector<float> outputChannelGains;

			const float invNumInChannels = 1.0f / inputChannels.size();

			// 1. Accumulate output channel gains
			for (const auto& channelGains : inputChannels)
			{
				for (uint32 outChannelI = 0; outChannelI < channelGains.size(); ++outChannelI)
					outputChannelGains[outChannelI] += channelGains[outChannelI] * channelGains[outChannelI];
			}

			// 2. Normalize resulting output gains
			for (float& g : outputChannelGains)
				g = Math::Sqrt(g * invNumInChannels);

			// ..but instead of mixing into the output channels here,
			// we bake in normalization into each input channel gains group,
			// which allows the inpout channels to be mixed simply by
			// adding into the corresponding output channels.
		*/

		namespace stdr = std::ranges;
		namespace stdv = std::views;

		// Clear all channel group gains
		stdr::for_each(sourceLayout.ChannelGroups, [&getOutGains](const ChannelGroup& channelGroup)
		{
			ChannelGainsRef channelGains = getOutGains(channelGroup.Channel);
			stdr::fill(channelGains, 0.0f);
		});

		// General idea is to generate new channel cap relative to nominal
		// pan direction then rotate it by channel offset + target pan

		JPL_ASSERT(updateData.Focus >= 0.0f && updateData.Focus <= 1.0f);
		JPL_ASSERT(updateData.Spread >= 0.0f && updateData.Spread <= 1.0f);

		// Buffer to mix in source channels and calculate the normalization coefficients
		float mixBufferData[Traits::MAX_CHANNELS]{ 0.0f };
		auto mixBuffer = mixBufferData | stdv::take(mNumChannels);

		// Large enough static buffer
		Vec3Type vsVecBuffer[(SourceLayoutType::GetMaxNumVirtualSources()) * 2];

		// Designate buffer space
		const size_t vsBufferSize = sourceLayout.GetNumVirtualSources();
		auto vsDirCanonBuffer = std::span(vsVecBuffer, vsBufferSize);
		auto vsDirScratchBuffer = std::span(&vsVecBuffer[vsBufferSize], vsBufferSize);

		// Initialize scratch buffer for virtual sources with correct weight,
		// wich is constant for panning SourceLayoutType
		std::array<VirtualSource, SourceLayoutType::GetMaxNumVirtualSources()> vsBuffer;
		for (VirtualSource& vs : vsBuffer)
			vs.Weight = sourceLayout.GetDefaultVSWeight();

		auto channelGroupVSs = std::span(vsBuffer.begin(), vsBufferSize);

		// Generate canon channel cap of virtual sources for the requrested focus
		sourceLayout.GenerateSpreadCap(vsDirCanonBuffer, updateData.Focus);

		const auto& qPan = panRotation;

		for (const auto& channelGroup : sourceLayout.ChannelGroups)
		{
#if JPL_SLERP_SPREAD

			// 1. Copy canon cap to scratch buffer for channel-specific modifications
			std::ranges::copy(vsDirCanonBuffer, vsDirScratchBuffer.begin());

			const Quat<Vec3Type>& qYaw = channelGroup.Rotation;
			const Basis totalRotation = (qPan * qYaw).ToBasis(); // yaw-after-pan (local up axis)
			//const Basis totalRotation = (qYaw * qPan).ToBasis(); // yaw-after-pan (local up axis)

			// 2. Rotate channel cap towards pan rotation + channel offset
			RotateChannelCap(vsDirScratchBuffer, totalRotation);

			// 3. Move virtual sources towards pan direction based on spread
			// (this is most expansive operation, requiring slerping each vs individually)
			SlerpChannelCap(vsDirScratchBuffer, updateData.SourceDirection, 1.0f - updateData.Spread);

#else
			const Quat<Vec3Type> qYaw = Math::Slerp(channelGroup.Rotation, Quat<Vec3Type>::Identity(), (1.0f - spread));
			const Basis<Vec3Type> totalRotation = (qPan * qYaw).ToBasis();

			const float capSpread = GetNominalChannelCapSpread() * updateData.Spread * (1.0f - updateData.Focus);
			GenerateSpreadCap(channelVSs, capSpread, numRings, numSamples);

			RotateChannelCap(channelVSs, totalRotation);
#endif

#if JPL_TEST
			// mainly for debugging and testing
			if constexpr (!std::same_as<std::remove_cvref_t<OnChannelGeneratedCallback>, std::identity>)
			{
				(void)callback(vsDirScratchBuffer, channelGroup.Channel);
			}
#endif
			// Assign new directions for our weighted virtual sources
			for (uint32 i = 0; i < vsDirScratchBuffer.size(); ++i)
				channelGroupVSs[i].Direction = vsDirScratchBuffer[i];

			ChannelGainsRef outGainsRef = getOutGains(channelGroup.Channel);
			JPL_ASSERT(outGainsRef.size() >= mNumChannels);

			auto gains = outGainsRef | stdv::take(mNumChannels);

			// Clear channel group gains
			stdr::fill(gains, 0.0f);

			// Compute new gains
			//ProcessVirtualSources(channelGroupVSs, gains);
			static constexpr bool bAccumulatePow = true;
			ProcessVirtualSourcesImpl<bAccumulatePow>(channelGroupVSs, gains);


			// Accumulate output channel gain squares
			stdr::transform(gains, mixBuffer, mixBuffer.begin(), std::plus<float>{});
		}

		// Normalize for the number of input channels
		{
			float normCoeffsData[Traits::MAX_CHANNELS]{ 0.0f };
			auto normCoeffs = normCoeffsData | stdv::take(mNumChannels);

			const auto numInChannels = static_cast<float>(sourceLayout.ChannelGroups.size());

			// Calculate normalization coefficients per output channel
			for (uint32 i = 0; i < mixBuffer.size(); ++i)
			{
				const float g = mixBuffer[i];
				normCoeffs[i] = g > 0.0f ? Math::InvSqrt(g * numInChannels) : 0.0f;
			}

			// Pre-normalize source out channel gains
			for (const auto& channelGroup : sourceLayout.ChannelGroups)
			{
				auto gains = getOutGains(channelGroup.Channel) | stdv::take(mNumChannels);

				for (uint32 i = 0; i < gains.size(); ++i)
					gains[i] *= normCoeffs[i];
			}

			// ..now the input channels can be mixed into the output channels with simple add

			// This is equivalent to the method above, only multiplying once per output channel during mixing.
			// 
			// We could instead of backing in normalization into source channel gains groups,
			// give the user the output gains coefficients to multiply when mixing.
			/*for (uint32 i = 0; i < mixBuffer.size(); ++i)
			{
				float& g = mixBuffer[i];
				float& c = normCoeffs[i];
				g *= c;
			}*/
		}
	}

	template<class PannerType, class Traits>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::ProcessVirtualSources(std::span<const VirtualSource> virtualSources,
																			 std::span<float> outGains) const
	{
		JPL_ASSERT(outGains.size() <= GetNumChannels());

		static constexpr bool bAccumulatePow = false;
		ProcessVirtualSourcesImpl<bAccumulatePow>(virtualSources, outGains);

		// Public ProcessVirtualSources is for the end use in mixing,
		// so has to be normalized.
		Algo::NormalizeL2(outGains);
	}

	template<class PannerType, class Traits>
	template<bool bAccumulatePow>
	inline void VBAPannerBase<PannerType, Traits>::ProcessVirtualSourcesImpl(std::span<const VirtualSource> virtualSources,
																			 std::span<float> outGains) const
	{
		JPL_ASSERT(outGains.size() <= GetNumChannels());

		using QueryType = typename LUTInterface::QueryType;

		if constexpr (requires{ typename QueryType::VBAPCell; })
		{
			// Note: this cannot be parallel for, because the writes to `outGains` can overlap
			std::ranges::for_each(virtualSources, [&](const VirtualSource& vs)
			{
				// Retrieve gains from the LUT
				typename QueryType::VBAPCell cell;

				LUTInterface::Query(*mLUT).GainsFor(vs.Direction, cell);

				if constexpr (bAccumulatePow)
				{
					// Accumulate linear gains
					outGains[cell.Speakers[0]] += vs.Weight * cell.Gains[0] * cell.Gains[0];
					outGains[cell.Speakers[1]] += vs.Weight * cell.Gains[1] * cell.Gains[1];
					outGains[cell.Speakers[2]] += vs.Weight * cell.Gains[2] * cell.Gains[2];
				}
				else
				{
					// Accumulate weighted linear gains (to be normalized later)
					outGains[cell.Speakers[0]] += vs.Weight * cell.Gains[0];
					outGains[cell.Speakers[1]] += vs.Weight * cell.Gains[1];
					outGains[cell.Speakers[2]] += vs.Weight * cell.Gains[2];
				}
			});
		}
		else
		{
			float buffer[Traits::MAX_CHANNELS]{ float(0.0) };
			std::span<float> vsSpeakerGains(buffer, outGains.size());

			// Note: this cannot be parallel for, because the writes to `outGains` can overlap
			std::ranges::for_each(virtualSources, [&](const VirtualSource& vs)
			{
				// Retrieve gains from the LUT
				// TODO: this -Z is weird
				GetSpeakerGains(Vec3Type(GetX(vs.Direction), GetY(vs.Direction), -GetZ(vs.Direction)), vsSpeakerGains);

				// Accumulate weighted linear gains (to be normalized later)
				for (uint32 s = 0; s < outGains.size(); ++s)
				{
					if constexpr (bAccumulatePow)
					{
						outGains[s] += vs.Weight * vsSpeakerGains[s] * vsSpeakerGains[s];
					}
					else
					{
						outGains[s] += vs.Weight * vsSpeakerGains[s];
					}
				}
			});
		}
	}

	template<class PannerType, class Traits>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::NormalizeWeights(std::span<VirtualSource> virtualSources)
	{
		const float sum = Accumulate(virtualSources, 0.0f, [](float acc, const VirtualSource& vs)
		{
			return acc += Math::Abs(vs.Weight);
		});

		const float invSum = sum == 0.0f ? 0.0f : 1.0f / sum;

		std::ranges::for_each(virtualSources, [invSum](VirtualSource& vs)
		{
			vs.Weight *= invSum;
		});
	}

	template<class PannerType, class Traits>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::RotateChannelCap(std::span<Vec3Type> virtualSources, const Basis<Vec3Type>& rotation)
	{
		for (Vec3Type& vs : virtualSources)
			vs = rotation.Transform(vs);
	}

	template<class PannerType, class Traits>
	JPL_INLINE void VBAPannerBase<PannerType, Traits>::SlerpChannelCap(std::span<Vec3Type> virtualSources, const Vec3Type& targetDirection, float t)
	{
		for (Vec3Type& vs : virtualSources)
			vs = Math::Slerp(vs, targetDirection, t);
	}

	template<class PannerType, class Traits>
	inline bool VBAPannerBase<PannerType, Traits>::VBAPLayoutBase::InitializeBase(ChannelMap channelMap, ChannelMap targetMap, uint32 numVirtualSourcesPerChannel)
	{
		mTargetChannelMap = targetMap;

		// Cache normalization factors for later
		mDefaultVSWeight = 1.0f / numVirtualSourcesPerChannel;

		const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

		// Source channel map is unsorted, we need to sort it
		// to make our lives easier
		Array<VBAP::ChannelAngle> sourceChannelsSorted(GetDefaultMemoryResource());
		VBAP::ChannelAngle::GetSortedChannelAngles(channelMap, sourceChannelsSorted, &Traits::GetChannelAngle);

		// Width of a singe source channel in radians
		const float channelWidth = JPL_TWO_PI / static_cast<float>(numChannels);

		// If we don't have center channel, we need to offset channel groups
		const float channelAngleOffset = !channelMap.Has(EChannel::FrontCenter)
			? 0.5f * channelWidth
			: 0.0f;

		// Create channel groups for source channels
		ChannelGroups.clear();
		ChannelGroups.resize(numChannels);

		static const Vec3Type cUP(0.0f, 1.0f, 0.0f);

		// 1. Find equal source channel positions for 100% spread
		// 2. Lay out virtual sources for source channel group evenly withing its equal section
		for (uint32 i = 0; i < sourceChannelsSorted.size(); ++i)
		{
			// Assign a centre angle of the next equal section of the source plane
			const float channelAngle = channelAngleOffset + channelWidth * i;

			// Create channel group for each source channel
			ChannelGroup& channelGroup = ChannelGroups[i];
			channelGroup.Rotation = Math::QuatRotation(cUP, -channelAngle); // right-handed rotation
			channelGroup.Channel = sourceChannelsSorted[i].ChannelId;

			/*
				To handle LFE:
				- channelGroup.Channel - is the index of the source channel
				- in the audio process block, instead of iterating actual inputs of thes block, iterate VBAP channel groups
				- if LFE pressent, simply copy input to output, applying other parameters, like distance attenuation, but skipping VBAP channel group

				There's no need to access channel group by ID when sending audio to the output, since we coppy each group to each output channel
			*/
		}

		return true;
	}

} // namespace JPL