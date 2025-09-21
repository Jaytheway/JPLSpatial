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

#include "JPLSpatial/Math/MinimalVec2.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Math/MinimalMat.h"

#include "JPLSpatial/Geometry/ConvexHullBuilder.h"
#include "JPLSpatial/Panning/DummySpeakers.h"

#include <vector>
#include <array>
#include <span>
#include <functional>
#include <compare>
#include <algorithm>
#include <memory>

namespace JPL
{
	/// Compute 3-dimentional VBAP for a point within triangle of speaker directions.
	/// @param sourceDirection : normalized direction from listener towards the source
	/// @returns : gain factors for the three speakers, not normalized
	template<CVec3Accessible Vec3Type>
	[[nodiscard]] inline Vec3Type ComputeVBAP(const Vec3Type& sourceDirection, const Vec3Type& triPointA, const Vec3Type& triPointB, const Vec3Type& triPointC)
	{
		const auto L = Math::Mat3<Vec3Type>::FromColumns(triPointA, triPointB, triPointC);
		Math::Mat3<Vec3Type> Linv;
		if (!JPL_ENSURE(L.TryInverse(Linv)))
		{
			// Loudspeakers are coplanar, cannot pan in 3D.

			// (this is the case for any speaker setup where spreakers are at Y=0 plane
			// and the source is also at Y=0, and instead of the vertical triangle,
			// the bottom plane triangle was selected)
			// 
			// In such case one can fall back to 2D or 1D panning
			return Vec3Type{ 0.0f, 0.0f, 0.0f };
		}

		return Linv.Transform(sourceDirection);
	}

	/// Compute 2-dimentional VBAP for a source direction within speaker arch defined by two speaker directions.
	/// Note: all directions must be normalized.
	/// @param sourceDirection : direction from listener towards the source
	/// @param speakerADirection : directoin of the first speaker
	/// @param speakerBDirection : direction of the second speaker
	/// @returns gain factors for the two speakers, or {0,0} if the speakers are collinear
	[[nodiscard]] inline Vec2 ComputeVBAP(const Vec2& sourceDirection, const Vec2& speakerADirection, const Vec2& speakerBDirection)
	{
		const auto L = Math::Mat2<Vec2>::FromColumns(speakerADirection, speakerBDirection);
		Math::Mat2<Vec2> Linv;
		if (!JPL_ENSURE(L.TryInverse(Linv)))
		{
			// 1D panning should handle this case.
			// Loudspeaker vectors are collinear.
			return Vec2{ 0.0f, 0.0f };
		}

		return Linv.Transform(sourceDirection);
	}

	namespace SpeakerTriangulation
	{
		using Vec3i = std::array<uint8, 3>;

		template<auto GetSpeakerVectorFunction, class Vec3ContainerType>
		inline bool GetSpeakerVectors(ChannelMap channelMap, Vec3ContainerType& outVectors)
		{
			if (!channelMap.IsValid())
				return false;

			const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

			outVectors.clear();
			outVectors.reserve(numChannels);

			// TODO: do we want to use indices of the channel map, or of the vector we store valid directions in?
			channelMap.ForEachChannel([&outVectors](EChannel channel)
			{
				if (channel != EChannel::LFE)
					outVectors.push_back(GetSpeakerVectorFunction(channel));
			});

			return true;
		}

		template<auto GetSpeakerVectorFunction, class Vec3ContainerType, class IndexContainerType>
		inline bool GetSpeakerVectors(ChannelMap channelMap, Vec3ContainerType& outVectors, IndexContainerType& outMapIndices)
		{
			if (!channelMap.IsValid())
				return false;

			const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

			outVectors.clear();
			outMapIndices.clear();
			outVectors.reserve(numChannels);
			outMapIndices.reserve(numChannels);

			channelMap.ForEachChannel([&outVectors](EChannel channel, uint32 index)
			{
				if (channel != EChannel::LFE)
				{
					outVectors.push_back(GetSpeakerVectorFunction(channel));
					outMapIndices.push_back(index);
				}
			});

			return true;
		}

		template< template<class> class AllocatorType, CVec3Accessible Vec3Type, class Vec3iContainerType>
		inline bool TriangulateSpeakerLayout(std::span<const Vec3Type> speakerVectors, Vec3iContainerType& outIndices)
		{
			const auto& vertices = speakerVectors;

			// For 3 speakers we only have one triangle, no need to build a hull.
			if (vertices.size() == 3)
			{
				outIndices.push_back({ 0, 1, 2 });
				return true;
			}

			/* TODO: we might want to rethink triangulation and instead of forcing creating closed convex hull
					create only viable triangulated sections.
					As per Pulkki:
						If the specified virtual source direction is outside of the panning
						directions possible with the current loudspeaker setup,
						vbap object finds the nearmost triangle and it applies the sound to it.
			*/

			using HullBuilderType = JPL::ConvexHullBuilder<Vec3Type, AllocatorType>;

			HullBuilderType builder(vertices);

			const char* errorMessage = nullptr;

			if (builder.Initialize(INT_MAX, 0.0f, errorMessage) != HullBuilderType::EResult::Success)
				return false;

			if (!JPL_ENSURE(builder.GetNumVerticesUsed() == vertices.size()))
				return false;

			builder.GetTriangles(outIndices);

			return true;
		}

		template<auto GetSpeakerVectorFunction, CVec3Accessible Vec3Type, class Vec3ContainerType, class Vec3iContainerType, template<class> class AllocatorType = std::allocator>
		inline bool TriangulateSpeakerLayout(ChannelMap channelMap, Vec3ContainerType& outVertices, Vec3iContainerType& outIndices)
		{
			if (!channelMap.IsValid())
				return false;

			// 2D speaker arrangement
			if (!channelMap.HasTopChannels())
				return false;

			const uint32 numChannels = channelMap.GetNumChannels() - channelMap.HasLFE();

			// We need at least 3 indices to form a triangle
			if (numChannels < 3)
				return false;

			std::vector<Vec3Type, AllocatorType<Vec3Type>> vertices;

			// TODO: do we want to use indices of the channel map?
			GetSpeakerVectors<GetSpeakerVectorFunction>(channelMap, vertices);

			VBAP::DummySpeakers<GetSpeakerVectorFunction, AllocatorType> dummySpeakers(channelMap, vertices);

			// Since at the moment ChannelMap doesn't support speakers on the bottom,
			// we at least need to add a dummy speaker there for a better topology
			// of the convex hull.
			dummySpeakers.AddDummy(Vec3Type(0.0f, -1.0f, 0.0f));

			const int numTopChannels = [channelMap]()
			{
				int numTopChannels = 0;
				channelMap.ForEachChannel([&numTopChannels](EChannel channel)
				{
					numTopChannels += channel >= EChannel::TOP_Channels;
				});
				return numTopChannels;
			}();

			// For more info about this see comments in LUTBuilder3D constructor
			if (numTopChannels == 6 || numTopChannels == 4)
			{
				dummySpeakers.AddIfChannelNotPresent(EChannel::TopCenter);
			}
			else
			{
				JPL_ASSERT(numTopChannels == 2);
			}

			if (TriangulateSpeakerLayout<AllocatorType>(std::span<const Vec3Type>(vertices), outIndices))
			{
				outVertices = std::move(vertices);
				return true;
			}
			else
			{
				return false;
			}
		}
	} // namespace SpeakerTriangulation

	namespace VBAP
	{
		//======================================================================
		struct ChannelAngle
		{
			float Angle;
			uint32 ChannelId; // Channel index or identifier

			// Operators necessary for sorting
			[[nodiscard]] JPL_INLINE constexpr std::strong_ordering operator<=>(const ChannelAngle& other) const noexcept
			{
				const float a1 = Angle < 0.0f ? Angle + JPL_TWO_PI : Angle;
				const float a2 = other.Angle < 0.0f ? other.Angle + JPL_TWO_PI : other.Angle;
				if (a1 < a2) return std::strong_ordering::less;
				if (a1 > a2) return std::strong_ordering::greater;
				return std::strong_ordering::equal;
			}
			[[nodiscard]] JPL_INLINE constexpr bool operator==(const ChannelAngle& other) const noexcept { return Math::Abs(Angle - other.Angle) < 1e-6f; }

			/// Get channel angles from ChannelMap, normalize to [0, Pi] and sort in assending order
			template<template<typename...> class ArrayType, class ...Args>
			static inline void GetSortedChannelAngles(
				ChannelMap channelMap,
				ArrayType<ChannelAngle, Args...>& sortedChannelAngles,
				std::function<float(EChannel)> getChannelAngle,
				bool skipLFO = true)
			{
				sortedChannelAngles.clear();
				sortedChannelAngles.reserve(channelMap.GetNumChannels() - skipLFO * channelMap.HasLFE());

				channelMap.ForEachChannel([&sortedChannelAngles, &getChannelAngle, skipLFO](EChannel channel, uint32 channelIndex)
				{
					// We don't use LFE for panning
					if (skipLFO && channel == EChannel::LFE)
						return;

					// Top channels of the source don't participate in VBAP
					if (channel >= EChannel::TOP_Channels)
						return;

					const float channelAngle = getChannelAngle(channel);

					sortedChannelAngles.emplace_back(channelAngle < 0.0f ? channelAngle + JPL_TWO_PI : channelAngle, channelIndex);
				});

				std::ranges::sort(sortedChannelAngles);
			}
		};
	} // namespace VBAP
} // namespace JPL