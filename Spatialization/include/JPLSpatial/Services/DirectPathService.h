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
#include "JPLSpatial/DistanceAttenuation.h"
#include "JPLSpatial/Containers/FlatMap.h"
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalBasis.h"
#include "JPLSpatial/Math/Position.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Utilities/IDType.h"

#include <algorithm>
#include <cmath>
#include <memory>
#include <iterator>
#include <vector>
#include <memory_resource>

namespace JPL
{
	//==========================================================================
	template<CVec3 Vec3Type>
	struct DirectPathResult
	{
		float Distance;					//< Distance from source to listener
		float DirectionDot;				//< Dot product between source direction relative to listener and listener's forward vector
		float InvDirectionDot;			//< Dot product listener direction relative to source and source's forward vector

		Position<Vec3Type> Position;	//< Direction and orientation relative to listener
	};

	struct AttenuationCone
	{
		float InnerAngle;	//< width of the inner sector in radians
		float OuterAngle;	//< width of the outer sector in radians, should be > InnerAngle
	};

	using AttenuationCurveRef = std::shared_ptr<AttenuationFunction>;

	// TODO: we might want Curve ID to be able to retrieve volume and other attenuations quickly
	struct CurveAttenuationCache
	{
		AttenuationCurveRef Curve;
		float AttenuationValue;
	};

	struct ConeAttenuationCache
	{
		AttenuationCone Cone;
		float AttenuationValue;
	};

	struct DirectPathServiceIDTag {};
	using DirectEffectHandle = IDType<DirectPathServiceIDTag>;

	struct DirectEffectInitParameters
	{
		AttenuationCurveRef BaseCurve;
		AttenuationCone AttenuationCone;
	};

	//==========================================================================
	class DirectPathService
	{
	public:
		// Alias to override allocator for the internal FlatMap we use
		template<class Key, class T>
		using FlatMapType = FlatMapWithAllocator<Key, T, std::pmr::polymorphic_allocator>;

	public:
		DirectPathService() = default;

		DirectPathService(const DirectPathService&) = delete;
		DirectPathService& operator=(const DirectPathService&) = delete;

		// High level API to:
		// - get distance-based volume attenuation and air absorption values

		JPL_INLINE DirectEffectHandle InitializeDirrectEffect(const DirectEffectInitParameters& initParameters);

		/// Release any data associated with the source handle.
		/// @returns		true if anything was cleared, false if no data was associated with the handle
		JPL_INLINE bool ReleaseEffectData(DirectEffectHandle source);

		/// Assign attenuation curve to source.
		/// The curve should not be owned by the user.
		/// `attenuationFunction` must be constructed with make_pmr_shared
		JPL_INLINE AttenuationCurveRef AssignAttenuationCurve(DirectEffectHandle source, AttenuationCurveRef attenuationFunction);

		/// Compute direct path parameters based on position of the soruce and listener.
		/// This can be used to compute parameters for position of a source
		/// relative to listener, as well as position of the listener relative
		/// to a source.
		/// 
		/// @param position			position to compute parameters for, in world space
		/// @param referencePoint		reference point to compute parameters from, in world space
		/// 
		/// @returns					paramteres of the position relative to the reference point
		template<CVec3 Vec3Type>
		static JPL_INLINE DirectPathResult<Vec3Type> ProcessDirectPath(const Position<Vec3Type>& source,
															 const Position<Vec3Type>& listener);
		template<CVec3 Vec3Type>
		static JPL_INLINE float ProcessAngleAttenuation(const Vec3Type& position,
														const Position<Vec3Type>& referencePoint,
														AttenuationCone cone);

		static JPL_INLINE float ProcessAngleAttenuation(float azimuth, AttenuationCone cone);

		static JPL_INLINE float EvaluateDistance(float distance, const AttenuationCurveRef& attenuationCurve);

		// TODO: mabye we could batch process multiple distances/sources per curve, if they are sharing curves?
		/*
			Essentially it is a tradeoff between:
				- efficient evaluation - which is better if we have a lot of curves per source
				- efficient cache look-up - which is better if we access values a lot and share curves between sources
		*/

		/// Evaluate distance with all the curves associated with the source.
		/// The values are cached internally and can be retrieved by calling
		/// GetDistanceAttenuation function.
		/// 
		/// @param source		source to evaluate the curves for
		/// @param distance	distance to evaluate
		/// 
		/// @returns			'true' if the source is valid, 'false' otherwise
		JPL_INLINE bool EvaluateDistance(DirectEffectHandle source, float distance);

		/// Evaluate direction angle using cone attenuation parameters assigned to the source.
		/// The value is cached internally and can be retrieved by calling
		/// GetDirectionAttenuation function.
		/// 
		/// @param source			source to evaluate the curves for
		/// @param directionDot	direction dot product relative to forward axis to evaluate
		/// 
		/// @returns				'true' if the source is valid, 'false' otherwise
		JPL_INLINE bool EvaluateDirection(DirectEffectHandle source, float directionDot);


		/// Get distance attenuation value for source evaluated from the curve.
		/// If either the soruce or the curve is not found in the cache,
		/// 1.0f is returned
		/// 
		/// @param source		source to get the value for
		/// @param curve		curve created by calling CreateAttenuationCurve function
		/// 
		/// @returns			last value evaluated from the curve for the source, or 1.0f
		///					if value is has not been evaluated yet
		JPL_INLINE float GetDistanceAttenuation(DirectEffectHandle source, const AttenuationCurveRef& curve) const;
		
		/// Get cone attenuation value for source relative to listener view.
		/// If the soruce is not found in the cache, 1.0f is returned
		/// 
		/// @param source		source to get the value for
		/// 
		/// @returns			last value evaluated for the source, or 1.0f
		///					if value is has not been evaluated yet
		JPL_INLINE float GetDirectionAttenuation(DirectEffectHandle source) const;

	private:
		static JPL_INLINE float ProcessAngleAttenuationImpl(float azimutCos, const AttenuationCone& cone);

	private:
		using CurveAttenuationCacheArray = std::pmr::vector<CurveAttenuationCache>;

		// TODO: can we store cache for more efficient access?
		FlatMapType<DirectEffectHandle, CurveAttenuationCacheArray> mAttenuationCache{ GetDefaultMemoryResource() };
		FlatMapType<DirectEffectHandle, ConeAttenuationCache> mDirectionAttenuationCache{ GetDefaultMemoryResource() };
	};
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL
{
	JPL_INLINE DirectEffectHandle DirectPathService::InitializeDirrectEffect(const DirectEffectInitParameters& initParameters)
	{
		const auto handle = DirectEffectHandle::New();
		mAttenuationCache.emplace(handle,
								  initParameters.BaseCurve
								  ? CurveAttenuationCacheArray({ {.Curve = initParameters.BaseCurve, .AttenuationValue = 1.0f } }, GetDefaultMemoryResource())
								  : CurveAttenuationCacheArray(GetDefaultMemoryResource()));

		mDirectionAttenuationCache.emplace(handle,
			ConeAttenuationCache{ .Cone = initParameters.AttenuationCone, .AttenuationValue = 1.0f });

		return handle;
	}

	JPL_INLINE bool DirectPathService::ReleaseEffectData(DirectEffectHandle source)
	{
		return mAttenuationCache.erase(source) + mDirectionAttenuationCache.erase(source);
	}

	JPL_INLINE AttenuationCurveRef DirectPathService::AssignAttenuationCurve(DirectEffectHandle source, AttenuationCurveRef attenuationFunction)
	{
		if (!source.IsValid() || !attenuationFunction)
			return nullptr;

		//std::shared_ptr<AttenuationFunction> curve = make_pmr_shared(attenuationFunction);
		auto& cache = mAttenuationCache[source];
		return cache.emplace_back(attenuationFunction, 1.0f).Curve;
		//return curve;
	}

	JPL_INLINE float DirectPathService::GetDistanceAttenuation(DirectEffectHandle source, const AttenuationCurveRef& curve) const
	{
		auto it = mAttenuationCache.find(source);
		if (it == mAttenuationCache.end())
			return 1.0f;

		auto cache = std::ranges::find(it->second, curve, [](const CurveAttenuationCache& cache) { return cache.Curve; });
		if (cache != std::ranges::end(it->second))
			return cache->AttenuationValue;

		return 1.0f;
	}

	JPL_INLINE float DirectPathService::GetDirectionAttenuation(DirectEffectHandle source) const
	{
		auto it = mDirectionAttenuationCache.find(source);
		if (it == mDirectionAttenuationCache.end())
			return 1.0f;

		return it->second.AttenuationValue;
	}

	JPL_INLINE float DirectPathService::EvaluateDistance(float distance, const AttenuationCurveRef& attenuationCurve)
	{
		return attenuationCurve->Evaluate(distance);
	}

	JPL_INLINE bool DirectPathService::EvaluateDistance(DirectEffectHandle source, float distance)
	{
		auto it = mAttenuationCache.find(source);
		if (it == mAttenuationCache.end())
			return false;

		for (CurveAttenuationCache& cache : it->second)
			cache.AttenuationValue = cache.Curve->Evaluate(distance);

		return true;
	}

	JPL_INLINE bool DirectPathService::EvaluateDirection(DirectEffectHandle source, float directionDot)
	{
		auto it = mDirectionAttenuationCache.find(source);
		if (it == mDirectionAttenuationCache.end())
			return false;

		JPL_ASSERT(directionDot >= -1.0f && directionDot <= 1.0f);

		it->second.AttenuationValue = ProcessAngleAttenuationImpl(directionDot, it->second.Cone);

		return true;
	}

	template<CVec3 Vec3Type>
	JPL_INLINE DirectPathResult<Vec3Type> DirectPathService::ProcessDirectPath(const Position<Vec3Type>& source, const Position<Vec3Type>& listener)
	{
		static const Vec3Type cForwardAxis(0, 0, -1); // TODO: this is very assuming
		static const Vec3Type cUpAxis(0, 1, 0); // TODO: this is very assuming
#if 1
		const Basis<Vec3Type> listenerBasis = listener.Orientation.ToBasisUnsafe();
		const Vec3Type sourceRelativePosition = source.Location - listener.Location;

		Vec3Type sourcePosInListenerFrame = listenerBasis.InverseTransform(sourceRelativePosition);
		// If source is directly on top, above or below the listener,
		// nudge it a bit forward
		//! Assuming Y axis is UP-DOWN
		if (Math::IsNearlyZero(GetX(sourcePosInListenerFrame)) && Math::IsNearlyZero(GetZ(sourcePosInListenerFrame)))
		{
			sourcePosInListenerFrame += cForwardAxis * 1e-5f;
		}

		// Get distance and cos of source in listener's frame
		const float distance = Length(sourcePosInListenerFrame);
		const Vec3Type dirRelativeToListener = sourcePosInListenerFrame / distance;
		const float directionDot = DotProduct(dirRelativeToListener, cForwardAxis);

		// Get cos of listener in source's frame
		const Vec3Type& sourceForward = source.Orientation.Forward;
		const Vec3Type listenerToSourceDir = listenerBasis.InverseTransform(-dirRelativeToListener);
		const float invDirectionDot = DotProduct(sourceForward, listenerToSourceDir);

		// Get orientation of source in listener's frame
		const Basis<Vec3Type> sourceToListenerOrientation = listenerBasis.InverseTransform(source.Orientation.ToBasisUnsafe());
#else
		Vec3Type sourcePosInListenerFrame = listener.Orientation.ToQuat().Rotate(source.Location);

		// If source is directly on top, above or below the listener,
		// nudge it a bit forward
		//! Assuming Y axis is UP-DOWN
		if (Math::IsNearlyZero(GetX(sourcePosInListenerFrame)) && Math::IsNearlyZero(GetZ(sourcePosInListenerFrame)))
		{
			sourcePosInListenerFrame += cForwardAxis * 1e-5f;
		}

		// Get distance and cos of source in listener's frame
		const float distance = Length(sourcePosInListenerFrame);
		const Vec3Type dirRelativeToListener = sourcePosInListenerFrame / distance;
		const float directionDot = DotProduct(dirRelativeToListener, cForwardAxis);

		// Get cos of listener in source's frame
		const Vec3Type sourceForward = source.Orientation.ToQuat().Rotate(cForwardAxis);
		const Vec3Type listenerToSourceDir = listener.Orientation.ToQuat().Conjugated().Rotate(-dirRelativeToListener);
		const float invDirectionDot = DotProduct(sourceForward, listenerToSourceDir);

		// Get orientation of source in listener's frame
		const Basis<Vec3Type> sourceToListenerOrientation =
			(listener.Orientation.ToQuat().Conjugated() * source.Orientation.ToQuat()).ToBasis();
#endif
		return DirectPathResult<Vec3Type>{
			.Distance = distance,
				.DirectionDot = directionDot,
				.InvDirectionDot = invDirectionDot,
				.Position = {
					.Location = dirRelativeToListener,
					.Orientation = {.Up = sourceToListenerOrientation.Y, .Forward = sourceToListenerOrientation.Z}
			}
		};
	}

	JPL_INLINE float DirectPathService::ProcessAngleAttenuationImpl(float azimutCos, const AttenuationCone& cone)
	{
		// Compute cosines of half of the cone sectors
		const float cutoffInner = std::cos(cone.InnerAngle * 0.5f); // TODO: can we cache actual dot isntead of angles?
		const float cutoffOuter = std::cos(cone.OuterAngle * 0.5f);

		float factor = 0.0f;

		if (azimutCos > cutoffInner)
			return factor;

		if (azimutCos > cutoffOuter)
		{
			// Between inner and outer cones
			factor = (cutoffInner - azimutCos) / (cutoffInner - cutoffOuter);
		}
		else
		{
			// Outside the outer cone
			factor = 1.0f;
		}

		return factor;
	}

	JPL_INLINE float DirectPathService::ProcessAngleAttenuation(float azimuth, AttenuationCone cone)
	{
		if (cone.InnerAngle >= JPL_TWO_PI)
		{
			// Inner angle is 360 degrees so no need to do any attenuation.
			return 0.0f;
		}

		return ProcessAngleAttenuationImpl(std::cos(azimuth), cone);
	}

	template<CVec3 Vec3Type>
	JPL_INLINE float DirectPathService::ProcessAngleAttenuation(const Vec3Type& position,
																const Position<Vec3Type>& referencePoint,
																AttenuationCone cone)
	{
		if (cone.InnerAngle >= JPL_TWO_PI)
		{
			// Inner angle is 360 degrees so no need to do any attenuation.
			return 0.0f;
		}

		// Position and reference must not be the same point
		JPL_ASSERT(!Math::IsNearlyEqual(position, referencePoint.Location));

		const Vec3Type referenceForward = referencePoint.Orientation.Forward;
		const Vec3Type sourceDirection = Normalized(position - referencePoint.Location);
		const float dot = DotProduct(referenceForward, sourceDirection);

		return ProcessAngleAttenuationImpl(dot, cone);
	}

} // namespace JPL