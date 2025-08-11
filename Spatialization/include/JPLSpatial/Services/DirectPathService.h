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

#include "../Core.h"
#include "../DistanceAttenuation.h"

#include <Jolt/Jolt.h>

#include <vector>
#include <memory>
#include <unordered_map>
#include <ranges>

namespace JPL
{
	//==========================================================================
	struct DirectPathResult
	{
		float Distance;		//< Distance from source to listener
		float Azimuth;		//< Angle between direction from listener to source and listener's forward vector
		float Altitude;		//< Altitude angle between direction from listener to source and listener's forward vector
		float InvAzimuth;	//< Angle between direction from source to listener and source's forward vector
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

	class DirectPathService;
	using DirectEffectHandle = IDType<DirectPathService>;

	struct DirectEffectInitParameters
	{
		AttenuationCurveRef BaseCurve;
		AttenuationCone AttenuationCone;
	};

	//==========================================================================
	class DirectPathService
	{
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
		JPL_INLINE AttenuationCurveRef AssignAttenuationCurve(DirectEffectHandle source, AttenuationFunction* attenuationFunction);

		/// Compute direct path parameters based on position of the soruce and listener.
		/// This can be used to compute parameters for position of a source
		/// relative to listener, as well as position of the listener relative
		/// to a source.
		/// 
		/// @param position			position to compute parameters for, in world space
		/// @param referencePoint		reference point to compute parameters from, in world space
		/// 
		/// @returns					paramteres of the position relative to the reference point
		static JPL_INLINE DirectPathResult ProcessDirectPath(const JPH::Mat44& source,
															 const JPH::Mat44& listener);

		static JPL_INLINE float ProcessAngleAttenuation(const JPH::Vec3& position,
														const JPH::Mat44& referencePoint,
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
		/// @param source		source to evaluate the curves for
		/// @param azimuth	direction angle relative to forward axis to evaluate
		/// 
		/// @returns			'true' if the source is valid, 'false' otherwise
		JPL_INLINE bool EvaluateDirection(DirectEffectHandle source, float azimuth);


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
		// TODO: can we store cache for more efficient access?
		std::unordered_map<DirectEffectHandle, std::vector<CurveAttenuationCache>> mAttenuationCache;
		std::unordered_map<DirectEffectHandle, ConeAttenuationCache> mDirectionAttenuationCache;
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
								  ? std::vector<CurveAttenuationCache>{{ .Curve = initParameters.BaseCurve, .AttenuationValue = 1.0f }}
								  : std::vector<CurveAttenuationCache>{});

		mDirectionAttenuationCache.emplace(handle,
										   ConeAttenuationCache{ .Cone = initParameters.AttenuationCone, .AttenuationValue = 1.0f });

		return handle;
	}

	JPL_INLINE bool DirectPathService::ReleaseEffectData(DirectEffectHandle source)
	{
		return mAttenuationCache.erase(source) + mDirectionAttenuationCache.erase(source);
	}

	JPL_INLINE AttenuationCurveRef DirectPathService::AssignAttenuationCurve(DirectEffectHandle source, AttenuationFunction* attenuationFunction)
	{
		if (!source.IsValid() || !attenuationFunction)
			return nullptr;

		std::shared_ptr<AttenuationFunction> curve;
		curve.reset(attenuationFunction);
		auto& cache = mAttenuationCache[source];
		cache.emplace_back(CurveAttenuationCache{ curve, 1.0f });
		return curve;
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

	JPL_INLINE bool DirectPathService::EvaluateDirection(DirectEffectHandle source, float azimuth)
	{
		auto it = mDirectionAttenuationCache.find(source);
		if (it == mDirectionAttenuationCache.end())
			return false;

		it->second.AttenuationValue = ProcessAngleAttenuation(azimuth, it->second.Cone);

		return true;
	}

	JPL_INLINE DirectPathResult DirectPathService::ProcessDirectPath(const JPH::Mat44& source, const JPH::Mat44& listener)
	{
		const JPH::Vec3 sourceToListener = listener.InversedRotationTranslation() * source.GetTranslation();

		const JPH::Vec3 xz(sourceToListener.GetX(), 0.0f, sourceToListener.GetZ());
		const JPH::Vec3 xy(sourceToListener.GetX(), 0.0f, sourceToListener.GetY());

		const float azimuth = xz.IsNearZero() ? 0.0f : JPH::ATan2(sourceToListener.GetX(), -sourceToListener.GetZ());
		// TODO: this altitude may not be correct
		const float altitude = xy.IsNearZero() ? 0.0f : JPH::ATan2(sourceToListener.GetY(), xy.Length());

		const JPH::Vec3 listenerToSource = source.InversedRotationTranslation() * listener.GetTranslation();
	
		const float listenerToSourceAzimuth =
			JPH::Vec3(listenerToSource.GetX(), 0.0f, listenerToSource.GetZ()).IsNearZero()
			? 0.0f
			: JPH::ATan2(listenerToSource.GetX(), -listenerToSource.GetZ());
		
		return DirectPathResult{
			.Distance = sourceToListener.Length(),
			.Azimuth = azimuth,
			.Altitude = altitude,
			.InvAzimuth = listenerToSourceAzimuth
		};
	}

	JPL_INLINE float DirectPathService::ProcessAngleAttenuationImpl(float azimutCos, const AttenuationCone& cone)
	{
		// Compute cosines of half of the cone sectors
		const float cutoffInner = std::cos(cone.InnerAngle * 0.5f);
		const float cutoffOuter = std::cos(cone.OuterAngle * 0.5f);

		float factor = 0.0f;

		auto lerp = [](auto a, auto b, auto t) { return a + t * (b - a); };

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
		static constexpr float two_pi = std::numbers::pi_v<float> * 2.0f;

		if (cone.InnerAngle >= two_pi)
		{
			// Inner angle is 360 degrees so no need to do any attenuation.
			return 0.0f;
		}

		return ProcessAngleAttenuationImpl(std::cos(azimuth), cone);
	}

	JPL_INLINE float DirectPathService::ProcessAngleAttenuation(const JPH::Vec3& position,
																const JPH::Mat44& referencePoint,
																AttenuationCone cone)
	{
		static constexpr float two_pi = std::numbers::pi_v<float> * 2.0f;

		if (cone.InnerAngle >= two_pi)
		{
			// Inner angle is 360 degrees so no need to do any attenuation.
			return 0.0f;
		}

		JPH::Vec3 listenerForward = referencePoint.GetRotation().GetAxisZ().Normalized();
		const JPH::Vec3 sourceDirection = (position - referencePoint.GetTranslation()).Normalized();

		return ProcessAngleAttenuationImpl(listenerForward.Dot(sourceDirection), cone);

#if 0 // Branchless vercorized version turned out to be actually slower

		const JPH::Vec3 listenerForward = referencePoint.GetRotation().GetAxisZ().Normalized();
		const JPH::Vec3 sourceDirection = (position - referencePoint.GetTranslation()).Normalized();
		const float d = listenerForward.Dot(sourceDirection);

		const float cutoffInner = std::cos(cone.InnerAngle * 0.5f);
		const float cutoffOuter = std::cos(cone.OuterAngle * 0.5f);

		// Prepare SIMD vectors
		const JPH::Vec4 dVec = JPH::Vec4::sReplicate(d);
		const JPH::Vec4 cutoffInnerVec = JPH::Vec4::sReplicate(cutoffInner);
		const JPH::Vec4 cutoffOuterVec = JPH::Vec4::sReplicate(cutoffOuter);
		const JPH::Vec4 outerGainVec = JPH::Vec4::sReplicate(cone.OuterGain);
		const JPH::Vec4 oneVec = JPH::Vec4::sReplicate(1.0f);
		const JPH::Vec4 zeroVec = JPH::Vec4::sZero();

		// Compute denominator and handle division by zero
		JPH::Vec4 denominatorVec = cutoffInnerVec - cutoffOuterVec;
		const float epsilon = 1e-6f;
		const JPH::Vec4 epsilonVec = JPH::Vec4::sReplicate(epsilon);
		denominatorVec = JPH::Vec4::sMax(denominatorVec, epsilonVec);

		// Compute tVec = (d - cutoffOuter) / (cutoffInner - cutoffOuter)
		JPH::Vec4 tVec = (dVec - cutoffOuterVec) / denominatorVec;

		// Clamp tVec to [0, 1] without branches
		tVec = JPH::Vec4::sMax(zeroVec, JPH::Vec4::sMin(tVec, oneVec));

		// Compute interpolated angular gain
		const JPH::Vec4 interpolatedGainVec = outerGainVec + (oneVec - outerGainVec) * tVec;

		// Compute masks
		const JPH::UVec4 mask_d_gt_cutoffInner = JPH::Vec4::sGreater(dVec, cutoffInnerVec);
		const JPH::UVec4 mask_d_gt_cutoffOuter = JPH::Vec4::sGreater(dVec, cutoffOuterVec);

		// Initialize angularGainVec with cone.OuterGain
		JPH::Vec4 angularGainVec = outerGainVec;

		// If d > cutoffOuter, select interpolatedGainVec
		angularGainVec = JPH::Vec4::sSelect(angularGainVec, interpolatedGainVec, mask_d_gt_cutoffOuter);

		// If d > cutoffInner, select oneVec (angularGain = 1.0f)
		angularGainVec = JPH::Vec4::sSelect(angularGainVec, oneVec, mask_d_gt_cutoffInner);

		// Extract the scalar result
		return angularGainVec.GetX();
#endif
	}

} // namespace JPL