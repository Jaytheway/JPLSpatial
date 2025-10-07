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
#include "JPLSpatial/Math/Curves.h"
#include "JPLSpatial/Memory/Memory.h"

#include <algorithm>
#include <cmath>
#include <vector>

namespace JPL
{
	enum AttenuationModel : int
	{
		None,			// No distance attenuation.
		Inverse,		// Equivalent to OpenAL's AL_INVERSE_DISTANCE_CLAMPED.
		Linear,			// Linear attenuation. Equivalent to OpenAL's AL_LINEAR_DISTANCE_CLAMPED.
		Exponential,	// Exponential attenuation. Equivalent to OpenAL's AL_EXPONENT_DISTANCE_CLAMPED.

		_Count
	};

	struct AttenuationFunction
	{
		virtual float Evaluate(float distance) const = 0;
		virtual ~AttenuationFunction() = default;
	};

	//==========================================================================
	// TODO
	/*struct AttenuationLUT : AttenuationFunction
	{
	};*/

	//==========================================================================
	struct AttenuationBaseModel : AttenuationFunction
	{
		AttenuationModel Model;
		struct DistanceAttenuationParameters
		{
			float MinDistance;
			float MaxDistance;
			float Rolloff;
		} Parameters;

		JPL_INLINE float Evaluate(float distance) const final
		{
			auto isMinMaxValid = [&] { return Parameters.MinDistance < Parameters.MaxDistance; };
			auto distanceClamped = [&] { return std::clamp(distance,  Parameters.MinDistance, Parameters.MaxDistance); };

			switch (Model)
			{
			case AttenuationModel::Inverse:
			{
				auto process = [&]
				{
					return  Parameters.MinDistance
						/ ( Parameters.MinDistance + Parameters.Rolloff * (distanceClamped() -  Parameters.MinDistance));
				};

				return isMinMaxValid()
					? process()
					: 1.0f; // To avoid division by zero. Do not attenuate.
			}
			case AttenuationModel::Linear:
			{
				auto process = [&]
				{
					return 1.0f - Parameters.Rolloff * (distanceClamped() -  Parameters.MinDistance)
						/ (Parameters.MaxDistance -  Parameters.MinDistance);
				};

				return isMinMaxValid()
					? process()
					: 1.0f;
			}
			case AttenuationModel::Exponential:
			{
				auto process = [&]
				{
					return std::pow(distanceClamped() /  Parameters.MinDistance, -Parameters.Rolloff);
				};

				return isMinMaxValid()
					? process()
					: 1.0f;
			}
			case AttenuationModel::None:
			default:
			{
				return 1.0f;
			}
			}
		}
	};

	// TODO: repurpose this as a generic curve that we may use for stuff like volume envelopes
	//==========================================================================
	/// Attenuation curve constructed from a set of points,
	/// each point can have a different function used to evaluate
	/// the following segment of the curve, up to the next point.
	struct AttenuationCurve : AttenuationFunction
	{
		struct Point
		{
			union
			{
				Curve::Point XY;
				struct
				{
					float Distance; //< The distance at which this point is defined
					float Value;    //< The attenuation value at this distance
				};
			};

			/// The function type to use from this point onward
			Curve::EType FunctionType; 
		};

		/// Curve points that should sorted by distance
		std::pmr::vector<Point> Points{ GetDefaultMemoryResource() };

		/// Sort curve points by distance. This should be called after changing Points.
		JPL_INLINE void SortPoints()
		{
			std::sort(Points.begin(), Points.end(), [](const AttenuationCurve::Point& a, const AttenuationCurve::Point& b)
			{
				return a.Distance < b.Distance;
			});
		}

		/// Evaluate curve at `distance`
		JPL_INLINE float Evaluate(float distance) const final;

	private:
		JPL_INLINE float EvaluateFunction(const Point& point, float distance, const Point* nextPoint) const;
	};

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

	JPL_INLINE float AttenuationCurve::Evaluate(float distance) const
	{
		if (Points.empty())
			return 1.0f; // Default attenuation

		// Handle distances before the first point
		if (distance <= Points.front().Distance)
		{
			const auto& point = Points.front();
			return EvaluateFunction(point, distance, nullptr);
		}

		// Handle distances beyond the last point
		if (distance >= Points.back().Distance)
		{
			// TODO: we might want to just return point.Value here,
			// i.e. treat out of bounds as constant
			const auto& point = Points.back();
			return EvaluateFunction(point, distance, nullptr);
		}

		// Binary search to find the point just before the given distance
		auto it = std::upper_bound(Points.begin(), Points.end(), distance, [](float value, const Point& point)
		{
			return value < point.Distance;
		});

		// Get the current point and the next point
		const auto& prevPoint = *(it - 1);
		const Point* nextPoint = (it != Points.end()) ? &(*it) : nullptr;

		// Evaluate the attenuation using the function type of the previous point
		// TODO: we may want to change this to evaluate function type of the next point
		return EvaluateFunction(prevPoint, distance, nextPoint);
	}

	static JPL_INLINE float ComputeAttenuation(Curve::EType curveType, float distance, float distance0, float distance1)
	{
		float t = (distance - distance0) / (distance1 - distance0);
		t = std::clamp(t, 0.0f, 1.0f);
		
		return EvaluateCurve(curveType, t);
	}

	JPL_INLINE float AttenuationCurve::EvaluateFunction(const Point& point, float distance, const Point* nextPoint) const
	{
		if (!nextPoint || point.FunctionType == Curve::EType::Constant)
			return point.Value;

		const float amplitude = nextPoint->Value - point.Value;
		return point.Value + amplitude * ComputeAttenuation(point.FunctionType, distance, point.Distance, nextPoint->Distance);
	}
} // namespace JPL