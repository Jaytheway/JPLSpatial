//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPL Spatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2026 Jaroslav Pevno, JPL Spatial is offered under the terms of the ISC license:
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
#include "JPLSpatial/FrequencyBands.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"
#include "JPLSpatial/SpeedOfSound.h"

namespace JPL
{
	// Conversion factor from dB to reciprocal meters: 1 / (20 * log10(e))
	constexpr float JPL_DB_TO_RECIP_M = 0.1151292546497022842f;
	
	// Conversion factor from reciprocal meters to dB: 20 * log10(e)
	constexpr float JPL_RECIP_M_TO_DB = 8.68588963807f;

	//==========================================================================
	/// Parameters to construct AirAbsorption object that can be used to
	/// compute attenuation for multiple frequencies, while reusing internal factors.
	struct AirAbsorptionParams
	{
		float AirTemperatureC = 20.0f;
		float RelativeHumidityPercent = 50.0f;
		float AtmosphericPressureKPa = 101.325f;
	};

	using FrequencyBands = simd;

	//==========================================================================
	/// Cached data required to compute sound attenuation due to air absorption.
	/// The data depends on the environment parameters and frequency bands
	/// it was calculated for.
	struct AirAbsorptionCache
	{
		const FrequencyBands FrequencyBandCenters;
		const FrequencyBands FrequencyBandLowerThirdCenters;

		/// Air attenuation calculated for frequency band centers, in dB/m.
		/// This can be used for short path propagation filtering
		/// (e.g. direct sound, early reflections, diffraction)
		const AbsorptionCoeffs CenterFrequencyAbsorption_dB;

		/// Air attenuation calculated for center frequency of lower third
		/// of the frequency bands, in dB/m.
		/// This is only valid for linear approximation of the non linear
		/// air absorption behavior computed internaly in AirAbsorption class.
		const AbsorptionCoeffs LowerThirdBandAbsorption_dB;

		/// Air attenuation corrected for high frequency overestimate, in dB/m.
		/// The last band is energy-weighted for more accurate HF decay slope.
		/// This is more accurate for longer propagation paths and is
		/// a middleground compromize between performance and accuracy.
		/// This can be used directly as `distance * absorption_dB`
		const AbsorptionCoeffs HighFreqWeightedAbsorption_dB;

		/// Air absorption in dB/s (effective contribution to RT60).
		/// Calculated as HighFreqWeightedAbsorption_dB * SpeedOfSound.
		const AbsorptionCoeffs FrequencyDecaySlope;

		//======================================================================
		/// Linearized approximation for non-linear frequency loss by distance
		struct
		{
			const simd_mask HasKnee;			// Flag whether frequncy band requires transition to absorption of the lower third
			const simd DistanceToKnee;			// rk
			const simd LinearizedHFLossFactor;	// m(f1) * rk + m(f2) * (r(t) - rk): with r(t) taken out as "lowerThirdFrequencyLoss"
		} LinAprox;
	};
	
	//==========================================================================
	/// AirAbsorption object can be constructed from environmental parameters
	/// and reused to compute absorption for different frequencies.
	/// This avoids a lot of computation when the environment doesn't change.
	/// 
	/// Otherwise AirAbsorption has static functions to compute air absorption
	/// for pure-tone feuquencies and for frequency bands.
	class AirAbsorption
	{
	public:

		//======================================================================
		[[nodiscard]] static inline AirAbsorptionCache CacheParameters(const FrequencyBands& frequencyBands,
																	   const AirAbsorptionParams& params,
																	   float nyquist = 22050.0f);

		/// Compute sound attenuation due to air absorption for `distance`
		/// in four frequency bands.
		/// Uses linearized approximation for non-linear high frequency decay
		/// which provides more accurate result than just `distanc * dB_loss`.
		[[nodiscard]] static inline simd ComputeForDistance(float distance, const AirAbsorptionCache& aaCache);
		
		//======================================================================
		/// Compute pure-tone sound attenuation for atmospheric absorption,
		/// in dB/m.
		/// 
		/// Note: this static function should be use only when a single frequency
		/// coefficient is needed, in case coefficients are needed for multiple
		/// frequency points and the same AirAbsorptionParams, it's better to
		/// construct AirAbsorption object with said AirAbsorptionParams,
		/// which reuses a lot of terms.
		[[nodiscard]] static inline float ComputeForFrequency(float frequencyHz, const AirAbsorptionParams& params);

		/// Compute pure-tone sound attenuation in four frequency bands
		[[nodiscard]] static inline AbsorptionCoeffs ComputeForFrequencies(const FrequencyBands& frequenciesHz, const AirAbsorptionParams& params);

		/// Compute energy-weighted band frequencies for more accurate high
		/// frequency loss over mid-long distance.
		/// I.e. for band with center frequency > 1kHz, use center frequency
		/// of the lower third.
		[[nodiscard]] static inline FrequencyBands ComputeHFWeightedBandCenters(const SplitFrequencies& splitFrequenciesHz);

		//======================================================================
		explicit AirAbsorption(const AirAbsorptionParams& params);

		/// Compute pure-tone sound attenuation coefficient in dB/m, for atmospheric absorption
		[[nodiscard]] float ComputeAbsorptionPerMeter(float frequencyHz) const;
		[[nodiscard]] simd ComputeAbsorptionPerMeter(simd frequenciesHz) const;

	private:
		// Classical absorption contribution.
		float mClassicalAbsorption;

		// Temperature scaling applied to the molecular relaxation terms.
		float mRelaxationTemperatureScale;

		// Oxygen relaxation frequency, frO, in Hz.
		float mOxygenRelaxationFrequency;

		// Nitrogen relaxation frequency, frN, in Hz.
		float mNitrogenRelaxationFrequency;

		// Oxygen molecular relaxation coefficient.
		float mOxygenRelaxationCoefficient;

		// Nitrogen molecular relaxation coefficient.
		float mNitrogenRelaxationCoefficient;

		// 1.0f / mOxygenRelaxationFrequency
		float mOxygenRelaxationFrequencyInv;
		
		// 1.0f / mNitrogenRelaxationFrequency
		float mNitrogenRelaxationFrequencyInv;
	};

	//==========================================================================
	/// Default air absorption used in JPL Spatial.
	inline const AirAbsorptionCache cDefaultAirAbsCache = AirAbsorption::CacheParameters(JPL::cBandCenters, AirAbsorptionParams{});

} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
	inline AirAbsorptionCache AirAbsorption::CacheParameters(const FrequencyBands& frequencyBands,
															 const AirAbsorptionParams& params,
															 float nyquist)
	{
		AirAbsorption aa(params);

		const SplitFrequencies splits = SplitFrequenciesFromBandCenters(frequencyBands, nyquist);
		const FrequencyBands lowerThirdCenters = ComputeBandLowerThirdCenters(splits);

		static constexpr float weightingThresholdHz = 1000.0f;
		const FrequencyBands weightedBands = simd::select(frequencyBands >= simd(weightingThresholdHz), lowerThirdCenters, frequencyBands);
		
		const AbsorptionCoeffs centerAbsorption = aa.ComputeAbsorptionPerMeter(frequencyBands);
		const AbsorptionCoeffs lowerThirdAbsorption = aa.ComputeAbsorptionPerMeter(lowerThirdCenters);
		const AbsorptionCoeffs weightedAbsorption = aa.ComputeAbsorptionPerMeter(weightedBands);

		// At 500 Hz, knee should be at 6 dB, we use 400 Hz to soften up the requirement
		const simd_mask hasKnee = frequencyBands > simd(400.0f);
		
		// Knee point is at 14dB for >= 1kHz and 6dB for 500Hz
		const simd deltaLk = simd::select(frequencyBands >= 1000.0f, 14.0f, 6.0f);
		const simd distanceToKnee = deltaLk / centerAbsorption;

		return AirAbsorptionCache{
			.FrequencyBandCenters = frequencyBands,
			.FrequencyBandLowerThirdCenters = lowerThirdCenters,
			.CenterFrequencyAbsorption_dB = centerAbsorption,
			.LowerThirdBandAbsorption_dB = lowerThirdAbsorption,
			.HighFreqWeightedAbsorption_dB = weightedAbsorption,
			.FrequencyDecaySlope = weightedAbsorption * SpeedOfSound::ForTemperature(params.AirTemperatureC),
			.LinAprox = {
				.HasKnee = hasKnee,
				.DistanceToKnee = distanceToKnee,
				.LinearizedHFLossFactor = distanceToKnee * (centerAbsorption - lowerThirdAbsorption)
			}
		};
	}

	inline simd AirAbsorption::ComputeForDistance(float distance, const AirAbsorptionCache& aaCache)
	{
		// Formula as per Rindel (2024):
		// 
		// if (r(t) < rk)
		//     return centerAbsorption * distance;
		// else
		//     return centerAbsorption * rk + lowerThirdAbsorption * (r(t) - rk);
		// 
		// ----------------------------------------------------------------------
		// 
		// const simd linearizedHighBandLoss = centerAbsorption * rk + lowerThirdAbsorption * (distance - rk);
		// ...simplifies to this:
		//const simd linearizedHighBandLoss = rk * (centerAbsorption - lowerThirdAbsorption) + lowerThirdFrequencyLoss;

		// const simd linearizedHighBandLoss = lowerThirdFrequencyLoss + aaCache.LinAprox.LinearizedHFLossFactor;
		const simd linearizedHighBandLoss =
			Math::FMA(distance, aaCache.LowerThirdBandAbsorption_dB, aaCache.LinAprox.LinearizedHFLossFactor);

		const simd centerFrequencyLoss = distance * aaCache.CenterFrequencyAbsorption_dB;

		return simd::select(aaCache.LinAprox.HasKnee & (simd(distance) >= aaCache.LinAprox.DistanceToKnee),
							linearizedHighBandLoss,
							centerFrequencyLoss);
	}

	inline FrequencyBands AirAbsorption::ComputeHFWeightedBandCenters(const SplitFrequencies& splitFrequenciesHz)
	{
		// Rindel (2024): "European Norm EN 12354-6:
		// For the 1 kHz to the 8 kHz octave bands, the coefficients are the pure-tone attenuation coefficients of the center frequency
		// of the lower one-third octave within the octave band.
		// This is assumed to handle the spectral change within the octave band during the sound propagation."

		// Note: because we can't guarantee that the frequencies are split in octave bands,
		// we are going to compute centers of geometric lower third.
		// This means we can't just multiply lower edge by a fixed ratio.

		const FrequencyBands bandCenters = ComputeBandCenters(splitFrequenciesHz);
		const FrequencyBands lowerThirdCenters = ComputeBandLowerThirdCenters(splitFrequenciesHz);
		return round(simd::select(bandCenters >= simd(1000.0f), lowerThirdCenters, bandCenters));
	}

	//==========================================================================
	inline float AirAbsorption::ComputeForFrequency(float frequencyHz, const AirAbsorptionParams& params)
	{
		// Spelled out formula for computed air absorption

		const float f = frequencyHz;
		const float f2 = f * f;
		const float pa = params.AtmosphericPressureKPa;

		// Reference atmospheric pressure
		const float pr = 101.325f;

		const float T = params.AirTemperatureC + 273.15f;

		// Reference temperature (C)
		const float To = 20.0f + 273.15f;

		// Tripple-point iusotherm temp (C)
		const float To1 = 0.01f + 273.15f;

		// Saturation vapor pressure
		const float psat = pr * ::pow(10, -6.8346 * ::pow(To1 / T, 1.261f) + 4.6151f);

		const float hr = params.RelativeHumidityPercent;

		// Molar concentration of water vapor, as a percentage
		const float h = hr * (psat / pa);

		// Oxygen relaxation frequency
		const float frO = (pa / pr) * (24.0f + 4.04f * ::pow(10, 4) * h * ((0.02f + h) / (0.391f + h)));

		// Nitrogen relaxation frequency
		const float frN = (pa / pr) * ::pow(T / To, -0.5f) * (9 + 280 * h * ::exp(-4.170 * (::pow(T / To, -1.0f / 3.0f) - 1.0f)));

		const float z = 0.1068f * ::exp(-3352 / T) * ::pow(frN + f2 / frN, -1);

		const float y = ::pow(T / To, -2.5f) * (0.01275f * ::exp(-2239.1 / T) * ::pow(frO + f2 / frO, -1) + z);

		// Attenuation coefficient
		const float a = JPL_RECIP_M_TO_DB * f2 * ((1.84f * ::pow(10, -11) * ::pow(pa / pr, -1) * ::pow(T / To, 0.5f)) + y);

		return a;
	}

	inline AbsorptionCoeffs AirAbsorption::ComputeForFrequencies(const FrequencyBands& frequenciesHz, const AirAbsorptionParams& params)
	{
		return AirAbsorption(params).ComputeAbsorptionPerMeter(frequenciesHz);
	}

	//==========================================================================
	inline AirAbsorption::AirAbsorption(const AirAbsorptionParams& params)
	{
		const float referencePressureKPa = 101.325f;
		const float absoluteTemperatureK = params.AirTemperatureC + 273.15f;
		const float referenceTemperatureK = 293.15f; // 20 C
		const float triplePointTemperatureK = 273.16f; // 0.01 C
		const float relativeTemperature = absoluteTemperatureK / referenceTemperatureK;
		const float relativePressure = params.AtmosphericPressureKPa / referencePressureKPa;

		// Saturation vapor pressure, in kPa.
		const float saturationVaporPressureKPa =
			referencePressureKPa *
			::pow(
				10.0f,
				-6.8346f * ::pow(triplePointTemperatureK / absoluteTemperatureK, 1.261f) + 4.6151f);

		// Molar concentration of water vapor as a percentage.
		const float waterVaporConcentrationPercent =
			params.RelativeHumidityPercent * (saturationVaporPressureKPa / params.AtmosphericPressureKPa);

		const float h = waterVaporConcentrationPercent;

		mOxygenRelaxationFrequency =
			relativePressure *
			(24.0f +
			 4.04e4f * h *
			 ((0.02f + h) / (0.391f + h)));

		mNitrogenRelaxationFrequency =
			relativePressure *
			::pow(relativeTemperature, -0.5f) *
			(9.0f +
			 280.0f * h * ::exp(-4.170f * (::pow(relativeTemperature, -1.0f / 3.0f) - 1.0f)));

		mClassicalAbsorption =
			1.84e-11f *
			::pow(relativePressure, -1.0f) *
			::sqrt(relativeTemperature);

		mRelaxationTemperatureScale =
			::pow(relativeTemperature, -2.5f);

		mOxygenRelaxationCoefficient =
			0.01275f *
			::exp(-2239.1f / absoluteTemperatureK);

		mNitrogenRelaxationCoefficient =
			0.1068f *
			::exp(-3352.0f / absoluteTemperatureK);

		mOxygenRelaxationFrequencyInv = 1.0f / mOxygenRelaxationFrequency;
		mNitrogenRelaxationFrequencyInv = 1.0f / mNitrogenRelaxationFrequency;
	}

	inline float AirAbsorption::ComputeAbsorptionPerMeter(float frequencyHz) const
	{
		const float f2 = frequencyHz * frequencyHz;

		const float oxygenRelaxationAbsorption = mOxygenRelaxationCoefficient / Math::FMA(f2, mOxygenRelaxationFrequencyInv, mOxygenRelaxationFrequency);
		const float nitrogenRelaxationAbsorption = mNitrogenRelaxationCoefficient / Math::FMA(f2, mNitrogenRelaxationFrequencyInv, mNitrogenRelaxationFrequency);
		const float molecularRelaxationAbsorption = mRelaxationTemperatureScale * (oxygenRelaxationAbsorption + nitrogenRelaxationAbsorption);
		const float totalAbsorptionBeforeFrequencyScale = mClassicalAbsorption + molecularRelaxationAbsorption;

		return JPL_RECIP_M_TO_DB * f2 * totalAbsorptionBeforeFrequencyScale;
	}

	inline simd AirAbsorption::ComputeAbsorptionPerMeter(simd frequenciesHz) const
	{
		const simd f2 = frequenciesHz * frequenciesHz;

		const simd oxygenRelaxationAbsorption = simd(mOxygenRelaxationCoefficient) / Math::FMA(f2, mOxygenRelaxationFrequencyInv, simd(mOxygenRelaxationFrequency));
		const simd nitrogenRelaxationAbsorption = simd(mNitrogenRelaxationCoefficient) / Math::FMA(f2, mNitrogenRelaxationFrequencyInv, simd(mNitrogenRelaxationFrequency));
		const simd molecularRelaxationAbsorption = simd(mRelaxationTemperatureScale) * (oxygenRelaxationAbsorption + nitrogenRelaxationAbsorption);
		const simd totalAbsorptionBeforeFrequencyScale = simd(mClassicalAbsorption) + molecularRelaxationAbsorption;

		return JPL_RECIP_M_TO_DB * f2 * totalAbsorptionBeforeFrequencyScale;
	}
} // namespac JPL
