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
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/Math/DecibelsAndGain.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"

#include <cmath>
#include <numbers>
#include <span>
#include <vector>

namespace JPL
{
    /// Settings to fit Energy Decay Curve
    /// to compute Reverberation Time
	struct RT60FitSettings
	{
		float BinSeconds = 0.003f;
        float StartTime = 0.08f;
		float FitMinDb = -25.0f;
		float FitMaxDb = -5.0f;
		float Eps = 1e-20f;
		uint32 MinFitPoints = 8;
	};

    /// Compute Energy Decay Curve from energy response histogram
    /// @param outEDC output buffer, where the EDC is going to be written to, has to be of size >= histogram.size()
    JPL_INLINE void ComputeEDC(std::span<const simd> histogram, std::span<simd> outEDC, const simd& tailIntegral = simd(0.0f));

    [[nodiscard]] inline simd ComputeDecaySlopeFromEDC(std::span<const simd> edc, const RT60FitSettings& settings);

    /// Compute energy decay slope in dB/s from the histogram based on fit settings
    /// @returns decay values in dB/s
    [[nodiscard]] JPL_INLINE simd ComputeDecaySlope(std::span<const simd> histogram, const RT60FitSettings& settings);

    /// Estimate RT60 by fitting energy decay curve of the `histogram` based on fit settings.
    /// @returns reverberation time in seconds
    [[nodiscard]] JPL_INLINE simd EstimateRT60(std::span<const simd> histogram, const RT60FitSettings& settings);

    [[nodiscard]] JPL_INLINE simd ComputeHistogramTailIntegral(std::span<const simd> histogram, float binTimeSeconds);

    //======================================================================
    /// Estimate RT60 using Sabine's equation, which tends to be more accurate
    /// for diffuse spaces, evenly distributed absorption and for rooms
    /// with low average absorption (< 0.4).
    [[nosiscard]] JPL_INLINE simd EstimateRT60_Sabine(float w, float l, float h, const simd& avgAbsorption);
    
    /// Estimate RT60 using Sabine's equation, taking into account air absorption
    [[nosiscard]] JPL_INLINE simd EstimateRT60_Sabine(float w, float l, float h, const simd& avgAbsorption, const simd& airAttenuation_dB);

    // Note: we keep Sabine estimator for material workflows/measures,
    // because absorption coefficients found online often assume Sabine measure.

    /// Estimate RT60 using Eyring-Norris equation, which is more accurate
    /// than Sabine for absorptive rooms, but works on the assumption that
    /// the room's sound field is perfectly diffused.
    [[nosiscard]] JPL_INLINE simd EstimateRT60_Eyring(float w, float l, float h, const simd& avgAbsorption);

    /// Estimate RT60 using Eyring-Norris equation, taking into account air absorption
    [[nosiscard]] JPL_INLINE simd EstimateRT60_Eyring(float w, float l, float h, const simd& avgAbsorption, const simd& airAttenuation_dB);

} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
    [[nodiscard]] JPL_INLINE simd EstimateRT60(std::span<const simd> histogram, const RT60FitSettings& settings)
    {
        const simd slope = ComputeDecaySlope(histogram, settings);

        // TODO: we probably want to return just slope, and use it directly in our reverb
        const simd rt60 = simd(-60.0f) / slope;

        return simd::select(slope == 0.0, simd(0.0f), rt60);
    }

    [[nodiscard]] JPL_INLINE simd ComputeDecaySlope(std::span<const simd> histogram, const RT60FitSettings& settings)
    {
        const int n = static_cast<int>(histogram.size());
        if (n <= 0)
            return simd(0.0f);

        // TODO: we could also give a shorter histogram to ComputeSlope from, i.e. cut off the tail that we go the integral from
        //const simd missingTailIntegral = ComputeHistogramTailIntegral(histogram, settings.BinSeconds);

        // TODO: plot EDC and play around with different fitting range and "knee" rejection
        std::vector<simd> edc(n);
        ComputeEDC(histogram, edc);

        // TODO: trying out tail integral over EDC instead of the source histogram
        /*const simd missingTailIntegral = ComputeHistogramTailIntegral(edc, settings.BinSeconds);
        for (simd& v : edc)
            v += missingTailIntegral;*/

        return ComputeDecaySlopeFromEDC(edc, settings);
    }

    JPL_INLINE simd ComputeHistogramTailIntegral(std::span<const simd> histogram, float binTimeSeconds)
    {
        // TODO: figure out how to make this Lundeby's Correction Method useful

        int lastBin = histogram.size() - 1;
        for (lastBin; lastBin >= 0; lastBin--)
        {
            if ((histogram[lastBin] > simd(0.0f)).all_of())
                break;
        }

        const uint32 tailN1 = lastBin * 0.3; // trying to cut out the "knee"
        const uint32 tailN2 = lastBin * 0.1;
        const simd energy1 = histogram[lastBin + 1 - tailN1];
        const simd energy2 = histogram[lastBin + 1 - tailN2];
        const simd deltaT = tailN1 * binTimeSeconds - tailN2 * binTimeSeconds;
        const simd decayConstant = -log(energy2 / energy1) / deltaT;

        const simd missingTailIntegral = histogram[tailN2] / decayConstant;
        return missingTailIntegral;
    }

    JPL_INLINE void ComputeEDC(std::span<const simd> histogram, std::span<simd> outEDC, const simd& tailIntegral)
    {
        JPL_ASSERT(outEDC.size() >= histogram.size());

        simd sum = tailIntegral;

        for (int i = histogram.size() - 1; i >= 0; --i)
        {
            sum += max(histogram[i], simd(0.0f));
            outEDC[i] = sum;
        }
    }

    [[nodiscard]] inline simd ComputeDecaySlopeFromEDC(std::span<const simd> edc, const RT60FitSettings& settings)
    {
        const int n = static_cast<int>(edc.size());
        JPL_ASSERT(n > 0);

        const int startBin = static_cast<int>(settings.StartTime / settings.BinSeconds);
        JPL_ASSERT(startBin < n);

        const simd totalEnergy = edc[startBin]; // If we want the FDN-late-tail estimate, normalize against edc[startBin], not edc[0]
        const simd eps = simd(settings.Eps);
        const simd invTotalEnergy = simd(1.0f) / max(totalEnergy, eps);

        simd count = simd(0.0f);
        simd sumX = simd(0.0f);
        simd sumY = simd(0.0f);
        simd sumXX = simd(0.0f);
        simd sumXY = simd(0.0f);

        for (int i = startBin; i < n; ++i)
        {
            const float tScalar = static_cast<float>(i) * settings.BinSeconds;
            const simd t = simd(tScalar);

            const simd yDb = -IntencityTodB(max(edc[i], eps) * invTotalEnergy);

            const simd_mask validEnergy = totalEnergy > eps;
            const simd_mask inFitRange =
                validEnergy &
                (edc[i] > eps) &
                (yDb <= simd(settings.FitMaxDb)) &
                (yDb >= simd(settings.FitMinDb));

            const simd w = simd::select(inFitRange, simd(1.0f), simd(0.0f));

            count += w;
            sumX += w * t;
            sumY += w * yDb;
            sumXX += w * t * t;
            sumXY += w * t * yDb;
        }

        const simd denom = count * sumXX - sumX * sumX;
        const simd slope = (count * sumXY - sumX * sumY) / denom;

        const simd_mask validFit =
            (count >= simd(settings.MinFitPoints)) &
            (abs(denom) > simd(1.0e-20f)) &
            (slope < simd(-1.0e-6f));

        return simd::select(validFit, slope, simd(0.0f));
    }

    //======================================================================
    namespace Impl
    {
        struct Sabine
        {
            [[nosidscard]] static JPL_INLINE simd Compute(float surface, const simd& avgAbsorption)
            {
                return surface * avgAbsorption;
            }
        };

        struct Eyring
        {
            [[nosidscard]] static JPL_INLINE simd Compute(float surface, const simd& avgAbsorption)
            {
                return -surface * log(simd(1.0f) - avgAbsorption);
            }
        };

        [[nosidscard]] JPL_INLINE simd ComputeAirAbsorptionFactor(float volume, const simd& airAttenuation_dB)
        {
            // Air attenuation as reciprocal meters (1/m).
            // I.e. dB converted to Nepers
            static const float napersCoeff = 1.0f / (10.0f * ::log10(std::numbers::e_v<float>));
            const simd m = airAttenuation_dB * napersCoeff;
            return 4.0f * m * volume;
        }

        template<class SurfaceFactor>
        [[nodiscard]] JPL_INLINE simd EstimateRT60(float w, float l, float h, const simd& avgAbsorption)
        {
            static const simd K(0.161f); // room constant
            const float surface = 2.0f * (l * w + l * h + w * h);
            const float volume = w * l * h;
            return simd(K * volume) / SurfaceFactor::Compute(surface, avgAbsorption);
        }

        template<class SurfaceFactor>
        [[nodiscard]] JPL_INLINE simd EstimateRT60(float w, float l, float h, const simd& avgAbsorption, const simd& airAttenuation_dB)
        {
            static const simd K(0.161f); // room constant
            const float surface = 2.0f * (l * w + l * h + w * h);
            const float volume = w * l * h;
            return simd(K * volume) /
                (SurfaceFactor::Compute(surface, avgAbsorption) + Impl::ComputeAirAbsorptionFactor(volume, airAttenuation_dB));
        }

    } // namespace Impl


    [[nodiscard]] JPL_INLINE simd EstimateRT60_Sabine(float w, float l, float h, const simd& avgAbsorption)
    {
        return Impl::EstimateRT60<Impl::Sabine>(w, l, h, avgAbsorption);
    }

    [[nodiscard]] JPL_INLINE simd EstimateRT60_Sabine(float w, float l, float h, const simd& avgAbsorption, const simd& airAttenuation_dB)
    {
        return Impl::EstimateRT60<Impl::Sabine>(w, l, h, avgAbsorption, airAttenuation_dB);

    }

    [[nodiscard]] JPL_INLINE simd EstimateRT60_Eyring(float w, float l, float h, const simd& avgAbsorption)
    {
        return Impl::EstimateRT60<Impl::Eyring>(w, l, h, avgAbsorption);
    }

    [[nodiscard]] JPL_INLINE simd EstimateRT60_Eyring(float w, float l, float h, const simd& avgAbsorption, const simd& airAttenuation_dB)
    {
        return Impl::EstimateRT60<Impl::Eyring>(w, l, h, avgAbsorption, airAttenuation_dB);
    }

} // namespace JPL
