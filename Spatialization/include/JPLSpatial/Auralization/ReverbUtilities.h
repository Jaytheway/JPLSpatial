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
