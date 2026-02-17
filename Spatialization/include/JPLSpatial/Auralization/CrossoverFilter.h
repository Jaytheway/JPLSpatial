//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
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
#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"

#include <algorithm>
#include <cmath>
#include <numbers>
#include <span>

namespace JPL
{
    //==========================================================================
    /// State Variable Filter with Topology-Preserving Transform structure
    struct StateVariableFilterTPT
    {
        struct Params
        {
            float g = 0.0f;
            float d = 0.0f;
            float MinusRg = 0.0f;
        };

        // State
        float s1 = 0.0f, s2 = 0.0f;

        /// Prepare parameters for a TPT SVGF Stage
        [[nodiscard]] static JPL_INLINE Params Prepare(float sampleRate, float frequency) noexcept
        {
            static constexpr float R = std::numbers::sqrt2_v<float>;
            const float g = std::tan(JPL_PI * (frequency / sampleRate));
            // d = 1.0f / (1.0f + R * g + g * g);
            return Params{
                .g = g,
                .d = 1.0f / Math::FMA(g, R + g, 1.0f),
                .MinusRg = -(R + g)
            };
        }

        JPL_INLINE void Reset() noexcept { s1 = 0.0f; s2 = 0.0f; }

        JPL_INLINE void Process(const Params& params, float x, float& yL, float& yB, float& yH) noexcept
        {
            // HP := (x-g1*s1-s2)*d;                    // g1=2R+g, d=1/(1+2Rg+g^2)
            // v1 := g*HP; BP := v1+s1; s1 := BP+v1;    // first integrator
            // v2 := g*BP; LP := v2+s2; s2 := LP+v2;    // second integrator

            // yH = (x - (R + g) * s1 - s2) * d;
            yH = Math::FMA(params.MinusRg, s1, x - s2) * params.d;
            yB = Math::FMA(params.g, yH, s1);
            s1 = Math::FMA(params.g, yH, yB);
            yL = Math::FMA(params.g, yB, s2);
            s2 = Math::FMA(params.g, yB, yL);
        }

        JPL_INLINE void ProcessBPLP(const Params& params, float x, float& yL, float& yB)
        {
            // BP: = (g * (x - s2) + s1) * d;           // d=1/(1+2Rg+g^2)
            // v1 := BP-s1; s1 := BP+v1;                // first integrator
            // v2 := g*BP; LP := v2+s2; s2 := LP+v2;    // second integrator

            yB = Math::FMA(params.g, (x - s2), s1) * params.d;
            s1 = yB + yB - s1;
            yL = Math::FMA(params.g, yB, s2);
            s2 = Math::FMA(params.g, yB, yL);
        }

        JPL_INLINE void ProcessBPHP(const Params& params, float x, float& yB, float& yH)
        {
            // HP := (x-g1*s1-s2)*d;                    // g1=2R+g, d=1/(1+2Rg+g^2)
            // v1 := g*HP; BP := v1+s1; s1 := BP+v1;    // first integrator
            // v22 := g*BP2; s2 := s2+v22;              // second integrator

            // yH = (x - (R + g) * s1 - s2) * d;
            yH = Math::FMA(params.MinusRg, s1, x - s2) * params.d;
            yB = Math::FMA(params.g, yH, s1);
            s1 = Math::FMA(params.g, yH, yB);
            s2 = Math::FMA(params.g, yB + yB, s2);
        }

        JPL_INLINE void ProcessBP(const Params& params, float x, float& yB)
        {
            // BP: = (g * (x - s2) + s1) * d;   // d=1/(1+2Rg+g^2)
            // BP2 := BP+BP; s1 := BP2-s1;      // first integrator
            // v22 := g*BP2; s2 := s2+v22;      // second integrator

            yB = Math::FMA(params.g, (x - s2), s1) * params.d;
            yB = yB + yB;
            s1 = yB - s1;
            s2 = Math::FMA(params.g, yB, s2);
        }

        JPL_INLINE void ProcessAP(const Params& params, float x, float& yAP)
        {
            static constexpr float minusR = -std::numbers::sqrt2_v<float>;
            float yL, yB, yH;
            Process(params, x, yL, yB, yH);
            yAP = Math::FMA(minusR, yB, yH + yL); // yL - R * yB + yH;
        }
    };

    //==========================================================================
    /// State Variable Filter with Topology-Preserving Transform structure.
    /// SIMD variant that can accommodate 4 different filters processed in parallel.
    struct TPTSVFStageSIMD
    {
        struct Params
        {
            simd g = 0.0f;
            simd d = 0.0f;
            simd MinusRg = 0.0f;
        };

        // State
        simd s1 = 0.0f, s2 = 0.0f;

        [[nodiscard]] static JPL_INLINE Params Prepare(const simd& sampleRate, const simd& frequency) noexcept
        {
            static const simd R = std::numbers::sqrt2_v<float>;
            const simd g = tan(JPL_PI * (frequency / sampleRate));
            // d = 1.0f / (1.0f + R * g + g * g);
            return Params{
                .g = g,
                .d = simd::c_1() / Math::FMA(g, R + g, 1.0f),
                .MinusRg = -(R + g)
            };
        }

        JPL_INLINE void Reset() noexcept { s1 = 0.0f; s2 = 0.0f; }

        JPL_INLINE void Process(const Params& params, const simd& x, simd& yL, simd& yB, simd& yH) noexcept
        {
            // yH = (x - (R + g) * s1 - s2) * d;
            yH = Math::FMA(params.MinusRg, s1, x - s2) * params.d;
            yB = Math::FMA(params.g, yH, s1);
            s1 = Math::FMA(params.g, yH, yB);
            yL = Math::FMA(params.g, yB, s2);
            s2 = Math::FMA(params.g, yB, yL);
        }

        JPL_INLINE void ProcessBPLP(const Params& params, simd x, simd& yL, simd& yB)
        {
            yB = Math::FMA(params.g, (x - s2), s1) * params.d;
            s1 = yB + yB - s1;
            yL = Math::FMA(params.g, yB, s2);
            s2 = Math::FMA(params.g, yB, yL);
        }

        JPL_INLINE void ProcessBPHP(const Params& params, simd x, simd& yB, simd& yH)
        {
            yH = Math::FMA(params.MinusRg, s1, x - s2) * params.d;
            yB = Math::FMA(params.g, yH, s1);
            s1 = Math::FMA(params.g, yH, yB);
            s2 = Math::FMA(params.g, yB + yB, s2);
        }

        JPL_INLINE void ProcessBP(const Params& params, simd x, simd& yB)
        {
            yB = Math::FMA(params.g, (x - s2), s1) * params.d;
            yB = yB + yB;
            s1 = yB - s1;
            s2 = Math::FMA(params.g, yB, s2);
        }

        JPL_INLINE void ProcessAP(const Params& params, simd x, simd& yAP)
        {
            static const simd minusR = -std::numbers::sqrt2_v<float>;
            simd yL, yB, yH;
            Process(params, x, yL, yB, yH);
            yAP = Math::FMA(minusR, yB, yH + yL); // yL - R * yB + yH;
        }
    };

    //==========================================================================
    struct TPTSVFAllpass : private StateVariableFilterTPT
    {
        typename StateVariableFilterTPT::Params Params;

        JPL_INLINE void Prepare(float sampleRate, float frequency) noexcept
        {
            Params = StateVariableFilterTPT::Prepare(sampleRate, frequency);
        }

        JPL_INLINE void Reset() noexcept
        {
            StateVariableFilterTPT::Reset();
        }

        [[nodiscard]] JPL_INLINE float Process(float x) noexcept
        {
            static constexpr float minusR = -std::numbers::sqrt2_v<float>;
            float yL, yB, yH;
            StateVariableFilterTPT::Process(Params, x, yL, yB, yH);
            return Math::FMA(minusR, yB, yH + yL); // yL - R * yB + yH;
        }
    };

    //==========================================================================
    struct TPTSVFAllpassSIMD : private TPTSVFStageSIMD
    {
        typename TPTSVFStageSIMD::Params Params;

        JPL_INLINE void Prepare(const simd& sampleRate, const simd& frequency) noexcept
        {
            Params = TPTSVFStageSIMD::Prepare(sampleRate, frequency);
        }

        JPL_INLINE void Reset() noexcept
        {
            TPTSVFStageSIMD::Reset();
        }

        [[nodiscard]] JPL_INLINE simd Process(const simd& x) noexcept
        {
            simd yAP;
            TPTSVFStageSIMD::ProcessAP(Params, x, yAP);
            return yAP;
        }
    };

    //==========================================================================
    /// 4th order Linkwitz-Riley crossover
    struct LR4Split
    {
        StateVariableFilterTPT Stage1;
        StateVariableFilterTPT Stage2;

        typename StateVariableFilterTPT::Params Params;

        JPL_INLINE void Prepare(float sampleRate, float frequency) noexcept
        {
            frequency = std::clamp(frequency, 1.0f, 0.49f * sampleRate);
            Params = StateVariableFilterTPT::Prepare(sampleRate, frequency);
            Reset();
        }

        JPL_INLINE void Reset() noexcept
        {
            Stage1.Reset();
            Stage2.Reset();
        }

        [[nodiscard]] JPL_INLINE float ProcessLP(float x) noexcept
        {
            // 1. TPT-SVF stage
            float yL, yB;
            Stage1.ProcessBPLP(Params, x, yL, yB);

            // 2. Cascade the second stage (to make LR4 LP)
            float yL2, yB2;
            Stage2.ProcessBPLP(Params, yL, yL2, yB2);

            return yL2;
        }

        [[nodiscard]] JPL_INLINE float ProcessHP(float x) noexcept
        {
            // 1. TPT-SVF stage
            float  yB, yH;
            Stage1.ProcessBPHP(Params, x, yB, yH);

            // 2. Cascade the second stage (to make LR4 LP)
            float yB2, yH2;
            Stage2.ProcessBPHP(Params, yH, yB2, yH2);
            // (for high pass the input to the second stage
            // is high-pass from the first, the rest is the same)
            return yH2;
        }

        // Process one sample through a proper LR4 crossover:
        // returns {low, high} such that low+high == x (exact) and magnitudes are LR4
        JPL_INLINE void Process(float x, float& low, float& high) noexcept
        {
            static constexpr float minusR = -std::numbers::sqrt2_v<float>;
            
            // 1. TPT-SVF stage
            float yL, yB, yH;
            Stage1.Process(Params, x, yL, yB, yH);

            // 2. Cascade the second stage (to make LR4 LP)
            float yL2, yB2, yH2;
            Stage2.Process(Params, yL, yL2, yB2, yH2);

            // Outputs
            low = yL2;
            // high = yL - R * yB + yH - yL2;
            high = Math::FMA(minusR, yB, yL + yH - yL2);
        }
    };

    //==========================================================================
    /// This version of L4 Split works specifically on
    /// the output of the first split (low, high),
    /// interleaved in 4 lanes as { low, high, low, high }
    /// and produces interleaved 4 bands as { b1, b3, b2, b4 }
    /// using 2 vectorized TPT SVF stages and allpass
    /// phase normalization.
    ///
    /// The interleaving is to be able to saturate
    /// 2 vectorized TPT SVF stages, while avoiding many more
    /// reshuffling in the Process block.
    struct LR4Split2Lanes
    {
        TPTSVFStageSIMD Stage1;
        TPTSVFStageSIMD Stage2;

        // 4 allpasses for phase equalization
        // f3 AP on bands 1 & 2
        // f1 AP on bands 3 & 4
        TPTSVFAllpassSIMD AllpassEqualizer;

        // Reusing filter parameters for the two stages
        typename TPTSVFStageSIMD::Params Params;

        // Input split frequencies must be as follows: f1 < f2 < f3,
        // where `f2` is the frequency of the previous splitter,
        // output of which will be fed into this splitter.
        JPL_INLINE void Prepare(float sampleRate, float f1, float f3) noexcept
        {
            // Interleave f1 and f3 to be able to vectorize as much
            // as possible with as little shuffling as possible
            const simd frequency = clamp(simd(f1, f3, f1, f3),
                                         simd(1.0f),
                                         simd(0.49f) * sampleRate);

            Params = TPTSVFStageSIMD::Prepare(sampleRate, frequency);

            // Interleave frequencies according to 2-lane
            // vectorized LR4 splitter output { b1, b3, b2, b4 }:
            // { f3, f1, f3, f1 }
            AllpassEqualizer.Prepare(sampleRate, reverse(frequency));
            // ..reversing `frequency` to reuse the clamp above,
            // however the relationship is not necessarily reversal)

            Reset();
        }

        JPL_INLINE void Reset() noexcept
        {
            Stage1.Reset();
            Stage2.Reset();
            AllpassEqualizer.Reset();
        }

        // Process `low` and `high` from the first split through proper LR4 crossover.
        // @param low : low part of the output from the first middle split
        // @param high : high part of the output from the first middle split
        // @param out_b1324 : output frequency bands, interleaved { b1, b3, b2, b4 }
        JPL_INLINE void ProcessInterleaved(float low, float high, simd& out_b1324) noexcept
        {
            // We SIMDI-fy this as follows:
            //  - Stage 1 works on the two input lanes interleaved (half saturated)
            //      - Stage1(low, high, low, high) -> L and H for f1 and f3
            //  - Stage 2 takes different inputs for each lane: (fully saturated)
            //      - Stage2(L_f1, L_f3, H_f1, H_f3) -> L2 and H2 for f1 and f3
            //
            //  Therefore the biggest SIMD benefit is in the Stage 2,
            //  while Stage 1 gives only half benefit compared to scalar version.

            const simd x(low, high, low, high);

            // yL = { yL_f1, yL_f3, yL_f1, yL_f3 }
            // yH = { yH_f1, yH_f3, yH_f1, yH_f3 }
            simd yL, yB, yH;
            Stage1.Process(Params, x, yL, yB, yH);

            // x2 = { yL_f1, yL_f3, yH_f1, yH_f3 }
            const simd x2 = combine_lo(yL, yH);
            simd yL2, yB2, yH2;
            Stage2.Process(Params, x2, yL2, yB2, yH2);

            // out = { yL2_f1, yL2_f3, yH2_f1, yH2_f3 }
            out_b1324 = combine_lohi(yL2, yH2);
            // (we leave it up to the caller to deinterleave the output if/when necessary)
            
            // Equalize the messed up phase
            //out_b1324 = AllpassEqualizer.Process(out_b1324);
            out_b1324 = AllpassEqualizer.Process(out_b1324);
        }

        // Same as ProcessInterleaved, but followed by deinterleaving
        // the bands for the output, since the caller most likely
        // expects bands in ascending frequency order.
        // @param out_b1234 : output frequency bands in order { b1, b2, b3, b4 }
        JPL_INLINE void Process(float low, float high, simd& out_b1234) noexcept
        {
            ProcessInterleaved(low, high, out_b1234);
            out_b1234 = interleave_lohi(out_b1234);
        }
    };

    //==========================================================================
    /// Split points to Prepare FourBandCrossover
    struct SplitFrequencies
    {
        float F1 = 176.0f, F2 = 775.0f, F3 = 3408.0f;
    };

    //==========================================================================
    /// 4th order Linkwitz-Riley 4-band crossover
    struct FourBandCrossover
    {
        // First splitter at the mid frequency
        LR4Split X2;

        // Combined splitter for the lower and higher
        // parts of the first splitter output
        LR4Split2Lanes X13;

        JPL_INLINE void Prepare(float sampleRate, SplitFrequencies splits = {}) noexcept
        {
            X2.Prepare(sampleRate, splits.F2);
            X13.Prepare(sampleRate, splits.F1, splits.F3);

            Reset();
        }

        JPL_INLINE void Reset() noexcept
        {
            X2.Reset();
            X13.Reset();
        }

        // Process block of float samples and return 4 bands per sample packed into simd lanes
        inline void ProcessBlock(std::span<const float> in, std::span<simd> outBands) noexcept
        {
            JPL_ASSERT(in.size() <= outBands.size());

            for (size_t i = 0; i < in.size(); ++i)
            {
                // Process the first split into `low` and `high` parts
                float low, high;
                X2.Process(in[i], low, high);

                // Process the secod layer splits from `low` and `high`
                // to produce the bands { b1, b2, b3, b4 }
                X13.Process(low, high, outBands[i]);
            }
        }

        // Process block of float samples, apply per band gains and return processed float samples
        inline void ProcessBlock(std::span<const float> in, simd gains, std::span<float> out) noexcept
        {
            // Interleave gains to match the 2-lane
            // vectorized LR4 splitter output { b1, b3, b2, b4 }
            // We do this here once, to avoid interleaving
            // the output of the splitter for each sample iteration.
            // Since we immediately recombine the bands here,
            // their order doesn't matter.
            gains = interleave_lohi(gains);

            for (size_t i = 0; i < in.size(); ++i)
            {
                // Process the first split into `low` and `high` parts
                float low, high;
                X2.Process(in[i], low, high);

#if 0           //? debugging with just single split
                out[i] = low * gains[0] + high * gains[3];
#else
                // Process the secod layer splits from `low` and `high`
                // to produce interleaved bands { b1, b3, b2, b4 }
                simd bands;
                X13.ProcessInterleaved(low, high, bands);

                // Sum up the 4 bands togther into the final output sample
                out[i] = (bands * gains).reduce();
#endif
            }
        }

        // Process block of float samples, apply per band gains with ramp from @startGains to @endGains
        // and return processed float samples
        inline void ProcessBlock(std::span<const float> in,
                                      simd startGains,
                                      simd endGains,
                                      std::span<float> out) noexcept
        {
            // Interleave gains to match the 2-lane
            // vectorized LR4 splitter output { b1, b3, b2, b4 }
            // We do this here once, to avoid interleaving
            // the output of the splitter for each sample iteration.
            // Since we immediately recombine the bands here,
            // their order doesn't matter.
            startGains = interleave_lohi(startGains);
            endGains = interleave_lohi(endGains);
            const simd increment = (endGains - startGains) / static_cast<float>(in.size());

            for (size_t i = 0; i < in.size(); ++i)
            {
                // Process the first split into `low` and `high` parts
                float low, high;
                X2.Process(in[i], low, high);
                // Process the secod layer splits from `low` and `high`
                // to produce interleaved bands { b1, b3, b2, b4 }
                simd bands;
                X13.ProcessInterleaved(low, high, bands);

                // Sum up the 4 bands togther into the final output sample
                out[i] = (bands * startGains).reduce();

                startGains += increment;
            }
        }
    };
} // namespace JPL