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
#include "JPLSpatial/PathTracing/BDTPDefinitions.h"
#include "JPLSpatial/Utilities/Variance.h"

#include <vector>
#include <ranges>

namespace JPL
{
    //==========================================================================
    /// Sample allocation strategy for Monte-Carlo path tracing
    struct BDPTSampleAllocation
    {
        // Estimation quality indicator,
        // essentially number of samples used for estimation.
        // Larger value will have less variance.
        int cQualityStandart = 100;

        // Initialization parameters
        int SampleBudget;
        int MaxBounces;
        int NumSampleBins;

#if 0
        struct IntegralInfo
        {
            float SampleBudgetStake;
            int AllocatedSamples;
        };
        std::vector<IntegralInfo> mIntegralInfos; // [numBounceOrders]

        struct PerIntegralBinInfo
        {
            float VarianceEstimation;
            int SampleCount;
            OnlineVariance SampleVariance;
            int SamplesTraced;
        };

        struct BinInfo
        {
            std::vector<PerIntegralBinInfo> PerIntegralInfos; // [numBounceOrders]
        };
        std::vector<BinInfo> mSampleBins;          // [numSampleBins]
#endif
        // TODO: flatten these 2D vectors

        // Updated between traces
        std::vector<float> mSampleProbability_xn;                   // [numBounceOrders] How much of or sample budget goes to each bounce order
        std::vector<std::vector<float>> mVarianceEstimation_smn;    // [numSampleBins][numBounceOrders]
        std::vector<std::vector<int>> mSampleCount_Qmn;             // [numSampleBins][numBounceOrders] How many samples to trace

        // Cleared for each trace
        std::vector<std::vector<int>> mSamplesTraced;                      // [numSampleBins][numBounceOrders] How many samples actually traced
        std::vector<std::vector<OnlineVariance>> mSampleVariance;    // [numSampleBins][numBounceOrders] Variance counter
        std::vector<int> mAllocatedSamples;                                     // [numBounceOrders] Number of connections to evaluate for each bounce order

        //? technically not used, we use acutal path candidates per integral for nomralization
        std::vector<float> mAllocatedSamplesInv;                                // [numBounceOrders] Inverse of mAllocatedSamples for normalization purposes

        /// Allocate internal buffers for the requested budget
        void Initialize(int totalSamples, int maxBounces, int numSampleBins);

        /// Allocate samples for a new trace. Reset the last trace counters.
        /// Must be called before starting a new trace.
        /// (no memory actually allocated)
        void Allocate();

        /// Update variance and optimize sample distribution for energy response
        /// Must be called after each trace to update the sample distribution
        /// @param a - optimization factor, typically in range [0.4, 0.7]
        void UpdateOptimize(float a /*0.4-0.7*/);

        bool HasSamplesFor(int bounceIndex) const { return mAllocatedSamples[bounceIndex] > 0; }

        JPL_INLINE void RecordContribution(int sampleBin, uint32 bounceIndex, float contribution)
        {
            mSamplesTraced[sampleBin][bounceIndex]++;
            mSampleVariance[sampleBin][bounceIndex].Add(contribution);
        }

        JPL_INLINE void UseSample(uint32 bounceIndex)
        {
            JPL_ASSERT(mAllocatedSamples[bounceIndex] > 0);
            mAllocatedSamples[bounceIndex]--;
        }

        //? not used
        /// Utility to tick all the counters for sample allocation.
        class Sample
        {
            BDPTSampleAllocation& mAllocation;
            int mBounceIndex;

            friend struct BDPTSampleAllocation;
            Sample(BDPTSampleAllocation& allocation, int bounceIndex)
                : mAllocation(allocation), mBounceIndex(bounceIndex)
            {}
        public:
            ~Sample() { UseSample(); };

            void RecordContribution(int sampleBin, float contribution)
            {
                mAllocation.mSamplesTraced[sampleBin][mBounceIndex]++;
                mAllocation.mSampleVariance[sampleBin][mBounceIndex].Add(contribution);
            }

            // Check if the sample is valid.
            operator bool() const { return mAllocation.HasSamplesFor(mBounceIndex); }

        private:
            void UseSample()
            {
                if (mAllocation.HasSamplesFor(mBounceIndex))
                    mAllocation.mAllocatedSamples[mBounceIndex]--;
            }
        };

        /// Get a sample for the given bounce index. Sample could be invalid.
        Sample GetSampleFor(int bounceIndex) { return Sample(*this, bounceIndex); }

    private:
        /// Reset trace counts.
        /// Called in Allocate()
        void ResetTrace();

        /// Update sample variance based on the last trace and quality factor. (Eq. 18)
        void UpdateVariance();

        /// Optimize the sample distribution based on the sample variance. (Eq. 17)
        /// This iteration is executed once during each frame,
        /// and the resultant distribution of the previous frame
        /// is used as the initial value for the current frame.
        void OptimizeEnergyResponse(float a /*0.4-0.7*/);

        void DebugPrintAllocation();
    };
} // namespace JPL

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

namespace JPL
{
    //==========================================================================
    inline void BDPTSampleAllocation::Initialize(int totalSamples, int maxBounces, int numSampleBins)
    {
        SampleBudget = totalSamples;
        MaxBounces = maxBounces;
        NumSampleBins = numSampleBins;

        //! Testing even distribution of the sample bugdet per bins
        cQualityStandart = SampleBudget / NumSampleBins;

        LOG_TMP("cQualityStandart (" << cQualityStandart << ")");

        // Contribution from the n-th integral to the m-th sample bin is Xmn,
        // TODO: do we need to track that?

        const int numIntegrals_N = MaxBounces;
        const int numSampleBins_M = NumSampleBins;

        // Initial sample allocation: equally among all bounce orders.
        const float initialSampleProbability = 1.0f / numIntegrals_N;

        // The sample probability for the n-th integral (n-th bounce), `xn`
        mSampleProbability_xn.resize(numIntegrals_N, initialSampleProbability);

        // The variance of a single sample contribution
        // from the n-th integral to the m-th bin
        mVarianceEstimation_smn.resize(numSampleBins_M, std::vector<float>(numIntegrals_N, 0.0f));

        mSampleVariance.resize(numSampleBins_M, std::vector<OnlineVariance>(numIntegrals_N));

        // Quality estimation, i.e. num samples per bin per integral
        mSampleCount_Qmn.resize(numSampleBins_M, std::vector<int>(numIntegrals_N, 0));
        mSamplesTraced.resize(numSampleBins_M, std::vector<int>(numIntegrals_N, 0));

        mAllocatedSamples.resize(numIntegrals_N, 0);
        mAllocatedSamplesInv.resize(numIntegrals_N, 1.0f);
    }

    inline void BDPTSampleAllocation::Allocate()
    {
        JPL_ASSERT(SampleBudget >= MaxBounces);

        // TODO: here we might get issues where the last integral/bounce gets a tiny amount of budget
        //      for the initial allocation, the `distribute()` function below would fix it


#if 0 // TODO: move to some math utility header, if useful for anything
        auto distribute = [](uint32 total, uint32 numBins, std::ranges::random_access_range auto& bins)
        {
            const uint32 base = total / numBins;
            const uint32 remainder = total % numBins;

            for (uint32 i = 0; i < numBins; ++i)
                bins[i] = base;

            for (uint32 i = 0; i < remainder; ++i)
                bins[i] += 1;
        };
#endif

        int allocated = 0;
        for (int i = 0; i < MaxBounces - 1; ++i)
        {
            const auto numToAllocate = static_cast<int>(mSampleProbability_xn[i] * SampleBudget);
            mAllocatedSamples[i] = numToAllocate;
            allocated += numToAllocate;
        }

        // Allocate all remaining samples to the last bounce order
        mAllocatedSamples[MaxBounces - 1] =
            std::min(static_cast<int>(mSampleProbability_xn[MaxBounces - 1] * SampleBudget),
                     SampleBudget - allocated);

        // Store inverse N for normalization purposes
        for (auto i = 0; i < mAllocatedSamples.size(); ++i)
            mAllocatedSamplesInv[i] = 1.0f / mAllocatedSamples[i];

        //DebugPrintAllocation();

        ResetTrace();
    }

    inline void BDPTSampleAllocation::ResetTrace()
    {
        std::ranges::for_each(mSampleVariance, [](std::vector<OnlineVariance>& v)
        {
            std::ranges::fill(v, OnlineVariance{});
        });

        std::ranges::for_each(mSamplesTraced, [](std::vector<int>& n)
        {
            std::ranges::fill(n, 0);
        });
    }

    inline void BDPTSampleAllocation::UpdateOptimize(float a /*0.4-0.7*/)
    {
        UpdateVariance();
        OptimizeEnergyResponse(a);
    }

    inline void BDPTSampleAllocation::OptimizeEnergyResponse(float a /*0.4-0.7*/)
    {
        JPL_ASSERT(a > 0.0f && a < 1.0f);

        // To avoid division by zero, xn0 must be all positive
        // and there must be at least one σmn > 0 for every m and n.
        // Sample bins and integrals that cannot satisfy
        // this restriction can be ignored without any consequence.

        const int numSampleBins_M = NumSampleBins;
        const int numIntegrals_N = MaxBounces;

        for (int n = 0; n < numIntegrals_N; ++n)
        {
            if (mSampleProbability_xn[n] <= 0.0f)
                continue;

            // NOTE: Paper uses 1-based indices here,
            // but we use 0-based indexing for consistency.

            float m_sum = 0.0f;
            for (int m = 0; m < numSampleBins_M; ++m)
            {
                if (mVarianceEstimation_smn[m][n] <= 0.0f)
                    continue;

                float k_sum = 0.0f;
                for (int k = 0; k < numIntegrals_N; ++k)
                {
                    if (mSampleProbability_xn[k] <= 0.0f)
                        continue;

                    k_sum += mVarianceEstimation_smn[m][k] / mSampleProbability_xn[k];
                }

                m_sum += (mVarianceEstimation_smn[m][n] / mSampleProbability_xn[n]) / k_sum;
            }

            const float xn_i1 = (a / numSampleBins_M) * m_sum + (1.0f - a) * mSampleProbability_xn[n];

            mSampleProbability_xn[n] = xn_i1;
        }

        // Normalize for sanity
        const float sum = std::reduce(mSampleProbability_xn.begin(),
                                      mSampleProbability_xn.end());
        if (sum > 0.0f)
        {
            const float invSum = 1.0f / sum;
            for (int n = 0; n < numIntegrals_N; ++n)
                mSampleProbability_xn[n] *= invSum;
        }
        else
        {
            // If for some reason all probabilities are 0,
            // assign equal probability to all integrals
            const float invN = 1.0f / numIntegrals_N;
            for (int n = 0; n < numIntegrals_N; ++n)
                mSampleProbability_xn[n] = invN;
        }
    }

    inline void BDPTSampleAllocation::DebugPrintAllocation()
    {
#if JPL_DEBUG_PRINT
        static int frame = 0;
        LOG_TMP("--- Allocattion, frame " << frame++ << " ---");

        for (auto i = 0; i < mAllocatedSamples.size(); ++i)
			LOG_TMP("   N[" << i << "] : " << mAllocatedSamples[i] << " | " << mSampleProbability_xn[i] << "");
#endif
    }

    inline void BDPTSampleAllocation::UpdateVariance()
    {
        // (Eq. 19)
        auto updateQualityIndicator = [Q_star = cQualityStandart](int Q0, int Q_prev)
        {
            // Q_star is predefined quality standard
            // (init to a "reasonable" value),
            // larger Q will have less variance)

            if (Q0 > Q_star)
            {
                return Q0;
            }
            else if (Q_prev + Q0 < Q_star)
            {
                return Q0 + Q_prev;
            }
            else
            {
                return Q_star;
            }
        };

        auto updateVarianceEst = [Q_star = cQualityStandart](int Q0, // Q of the corrent frame 
                                                             int Q_prev, // Q of the previous frame
                                                             float sigma_0, // variance of the current frame
                                                             float sigma_prev) // variance of the previous frame
        {
            if (Q0 > Q_star)
            {
                return sigma_0;
            }
            else if (Q_prev + Q0 < Q_star)
            {
                return (Q_prev * sigma_prev + Q0 * sigma_0) / float(Q_prev + Q0);
            }
            else
            {
                // Calculate gamma as per Eq. 20
                float gamma = 0.0f;

                if (Q_prev == Q_star)
                {
                    gamma = std::min(1.0f, (Q_star - Q0) / float(Q_star + Q0));
                }
                else
                {
                    const int Q_prev_and_cur = Q_prev + Q0;
                    gamma = (Q_prev - std::sqrt(Q_prev * Q0 * (Q_prev_and_cur / float(Q_star) - 1))) / Q_prev_and_cur;
                }

                return gamma * sigma_prev + (1.0f - gamma) * sigma_0;
            }
        };

        const int numSampleBins_M = NumSampleBins;
        const int numIntegrals_N = MaxBounces;

        for (int m = 0; m < numSampleBins_M; ++m)
        {
            for (int n = 0; n < numIntegrals_N; ++n)
            {
                const int Q_prev = mSampleCount_Qmn[m][n];
                const int Q0 = mSamplesTraced[m][n]; // updated during or after tracing

                if ((Q_prev + Q0) == 0)
                {
                    // Skip estimation if we havent yet traced any samples for [m][n]
                    mVarianceEstimation_smn[m][n] = 0.0f;
                    continue;
                }

                mSampleCount_Qmn[m][n] = updateQualityIndicator(Q0, Q_prev); // Eq. 19

                const float sigma_0 = mSampleVariance[m][n].GetVariance(); // updated during or after tracing
                const float sigma_prev = mVarianceEstimation_smn[m][n];

                mVarianceEstimation_smn[m][n] = updateVarianceEst(Q0, Q_prev, sigma_0, sigma_prev);
            }
        }
    }
} // namespace JPL