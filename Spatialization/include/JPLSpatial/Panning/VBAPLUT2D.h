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
#include "JPLSpatial/ChannelConversion.h"

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec2.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Math/MinimalMat.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Algo/Algorithm.h"
#include "JPLSpatial/Math/DirectionEncoding.h"

#include "JPLSpatial/Panning/VBAPEx.h"

#include <cmath>
#include <limits>
#include <span>
#include <type_traits>
#include <bit>
#include <vector>
#include <cstring> // std::memcpy

namespace JPL::VBAP
{
    //======================================================================
    /// Forward declarations
    template<auto GetSpeakerAngleFunction>
    class LUTBuilder2D;

    //======================================================================
    /// Look-up table containing channel gains for each direction
    class LUT2D
    {

    public:
        //==================================================================
        /* Note on resolution:
            (values in degrees)
            
            at 256:
            - LUT step mean width: 1.40625
            - step variance: 0.288304
            - step min: 0.909371
            - step max: 1.78992
            - step max-min: 0.880548

            at 512:
            - LUT step mean width: 0.703125
            - step variance: 0.144059
            - step min: 0.45112
            - step max: 0.895192
            - step max-min: 0.444072
            
            at 1024:
            - LUT step mean width: 0.351562
            - step variance: 0.072
            - step min: 0.224686 
            - step max: 0.447623 
            - step max-min: 0.222937
            
            at 2048:
            - LUT step mean width: 0.175781
            - step variance: 0.0359921
            - step min: 0.112124 
            - step max: 0.223812 
            - step max-min: 0.111687
        */
        static constexpr struct LUTStats // These values were pre-computed in tests
        {
            static constexpr uint16 Resolution = 1024;
            static constexpr float StepWidth = 0.351562f;
            static constexpr float StepVariance = 0.072f;
            static constexpr float StepMin = 0.224686f;
            static constexpr float StepMax = 0.447623f;
            static constexpr float StepMinMaxGap = StepMax - StepMin;
        } cLUTStats;

        template<class T>
        using Array = std::pmr::vector<T>;

        //==================================================================
        LUT2D() = default;

        [[nodiscard]] JPL_INLINE bool IsInitialized() const noexcept { return !mData.empty(); }

        /// Get gain value at LUT position. Channel stride should be handled by the caller.
        /// 
        /// E.g. to get gain value for channel 3, knowing LUT position (e.g. retrieved by
        /// calling AngleToLUTPosition), `positionInLUT` parameter should be:
        /// AngleToLUTPosition * NumberOfChannels + 3.
        /// 
        /// Channel gain values for each angle are laid out continuously in the LUT.
        /// So to get all the target channel gains for a specific angle, simply get the beginning
        /// of the gain data, e.g.: GetLUTValue(AngleToLUTPosition * NumberOfChannels)
        /// ..and copy the contiguous gains from that position (LFE is inlude if present in
        /// the channel map, though the gain for LFE is always 0.0).
        [[nodiscard]] JPL_INLINE float GetLUTValue(uint32 positionInLUT) const { return mData[positionInLUT]; }

        [[nodiscard]] JPL_INLINE float& operator[](int i) { return mData[i]; }
        [[nodiscard]] JPL_INLINE const float& operator[](int i) const { return mData[i]; }

        /// Size of the LUT = LUT Resolution  * Number of Channels
        [[nodiscard]] JPL_INLINE size_t GetLUTSize() const noexcept { return mData.size(); }

        /// Resolution of the LUT
        [[nodiscard]] JPL_INLINE size_t GetLUTResolution() const noexcept { return mLUTResolution; }

        /// Get LUT position from direction angle, where 0.0 is forward
        /// @param angleNormalized angle in radians in [0, 2Pi]
        [[nodiscard]] JPL_INLINE int AngleNormalizedToLUTPosition(float angleNormalised) const;

        /// Get LUT position from direction angle, where 0.0 is forward
        /// @param angleInRadians in radians in [-Pi, Pi]
        [[nodiscard]] JPL_INLINE int AngleToLUTPosition(float angleInRadians) const;

        /// Convert LUT position to angle (within LUT resolution, irrespective
        /// of number of channels of the channel map)
        [[nodiscard]] JPL_INLINE float LUTPositionToAngle(int pos) const;

        /// Get preprocessed speaker gains at specific LUT posotion
        JPL_INLINE void GetSpeakerGains(int lutPosition, std::span<float> outGains) const;

        /// Get preprocessed speaker gains from LUT based on direction vector
        JPL_INLINE void GetSpeakerGains(const Vec2& direction, std::span<float> outGains) const;

        /// Get preprocessed speaker gains from LUT based on direction vector
        JPL_INLINE void GetSpeakerGains(const simd& dirX, const simd& dirY, std::span<simd> outGains) const;

        /// Get LUT position from direction vector
        [[nodiscard]] JPL_INLINE int CartesianToLUTPosition(float x, float y) const;

        /// Get LUT position from direction vector
        [[nodiscard]] JPL_INLINE simd_mask CartesianToLUTPosition(const simd& x, const simd& y) const;

        /// Build a table of size M_corr that maps
        /// diamond-parameter bins to 'uniform-angle' indices in [0, N_uniform).
        static void BuildDiamondToUniformIndexLUT(Array<uint32>& outCorrectionLUT, uint32 N_uniform, uint32 M_corr = 1024);

        /// Build p -> normalized angle in [0,1)
        /// M_corr: power-of-two size
        static void BuildDiamondToAngleNormLUT(Array<float>& outCorrectionLUT, uint32 M_corr = 1024);

    private:
        void Resize(uint16 resolution, uint32 numTargetChannels);

    private:
        template<auto GetSpeakerAngleFunction>
        friend class LUTBuilder2D;
        Array<float> mData{ GetDefaultMemoryResource() };

        /// Total number of discreet values for 360-degrees field.
        /// The actual size of the LUT data is mLUTResolution * number of output channels.
        uint16 mLUTResolution = 0;
        uint16 mLUTResolutionMask = 0;
        float mInvLUTResolution = 0.0f;
        uint8 mNumTargetChannels = 0;
    };

    //=======================================================================
    /// Interface to query LUT gains for a direction
    class LUTQuery2D
    {
    public:
        JPL_INLINE explicit LUTQuery2D(const LUT2D& lut) noexcept : LUT(lut) {}

        /// Function to query LUT gains for a direction.
        /// @param direction : has to be normalized unit vector
        /// @param outGains : must be of size at least number of target speakers the LUT was built for
        template<CVec3 Vec3Type>
        JPL_INLINE void GainsFor(const Vec3Type& direction, std::span<float> outGains) const
        {
            // Normalize Vec2 we query.
            //! Note: this is needed unless we use atan2 path for scalar query.
            Vec3Type dir(direction);
            SetY(dir, 0.0f);
            Normalize(dir);

            LUT.GetSpeakerGains({
                static_cast<float>(GetX(dir)),
                static_cast<float>(GetZ(dir)) },
                outGains);
        }

        /// Function to query LUT gains for a direction.
        /// @param direction : has to be normalized unit vector
        /// @param outGains : must be of size at least number of target speakers the LUT was built for
        JPL_INLINE void GainsFor(const simd& dirX, const simd& dirY, const simd& dirZ, std::span<simd> outGains) const
        {
            LUT.GetSpeakerGains(
                dirX,
                dirZ,
                outGains);
        }

    public:
        const LUT2D& LUT;
    };

    //======================================================================
    /// Interface to contsruct LUTBuilder and LUTQuery without having to
    /// retype long template parameter lists
    template<auto GetSpeakerVectorFunction, auto GetSpeakerAngleFunction>
    class LUTInterface2D
    {
    public:
        using Vec3Type = std::remove_cvref_t<decltype(GetSpeakerVectorFunction(EChannel{}))>;
        using LUTType = LUT2D;
        using BuilderType = LUTBuilder2D<GetSpeakerAngleFunction>;
        using QueryType = LUTQuery2D;

        //======================================================================
        /// Make LUTBuilder object to build LUT for given 'channelMap' and 'LUTType'
        [[nodiscard]] static JPL_INLINE BuilderType MakeBuilder(ChannelMap channelMap, LUT2D& lut)
        {
            return BuilderType(channelMap, lut);
        }

        /// Make LUTQuery object to query 'LUT' for speaker gains
        [[nodiscard]] static JPL_INLINE QueryType Query(const LUT2D& LUT)
        {
            return QueryType(LUT);
        }
    };

    //======================================================================
    /// Helper class to build a LUT for a set of directions and indices
    template<auto GetSpeakerAngleFunction>
    class LUTBuilder2D
    {
        template<class T>
        using Array = std::pmr::vector<T>;
        using ChannelAngleArray = Array<ChannelAngle>;
        using LUTType = LUT2D;

    public:
        LUTBuilder2D(ChannelMap channelMap, LUTType& LUT);

        /// If target channel map has < 4 speakers, we use intermediary
        /// quad map to compute the gains, which are then converte
        /// to target map and stored in the LUT
        [[nodiscard]] JPL_INLINE bool RequiresChannelConversion() const noexcept { return mChannelMapInternal != mChannelMapTarget; }
        
        /// Find shortest aperture between two speakers of the target map
        [[nodiscard]] float FindShortestAperture() const;
        
        /// Comput LUT gains for given direction and LUT offset
        [[nodiscard]] bool ComputeCellFor(const Vec2& direction, int lutOffset);

        /// Build the entire LUT for all directions
        [[nodiscard]] bool BuildForAllDirections();

#if JPL_VALIDATE_VBAP_LUT
        void ValidateLUT() const;
#endif

    private:
        void ComputePairMatrices();

        // 'ThisType' is to deduce constness and avoid two otherwise identical member functions.
        // "Deducing this" is not available until C++23.
        template<class ThisType, class CallbackType>
        static void ForEachChannelAnglePair(ThisType& self, CallbackType&& callback);

        // If target channel map requires conversion (i.e. if it has < 4 channels, and we use intermediary quad map),
        // then we call this function to convert and store converted gains in the LUT
        JPL_INLINE void StoreChannelGainsConverted(uint32 channelId1, uint32 channelId2, const Vec2& gains, uint32 lutOffset);

        void ApplyChannelConversion(const std::pair<uint32, uint32>& inChannelIds, const Vec2& inGains, std::span<float> outValues) const;

    private:
        LUT2D& mLUT;

        ChannelMap mChannelMapInternal;
        ChannelMap mChannelMapTarget;

        uint32 mNumInternalChannels;
        uint32 mNumTargetChannels;
        uint32 mLFEIndex;

        Array<ChannelAngle> mChannelAngels{ GetDefaultMemoryResource() };

        ChannelConversionWeights mChannelConversionWeights;

        struct ChannelPair
        {
            Math::Mat2<Vec2> invL;
            uint32 ChannelId1;
            uint32 ChannelId2;
        };

        Array<ChannelPair> mChannelPairs{ GetDefaultMemoryResource() };
    };
} // namespace JPL::VBAP

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================
namespace JPL::VBAP
{
    //==========================================================================
    JPL_INLINE int LUT2D::AngleNormalizedToLUTPosition(float angleNormalised) const
    {
        JPL_ASSERT(IsInitialized());

        return static_cast<int>(
            ToDiamond({ std::sinf(angleNormalised), -std::cosf(angleNormalised) })
            * mLUTResolution + 0.5f
            ) & mLUTResolutionMask;
    }

    JPL_INLINE int LUT2D::AngleToLUTPosition(float angleInRadians) const
    {
        // Normalize to [0, 2Pi]
        if (angleInRadians < float(0.0))
            angleInRadians += JPL_TWO_PI;
        return AngleNormalizedToLUTPosition(angleInRadians);
    }

    JPL_INLINE int LUT2D::CartesianToLUTPosition(float x, float y) const
    {
#if 1 // The LUT built with uniform steps of diamond encoding
        const float diamond = ToDiamond(Vec2(x, y));
        return static_cast<int>(Math::FMA(static_cast<float>(mLUTResolution), diamond, 0.5f)) & mLUTResolutionMask;
#elif 0
        JPL_ASSERT(!mDiamondCorrectionLUT.empty());

        // Diamond to uniform index
        const float diamond = ToDiamond(Vec2(x, y));
        const auto angleT = static_cast<int>(diamond * sCorrectionLUTSize + 0.5f) & sCorrectionLUTMask;

        //return static_cast<int>(diamond * mLUTResolution + 0.5f) & mLUTResolutionMask;
        return mDiamondCorrectionLUT[angleT];

#if 0   // If correction LUT is just uniforming the angles, not directly to target indices
        // normalized angle in [0,1)
        const float uniformAngleT = mDiamondCorrectionLUT[angleT];

        // final uniform-angle index in [0, mLUTResolution)
        return static_cast<int>(uniformAngleT * mLUTResolution + 0.5f) & mLUTResolutionMask;
#endif

#else
        // Angle in [-Pi, Pi]
        const float angle = std::atan2(x, z);
        return AngleToLUTPosition(angle);
#endif
    }

    JPL_INLINE simd_mask LUT2D::CartesianToLUTPosition(const simd& x, const simd& y) const
    {
        //! For diamond encoding we need to normalize Vec2(x, y) direction
        const simd invLen = Math:: InvSqrtFast(x * x + y * y);
        const simd xN = x * invLen;
        const simd yN = y * invLen;

        const simd diamond = ToDiamond(xN, yN);
        return FMA(diamond, simd(mLUTResolution), 0.5f).to_mask() & mLUTResolutionMask;
    }

    inline void LUT2D::Resize(uint16 resolution, uint32 numTargetChannels)
    {
        // We should not have more than 255 channels
        JPL_ASSERT(numTargetChannels <= std::numeric_limits<uint8>::max());

        // for the sake of sanity
        resolution = std::bit_ceil(resolution);

        mData.clear();
        mData.resize(numTargetChannels * resolution, 0.0f);

        mLUTResolution = resolution;
        mLUTResolutionMask = mLUTResolution - 1;
        mInvLUTResolution = 1.0f / mLUTResolution;
        mNumTargetChannels = static_cast<uint8>(numTargetChannels);
    }

    JPL_INLINE auto LUT2D::LUTPositionToAngle(int pos) const -> float
    {
        const Vec2 direction = FromDiamond((static_cast<float>(pos) * static_cast<float>(mInvLUTResolution)));
        float angle = std::atan2f(direction.Y, direction.X);
        if (angle < 0.0f)
            angle += JPL_TWO_PI;
        return angle;
    }

    JPL_INLINE void LUT2D::GetSpeakerGains(int lutPosition, std::span<float> outGains) const
    {
        JPL_ASSERT(outGains.size() <= mNumTargetChannels);

        const float* speakerGain = &mData[mNumTargetChannels * lutPosition];
        std::memcpy(outGains.data(), speakerGain, sizeof(float) * outGains.size());
    }

    JPL_INLINE void LUT2D::GetSpeakerGains(const Vec2& direction, std::span<float> outGains) const
    {
        GetSpeakerGains(CartesianToLUTPosition(direction.X, direction.Y), outGains);
    }

    JPL_INLINE void LUT2D::GetSpeakerGains(const simd& dirX, const simd& dirY, std::span<simd> outGains) const
    {
        // For each speaker we compute 4 directions.
        // We can vectorize the encoding of the direction into LUT index,
        // however we have to retrieve the channel gains individually per direction index,
        // since the location for each simd lane in LUT differs.

        uint32 lutPositions[4];
        (CartesianToLUTPosition(dirX, dirY) * mNumTargetChannels).store(lutPositions);

        static constexpr std::size_t bufferSize = 32 * simd::size();
        JPL_ASSERT(bufferSize >= outGains.size() * simd::size());

        float buffer[bufferSize];
        const uint32 offsets[]{
            0,
            mNumTargetChannels,
            static_cast<uint32>(mNumTargetChannels) << 1,
            (static_cast<uint32>(mNumTargetChannels) << 1) + mNumTargetChannels
        };

        // Retrieve gains from the LUT and write contiguously
        // d1[ch1, ch2], dr2[ch1, ch2]...
        for (uint32 i = 0, dest = 0; i < simd_mask::size(); ++i, dest += mNumTargetChannels)
        {
            std::memcpy(&buffer[dest], &mData[lutPositions[i]], sizeof(float) * mNumTargetChannels);
        }

        // Copy gains from the buffer strided into out simd lanes
        for (uint32 si = 0; si < outGains.size(); ++si)
        {
            // dr1[ch1], dr2[ch1], dr3[ch1], dr4[ch1]
            outGains[si] = simd(buffer[si], buffer[si + offsets[1]], buffer[si + offsets[2]], buffer[si + offsets[3]]);
        }
    }

    inline void LUT2D::BuildDiamondToUniformIndexLUT(Array<uint32>& outCorrectionLut, uint32 N_uniform, uint32 M_corr /*= 1024*/)
    {
        // Require power-of-two sizes for cheap masking
        auto isPow2 = [](uint32 n) { return n && ((n & (n - 1)) == 0); };
        JPL_ASSERT(isPow2(N_uniform) && "N_uniform must be power-of-two");
        JPL_ASSERT(isPow2(M_corr) && "M_corr must be power-of-two");

        outCorrectionLut.resize(M_corr);

        const float invM_Corr = 1.0f / M_corr;

        for (uint32 j = 0; j < M_corr; ++j)
        {
            // diamond parameter for this table cell
            const float p = static_cast<float>(j) * invM_Corr;

            // decode to unit vector on circle
            const Vec2 v = FromDiamond(p);

            // recover true polar angle in [theta, 2PI)
            float theta = std::atan2(v.X, v.Y);
            if (theta < 0.0f)
                theta += JPL_TWO_PI;

            // map to our uniform-angle LUT index
            const float t = (theta * JPL_INV_TWO_PI) * static_cast<float>(N_uniform);
            const auto idx = static_cast<uint32>(std::llround(t)) & (N_uniform - 1);

            outCorrectionLut[j] = idx;
        }
    }

    inline void LUT2D::BuildDiamondToAngleNormLUT(Array<float>& outCorrectionLUT, uint32 M_corr)
    {
        auto isPow2 = [](uint32_t n) { return n && ((n & (n - 1)) == 0); };
        JPL_ASSERT(isPow2(M_corr));

        outCorrectionLUT.resize(M_corr);

        const float invM_Corr = 1.0f / M_corr;

        for (uint32_t j = 0; j < M_corr; ++j)
        {
            // diamond parameter for this table cell
            const float p = static_cast<float>(j) * invM_Corr;

            // decode to unit vector on circle
            const Vec2 v = FromDiamond(p);

            float theta = std::atan2(v.X, v.Y); // (-PI, PI]
            if (theta < 0.0f)
                theta += JPL_TWO_PI; // [0, 2PI)

            outCorrectionLUT[j] = theta * JPL_INV_TWO_PI; // normalized angle
        }
    }


#if 0
    JPL_INLINE float CircularLerp01(float a, float b, float t)
    {
        // both in [0,1); treat 1 as 0 on the circle
        float d = b - a;
        if (d > 0.5f) d -= 1.0f;
        if (d < -0.5f) d += 1.0f;
        float u = a + t * d;
        if (u < 0.0f) u += 1.0f;
        if (u >= 1.0f) u -= 1.0f;
        return u;
    }
#endif

    //==========================================================================
    template<auto GetSpeakerAngleFunction>
    inline LUTBuilder2D<GetSpeakerAngleFunction>::LUTBuilder2D(ChannelMap channelMap, LUTType& LUT)
        : mLUT(LUT)
    {
        mChannelMapTarget = channelMap;
        mNumTargetChannels = channelMap.GetNumChannels();

        // We need at least quad layout for VBAP to work
        mChannelMapInternal = channelMap.GetNumChannels() < 4 ? ChannelMap::FromChannelMask(ChannelMask::Quad) : channelMap;
        mNumInternalChannels = mChannelMapInternal.GetNumChannels();
        mLFEIndex = mChannelMapInternal.GetChannelIndex(EChannel::LFE);

        // Extract sortet speaker angles
        static constexpr bool skipLFE = false;
        VBAP::ChannelAngle::GetSortedChannelAngles(mChannelMapInternal, mChannelAngels, GetSpeakerAngleFunction, skipLFE);
        JPL_ASSERT(mChannelAngels.size() == mNumInternalChannels);

        //JPL_ASSERT(intermNumChannels <= Traits::MAX_CHANNELS);

        ComputePairMatrices();

        if (RequiresChannelConversion())
        {
            mChannelConversionWeights.Resize(mNumTargetChannels, mNumInternalChannels);
            ComputeChannelConversionRectangularWeights(mChannelMapInternal, mChannelMapTarget, mChannelConversionWeights);
        }

        mLUT.Resize(LUTType::LUTStats::Resolution, mNumTargetChannels);
    }

    template<auto GetSpeakerAngleFunction>
    inline float LUTBuilder2D<GetSpeakerAngleFunction>::FindShortestAperture() const
    {
        float shortestAperture = std::numeric_limits<float>::max();

        const uint32 lastSpeakerId = mChannelAngels.back().ChannelId;

        auto findShortestAperture = [&](const ChannelAngle& cha1, const ChannelAngle& cha2)
        {
            if (cha1.ChannelId == lastSpeakerId)
                shortestAperture = std::min(shortestAperture, JPL_TWO_PI - cha1.Angle + cha2.Angle);
            else
                shortestAperture = std::min(shortestAperture, cha2.Angle - cha1.Angle);
        };

        ForEachChannelAnglePair(*this, findShortestAperture);

        return shortestAperture;
    }

    template<auto GetSpeakerAngleFunction>
    inline bool LUTBuilder2D<GetSpeakerAngleFunction>::ComputeCellFor(const Vec2& direction, int lutOffset)
    {
        // Assign speaker contribution values
        for (const ChannelPair& pair : mChannelPairs)
        {
            Vec2 gains = pair.invL.Transform(direction);

            if (gains.X < 0.0f || gains.Y < 0.0f)
                continue; // not our pair

            gains.Normalize();

            if (RequiresChannelConversion())
            {
                StoreChannelGainsConverted(pair.ChannelId1, pair.ChannelId2, gains, lutOffset);
            }
            else
            {
                //! Note: we could use indices here instead of ChannelIds
                //! if we'd want to skip LFE in our table
                mLUT.mData[lutOffset + pair.ChannelId1] = gains.X;
                mLUT.mData[lutOffset + pair.ChannelId2] = gains.Y;
            }

            return true;
        }

        // This should be unreachable
        JPL_ASSERT(false);
        return false;
    }

    template<auto GetSpeakerAngleFunction>
    inline bool LUTBuilder2D<GetSpeakerAngleFunction>::BuildForAllDirections()
    {
        bool bAnyFailed = false;

        // Build a LUT with uniform diamond steps
        const float step = 1.0f / mLUT.GetLUTResolution();
        float diamond = 0.0f;

        for (uint32 pos = 0; pos < mLUT.GetLUTResolution(); ++pos, diamond += step)
        {
            JPL_ASSERT(diamond <= 1.0f);

            const Vec2 direction = FromDiamond(diamond);

            // Position of the next diamond step value in the LUT
            // (number of channels stride)
            const uint32 offset = mNumTargetChannels * pos;

            // TODO: we may or may not want to terminate if any cell fails
            bAnyFailed |= ComputeCellFor(direction, offset);
        }

        return bAnyFailed;
    }

    template<auto GetSpeakerAngleFunction>
    inline void LUTBuilder2D<GetSpeakerAngleFunction>::ComputePairMatrices()
    {
        auto makeInvMat = [&](const ChannelAngle& cha1, const ChannelAngle& cha2)
        {
            //JPL_ASSERT(Math::IsPositiveAndBelow(cha2.Angle - cha1.Angle, JPL_PI + 1e-6f));

            const auto [s1, c1] = Math::SinCos(cha1.Angle);
            const auto [s2, c2] = Math::SinCos(cha2.Angle);

            const auto mat =
                Math::Mat2<Vec2>::FromColumns(
                    Vec2(s1, -c1),
                    Vec2(s2, -c2));

            // Inverse will fail for wide aperture angles,
            // therefore we need at least quad layout for <= 90 degree max aperture
            Math::Mat2<Vec2> invL;
            (void)JPL_ENSURE(mat.TryInverse(invL));

            mChannelPairs.emplace_back(
                invL,
                cha1.ChannelId,
                cha2.ChannelId);
        };

        ForEachChannelAnglePair(*this, makeInvMat);
    }

    template<auto GetSpeakerAngleFunction>
    template<class ThisType, class CallbackType>
    inline void LUTBuilder2D<GetSpeakerAngleFunction>::ForEachChannelAnglePair(ThisType& self,
                                                                                                   CallbackType&& callback)
    {
        auto sanitizeLFEIndex = [&self](uint32 idx)
        {
            return idx + (self.mChannelAngels[idx].ChannelId == self.mLFEIndex);
        };

        for (uint32 ch = 0; ch < self.mChannelAngels.size() - 1; ++ch)
        {
            const uint32 ch1 = sanitizeLFEIndex(ch);
            const uint32 ch2 = sanitizeLFEIndex(ch1 + 1);

            const ChannelAngle& cha1 = self.mChannelAngels[ch1];
            const ChannelAngle& cha2 = self.mChannelAngels[ch2];
            callback(cha1, cha2);
        }

        // Handle wrap around
        {
            const uint32 ch1 = static_cast<uint32>(self.mChannelAngels.size()) - 1;
            const uint32 ch2 = sanitizeLFEIndex(0);
            const ChannelAngle& cha1 = self.mChannelAngels[ch1];
            const ChannelAngle& cha2 = self.mChannelAngels[ch2];
            callback(cha1, cha2);
        }
    }

    template<auto GetSpeakerAngleFunction>
    JPL_INLINE void LUTBuilder2D<GetSpeakerAngleFunction>::StoreChannelGainsConverted(uint32 channelId1,
                                                                                                          uint32 channelId2,
                                                                                                          const Vec2& gains,
                                                                                                          uint32 lutOffset)
    {
        std::span<float> targetGains(&mLUT.mData[lutOffset], mNumTargetChannels);

        // Convert from intermediary to target
        // channel map that we store in the LUT
        ApplyChannelConversion({ channelId1, channelId2 }, gains, targetGains);

        // Normalize
        //! This may or may not be desireable
        Algo::NormalizeL2(targetGains);
    }

    template<auto GetSpeakerAngleFunction>
    inline void LUTBuilder2D<GetSpeakerAngleFunction>::ApplyChannelConversion(const std::pair<uint32, uint32>& inChannelIds,
                                                                                                  const Vec2& inGains,
                                                                                                  std::span<float> outValues) const
    {
        for (uint32 iChannelOut = 0; iChannelOut < outValues.size(); ++iChannelOut)
        {
            const float accumulation =
                inGains.X * mChannelConversionWeights[iChannelOut][inChannelIds.first] +
                inGains.Y * mChannelConversionWeights[iChannelOut][inChannelIds.second];

            outValues[iChannelOut] = accumulation;
        }
    }

#if JPL_VALIDATE_VBAP_LUT
    template<auto GetSpeakerAngleFunction>
    void LUTBuilder2D<GetSpeakerAngleFunction>::ValidateLUT() const
    {
        
#if 0   // TODO: implement for 2D

        JPL_ASSERT(mLUT != nullptr);

        for (uint32 i = 0; i < mLUT->Speakers.size(); ++i)
        {
            if (!LUTCodec::IsValidCode(i))
                continue;

            const auto& speakers = mLUT->Speakers[i];
            JPL_ASSERT(speakers[0] != speakers[1]);
            JPL_ASSERT(speakers[1] != speakers[2]);
            JPL_ASSERT(speakers[2] != speakers[0]);
        }

        for (uint32 i = 0; i < mLUT->Gains.size(); ++i)
        {
            if (!LUTCodec::IsValidCode(i))
                continue;

            const auto& gains = mLUT->Gains[i];
            // Gains can be encoded in 24 or even 16 bit
            const std::array<float, 3> gainsDecoded
            {
                gains[0],
                gains[1],
                gains[2]
            };
            JPL_ASSERT(Algo::IsNormalizedL2(gainsDecoded));
        }
#endif
    }
#endif

} // namespace JPL::VBAP
