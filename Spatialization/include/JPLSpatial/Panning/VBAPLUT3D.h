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
#include "JPLSpatial/Panning/VBAPEx.h"
#include "JPLSpatial/Panning/DummySpeakers.h"
#include "JPLSpatial/Math/MinimalMat.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Utilities/GainEncoding.h"

#include <array>
#include <cstring>
#include <vector>
#include <span>
#include <concepts>
#include <type_traits>
#include <algorithm>
#include <memory>

namespace JPL::VBAP
{
    /// VBAP LUT interfaces only accept LUT types defined below
    template<class T>
    concept CLUT = requires { typename T::GainType; }&& requires { typename T::SpeakerIndexType; };

    //==========================================================================
    /// Forward declarations
    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    class LUTBuilder;

    template<CLUT LUTType, class LUTCodec>
    class LUTQuery;

    //==========================================================================
    /// Interface to contsruct LUTBuilder and LUTQuery without having to
    /// retype long template parameter lists
    template<auto GetSpeakerVectorFunction, class LUTCodec, CLUT LUTType>
    class LUTInterface
    {
    public:
        using Vec3Type = std::remove_cvref_t<decltype(GetSpeakerVectorFunction(EChannel{}))>;
        
        using BuilderType = LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>;
        using QueryType = LUTQuery<LUTType, LUTCodec>;

        //======================================================================
        /// Make LUTBuilder object to build LUT for given 'channelMap' and 'LUTType'
        [[nodiscard]] static JPL_INLINE BuilderType MakeBuilder(ChannelMap channelMap, LUTType& lut)
        {
            return BuilderType(channelMap, lut);
        }

        /// Make LUTQuery object to query 'LUT' for speaker gains
        [[nodiscard]] static JPL_INLINE QueryType Query(const LUTType& LUT)
        {
            return QueryType(LUT);
        }
    };

    //==========================================================================
    /// LUT type selector parameter based on approximate size of the static data.
    /// Sizes are approximate for Octahedron16Bit encoding,
    /// not including dynamic data.
    enum class ELUTSize
    {
        KB_983, // No gain compression, direct speaker indices, no dynamic data
        KB_851, // No gain compression, speaker tiplet indices, dynamic data
        KB_786, // 24-bit gain compression, direct speaker indices, no dynamic data
        KB_655, // 24-bit gain compression, speaker triplet indices, dynamic data

        // For this type gains have to be computed on query, by a single matrix multiplication
        // -----------------------------------------------------------------
        KB_65   // No precomputed gains, speaker triplet indices, dynamic data with inv matrices
    };

    template<class T>
    concept CLUTType = std::same_as<T, ELUTSize>;

    /// Forward declaration of LUT type
    /// 'N' is the size of the LUT (e.g. indice count of octahedron direction codec)
    /// 'Vec3Type' is needed by LUT type that doesn't store precomputed gains.
    template<CLUTType auto T, size_t N, CVec3 Vec3Type = void*>
    struct LUT;
     
    //==========================================================================
    /// Alias for speaker triplet gains
    template<class T>
    using GainPack = std::array<T, 3>;

    /// LUT can be made out of just the selected speaker triplets, without precomputed gains
    using GainTypeNone = void*;

    /// Index of a speaker/channel
    using SpeakerIdx = uint8;

    /// Direct indices of the speakers, corersponding to channels in the output buffer
    using SpeakerTripletIdx = std::array<SpeakerIdx, 3>;

    /// Index of a 'SpeakerTripletIdx' (one extra indirection to select actual speakers/channels)
    /// For this, a small array of all triplets has to be stored alongside the LUT
    using TripletIdx = uint8;

    //==========================================================================
    /// Data required to retrieve the gains from the given LUT type,
    /// dependent on the number of triangles of the target speaker layout
    struct DynamicDataTri
    {
        SpeakerTripletIdx Tri;
        uint8 DummyIndex; // Index where the dummy is in a triplet (index 3 if none)
    };

    template<CVec3 Vec3Type>
    struct DynamicDataWithMat : DynamicDataTri
    {
        static_assert(!std::same_as<Vec3Type, void*> && "Vec3Type is not specified for the LUT that requires it.");

        Math::Mat3<Vec3Type> TrisInvMat;
    };

    using DynamicDataTypeNone = void*;

    template<class DynamicDataType_>
    struct DynamicDataTrait
    {
        using DynamicDataType = DynamicDataType_;
        std::pmr::vector<DynamicDataType> Data{ GetDefaultMemoryResource() };
    };

    template<>
    struct DynamicDataTrait<DynamicDataTypeNone> {};

    //==========================================================================
    /// Base to compose LUT specializations
    template<size_t N, class GainType_, class SpeakerIndexType_, class DynamicDataType_ = DynamicDataTypeNone>
    struct LUTBase : DynamicDataTrait<DynamicDataType_>
    {
        using GainType = GainType_;
        using SpeakerIndexType = SpeakerIndexType_;

        std::array<GainPack<GainType>, N> Gains;    // Speaker gains for each LUT index
        std::array<SpeakerIndexType, N> Speakers;   // Indices of the speakers to apply gains to
    };

    /// Specialization of the base without precomputed gains
    template<size_t N, class SpeakerIndexType_, class DynamicDataType_>
    struct LUTBase<N, GainTypeNone, SpeakerIndexType_, DynamicDataType_> : DynamicDataTrait<DynamicDataType_>
    {
        using GainType = GainTypeNone;
        using SpeakerIndexType = SpeakerIndexType_;

        std::array<SpeakerIndexType, N> Speakers;    // Indices of the speakers to compute gains for
    };

    //==========================================================================
    /// Available LUT specializations
        
    template<size_t N, CVec3 Vec3Type>
    struct LUT<ELUTSize::KB_983, N, Vec3Type>
        : LUTBase<N, float, SpeakerTripletIdx> {};
    
    template<size_t N, CVec3 Vec3Type>
    struct LUT<ELUTSize::KB_851, N, Vec3Type>
        : LUTBase<N, float, TripletIdx, DynamicDataTri> {};
    
    template<size_t N, CVec3 Vec3Type>
    struct LUT<ELUTSize::KB_786, N, Vec3Type>
        : LUTBase<N, Gain24Bit, SpeakerTripletIdx> {};
    
    template<size_t N, CVec3 Vec3Type>
    struct LUT<ELUTSize::KB_655, N, Vec3Type>
        : LUTBase<N, Gain24Bit, TripletIdx, DynamicDataTri> {};
    
    template<size_t N, CVec3 Vec3Type>
    struct LUT<ELUTSize::KB_65, N,  Vec3Type>
        : LUTBase<N, GainTypeNone, TripletIdx, DynamicDataWithMat<Vec3Type>> {};
        
#if 0 // Note: 16-bit gains may be too noisy below -60 dB
    template<size_t N, CVec3 Vec3Type> struct LUT<ELUTSize::KB_589, N, Vec3Type> : LUTBase<N, Gain16Bit, SpeakerTripletIdx> {};
    template<size_t N, CVec3 Vec3Type> struct LUT<ELUTSize::KB_458, N, Vec3Type> : LUTBase<N, Gain16Bit, TripletIdx, DynamicDataTri> {};
#endif
        
    //==========================================================================
    /// Just some more convenient aliases

    template<size_t N>
    using LUT_983 = LUTBase<N, float, SpeakerTripletIdx>;
    
    template<size_t N>
    using LUT_851 = LUTBase<N, float, TripletIdx, DynamicDataTri>;
    
    template<size_t N>
    using LUT_786 = LUTBase<N, Gain24Bit, SpeakerTripletIdx>;
    
    template<size_t N>
    using LUT_655 = LUTBase<N, Gain24Bit, TripletIdx, DynamicDataTri>;
    
    template<size_t N, CVec3 Vec3Type>
    using LUT_65 = LUTBase<N, GainTypeNone, TripletIdx, DynamicDataWithMat<Vec3Type>>;
        
    //==========================================================================
    /// Interface to query LUT gains for a direction
    template<CLUT LUTType, class LUTCodec>
    class LUTQuery
    {
    public:
        //==========================================================================
        /// If LUT doesn't have precomputed gains, querying computes
        /// inverse speaker matrix multiplication to get the gains for the query direction
        static constexpr bool bLUTHasGains = !std::same_as<typename LUTType::GainType, GainTypeNone>;

        //==========================================================================
        /// Data extracted from LUT for specific source direction
        struct VBAPCell
        {
            GainPack<float> Gains;        /// Represents gains of speaker/out-channel triplet
            SpeakerTripletIdx Speakers;   /// Represents indices of speaker/out-channel triplet
        };

        JPL_INLINE explicit LUTQuery(const LUTType& lut) noexcept : LUT(lut) {}

        /// Function to query LUT gains for a direction.
        /// @param direction : has to be normalized unit vector
        template<CVec3 Vec3Type>
        void GainsFor(const Vec3Type& direction, VBAPCell& outSpeakerGains) const;

        /// Function to query LUT gains for a direction.
        /// @param direction : has to be normalized unit vector
        /// @param outGains : must be of size at least number of target speakers the LUT was built for
        template<CVec3 Vec3Type>
        JPL_INLINE void GainsFor(const Vec3Type& direction, std::span<float> outGains) const;

    private:
        JPL_INLINE void ExtractGains(int index, std::span<float, 3> outGains) const requires (bLUTHasGains);

        // If LUT doen't have precomputed gains, we have to do things manually
        template<CVec3 Vec3Type>
        JPL_INLINE void ExtractGains(int index, const Vec3Type& direction, std::span<float, 3> outGains) const requires (!bLUTHasGains);

    public:
        const LUTType& LUT;
    };

    //==========================================================================
    /// Helper class to build a LUT for a set of directions and indices
    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    class LUTBuilder
    {
    public:
        static constexpr bool cLUTHasDynamicData = requires { typename LUTType::DynamicDataType; };
    public:
        LUTBuilder(ChannelMap channelMap, LUTType& outLUT);

        [[nodiscard]] JPL_INLINE uint32 GetNumDummies() const noexcept{ return mDummySpeakers.GetNumDummies(); }
        [[nodiscard]] JPL_INLINE uint32 GetNumRealChannels() const noexcept { return static_cast<uint32>(mVectors.size()) - mDummySpeakers.GetNumDummies(); }
        [[nodiscard]] JPL_INLINE const std::pmr::vector<SpeakerTripletIdx>& GetTris() const noexcept { return mTris; }
        [[nodiscard]] float FindShortestAperture() const;

        /// This has to be called for each direction the LUT needs to
        /// have values for
        /// @param direction :  must be a normalized unit vector
        /// @param lutIndex :   index in the LUT at which the values for
        ///                     the 'direction' have to be stored
        [[nodiscard]] bool ComputeCellFor(const Vec3Type& direction, int lutIndex);

        /// Build the entire LUT for all directions
        [[nodiscard]] JPL_INLINE bool BuildForAllDirections();

#if JPL_VALIDATE_VBAP_LUT
        void ValidateLUT() const;
#endif

    private:
#if 0   // W0 may need 2D matrices in the future
        JPL_INLINE void Compute2DMats();
#endif
        JPL_INLINE bool Triangulate();
        JPL_INLINE void ComputeTriMatrices();

        void ExtractDynamicData() const requires(cLUTHasDynamicData);
        JPL_INLINE uint32 FindReaplacementForDummy(const SpeakerTriangulation::Vec3i& tri) const;
    private:
        LUTType& mLUT;

        std::pmr::vector<Vec3Type> mVectors; // speaker direction vectors
        
        DummySpeakers<GetSpeakerVectorFunction> mDummySpeakers;

        uint32 mNumRealSpeakers = 0;
        uint32 mNumGroundSpeakers = 0;
        uint32 mNumTopSpeakers = 0;

        // Potential dynamic data stored in some LUT types
        std::pmr::vector<SpeakerTripletIdx> mTris;
        std::pmr::vector<Math::Mat3<Vec3Type>> mTrisInvMats;
    };
} // namespace JPL::VBAP

//==============================================================================
//
//   Code beyond this point is implementation detail...
//
//==============================================================================

// It may or may not sound more natural to normalize speaker
// triplet gains including "dummy" speaker, which would result
// in slight dip in volume in the lobe where "dummy" is "active".
//? Note: if we normalize with dummy, we won't be able to obtain
//? overall consistent gain normalization of the entire output of the panning
#define JPL_NORMALIZE_GAINS_WITH_DUMMY 0

namespace JPL::VBAP
{
    //==========================================================================
    template<auto GetSpeakerVectorFunction, class LUTCodec , CVec3 Vec3Type, CLUT LUTType>
    inline LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::LUTBuilder(ChannelMap channelMap, LUTType& outLUT)
        : mVectors(GetDefaultMemoryResource())
        , mDummySpeakers(channelMap, mVectors), mLUT(outLUT)
        , mTris(GetDefaultMemoryResource())
        , mTrisInvMats(GetDefaultMemoryResource())
    {
        // TODO: do we want to use real indices for somethin?
        channelMap.ForEachChannel([this](EChannel channel/*, uint32 index*/)
        {
            if (channel != EChannel::LFE)
            {
                mVectors.push_back(GetSpeakerVectorFunction(channel));

                mNumGroundSpeakers += channel < EChannel::TOP_Channels;
                mNumTopSpeakers += channel >= EChannel::TOP_Channels;
            }
        });

        // We have to use "dummy" speakers to calculate proper gains
        // when we don't have speakers all around the listener.
        // These dummy speakeres can be discarded after building the LUT.
        // 
        // For symetrical layout, non-side planes that don't have
        // center channel (top plane, back plane, bottom plane),
        // are triangulated non-symetrically, which results in
        // non-symetrical gain distribution for some symetrical directions.

        // Since at the moment ChannelMap doesn't support speakers on the bottom,
        // and most of the common layouts don't have speaker below listening plane,
        // we at least need to add a dummy speaker there for a better topology
        // of the convex hull.
        mDummySpeakers.AddDummy(Vec3Type(0.0f, -1.0f, 0.0f));

        // We have to accept potential asymetry of the back face,
        // to avoid making holes in panning where there's only
        // one real speaker per triangle (i.e the other 2 are dummies).

        // We can at least ensure symmetrical topology on the top
        if (mNumTopSpeakers == 6 || mNumTopSpeakers == 4)
        {
            mDummySpeakers.AddIfChannelNotPresent(EChannel::TopCenter);
        }
        else
        {
            JPL_ASSERT(mNumTopSpeakers == 2);
        }

        mNumRealSpeakers = static_cast<uint32>(mVectors.size() - GetNumDummies());

#if 0
        Compute2DMats();
#endif

        // Triangulate and compute matrices
        if (Triangulate())
        {
            if constexpr (cLUTHasDynamicData)
            {
                ExtractDynamicData();
            }
        }
        else
        {
            JPL_ASSERT(false, "Failed to triangulate speaker setup.");
        }

    }

#if 0
    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    JPL_INLINE void LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::Compute2DMats()
    {
        const ChannelMap channelMap = mDummySpeakers.GetChannelMap();

        // Sort channels to ensure we get valid edges form consecutive indices
        // (this will skip top channels, which is what we need)
        using ChannelAngleArray = typename Traits::template Array<ChannelAngle<Traits>>;
        ChannelAngleArray sourceChannelsSorted;
        ChannelAngle<Traits>::GetSortedChannelAngles(channelMap, sourceChannelsSorted);

        JPL_ASSERT(sourceChannelsSorted.size() >= mNumGroundSpeakers);

        auto createEdgeMat = [this](uint32 a, uint32 b)
        {
            const Vec2 A{ mVectors[a].X, mVectors[aGetZ(]) };
            const Vec2 B{ mVectors[b].X, mVectors[bGetZ(]) };
            Math::Mat2<Vec2> L{ A, B };

            auto& Linv = m2DMats.emplace_back();
            JPL_ENSURE(L.TryInverse(Linv));
        };

        // TODO: this ChannelId may be incorrect (relative to our vectors) past LFE, since it represent index in channel map including LFE

        // Start with wrap around (last->first)
        uint32 firstChannelIdx = sourceChannelsSorted[mNumGroundSpeakers - 1].ChannelId;

        // Ground speakers must be laid out first in mVectors
        for (uint32 i = 0; i < sourceChannelsSorted.size() && i < mNumGroundSpeakers; ++i)
        {
            const uint32 secondChannelIdx = sourceChannelsSorted[i].ChannelId;
            createEdgeMat(firstChannelIdx, secondChannelIdx);
            firstChannelIdx = secondChannelIdx;
        }
    }
#endif

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    JPL_INLINE bool LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::Triangulate()
    {
        if (SpeakerTriangulation::TriangulateSpeakerLayout(std::span<const Vec3Type>(mVectors), mTris))
        {
            ComputeTriMatrices();
            return true;
        }
        return false;
    }

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    JPL_INLINE void LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::ComputeTriMatrices()
    {
        mTrisInvMats.clear();
        mTrisInvMats.reserve(mTris.size());
        for (const SpeakerTriangulation::Vec3i& tri : mTris)
        {
            Math::Mat3<Vec3Type> L{ mVectors[tri[0]], mVectors[tri[1]], mVectors[tri[2]] };
            Math::Mat3<Vec3Type>& Linv = mTrisInvMats.emplace_back();
            (void)JPL_ENSURE(L.TryInverse(Linv));
        }
    }

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    inline float LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::FindShortestAperture() const
    {
        float maxDot = -std::numeric_limits<float>::max();
        for (const SpeakerTriangulation::Vec3i& tri : mTris)
        {
            const Vec3Type& A = mVectors[tri[0]];
            const Vec3Type& B = mVectors[tri[1]];
            const Vec3Type& C = mVectors[tri[2]];

            maxDot = std::max({ maxDot, DotProduct(A, B), DotProduct(B, C), DotProduct(C, A) });
        }
        JPL_ASSERT(maxDot > -std::numeric_limits<float>::max());
        return maxDot;
    }

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    inline bool LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::ComputeCellFor(const Vec3Type& direction, int lutIndex)
    {
        using LUTGainType = typename LUTType::GainType;
        using LUTSpeakerIdxType = typename LUTType::SpeakerIndexType;
        static constexpr bool bLUTHasGains = !std::same_as<LUTGainType, GainTypeNone>;
        static constexpr bool bLUTStoresTripletIdx = std::same_as<LUTSpeakerIdxType, TripletIdx>;

        Vec3Type directionSafe = direction;

        // TODO: if direction falls directly onto a dummy speaker, we might need to shift it over a bit
        //      otherwise we might get weirdness where at there're a hole in just one spot
        //      without fade if we normalize after dummy.
        /* if (mDummySpeakers.HasDummyAt(direction, static_cast<float>(LUTCodec::cMaxVectorError)))
        {
            static constexpr float offset =
                static_cast<float>(LUTCodec::cMaxVectorError) * 2.0f;

            GetZ(directionSafe) += offset;
            Normalize(directionSafe);
        }
        JPL_ASSERT(!mDummySpeakers.HasDummyAt(directionSafe, static_cast<float>(LUTCodec::cMaxVectorError)));*/

        // Find speaker triplet for our direction,
        // the one that has all gains positive
        for (int triI = 0; triI < mTrisInvMats.size(); ++triI)
        {
            Vec3Type gains = mTrisInvMats[triI].Transform(directionSafe);

            if (GetX(gains) < 0.0f || GetY(gains) < 0.0f || GetZ(gains) < 0.0)
                continue; // direction is outside of this trignale

            if (!JPL_ENSURE(!Math::IsNearlyZero(LengthSquared(gains))))
            {
                // TODO: Loudspeaker vectors are collinear, a case for 1D panning
                // (maybe if we don't find proper triplet, fall back to best collinear case and compute 1D gains)
                continue;
            }
#if JPL_NORMALIZE_GAINS_WITH_DUMMY
            // Normalize before zeroing out "dummy" speaker
            Normalize(gains);
#endif

            SpeakerTriangulation::Vec3i tri = mTris[triI];

#if defined(JPL_ENABLE_ASSERTS)
            // Ensure we don't create triangles with > 1 dummy speaker
            {
                [[maybe_unused]] int numDummies = 0;
                for (const uint32 idx : tri)
                    numDummies += mDummySpeakers.Contains(idx);
                JPL_ASSERT(numDummies <= 1);
            }
#endif

            // If we have precalculated gains, one of the seakers may be a dummy.
            // We need to silence it and renormalize the gains.
            if constexpr (bLUTHasGains)
            {
                int dummy = -1;

                if (mDummySpeakers.Contains(tri[0]))
                {
                    dummy = 0;
                    SetX(gains, 0.0f);
                }
                else if (mDummySpeakers.Contains(tri[1]))
                {
                    dummy = 1;
                    SetY(gains, 0.0f);
                }
                else if (mDummySpeakers.Contains(tri[2]))
                {
                    dummy = 2;
                    SetZ(gains, 0.0f);
                }

                // If our tri contains a dummy, we need to find
                // a different speaker to assign 0 gain to in our table
                if (dummy >= 0)
                    tri[dummy] = FindReaplacementForDummy(tri);
            }

#if !JPL_NORMALIZE_GAINS_WITH_DUMMY
            // Normalize after "dummy" speaker is zeroed out to avoid making a "hole"
            Normalize(gains);

            //JPL_ASSERT(!Math::HasNans(gains));
            //JPL_ASSERT(!Math::IsNearlyZero(gains.LengthSquared()));
#endif

            if constexpr (bLUTHasGains)
            {
                mLUT.Gains[lutIndex] = { LUTGainType(GetX(gains)), LUTGainType(GetY(gains)), LUTGainType(GetZ(gains)) };
            }

            if constexpr (bLUTStoresTripletIdx)
            {
                mLUT.Speakers[lutIndex] = triI;
            }
            else
            {
                mLUT.Speakers[lutIndex] = {
                    static_cast<SpeakerIdx>(tri[0]),
                    static_cast<SpeakerIdx>(tri[1]),
                    static_cast<SpeakerIdx>(tri[2])
                };
            }

            return true;
        }
        //if (GetY(direction) >= 0.0f)
            //std::cout << "No tri found for: " << std::format("{{{}, {}, {}}}", direction.X, GetY(direction), GetZ(direction)) << "\n";

        // Should be unreachable
        JPL_ASSERT(false, "Computing VBAP LUT failed.");
        if constexpr (bLUTHasGains)
        {
            mLUT.Gains[lutIndex] = { LUTGainType(0.0f), LUTGainType(0.0f), LUTGainType(0.0f) };
        }

        if constexpr (bLUTStoresTripletIdx)
        {
            mLUT.Speakers[lutIndex] = std::numeric_limits<SpeakerIdx>::max();
        }
        else
        {
            mLUT.Speakers[lutIndex] = {
                std::numeric_limits<SpeakerIdx>::max(),
                std::numeric_limits<SpeakerIdx>::max(),
                std::numeric_limits<SpeakerIdx>::max()
            };
        }

        return false;
    }

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    inline bool LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::BuildForAllDirections()
    {
        bool bAnyFailed = false;

        //const auto yminuscode = LUTCodec::Encode(Vec3Type(0, -1, 0));

        // Compute LUT values for every possible direction
        for (uint16_t dy = 0; dy < LUTCodec::cAxisRange; ++dy)
        {
            for (uint16_t dx = 0; dx < LUTCodec::cAxisRange; ++dx)
            {
                if (!LUTCodec::AreValidComponents(dx, dy))
                    continue; // skip padded cells that have no direction

                const uint32_t code = LUTCodec::CombineComponents(dx, dy);
                const Vec3Type dir = LUTCodec::template Decode<Vec3Type>(code);

                // TODO: we may or may not want to terminate if any fails
                bAnyFailed |= ComputeCellFor(dir, code);
            }
        }

        return bAnyFailed;
    }

#if JPL_VALIDATE_VBAP_LUT
    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    void LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::ValidateLUT() const
    {
        for (uint32 i = 0; i < mLUT.Speakers.size(); ++i)
        {
            if (!LUTCodec::IsValidCode(i))
                continue;

            if constexpr (std::same_as<typename LUTType::SpeakerIndexType, SpeakerTripletIdx>)
            {
                const auto& speakers = mLUT.Speakers[i];
                JPL_ASSERT(speakers[0] != speakers[1]);
                JPL_ASSERT(speakers[1] != speakers[2]);
                JPL_ASSERT(speakers[2] != speakers[0]);
            }
            else
            {
                // Extract speakers.
                // At this poitn LUT should have dynamic data
                // with speaker triplet mappings
                const TripletIdx tripletIdx = mLUT.Speakers[i];
                const SpeakerTripletIdx& speakers = mLUT.Data[tripletIdx].Tri;

                JPL_ASSERT(speakers[0] != speakers[1]);
                JPL_ASSERT(speakers[1] != speakers[2]);
                JPL_ASSERT(speakers[2] != speakers[0]);
            }
        }

        if constexpr (requires{ mLUT.Gains; })
        {
            for (uint32 i = 0; i < mLUT.Gains.size(); ++i)
            {
                if (!LUTCodec::IsValidCode(i))
                    continue;

                const auto& gains = mLUT.Gains[i];
                // Gains can be encoded in 24 or even 16 bit
                const std::array<float, 3> gainsDecoded
                {
                    gains[0],
                    gains[1],
                    gains[2]
                };
                JPL_ASSERT(Algo::IsNormalizedL2(gainsDecoded));
            }
        }
    }
#endif

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    inline void LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::ExtractDynamicData() const requires(cLUTHasDynamicData)
    {
        mLUT.Data.resize(mTris.size());

        for (uint32 i = 0; i < mLUT.Data.size(); ++i)
        {
            auto& data = mLUT.Data[i];

            if constexpr (requires{ data.Tri; })
            {
                static_assert(requires{ data.DummyIndex; });

                // Copy index of the speaker triplet
                data.Tri = mTris[i];

                // Find and replace dummy speaker with a real one if needed
                for (; data.DummyIndex < 3; ++data.DummyIndex)
                {
                    if (mDummySpeakers.Contains(data.Tri[data.DummyIndex]))
                    {
                        data.Tri[data.DummyIndex] = FindReaplacementForDummy(data.Tri);
                        break;
                    }
                }
            }

            // Copy inverse matrix for the gain calculation done for the LUT
            // that doesn't store precomputed gains, only selected speaker triplet
            if constexpr (requires{ data.TrisInvMat; })
                data.TrisInvMat = mTrisInvMats[i];
        }
    }

    template<auto GetSpeakerVectorFunction, class LUTCodec, CVec3 Vec3Type, CLUT LUTType>
    JPL_INLINE uint32 LUTBuilder<GetSpeakerVectorFunction, LUTCodec, Vec3Type, LUTType>::FindReaplacementForDummy(const SpeakerTriangulation::Vec3i& tri) const
    {
        // Simply increment index until we find one not already in the 'tri'
        uint32 newI = 0;
        while (std::ranges::find(tri, newI) != std::ranges::end(tri))
            ++newI;

        // Should be impossible
        JPL_ASSERT(newI < mNumRealSpeakers);

        return newI;
    }

    //==========================================================================
    template<CLUT LUTType, class LUTCodec>
    template<CVec3 Vec3Type>
    inline void LUTQuery<LUTType, LUTCodec>::GainsFor(const Vec3Type& direction, VBAPCell& outSpeakerGains) const
    {
        GainPack<float>& outGains = outSpeakerGains.Gains;
        SpeakerTripletIdx& outSpeakers = outSpeakerGains.Speakers;

        // Convert direction to LUT index
        //const auto index = GetIndexFromDirection(direction);
        const auto index = LUTCodec::Encode(direction);

        if constexpr (std::same_as<typename LUTType::SpeakerIndexType, SpeakerTripletIdx>)
        {
            outSpeakers = LUT.Speakers[index];
            ExtractGains(index, outGains);
        }
        else
        {
            // Extract speakers
            const TripletIdx tripletIdx = LUT.Speakers[index];
            outSpeakers = LUT.Data[tripletIdx].Tri;

            // Extract gains
            std::array<float, 4> gains{ 0.0f, 0.0f, 0.0f, 0.0f };

            if constexpr (bLUTHasGains)
                ExtractGains(index, std::span<float, 3>(gains.data(), 3));
            else
                ExtractGains(index, direction, std::span<float, 3>(gains.data(), 3));

            // If one of the speakedrs is dummy, we need to silence it
            const uint8 dummyIndex = LUT.Data[tripletIdx].DummyIndex;

            if constexpr (bLUTHasGains)
            {
                gains[dummyIndex] = 0.0f;
            }
            else
            {
#if !JPL_NORMALIZE_GAINS_WITH_DUMMY
                // If the direction falls directly onto a dummy,
                // we just assign the gains to the other speakers
                // to ensure consistent output
                if (Math::IsNearlyEqual(gains[dummyIndex], 1.0f))
                {
                    gains[0] = 1.0f;
                    gains[1] = 1.0f;
                    gains[2] = 1.0f;
                }
                // Now it's safe to silence the dummy
                gains[dummyIndex] = 0.0f;

                // If we just computed gains, we need to normalize
                // here after silencing the dummy.
                Algo::NormalizeL2(gains);

                //JPL_ASSERT(!Math::HasNans(Vec3Type(gains[0], gains[1], gains[2])));
#else
                gains[dummyIndex] = 0.0f;
#endif
            }

            // Copy valid speaker gains to the output
            std::memcpy(outGains.data(), gains.data(), sizeof(float) * 3);

            JPL_ASSERT(Algo::IsNormalizedL2(outGains));
        }
    }

    template<CLUT LUTType, class LUTCodec>
    template<CVec3 Vec3Type>
    JPL_INLINE void LUTQuery<LUTType, LUTCodec>::GainsFor(const Vec3Type& direction, std::span<float> outGains) const
    {
        VBAPCell cell;
        GainsFor(direction, cell);

        outGains[cell.Speakers[0]] = cell.Gains[0];
        outGains[cell.Speakers[1]] = cell.Gains[1];
        outGains[cell.Speakers[2]] = cell.Gains[2];
    }

    template<CLUT LUTType, class LUTCodec>
    JPL_INLINE void LUTQuery<LUTType, LUTCodec>::ExtractGains(int index, std::span<float, 3> outGains) const requires (bLUTHasGains)
    {
        if constexpr (std::same_as<typename LUTType::GainType, float>)
        {
            std::memcpy(outGains.data(), LUT.Gains[index].data(), sizeof(float) * 3);
        }
        else
        {
            static_assert(std::same_as<typename LUTType::GainType, Gain24Bit>);

            // Unpack the gains to convert to float
            const GainPack<Gain24Bit>& pack = LUT.Gains[index];
            outGains[0] = pack[0]; outGains[1] = pack[1]; outGains[2] = pack[2];
        }
    }

    template<CLUT LUTType, class LUTCodec>
    template<CVec3 Vec3Type>
    JPL_INLINE void LUTQuery<LUTType, LUTCodec>::ExtractGains(int index, const Vec3Type& direction, std::span<float, 3> outGains) const requires (!bLUTHasGains)
    {
        // Get the triplet index from the LUT
        const TripletIdx triplet = LUT.Speakers[index];

        // Compute gains
        const Vec3Type gainsV = LUT.Data[triplet].TrisInvMat.Transform(direction);
        outGains[0] = GetX(gainsV); outGains[1] = GetY(gainsV); outGains[2] = GetZ(gainsV);
    }

} // namespace JPL::VBAP