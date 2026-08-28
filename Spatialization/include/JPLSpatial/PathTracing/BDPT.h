#pragma once
#if defined(JPL_HAS_PATH_TRACING) && JPL_HAS_PATH_TRACING
#include "JPLSpatial/Core.h"
#include "JPLSpatial/ErrorReporting.h"
#include "JPLSpatial/AirAbsorption.h"
#include "JPLSpatial/Algo/Algorithm.h"
#include "JPLSpatial/Algo/Random.h"
#include "JPLSpatial/Containers/ScratchHashSet.h"
#include "JPLSpatial/Memory/Memory.h"
#include "JPLSpatial/Math/SIMD.h"
#include "JPLSpatial/Math/SIMDMath.h"
#include "JPLSpatial/PathTracing/BDPTTraits.h"
#include "JPLSpatial/PathTracing/BDTPDefinitions.h"
#include "JPLSpatial/PathTracing/EnergyResponse.h"
#include "JPLSpatial/PathTracing/Math.h"
#include "JPLSpatial/PathTracing/BDTPSampleAllocation.h"
#include "JPLSpatial/Utilities/Variance.h"

#include <algorithm>
#include <cmath>
#include <bit>
#include <vector>
#include <span>
#include <cmath>
#include <numeric>
#include <memory_resource>
#include <ranges>
#include <random>
#include <utility>

#ifndef JPL_DEV_BDPT_AIRABS_INTEGRATED
#define JPL_DEV_BDPT_AIRABS_INTEGRATED 0
#endif // !JPL_DEV_BDPT_AIRABS_INTEGRATED

#ifndef JPL_BDPT_PROFILE_PRINT
#define JPL_BDPT_PROFILE_PRINT 0
#endif


#if JPL_BDPT_PROFILE_PRINT
 //? temp
#include <chrono>
class TestTimer
{
public:
    TestTimer() { Reset(); }
    inline void Reset() { mStart = std::chrono::high_resolution_clock::now(); }
    inline float Elapsed() const { return std::chrono::duration<float, std::nano>(std::chrono::high_resolution_clock::now() - mStart).count(); }
    inline float ElapsedMicro() const { return std::chrono::duration<float, std::micro>(std::chrono::high_resolution_clock::now() - mStart).count(); }
    inline float ElapsedMillis() const { return std::chrono::duration<float, std::milli>(std::chrono::high_resolution_clock::now() - mStart).count(); }
    inline float ElapsedMillisD() const { return std::chrono::duration<double, std::milli>(std::chrono::high_resolution_clock::now() - mStart).count(); }

private:
    std::chrono::time_point<std::chrono::high_resolution_clock> mStart;
};
struct ScopedTimePrint
{
    explicit ScopedTimePrint(std::string_view tag) : Tag(tag) {}
    ~ScopedTimePrint() { std::cout << Tag << " " << mTimer.ElapsedMillis() << " ms"; }
    std::string_view Tag;
    TestTimer mTimer;
};
#endif


namespace JPL
{
    // TODO: move to math header
    namespace Math
    {
        template<class Vec3>
        [[nodiscard]] JPL_INLINE auto ComputePropagationTime(const Vec3& pointA, const Vec3& pointB) -> Internal::FloatOf<Vec3>
        {
            return Length(pointB - pointA) * JPL_INV_SPEAD_OF_SOUND;
        }
    }

    // Dot product clamped to >= 0.0
    template<class Vec3>
    [[nodiscard]] JPL_INLINE auto DotPositive(const Vec3& a, const Vec3& b) -> Internal::FloatOf<Vec3>
    {
        using FloatType = Internal::FloatOf<Vec3>;
        return std::max(FloatType(0.0), DotProduct(a, b));
    }

    
    //==========================================================================
    /// BDPT initialization parameters
    struct BDPTParameters
    {
        int SubPathBudget = 100;
        int MaxBounces = 4;
        int TotalSamples = 1024;
        float SampleBinDt = 0.003f;
        float EnergyResponseTime = 2.0f;

        float AllocAdaptationAlpha = 0.5f; // [0.4, 0.7] is a good range
    };

    //==========================================================================
    namespace BDPTUtils
    {
        // Probability density function for source emission
        static constexpr float Source_PDF()
        {
            // Uniform over the sphere
            constexpr float PDF = 1.0f / JPL_FOUR_PI;
            return PDF;
        }
    }

    //==========================================================================
    /// Biderectional Path Tracing (Cao et al. 2016)
    template<class Traits = BDPTDefaultTraits>
    class BDPT
    {
    private:
        using Vec3 = typename Traits::Vec3;

    public:
        //BDPT();
        explicit BDPT(const BDPTParameters& parameters);
        ~BDPT() = default;

		/// Trace the scene using Bidirectional Path Tracing
        /// from source and listener, connecting the paths
		/// and applying their contributions to the energy response.
        template<CScene SceneType,
            CSource<typename SceneType::Ray> SourceType,
            CListener<typename SceneType::Ray> ListenerType,
            class EnergyResponseType>
        void Trace(const SceneType& scene,
                   const SourceType& source,
                   const ListenerType& listener,
                   EnergyResponseType& response,
                   bool bOptimizeAllocations); //? temp. for testing

        [[nodiscard]] JPL_INLINE std::span<const float> GetAllocationRatios() { return mSampleAllocation.mSampleProbability_xn; }

    private:
        struct SplitWeights
        {
            std::span<const float> C;   // all s-split weights for n-th bounce index (s + t - 1)
            uint8 s;                    // current split s
            uint8 t;                    // current split t

        };

        // Note: just using 16bit integers instead of 32bit
        // doubled the performance of the build candidates loops,
        // reducing to 8bit further increased the performance by about 2x
        struct Candidate
        {
            uint8 fPath;    // forward path index
            uint8 bPath;    // backward path indes
            uint8 s;        // forward path prefix
            uint8 t;        // backward path prefix

            bool operator ==(const Candidate&) const noexcept = default;
        };

        struct ConnectedSample
        {
            simd Contribution;      // energy contribution of this sample
            float PropagationTime;  // path propagation time of this sample
            bool bConnected;        // will be 'false' if propagation time exceeds ER length
        };

        struct PathData;
        struct SubpathData;

    private:
        /// Initialize internal data structures
        void Initialize();

        // TODO: maybe move this out as the first stage, and then have use call "process traces"
        // taking backward paths, a set of forward paths per source and caches
        template<CScene SceneType>
        static PathData Trace_AreaMeasure(const Vec3& sourcePos,
                                          const Vec3& initialRayDirection,
                                          const SceneType& scene,
                                          uint32 maxBounces);

        void BuildCandidateLists(std::span<PathData> forwardPaths,
                                 std::span<PathData> backwardPaths,
                                 std::vector<std::vector<Candidate>>& outCandidatesPerOrder,
                                 std::vector<std::vector<float>>& outSplitWeights);


		[[nodiscard]] ConnectedSample Connect(const SubpathData& forwardSubpath,
											  const SubpathData& backwardSubpath,
											  const SplitWeights& splitWeights);

    private:
        struct Vertex
        {
            // --- Doesn't exist for x-last
            float PDFA;        // PDFA of x -> x+1

            float PDFAR;       // PDFA of x+1 -> x

            // --- Doesn't exist for x-last
            double MISRatio;  // PDFAR x+1 / PDFA x-1 (for x0 = PDFAR[x1])

            // --- Doesn't exist for x-last
            float G;           // G-term of x -> x+1
            float R;           // accumulated distance from x to x+1
            simd F;            // F of x -> x+1
            simd Throughput;   // product of `F * (G / PDFA)` up to x+1 (i.e. energy arriving at x+1)

            // --- Per vertex data
            float Diffusion;   // material diffusion at x (incl. x0)
            Vec3 Position;    // position of x
            Vec3 Normal;      // normal of x
        };

        struct PathData
        {
            std::vector<Vertex> Verts;

            std::vector<uint8> IsSpecular; // flag if x is a specular vertex (incl. x0)
            // ..we use these flags when building candidates,
            // that's why we keep them as dense array

            JPL_INLINE void Resize(uint32 numVertices)
            {
                Verts.resize(numVertices);
                IsSpecular.resize(numVertices);
            }

            JPL_INLINE void OptimizeMemory(uint32 actualNumVertices)
            {
                Resize(actualNumVertices);
                Verts.shrink_to_fit();
                IsSpecular.shrink_to_fit();
            }

            [[nodiscard]] JPL_INLINE std::size_t size() const { return Verts.size(); }
        };

        struct SubpathData
        {
            SubpathData(const PathData& pathData, uint32 numVertices)
                : Verts(pathData.Verts.data(), numVertices)
                , IsSpecular(pathData.IsSpecular.data(), numVertices)
            {
                JPL_ASSERT(numVertices <= pathData.Verts.size());
            }

            std::span<const Vertex> Verts;
            std::span<const uint8> IsSpecular;

            [[nodiscard]] JPL_INLINE std::size_t size() const { return Verts.size(); }
        };

    private:
        // Num subpaths to trace forward and backward
        uint8 mSubPathBudget;

        // Max boucne order per path, e.g. 20
        int mMaxBounces;

        // Adaptive sample allocation strategy
        BDPTSampleAllocation mSampleAllocation;

        // TODO: wrap this stuff into a utility class
        float mSampleBinDt;
        float mERTotalTime;
        float mAllocAdaptationAlpha;

        //? temp
        static float ComputeMeanFreePath(std::span<const PathData> paths);
    public:
        float MeanFreePath = 0.0f;

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
    /*template<class Traits>
    BDPT<Traits>::BDPT() // TODO: should we just delete default constuctor?
        : mSubPathBudget(0)
        , mMaxBounces(0)
        , mERTotalTime(0.0f)
    {
    }*/

    template<class Traits>
    BDPT<Traits>::BDPT(const BDPTParameters& parameters)
        : mSubPathBudget(std::min(255, parameters.SubPathBudget)) // TODO: investigate if using uint16 doesn't hurt performance of the candidate building, in case we need to increase max sub path budget from 255
        , mMaxBounces(parameters.MaxBounces)
        , mSampleBinDt(parameters.SampleBinDt)
        , mERTotalTime(parameters.EnergyResponseTime)
        , mAllocAdaptationAlpha(parameters.AllocAdaptationAlpha)
    {
        Initialize();

#if JPL_DEBUG_PRINT
        // All possible connections
        const int totalSamplesEstimate =
            parameters.SubPathBudget * parameters.SubPathBudget * parameters.MaxBounces;

        if (totalSamplesEstimate < parameters.TotalSamples)
        {
            LOG_TMP("Initialize: Total Samples Estimate (" << totalSamplesEstimate << ") "
                    "for SubPathBudget (" << mSubPathBudget <<") "
                    "and MaxBounces (" << mMaxBounces << ") "
                    "is less than Total Sampbles budget (" << parameters.TotalSamples << ").");
        }
#endif

		const int numSampleBins =
            parameters.SampleBinDt > 0.0f
			? int(std::ceil(parameters.EnergyResponseTime / parameters.SampleBinDt)) + 1
			: 0;

        mSampleAllocation.Initialize(parameters.TotalSamples, parameters.MaxBounces, numSampleBins);
    }

    template<class Traits>
    inline void BDPT<Traits>::Initialize()
    {
    }

	template<class Traits>
	template<CScene SceneType>
	inline auto JPL::BDPT<Traits>::Trace_AreaMeasure(const Vec3& sourcePos,
													 const Vec3& initialRayDirection,
													 const SceneType& scene,
													 uint32 maxBounces) -> PathData
	{
		// We can combine this "forced" specular/diffuse with the "branched" version,
        // as long as we divide by the right probability of the choice (which is 1.0f if "forced")
//#define JPL_FORCE_SPECULAR 0

        using Vec3 = typename SceneType::Vec3;
        using FloatType = Internal::FloatOf<Vec3>;
        using Ray = typename SceneType::Ray;
        using Intersection = typename SceneType::Intersection;

        PathData pathData;
        pathData.Resize(maxBounces + 1); // TODO: this allocation takes significant amount of CPU time
        auto& verts = pathData.Verts;
        auto& isSpecular = pathData.IsSpecular;

        Ray ray;
        ray.Origin = sourcePos;
        ray.Direction = initialRayDirection;

        struct TraceCache
        {
            Ray IncidentRay;
            Intersection Hit;
            simd Reflectance;
        };

        std::vector<TraceCache> traceCache;
        traceCache.reserve(maxBounces + 1);

        // for x0
        auto& vert0 = verts[0];
        vert0.Position = sourcePos;
        vert0.Normal = ray.Direction;
        vert0.Diffusion = 1.0f;

        isSpecular[0] = false;

        for (uint32 x = 1; x <= maxBounces; ++x)
        {
            Intersection hit;
            if (not scene.Intersect(ray, hit)) // trace the first surface patch x1
                break;

            auto [diffusion, absorption] = scene.GetMaterialFactor(hit.Material);
            simd reflectance = simd(1.0f) - absorption;

            traceCache.emplace_back(ray, hit, reflectance);

            // Sample next outgoing direction from current hit
            const bool bIsSpecular = Math::InternalUtils::RandFloat<FloatType>() >= diffusion;

            const Vec3 outDirection = bIsSpecular
                ? Math::SpecularReflection(ray.Direction, hit.Normal)
                : Math::SampleHemisphereCosine(hit.Normal);

            ray.Origin = hit.Position + hit.Normal * 0.001f; // small offset to avoide self intersection;
            ray.Direction = outDirection;

            auto& vertX = verts[x];
            vertX.Diffusion = diffusion;
            vertX.Position = hit.Position;
            vertX.Normal = hit.Normal;

            isSpecular[x] = bIsSpecular;
        }

        // If no hits...
        if (traceCache.empty())
        {
            auto& vert0 = verts[0];
            vert0.F = 1.0f / JPL_FOUR_PI;
            vert0.G = 1.0f;
            vert0.PDFA = 1.0f;
            vert0.PDFAR = 1.0f;
            vert0.MISRatio = 1.0f;
            vert0.R = 0.0f;
            vert0.Throughput = vert0.F;

            pathData.OptimizeMemory(1);
            return pathData;
        }

        float r2 = 0.0f;

        // Segment x0 -> x1
        {
            const auto& [insidentRay, hit, reflectance] = traceCache[0];

            // Converting intencity to radiance
            const float cosArrival = DotPositive(verts[1].Normal, -insidentRay.Direction);

            r2 = LengthSquared(verts[0].Position - verts[1].Position);

            const float g = cosArrival / r2;
            const float pdf = 1.0f / JPL_FOUR_PI;
            auto& vert0 = verts[0];
            vert0.F = 1.0f / JPL_FOUR_PI;
            vert0.G = g;
            vert0.PDFA = pdf * g;
            vert0.PDFAR = 1.0f;
            vert0.R = Math::Sqrt(r2);
            vert0.Throughput = vert0.F * (vert0.G / vert0.PDFA);
        }


        for (uint32 x = 1; x < traceCache.size(); ++x)
        {
            // Trace cache is indexed x-1, so traceCache[x] is for the next vertex

            // Ray that sampled vertex x
            const auto& incidentRay = traceCache[x - 1].IncidentRay;

            // Hit at vertex x
            const auto& hit = traceCache[x - 1].Hit;

            // Relfectance of the vertex x
            const auto& reflectance = traceCache[x - 1].Reflectance;

            const auto& nextHit = traceCache[x].Hit;
            const auto& nextIncRay = traceCache[x].IncidentRay;

            // Converting intencity to radiance
            const auto& vertPrev = verts[x - 1];
            auto& vertCur = verts[x];
            const auto& vertNext = verts[x + 1];

            const float cosThetaIn = DotPositive(vertNext.Normal, -nextIncRay.Direction);
            const float cosThetaInReverse = DotPositive(vertPrev.Normal, incidentRay.Direction);

            // Keep the distance squared of the last segment for reverse PDF
            const float r2Reverse = r2;

            // Compute new r2 for the outgoing segment
            r2 = LengthSquared(vertCur.Position - vertNext.Position);
            vertCur.R += vertPrev.R + Math::Sqrt(r2);

            float pdf; // probability of picking next vertex (angle measure)
            float pdfr; // probability of picking previous vertex in reverse (angle measure)

            if (isSpecular[x])
            {
                // Specular portion of the eradiance
                vertCur.F = reflectance * (1.0f - vertCur.Diffusion); // division by cosTheta out cancels out with G
                pdf = 1.0f - vertCur.Diffusion;
                vertCur.G = cosThetaIn / r2; // cosThetaOut of G cancels out with cosThetaOut needed for specular F


                // Reverse PDF for specular branch is symmetrical
                pdfr = pdf;

                // Throughput contribution boils down to F[x] / pdf:
                //
                // F[x] * (G[x] / PDFA[x]):
                // 
                //  F[x] * (cosThetaIn / r2) / (pdf * cosThetaIn / r2)
                //  F[x] * (cosThetaIn / r2) * (r2 / cosThetaIn * pdf)
                //  F[x] * (cosThetaIn * r2) / (r2 * cosThetaIn * pdf)
                //  F[x] / pdf
                // (for the sake of clarity, we'll use full form of throughput contribution below)
            }
            else
            {
                // Diffuse portion of the eradiance
                vertCur.F = (reflectance * vertCur.Diffusion) * JPL_INV_PI;

                const float cosThetaOut = DotPositive(vertCur.Normal, nextIncRay.Direction);
                pdf = vertCur.Diffusion * (cosThetaOut * JPL_INV_PI);
                vertCur.G = cosThetaOut * cosThetaIn / r2;

                // Reverse PDF for diffuse branch is non-symmetrical
                const auto dirToLastVertex = -incidentRay.Direction;
                const float cosThetaOutReverse = DotPositive(vertCur.Normal, dirToLastVertex);
                pdfr = vertCur.Diffusion * (cosThetaOutReverse * JPL_INV_PI);

                // Throughput contribution boils down to F[x] / (diffusions[x] * JPL_INV_PI):
                // 
                // F[x] * (G[x] / PDFA[x]):
                // 
                //  F[x] * (cosThetaOut * cosThetaIn / r2) / (pdf * cosThetaIn / r2)
                //  F[x] * (cosThetaOut * cosThetaIn / r2) * (r2 / pdf * cosThetaIn)
                //  F[x] * (cosThetaOut * cosThetaIn * r2) / (r2 * pdf * cosThetaIn)
                //  F[x] * (cosThetaOut) / (pdf)
                //  F[x] / (diffusions[x] * JPL_INV_PI)
                // (for the sake of clarity, we'll use full form of throughput contribution below)
            }

            // Probability dencity function converted to area measure
            vertCur.PDFA = pdf * cosThetaIn / r2;

            // Throughput is effectively a product of weight factors mapped to area measure
            vertCur.Throughput = vertPrev.Throughput * vertCur.F * (vertCur.G / vertCur.PDFA);

            // Reverse PDF in area measure
            vertCur.PDFAR = pdfr * cosThetaInReverse / r2Reverse;
        }

        JPL_ASSERT(traceCache.size() < maxBounces + 1);

        // Segment x-last -> ...
        {
            const float r2Reverse = r2;

            const uint32 x = traceCache.size();
            auto& vertX = verts[x];
            auto& vertXPrev = verts[x - 1];
            vertX.R = vertXPrev.R;
            vertX.F = 1.0f;
            vertX.G = 1.0f;
            vertX.PDFA = 1.0f;
            vertX.Throughput = vertXPrev.Throughput; // TODO: should this be just 0, since this depends on the "receiveing" vetex?

            // Ray that sampled this last vertex
            const auto& incidentRay = traceCache[x - 1].IncidentRay;
            const float cosThetaInReverse = DotPositive(vertXPrev.Normal, incidentRay.Direction);

            float pdfr;
            if (isSpecular[x])
            {
                const float pdf = 1.0f - vertX.Diffusion;
                pdfr = pdf;
            }
            else
            {
                const auto dirToLastVertex = -incidentRay.Direction;
                const float cosThetaOutReverse = DotPositive(vertX.Normal, dirToLastVertex);
                pdfr = vertX.Diffusion * (cosThetaOutReverse * JPL_INV_PI);
            }
            vertX.PDFAR = pdfr * cosThetaInReverse / r2Reverse;
        }

        // For the source vertex: PDFAR of x1 -> x0 / PDFA x-1 -> x0 (x-1 doesn't exist, so its PDFA is 1.0)
        // For the endpoint vertex: MIS depends on the endpoint on the other side of the future connection

        verts[0].MISRatio = verts[1].PDFAR;
        verts[traceCache.size()].MISRatio = 0.0f;

        for (uint32 xmr = 1; xmr < traceCache.size(); ++xmr)
        {
            verts[xmr].MISRatio = verts[xmr + 1].PDFAR / verts[xmr - 1].PDFA;
        }

        pathData.OptimizeMemory(traceCache.size() + 1); // +1 for source vertex
        
#if 0       
            //! Useful tool, we may play around with it later,
            //! for now, it results in a very quick die off of the paths
            //! after the tested bounce order and a very noisy historgram
            //! after that point as well.

            // Roulette
            if (bounce > 5)
            {
                //const float q = std::clamp(throughput, 0.1f, 0.95f); // TODO: for SIMD throughput, use max component
                const float q = std::clamp(reflectance, 0.1f, 0.95f); // TODO: for SIMD throughput, use max component
                if (Math::InternalUtils::RandFloat<float>() > q)
                {
                    break;
                }
                throughput /= q;
            }
#endif
        return pathData;
    }


    template<class Traits>
	auto BDPT<Traits>::Connect(const SubpathData& forwardSubpath,
												 const SubpathData& backwardSubpath,
												 const SplitWeights& splitWeights) -> ConnectedSample
	{
        JPL_ASSERT(not forwardSubpath.IsSpecular.back() and not backwardSubpath.IsSpecular.back(),
                   "Shouldn't attempt to connect specular endpoints, the probability of such connection is 0");

        // Sanity check
        JPL_ASSERT(forwardSubpath.size() == splitWeights.s + 1);
        JPL_ASSERT(backwardSubpath.size() == splitWeights.t + 1);

        // Source/listener "emit" energy omnidirectionlly,
        // we only need to use the cosine on the other side when connecting.
        const bool isForwardDirect = splitWeights.s == 0;
        const bool isBackwardDirect = splitWeights.t == 0;


        // Accumulated R is stored for the segment forward (i.e. s -> s+1) so ignore the last index
        const float distance =
            (isForwardDirect ? 0.0f : forwardSubpath.Verts[splitWeights.s - 1].R) +
            (isBackwardDirect ? 0.0f : backwardSubpath.Verts[splitWeights.t - 1].R);

        // --- Connect endpoint vertices
        const auto& vertS = forwardSubpath.Verts[splitWeights.s];
        const auto& vertT = backwardSubpath.Verts[splitWeights.t];

        const Vec3 connectionVector = vertT.Position - vertS.Position;
        const float connectionR2 = LengthSquared(connectionVector);

        const float connectionDistance = Math::Sqrt(connectionR2);
        const float propagationTime = (distance + connectionDistance) * JPL_INV_SPEAD_OF_SOUND;
        if (propagationTime > mERTotalTime)
        {
            return ConnectedSample{ .bConnected = false };
        }

        // Accumulated throughput is also stored for the segment forward
        const simd throughput =
            (isForwardDirect ? simd(1.0f) : forwardSubpath.Verts[splitWeights.s - 1].Throughput) *
            (isBackwardDirect ? simd(1.0f) : backwardSubpath.Verts[splitWeights.t - 1].Throughput);

        const Vec3 connectionDir = connectionVector / connectionDistance;

        const float cosThetaOut = isForwardDirect ? 1.0f : DotPositive(vertS.Normal, connectionDir);
        const float cosThetaIn = isBackwardDirect ? 1.0f : DotPositive(vertT.Normal, -connectionDir);

        const float connectionG = cosThetaOut * cosThetaIn / connectionR2;
        const simd connectionF = vertS.F * connectionG * vertT.F;
        const simd contribution = throughput * connectionF;


        // --- Compute MIS weight

        double sum = 1.0;

        const float cCurrentInv =
            splitWeights.C[splitWeights.s] > 0.0f
            ? 1.0f / splitWeights.C[splitWeights.s]
            : 0.0f;

        auto cRatio = [&](uint32 altS)
        {
            return splitWeights.C[altS] * cCurrentInv;
        };

        // Walk towards the source and compute PDFA ratios
        {
            // Connection reverse PDFA
            const float pdfr =
                isBackwardDirect
                ? 1.0 / JPL_FOUR_PI // if the other side is the receiver vertex
                : vertT.Diffusion * (cosThetaIn * JPL_INV_PI);
           
            const float connectionPDFAReverse = pdfr * cosThetaOut / connectionR2;

            const float connectionPDFAForward =
                isForwardDirect
                ? 1.0                           // PDF of selecting the source vertex
                : forwardSubpath.Verts[splitWeights.s - 1].PDFA;   // PDFA towards the last fwd vertex 


            double ratioProductForward = connectionPDFAReverse / connectionPDFAForward;
            sum += ratioProductForward;


            for (int altS = int(splitWeights.s) - 1; altS >= 0; altS--)
            {
                if (forwardSubpath.IsSpecular[altS])
                {
                    break;
                }
                ratioProductForward *= forwardSubpath.Verts[altS].MISRatio;
                sum += cRatio(altS) * ratioProductForward;
            }
        }

        // Walk towards the receiver and compute PDFA ratios
        {
            // Connection reverse PDFA
            const float pdfr =
                isForwardDirect
                ? 1.0 / JPL_FOUR_PI // if the other side is the source vertex vertex
                : vertS.Diffusion * (cosThetaOut * JPL_INV_PI);

            const float connectionPDFAReverse = pdfr * cosThetaIn / connectionR2;

            const float connectionPDFAForward =
                isBackwardDirect
                ? 1.0                           // PDFA of selecting the receiver vertex
                : backwardSubpath.Verts[splitWeights.t - 1].PDFA;  // PDFA towards the last bwd vertex 

            double ratioProductBackward = connectionPDFAReverse / connectionPDFAForward;
            sum += ratioProductBackward;

            for (int altT = int(splitWeights.t) - 1, altS = splitWeights.s + 1; altT >= 0; altT--, altS++)
            {
                if (backwardSubpath.IsSpecular[altT])
                {
                    break;
                }
                ratioProductBackward *= backwardSubpath.Verts[altT].MISRatio;
                sum += cRatio(altS) * ratioProductBackward;
            }
        }
        const float misWeight = 1.0 / sum;

#if JPL_DEV_BDPT_AIRABS_INTEGRATED
        const simd airAbs = dBToIntencity(-AirAbsorption::cRT60Coeffs_dB * (distance + connectionDistance));
     
        return { contribution * airAbs * misWeight, propagationTime, true };
#else
        return { contribution * misWeight, propagationTime, true };
#endif
    }


    template<class Traits>
    inline void BDPT<Traits>::BuildCandidateLists(std::span<PathData> forwardPaths,
                                                  std::span<PathData> backwardPaths,
                                                  std::vector<std::vector<Candidate>>& outCandidatesPerOrder,
                                                  std::vector<std::vector<float>>& splitWeights)
    {
        //ScopedTimePrint timer(", build candidates:");

        outCandidatesPerOrder.resize(mMaxBounces);
        splitWeights.resize(mMaxBounces);
        
        // Preallocate max expected memory
        for (uint32 n = 0; n < mMaxBounces; ++n)
        {
            splitWeights[n].reserve(n + 2);

            const auto allocatedSamples = static_cast<std::size_t>(mSampleAllocation.mAllocatedSamples[n]);
            outCandidatesPerOrder[n].reserve(allocatedSamples);
        }

        // Generate pools of all valid s,t from forward and backward paths
        std::vector<std::vector<uint8>> fByS(mMaxBounces + 1);
        std::vector<std::vector<uint8>> bByT(mMaxBounces + 1);

        for (std::vector<uint8>& ss : fByS)
            ss.reserve(forwardPaths.size());

        for (std::vector<uint8>& ts : bByT)
            ts.reserve(backwardPaths.size());

        for (uint8 fPath = 0; fPath < forwardPaths.size(); ++fPath)
        {
            PathData& forwardPathData = forwardPaths[fPath];

            for (uint16 s = 0; s < forwardPathData.IsSpecular.size(); ++s)
            {
                if (not forwardPathData.IsSpecular[s])
                    fByS[s].push_back(fPath);
            }
        }

        for (uint8 bPath = 0; bPath < backwardPaths.size(); ++bPath)
        {
            PathData& backwardPathData = backwardPaths[bPath];

            for (uint16 t = 0; t < backwardPathData.IsSpecular.size(); ++t)
            {
                if (not backwardPathData.IsSpecular[t])
                    bByT[t].push_back(bPath);
            }
        }

        struct SplitChoice
        {
            uint8 s;
            uint8 t;
            float c; // c_s, probability of choosing this split
        };
        std::vector<SplitChoice> splits;
        splits.reserve(mMaxBounces + 2);

        std::vector<uint64> budgets;
        std::vector<float> budgetRemainders;

        // TODO: use proper seeding, for now determinism is needed for development and testing
        Rand rng(534543789);

        for (uint8 n = 0; n < mMaxBounces; ++n)
        {
            splitWeights[n].assign(n + 2, 0.0f);

            const uint32 allocatedSamples = mSampleAllocation.mAllocatedSamples[n];
            if (allocatedSamples == 0)
                continue;

            splits.clear();

            // Generate all possible s splits for n-th order
            for (uint8 s = 0; s <= n + 1; ++s)
            {
                const uint8 t = n + 1 - s;

                if (s >= fByS.size() || t >= bByT.size())
                    continue;

                if (fByS[s].empty() || bByT[t].empty())
                    continue;

                splits.push_back({ .s = s, .t = t, .c = 0.0f });
            }

            if (splits.empty())
                continue;

            // Compute probability weights for s splits
            uint64 totalPairs = 0;

            for (SplitChoice& split : splits)
            {
                const uint64 pairCount =
                    static_cast<uint64>(fByS[split.s].size()) *
                    static_cast<uint64>(bByT[split.t].size());

                split.c = static_cast<float>(pairCount); // temporarily
                totalPairs += pairCount;
            }

            if (totalPairs == 0)
                continue;

            const float invTotalPairs = 1.0f / totalPairs;
            for (SplitChoice& split : splits)
            {
                split.c *= invTotalPairs;
            }

            // Comptue budgets for each s split based on its probability weight
            budgets.clear();
            budgets.resize(splits.size());
            budgetRemainders.clear();
            budgetRemainders.resize(splits.size());

            uint64 budgetSum = 0;

            for (uint32 i = 0; i < splits.size(); ++i)
            {
                const float quota = splits[i].c * allocatedSamples;
                const uint64 base = static_cast<uint64>(std::floor(quota));

                budgets[i] = base;
                budgetRemainders[i] = quota - static_cast<float>(base);
                budgetSum += base;
            }

            const uint64 leftover = allocatedSamples - budgetSum;
            if (leftover > 0)
            {
                // TODO: this is the heaviest part of the entire function,
                //      discrete_distribution copies our vector and then allocates another one internally
                std::discrete_distribution<uint32> pickRemainder(
                    budgetRemainders.begin(),
                    budgetRemainders.end());

                for (uint64 k = 0; k < leftover; ++k)
                    budgets[pickRemainder(rng)]++;
            }

            budgetSum = Algo::Accumulate(budgets, uint64(0));
            JPL_ASSERT(budgetSum == allocatedSamples);

            // Store split weights to later use in MIS
            const float invBudgetSum = budgetSum > 0 ? 1.0f / float(budgetSum) : 0.0f;
            for (uint32 i = 0; i < splits.size(); ++i)
            {
                const SplitChoice& split = splits[i];
                splitWeights[n][split.s] = budgets[i] * invBudgetSum;
            }

            // For each split, uniformly sample candidates from s,t pools
            // based on the weight-distributed budgets
            for (uint32 i = 0; i < splits.size(); ++i)
            {
                const SplitChoice& split = splits[i];
                const uint64 budgetForSplit = budgets[i];

                const auto& fPool = fByS[split.s];
                const auto& bPool = bByT[split.t];

                std::uniform_int_distribution<std::size_t> pickF(0, fPool.size() - 1);
                std::uniform_int_distribution<std::size_t> pickB(0, bPool.size() - 1);

                for (uint64 j = 0; j < budgetForSplit; ++j)
                {
                    const uint8 fPath = fPool[pickF(rng)];
                    const uint8 bPath = bPool[pickB(rng)];

                    outCandidatesPerOrder[n].push_back(
                        Candidate{
                            .fPath = fPath,
                            .bPath = bPath,
                            .s = split.s,
                            .t = split.t
                        });
                }
            }
        }
    }

    //==========================================================================
    template<class Traits>
    float BDPT<Traits>::ComputeMeanFreePath(std::span<const PathData> paths)
    {
        uint64 numSegments = 0;
        float lengthSum = 0.0f;

        for (const PathData& path : paths)
        {
            if (path.Verts.size() < 2)
                continue;

            numSegments += path.Verts.size() - 1;

            for (uint32 nodeIdx = 2; nodeIdx < path.Verts.size(); ++nodeIdx)
            {
                const Vertex& prevVertex = path.Verts[nodeIdx - 1];
                const Vertex& curVertex = path.Verts[nodeIdx];

                lengthSum += Length(curVertex.Position - prevVertex.Position);
            }
        }

        // TODO: do something with this
        const float meanPathLength = numSegments ? lengthSum / numSegments : 0.0;

        //const float totalT = lengthSum / 343.0f;// SpeedOfSound::cReference;
        //const float reflectionsPerSecond = numSegments / totalT;
        //simd absorptionPerSecond = reflectionsPerSecond * meanAbsorption;
        //const simd absorptionPerSecond = absorptionSum / totalT;

        return meanPathLength;
    }


    template<class Traits>
    template<CScene SceneType,
        CSource<typename SceneType::Ray> SourceType,
        CListener<typename SceneType::Ray> ListenerType,
        class EnergyResponseType>
    inline void BDPT<Traits>::Trace(
        const SceneType& scene,
        const SourceType& source,
        const ListenerType& listener,
        EnergyResponseType& response,
        bool bOptimizeAllocations)
    {
#if JPL_BDPT_PROFILE_PRINT
        using OnlineVarianceD = typename Variance<double>::Online;

        static OnlineVarianceD varFullTrace;
#if 0
        static OnlineVarianceD varTrace;
        static OnlineVarianceD varBuildCands;
       // static OnlineVarianceD varConnect;
        static OnlineVarianceD varConnectLoop;
        static OnlineVarianceD varOptimize;

#endif
        TestTimer timerFullTrace;
#endif

        // TODO: can we enforce this without assertion?
        JPL_ASSERT(mSampleBinDt == response.GetTimeStep());
        JPL_ASSERT(mERTotalTime <= response.GetTotalTimeCapacity());

        using Vec3 = typename SceneType::Vec3;

        mSampleAllocation.Allocate();

        //? TEMP
        const Vec3 listenerPosition = listener.SampleRay().Origin;
        const Vec3 sourcePosition = source.SampleRay().Origin;

        std::vector<PathData> forwardPaths;
        std::vector<PathData> backwardPaths;


		//! Perf notes
        
		/* Before optimization
			Total: 632.5 ms
			- trace 1.72 ms
			- build candidates 341 ms
			- connect 273.6 ms

		*/
        // ASNR before refactor 7.1433043 dB

		/* During/after refactor & optimization

			Total: 6.5-8 ms
			- build candidates 64-79 ms -> using Reservoirs -> 119-122 ms -> precomputing N -> 46-65 ms -> weighted sampling -> 1.5 ms
			- connect 240-306 ms -> using randomized candidate buildup -> 4.26-5.47 ms

            ..switching from SoA to AoS reduced from about 12 ms to 6.5 ms total time
		*/
        // ASNR after refactor 7.365868 dB

        // We can shave off about 0.1-0.2 ms if we don't allocate each frame

        //! Perf note: we can get a lot better performance tracing fewer boucnes and fewer samples
        //! however, the EDC doesn't have enough of the slope length to compute the correct RT60 values


        // 1. Trace forward and backward paths
        {
#if JPL_BDPT_PROFILE_PRINT
            //TestTimer timerTrace;
#endif

            forwardPaths.reserve(mSubPathBudget);
            backwardPaths.reserve(mSubPathBudget);

            for (uint32 i = 0; i < mSubPathBudget; ++i)
            {
                const Vec3 initialSourceRayDir = source.SampleRay().Direction;
                forwardPaths.emplace_back(Trace_AreaMeasure(sourcePosition, initialSourceRayDir, scene, mMaxBounces));

                const Vec3 initialListenerRayDir = listener.SampleRay().Direction;
                backwardPaths.emplace_back(Trace_AreaMeasure(listenerPosition, initialListenerRayDir, scene, mMaxBounces));
            }
#if JPL_BDPT_PROFILE_PRINT
            //varTrace.Add(timerTrace.ElapsedMillisD());
#endif
        }

#if 1
        // TEMP. Compute Mean Free Path for valdiation
        MeanFreePath = 0.5f * (ComputeMeanFreePath(forwardPaths) + ComputeMeanFreePath(backwardPaths));

#endif

        // TODO: enforce this where we take the parameters
        JPL_ASSERT(mSubPathBudget <= std::numeric_limits<uint8>::max());

        std::vector<std::vector<float>> splitWeights(mMaxBounces);
        
#if JPL_BDPT_PROFILE_PRINT
        //TestTimer timerBuildCands;
#endif        
        std::vector<std::vector<Candidate>> selectedCandidates(mMaxBounces);

        BuildCandidateLists(forwardPaths, backwardPaths, selectedCandidates, splitWeights);
#if JPL_BDPT_PROFILE_PRINT
        //varBuildCands.Add(timerBuildCands.ElapsedMillisD());

        //double timeConnect = 0.0;
        //static OnlineVarianceD connectTimeVar;
#endif
        // 4. Select up to mAllocatedSamples[n] candidates
        {
#if JPL_BDPT_PROFILE_PRINT
            //TestTimer timerConnectLoop;
           // double timeConnect = 0.0;
    #endif

            for (uint32 n = 0; n < mMaxBounces; ++n)
            {
                std::vector<Candidate>& candidates = selectedCandidates[n];
            
                JPL_ASSERT(candidates.size() <= mSampleAllocation.mAllocatedSamples[n]);

                const uint32 selectedCount = candidates.size();
                const float norm = selectedCount > 0 ? 1.0f / selectedCount : 0.0f;

                // For each candidate, try to Connect
                for (const Candidate& candidate : candidates)
                {
                    const auto forwardSubpath = SubpathData(forwardPaths[candidate.fPath], candidate.s + 1);
                    const auto backwardSubpath = SubpathData(backwardPaths[candidate.bPath], candidate.t + 1);

                    // "Consume" occluded but technically connectible candidate
                    //! Note: in the non-shoe-box scene this might affect ASNR significantly, needs testing
                    if (scene.IsOccluded(forwardSubpath.Verts.back().Position, backwardSubpath.Verts.back().Position))
                    {
                        // Accumulated R is stored for the segment forward (i.e. s -> s+1) so ignore the last index
                        const float distance =
                            (candidate.s == 0 ? 0.0f : forwardSubpath.Verts[candidate.s - 1].R) +
                            (candidate.t == 0 ? 0.0f : backwardSubpath.Verts[candidate.t - 1].R) +
                            Length(forwardSubpath.Verts.back().Position - backwardSubpath.Verts.back().Position);

                        const float propagationTime = distance * JPL_INV_SPEAD_OF_SOUND;

                        if (propagationTime <= mERTotalTime)
                        {
                            const auto sampleBin = response.GetBinFor(propagationTime);
                            mSampleAllocation.RecordContribution(sampleBin, n, 0.0f);
                        }
      
                        continue;
                    }

                    const SplitWeights sw{
                        .C = std::span<const float>(splitWeights[n]),
                        .s = candidate.s,
                        .t = candidate.t
                    };
#if JPL_BDPT_PROFILE_PRINT
                    //TestTimer timerConnect;
#endif                    
                    auto [contribution, propagationTime, bConnected] = Connect(forwardSubpath, backwardSubpath, sw);
#if JPL_BDPT_PROFILE_PRINT              
                    //timeConnect += timerConnect.ElapsedMillisD();
#endif
                    if (not bConnected)
                        continue; // TODO: we might want to report if some paths/boucne-ordre exceding our ER time, and adjust accordingly

                    // Normalize by the number of selected candidates for bounce order `n`
                    contribution *= norm;

                    const auto sampleBin = response.AddSample(propagationTime, contribution, /* direction */ 0); // we don't use direction

                    mSampleAllocation.RecordContribution(sampleBin, n, contribution.reduce_mean());
                }
            }

#if JPL_BDPT_PROFILE_PRINT
           // varConnect.Add(timeConnect);

            //const double fullLoopT = timerConnectLoop.ElapsedMillisD();
            //varConnectLoop.Add(fullLoopT - timeConnect);
            //varConnectLoop.Add(fullLoopT);// -timeConnect);
#endif
        }
#if JPL_BDPT_PROFILE_PRINT
        //TestTimer timerOptimize;
#endif
        if (bOptimizeAllocations)
        {
            // Update sample allocation based on the traced paths and their contributions
            const float alpha = mAllocAdaptationAlpha; // [0.4-0.7], affects responsiveness of the allocation adaptation to geometry changes
            mSampleAllocation.UpdateOptimize(alpha);
        }
#if JPL_BDPT_PROFILE_PRINT
        //varOptimize.Add(timerOptimize.ElapsedMillisD());

        const double timeFullTrace = timerFullTrace.ElapsedMillisD();
        varFullTrace.Add(timeFullTrace);

        std::cout << std::fixed << std::setprecision(3)
            << "Count: " << varFullTrace.Count << ", "
            << "Full Trace: " << varFullTrace.Mean << " ms"
            << '\n';
/*
        std::cout << std::fixed << std::setprecision(3)
            << "Count: " << varFullTrace.Count << ", "
            << "Full Trace: " << varFullTrace.Mean << " ms, "
            << "Trace: " << varTrace.Mean << " ms, "
            << "Build Cands: " << varBuildCands.Mean << " ms, "
            //<< "Connect: " << varConnect.Mean << " ms, "
            << "Connect loop: " << varConnectLoop.Mean << " ms, "
            << "Optimize: " << varOptimize.Mean << " ms"
            << '\n';*/
        
        /*std::cout << std::fixed << std::setprecision(6)
            << " Current: " << elapsed
            << " ms, count: " << fullTraceVar.Count
            << ", mean: " << fullTraceVar.Mean << " ms"
            << '\n';*/

        /*std::cout << std::fixed << std::setprecision(6)
            << " Current total: " << elapsed << " ms"
            << ", count: " << fullTraceVar.Count
            << ", total mean: " << fullTraceVar.Mean << " ms"
            << ", mean: " << connectTimeVar.Mean << " ms"
            << '\n';*/
#endif
    }

} // namespace JPL

#endif
