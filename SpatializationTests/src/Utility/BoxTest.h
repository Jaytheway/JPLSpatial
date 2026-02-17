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

#include <cmath>
#include <numbers>

namespace JPL
{
    /// Box - axis-aligned, centred, built from half-extents
    template<class Vec3>
    class Box
    {
        // Fast component-wise reciprocal with “infinite” for 0.0f
        static inline Vec3 SafeRcp(const Vec3& v, float huge = 1.0e+30f)
        {
            return { v.X != 0.0f ? 1.0f / v.X : huge,
                     v.Y != 0.0f ? 1.0f / v.Y : huge,
                     v.Z != 0.0f ? 1.0f / v.Z : huge };
        }

    public:
        Box(const Vec3& inCenter, const Vec3& inHalfExtent)
            : mCentre(inCenter), mHalfExtent(inHalfExtent)
        {
            mMin = mCentre - mHalfExtent;
            mMax = mCentre + mHalfExtent;
        }

        struct Hit
        {
            bool bHit = false;
            float Distance = 0.0f;  // along the ray
            Vec3 Position;          // world-space point
            Vec3 Normal;            // outward unit normal
        };

        /// Cast a ray: origin + t * direction, t >= 0.
        /// Returns the first intersection with the box (entry if outside, exit if inside).
        /// maxDist = +inf by default – supply a finite value to clamp travel distance.
        Hit CastRay(const Vec3& origin,
                    const Vec3& dir,
                    float maxDist = std::numeric_limits<float>::infinity()) const
        {
            const Vec3 invDir = SafeRcp(dir);
            const Vec3 t1 = (mMin - origin) * invDir;   // distance to the  “min” planes
            const Vec3 t2 = (mMax - origin) * invDir;   // distance to the  “max” planes

            // For each axis pick the nearer (entry) and farther (exit) intersection.
            const Vec3 tEntryV{ std::min(t1.X, t2.X),
                                std::min(t1.Y, t2.Y),
                                std::min(t1.Z, t2.Z) };

            const Vec3 tExitV{ std::max(t1.X, t2.X),
                                std::max(t1.Y, t2.Y),
                                std::max(t1.Z, t2.Z) };

            const float tEntry = std::max({ tEntryV.X, tEntryV.Y, tEntryV.Z });
            const float tExit = std::min({ tExitV.X,  tExitV.Y,  tExitV.Z });

            Hit hit;

            // Reject if the slabs miss, or exit is behind the origin, or entry > maxDist.
            if (tEntry > tExit || tExit < 0.0f || tEntry > maxDist)
                return hit;  // hit.bHit == false

            // Choose the first positive intersection:
            hit.Distance = (tEntry >= 0.0f) ? tEntry : tExit;

            if (hit.Distance > maxDist)
                return hit;

            hit.bHit = true;
            hit.Position = origin + dir * hit.Distance;

            // Determine the normal: which face did we hit?
            // Compare the intersection point to the box planes with a small ε.
            static constexpr float eps = 1.0e-4f;
                 if (std::abs(hit.Position.X - mMin.X) < eps) hit.Normal = { -1, 0, 0 };
            else if (std::abs(hit.Position.X - mMax.X) < eps) hit.Normal = { 1, 0, 0 };
            else if (std::abs(hit.Position.Y - mMin.Y) < eps) hit.Normal = { 0,-1, 0 };
            else if (std::abs(hit.Position.Y - mMax.Y) < eps) hit.Normal = { 0, 1, 0 };
            else if (std::abs(hit.Position.Z - mMin.Z) < eps) hit.Normal = { 0, 0,-1 };
            else                                              hit.Normal = { 0, 0, 1 };  // fallback

            return hit;
        }

        // Check if this box contains a point
        bool Contains(const Vec3& point) const
        {
            return mMin.X <= point.X
                && mMin.Y <= point.Y
                && mMin.Z <= point.Z
                && mMax.X >= point.X
                && mMax.Y >= point.Y
                && mMax.Z >= point.Z;
        }

    private:
        Vec3 mCentre;
        Vec3 mHalfExtent;   // half-extent
        Vec3 mMin, mMax;    // cached slabs
    };
} // namespace JPL