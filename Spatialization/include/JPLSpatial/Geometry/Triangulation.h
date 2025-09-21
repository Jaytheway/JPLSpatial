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

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/MinimalVec2.h"
#include "JPLSpatial/Math/Vec3Traits.h"

#include <vector>
#include <span>
#include <cmath>
#include <algorithm>

namespace JPL
{
	namespace Triangulation
	{
        //======================================================================
        //  Helper types / maths
        //======================================================================
    
        static float GetPolygonArea(std::span<const Vec2> p)
        {
            float a = 0.0f;
            const size_t n = p.size();
            for (size_t i = 0, j = n - 1; i < n; j = i++)
                a += CrossProduct(p[j], p[i]);
            return 0.5f * a;
        }

        static bool IsPointInTri(const Vec2& p, const Vec2& a, const Vec2& b, const Vec2& c)
        {
            // barycentric sign tests (CCW triangle assumed)
            const float w0 = CrossProduct(Vec2{ b.X - a.X, b.Y - a.Y }, Vec2{ p.X - a.X, p.Y - a.Y });
            const float w1 = CrossProduct(Vec2{ c.X - b.X, c.Y - b.Y }, Vec2{ p.X - b.X, p.Y - b.Y });
            const float w2 = CrossProduct(Vec2{ a.X - c.X, a.Y - c.Y }, Vec2{ p.X - c.X, p.Y - c.Y });
            return (w0 >= 0 && w1 >= 0 && w2 >= 0);
        }

        static float GetMinAngleDeg(const Vec2& a, const Vec2& b, const Vec2& c)
        {
            auto angle = [&](const Vec2& p, const Vec2& q, const Vec2& r)
            {
                Vec2 u{ p.X - q.X, p.Y - q.Y };
                Vec2 v{ r.X - q.X, r.Y - q.Y };
                float d = DotProduct(u, v) / std::sqrt(DotProduct(u, u) * DotProduct(v, v));
                d = std::clamp(d, -1.0f, 1.0f);
                return std::acos(d);                  // radians
            };
            const float a1 = angle(b, a, c);
            const float a2 = angle(a, b, c);
            const float a3 = angle(a, c, b);
            return std::min({ a1, a2, a3 }) * 57.29578f; // to degrees
        }

        //======================================================================
        // Perform an ear-clipping triangulation that always clips the ear
        // whose triangle has the largest minimum interior angle
        // - a cheap heuristic that strongly discourages long, skinny slivers.
        template<CVec3Accessible Vec3, class Vec3i>
        void TriangulatePoly(std::span<const Vec3> inPositions, std::span<int> poly, std::vector<Vec3i>& outTris)
        {
            using IndexType = std::remove_cvref_t<decltype(std::declval<Vec3i>()[0])>;

            // 1. Build projection basis

            // Robust polygon normal (Newell)
            Vec3 n{ 0,0,0 };
            for (size_t i = 0, j = poly.size() - 1; i < poly.size(); j = i++)
            {
                const Vec3& p = inPositions[poly[i]];
                const Vec3& q = inPositions[poly[j]];
                SetX(n, GetX(n) + (GetY(p) - GetY(q)) * (GetZ(p) + GetZ(q)));
                SetY(n, GetY(n) + (GetZ(p) - GetZ(q)) * (GetX(p) + GetX(q)));
                SetZ(n, GetZ(n) + (GetX(p) - GetX(q)) * (GetY(p) + GetY(q)));
            }
            Normalize(n);

#if 0
            // Drop the dominant axis of the normal to get a 2-D view
            enum Axis { X = 0, Y, Z } drop =
                (Math::Abs(GetX(n)) > Math::Abs(GetY(n)) && Math::Abs(GetX(n)) > Math::Abs(GetZ(n))) ? X :
                (Math::Abs(GetY(n)) > Math::Abs(GetZ(n))) ? Y : Z;

            auto project = [&](const Vec3& v) -> Vec2
            {
                switch (drop)
                {
                case X: return { GetY(v), GetZ(v) };
                case Y: return { GetZ(v), GetX(v) };
                case Z: return { GetX(v), GetY(v) };
                }
                return { 0,0 };
            };
#else
           // Better angle precision, but slightly more expansive than simply dropping an axis

           // build orthonormal basis (u,v) in the plane of 'n'
           const Vec3 u = Normalized(Math::Abs(GetZ(n)) > 0.9f ? Vec3{ 1,0,0 } : CrossProduct(Vec3{ 0,0,1 }, n));
           const Vec3 v = CrossProduct(n, u); // already unit length
           auto project = [&](const Vec3& p) -> Vec2
           {
               return { DotProduct(p, u),  DotProduct(p, v) };
           };
#endif
            std::vector<Vec2> pts2;
            pts2.reserve(poly.size());
            for (int idx : poly)
                pts2.push_back(project(inPositions[idx]));

            // 3. Ensure counter-clockwise orientation

            if (GetPolygonArea(pts2) < 0.0f)
            {
                std::reverse(poly.begin(), poly.end());
                std::reverse(pts2.begin(), pts2.end());
            }

            // 4. Ear-clipping with quality heuristic

            struct Node
            {
                int Idx;          // original vertex index
                Vec2 P;           // projected coords
                int Prev, Next;   // doubly-linked list in 'nodes'
            };
            std::vector<Node> nodes;
            nodes.reserve(poly.size());
            for (size_t i = 0; i < poly.size(); ++i)
            {
                nodes.push_back({ poly[i], pts2[i],
                                  int(i == 0 ? poly.size() - 1 : i - 1),
                                  int((i + 1) % poly.size()) });
            }

            auto isConvex = [&](int i)
            {
                const Vec2& a = nodes[nodes[i].Prev].P;
                const Vec2& b = nodes[i].P;
                const Vec2& c = nodes[nodes[i].Next].P;
                return CrossProduct(Vec2{ b.X - a.X, b.Y - a.Y },
                                    Vec2{ c.X - b.X, c.Y - b.Y }) >= 0.0f;
            };

            int remaining = int(nodes.size());
            while (remaining > 3)
            {
                // Find the "best" ear this sweep
                int best = -1;
                float bestQuality = -1.0f; // larger is better
                for (int i = 0; i < nodes.size(); ++i)
                {
                    if (nodes[i].Idx == -1)
                        continue; // already clipped

                    if (!isConvex(i))
                        continue;

                    const Vec2& a = nodes[nodes[i].Prev].P;
                    const Vec2& b = nodes[i].P;
                    const Vec2& c = nodes[nodes[i].Next].P;

                    bool ear = true;
                    for (int j = 0; j < nodes.size() && ear; ++j)
                    {
                        if (nodes[j].Idx == -1 || j == i ||
                            j == nodes[i].Prev || j == nodes[i].Next)
                            continue;

                        ear &= !IsPointInTri(nodes[j].P, a, b, c);
                    }
                    if (!ear)
                        continue;

                    const float q = GetMinAngleDeg(a, b, c); // quality metric
                    if (q > bestQuality)
                    {
                        bestQuality = q;
                        best = i;
                    }
                }

                if (best == -1)
                    break; // should not happen

                // Emit triangle (original vertex indices, still CCW)
                outTris.emplace_back(
                    Vec3i{
                        IndexType(nodes[nodes[best].Prev].Idx),
                        IndexType(nodes[best].Idx),
                        IndexType(nodes[nodes[best].Next].Idx)
                    });

                // Remove the ear-tip from the polygon
                const int prev = nodes[best].Prev;
                const int next = nodes[best].Next;
                nodes[prev].Next = next;
                nodes[next].Prev = prev;
                nodes[best].Idx = -1; // mark removed
                --remaining;
            }

            // 5. Final triangle

            int vi[3], k = 0;
            for (const Node& n : nodes)
            {
                if (n.Idx != -1)
                    vi[k++] = n.Idx;
            }

            if (k == 3)
            {
                outTris.emplace_back(
                    Vec3i{
                        IndexType(vi[0]),
                        IndexType(vi[1]),
                        IndexType(vi[2])
                    });
            }
        }

	} // namespace Triangulation
} // namespace JPL
