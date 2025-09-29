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

#include "JPLSpatial/ErrorReporting.h"

#include "JPLSpatial/Math/Math.h"
#include "JPLSpatial/Math/Vec3Traits.h"
#include "JPLSpatial/Math/ClosestPoint.h"
#include "JPLSpatial/Geometry/Triangulation.h"

#include <vector>
#include <memory>
#include <span>
#include <unordered_set>

#ifndef JPL_ENABLE_VALIDATION
//#define JPL_ENABLE_VALIDATION
#endif

#if JPL_ENABLE_VALIDATION
#include <format>
#include <iostream>
#endif // JPL_ENABLE_VALIDATION

namespace JPL
{

	//==========================================================================
	/// Class to generate convex hull from a list of points,
	/// suitable for generating speaker mesh for VBAP.
	/// 
	/// Note: This is a slightly modified version of ConvexHullBuilder
	/// from JoltPhysics library
	template<CVec3 Vec3Type, template<class> class VectorAllocatorType = std::allocator>
	class ConvexHullBuilder
	{
		using Vec3 = Vec3Type;

		template<class T>
		using Array = std::vector<T, VectorAllocatorType<T>>;

		ConvexHullBuilder(const ConvexHullBuilder&) = delete;
		ConvexHullBuilder& operator=(const ConvexHullBuilder&) = delete;
	public:
		// Forward declare
		class Face;

		/// Class that holds the information of an edge
		class Edge
		{
			Edge(const Edge&) = delete;
			Edge& operator =(const Edge&) = delete;
		public:
			Edge(Face* inFace, int inStartIdx);

			/// Get the previous edge
			Edge* GetPreviousEdge();

			Face* mFace;					///< Face that this edge belongs to
			Edge* mNextEdge = nullptr;		///< Next edge of this face
			Edge* mNeighbourEdge = nullptr;	///< Edge that this edge is connected to
			int	mStartIdx;					///< Vertex index in mPositions that indicates the start vertex of this edge
		};

		using ConflictList = Array<int>;

		/// Class that holds the information of one face
		class Face final
		{
			Face(const Face&) = delete;
			Face& operator=(const Face&) = delete;
		public:
			Face() = default;
			~Face();

			/// Initialize a face with three indices
			void Initialize(int inIdx0, int inIdx1, int inIdx2, std::span<const Vec3> inPositions);

			/// Calculates the centroid and normal for this face
			void CalculateNormalAndCentroid(std::span<const Vec3> inPositions);

			template<class Vec3i>
			bool Triangulate(std::span<const Vec3> inPositions, Array<Vec3i>& outTris) const;

			/// Check if face inFace is facing inPosition
			bool IsFacing(const Vec3& inPosition) const;

			Vec3 mNormal;							///< Normal of this face, length is 2 times area of face
			Vec3 mCentroid;							///< Center of the face
			ConflictList mConflictList;				///< Positions associated with this edge (that are closest to this edge). The last position in the list is the point that is furthest away from the face.
			Edge* mFirstEdge = nullptr;				///< First edge of this face
			float mFurthestPointDistanceSq = 0.0f;	///< Squared distance of furthest point from the conflict list to the face
			bool mRemoved = false;					///< Flag that indicates that face has been removed (face will be freed later)
#ifdef JPL_CONVEX_BUILDER_DEBUG
			int	mIteration;							///< Iteration that this face was created
#endif
		};

		// Typedefs
		using Positions = std::span<const Vec3>;
		using Faces = Array<Face*>;

		explicit ConvexHullBuilder(Positions inPositions);
		~ConvexHullBuilder();

		/// Result enum that indicates how the hull got created
		enum class EResult
		{
			Success,			///< Hull building finished successfully
			MaxVerticesReached,	///< Hull building finished successfully, but the desired accuracy was not reached because the max vertices limit was reached
			TooFewPoints,		///< Too few points to create a hull
			TooFewFaces,		///< Too few faces in the created hull (signifies precision errors during building)
			Degenerate,			///< Degenerate hull detected
		};

		/// Takes all positions as provided by the constructor and use them to build a hull
		/// Any points that are closer to the hull than inTolerance will be discarded
		/// @param inMaxVertices Max vertices to allow in the hull. Specify INT_MAX if there is no limit.
		/// @param inTolerance Max distance that a point is allowed to be outside of the hull
		/// @param outError Error message when building fails
		/// @return Status code that reports if the hull was created or not
		EResult Initialize(int inMaxVertices, float inTolerance, const char*& outError);

		/// Returns the amount of vertices that are currently used by the hull
		int	GetNumVerticesUsed() const;

		/// Returns true if the hull contains a polygon with inIndices (counter clockwise indices in mPositions)
		bool ContainsFace(const Array<int>& inIndices) const;

#if 0
		/// Determines the point that is furthest outside of the hull and reports how far it is outside of the hull (which indicates a failure during hull building)
		/// @param outFaceWithMaxError The face that caused the error
		/// @param outMaxError The maximum distance of a point to the hull
		/// @param outMaxErrorPositionIdx The index of the point that had this distance
		/// @param outCoplanarDistance Points that are less than this distance from the hull are considered on the hull. This should be used as a lowerbound for the allowed error.
		void DetermineMaxError(Face*& outFaceWithMaxError, float& outMaxError, int& outMaxErrorPositionIdx, float& outCoplanarDistance) const;
#endif

		/// Access to the created faces. Memory is owned by the convex hull builder.
		const Faces& GetFaces() const { return mFaces; }

		/// Get triangle indices for all faces in the hull.
		/// Note: any faces that are polygons will be triangulated.
		template<class Vec3i>
		void GetTriangles(Array<Vec3i>& outTris);

	private:
		/// Minimal square area of a triangle (used for merging and checking if a triangle is degenerate)
		static constexpr float cMinTriangleAreaSq = 1.0e-12f;

#ifdef JPL_CONVEX_BUILDER_DEBUG
		/// Factor to scale convex hull when debug drawing the construction process
		static constexpr Real cDrawScale = 10;
#endif

		/// Class that holds an edge including start and end index
		class FullEdge
		{
		public:
			Edge* mNeighbourEdge;	///< Edge that this edge is connected to
			int	mStartIdx;			///< Vertex index in mPositions that indicates the start vertex of this edge
			int	mEndIdx;			///< Vertex index in mPosition that indicates the end vertex of this edge
		};

		// Private typedefs
		using FullEdges = Array<FullEdge>;

		/// Determine a suitable tolerance for detecting that points are coplanar
		float DetermineCoplanarDistance() const;

		/// Find the face for which inPoint is furthest to the front
		/// @param inPoint Point to test
		/// @param inFaces List of faces to test
		/// @param outFace Returns the best face
		/// @param outDistSq Returns the squared distance how much inPoint is in front of the plane of the face
		void GetFaceForPoint(const Vec3& inPoint, const Faces& inFaces, Face*& outFace, float& outDistSq) const;

		/// @brief Calculates the distance between inPoint and inFace
		/// @param inFace Face to test
		/// @param inPoint Point to test
		/// @return If the projection of the point on the plane is interior to the face 0, otherwise the squared distance to the closest edge
		float GetDistanceToEdgeSq(const Vec3& inPoint, const Face* inFace) const;

		/// Assigns a position to one of the supplied faces based on which face is closest.
		/// @param inPositionIdx Index of the position to add
		/// @param inFaces List of faces to consider
		/// @param inToleranceSq Tolerance of the hull, if the point is closer to the face than this, we ignore it
		/// @return True if point was assigned, false if it was discarded or added to the coplanar list
		bool AssignPointToFace(int inPositionIdx, const Faces& inFaces, float inToleranceSq);

		/// Add a new point to the convex hull
		void AddPoint(Face* inFacingFace, int inIdx, float inCoplanarToleranceSq, Faces& outNewFaces);

		/// Remove all faces that have been marked 'removed' from mFaces list
		void GarbageCollectFaces();

		/// Create a new face
		Face* CreateFace();

		/// Create a new triangle
		Face* CreateTriangle(int inIdx1, int inIdx2, int inIdx3);

		/// Delete a face (checking that it is not connected to any other faces)
		void FreeFace(Face* inFace);

		/// Release all faces and edges
		void FreeFaces();
		/// Link face edge to other face edge
		static void	 sLinkFace(Edge* inEdge1, Edge* inEdge2);

		/// Unlink this face from all of its neighbours
		static void sUnlinkFace(Face* inFace);

		/// Given one face that faces inVertex, find the edges of the faces that are not facing inVertex.
		/// Will flag all those faces for removal.
		void FindEdge(Face* inFacingFace, const Vec3& inVertex, FullEdges& outEdges) const;

		/// Merges the two faces that share inEdge into the face inEdge->mFace
		void MergeFaces(Edge* inEdge);

		/// Merges inFace with a neighbour if it is degenerate (a sliver)
		void MergeDegenerateFace(Face* inFace, Faces& ioAffectedFaces);

		/// Merges any coplanar as well as neighbours that form a non-convex edge into inFace.
		/// Faces are considered coplanar if the distance^2 of the other face's centroid is smaller than inToleranceSq.
		void MergeCoplanarOrConcaveFaces(Face* inFace, float inCoplanarToleranceSq, Faces& ioAffectedFaces);

		/// Mark face as affected if it is not already in the list
		static void sMarkAffected(Face* inFace, Faces& ioAffectedFaces);

		/// Removes all invalid edges.
		/// 1. Merges inFace with faces that share two edges with it since this means inFace or the other face cannot be convex or the edge is colinear.
		/// 2. Removes edges that are interior to inFace (that have inFace on both sides)
		/// Any faces that need to be checked for validity will be added to ioAffectedFaces.
		void RemoveInvalidEdges(Face* inFace, Faces& ioAffectedFaces);

		/// Removes inFace if it consists of only 2 edges, linking its neighbouring faces together
		/// Any faces that need to be checked for validity will be added to ioAffectedFaces.
		/// @return True if face was removed.
		bool RemoveTwoEdgeFace(Face* inFace, Faces& ioAffectedFaces) const;

#ifdef JPL_ENABLE_VALIDATION
		/// Dumps the text representation of a face to the TTY
		void DumpFace(const Face* inFace) const;

		/// Check consistency of 1 face
		void ValidateFace(const Face* inFace) const;

		/// Check consistency of all faces
		void ValidateFaces() const;
#endif

#ifdef JPL_CONVEX_BUILDER_DEBUG
		/// Draw state of algorithm
		void DrawState(bool inDrawConflictList = false) const;

		/// Draw a face for debugging purposes
		void DrawWireFace(const Face* inFace, ColorArg inColor) const;

		/// Draw an edge for debugging purposes
		void DrawEdge(const Edge* inEdge, ColorArg inColor) const;
#endif

#ifdef JPL_CONVEX_BUILDER_DUMP_SHAPE
		void DumpShape() const;
#endif

		const Positions& mPositions;				///< List of positions (some of them are part of the hull)
		Faces mFaces;								///< List of faces that are part of the hull (if !mRemoved)

		struct Coplanar
		{
			int mPositionIdx;						///< Index in mPositions
			float mDistanceSq;						///< Distance to the edge of closest face (should be > 0)
		};
		using CoplanarList = Array<Coplanar>;

		CoplanarList mCoplanarList;					///< List of positions that are coplanar to a face but outside of the face, these are added to the hull at the end

#ifdef JPL_CONVEX_BUILDER_DEBUG
		int mIteration;			///< Number of iterations we've had so far (for debug purposes)
		mutable RVec3 mOffset;	///< Offset to use for state drawing
		Vec3 mDelta;			///< Delta offset between next states
#endif
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
	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::Edge::Edge(ConvexHullBuilder<Vec3Type, VectorAllocator>::Face* inFace, int inStartIdx)
		: mFace(inFace), mStartIdx(inStartIdx)
	{
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::Edge* ConvexHullBuilder<Vec3Type, VectorAllocator>::Edge::GetPreviousEdge()
	{
		Edge* prev_edge = this;
		while (prev_edge->mNextEdge != this)
			prev_edge = prev_edge->mNextEdge;
		return prev_edge;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::Face::~Face()
	{
		// Free all edges
		if (Edge* e = mFirstEdge)
		{
			do
			{
				Edge* next = e->mNextEdge;
				delete e;
				e = next;
			} while (e != mFirstEdge);
		}
	}

	//==========================================================================
	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::Face::Initialize(int inIdx0, int inIdx1, int inIdx2, std::span<const Vec3> inPositions)
	{
		JPL_ASSERT(mFirstEdge == nullptr);
		JPL_ASSERT(inIdx0 != inIdx1 && inIdx0 != inIdx2 && inIdx1 != inIdx2);

		// Create 3 edges
		auto* e0 = new Edge(this, inIdx0);
		auto* e1 = new Edge(this, inIdx1);
		auto* e2 = new Edge(this, inIdx2);

		// Link edges
		e0->mNextEdge = e1;
		e1->mNextEdge = e2;
		e2->mNextEdge = e0;
		mFirstEdge = e0;

		CalculateNormalAndCentroid(inPositions);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::Face::CalculateNormalAndCentroid(std::span<const Vec3> inPositions)
	{
		// Get point that we use to construct a triangle fan
		Edge* e = mFirstEdge;
		Vec3 y0 = inPositions[e->mStartIdx];

		// Get the 2nd point
		e = e->mNextEdge;
		Vec3 y1 = inPositions[e->mStartIdx];

		// Start accumulating the centroid
		mCentroid = y0 + y1;
		int n = 2;

		// Start accumulating the normal
		mNormal = Vec3{ 0.0f, 0.0f, 0.0f };

		// Loop over remaining edges accumulating normals in a triangle fan fashion
		for (e = e->mNextEdge; e != mFirstEdge; e = e->mNextEdge)
		{
			// Get the 3rd point
			Vec3 y2 = inPositions[e->mStartIdx];

			// Calculate edges (counter clockwise)
			Vec3 e0 = y1 - y0;
			Vec3 e1 = y2 - y1;
			Vec3 e2 = y0 - y2;

			// The best normal is calculated by using the two shortest edges
			// See: https://box2d.org/posts/2014/01/troublesome-triangle/
			// The difference in normals is most pronounced when one edge is much smaller than the others (in which case the others must have roughly the same length).
			// Therefore we can suffice by just picking the shortest from 2 edges and use that with the 3rd edge to calculate the normal.
			mNormal += LengthSquared(e1) < LengthSquared(e2) ? CrossProduct(e0, e1) : CrossProduct(e2, e0);

			// Accumulate centroid
			mCentroid += y2;
			n++;

			// Update y1 for next triangle
			y1 = y2;
		}

		// Finalize centroid
		mCentroid /= float(n);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	template<class Vec3i>
	inline bool ConvexHullBuilder<Vec3Type, VectorAllocator>::Face::Triangulate(std::span<const Vec3> inPositions, Array<Vec3i>& outTris) const
	{
		using IndexType = std::remove_cvref_t<decltype(std::declval<Vec3i>()[0])>;

		Array<int> poly;
		poly.reserve(32);

		// Collect boundary indices
		poly.push_back(mFirstEdge->mStartIdx);
		for (Edge* e = mFirstEdge->mNextEdge; e != mFirstEdge; e = e->mNextEdge)
			poly.push_back(e->mStartIdx);

		if (poly.size() < 3)
			return false; // degenerate

		if (poly.size() == 3)
		{
			// Single triangle face
			outTris.push_back(Vec3i{
				static_cast<IndexType>(poly[0]),
				static_cast<IndexType>(poly[1]),
				static_cast<IndexType>(poly[2])
							  });
		}
		else
		{
			// Polygon face, needs triangulation
			Triangulation::TriangulatePoly(inPositions, poly, outTris);
		}

		return true;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline bool ConvexHullBuilder<Vec3Type, VectorAllocator>::Face::IsFacing(const Vec3& inPosition) const
	{
		JPL_ASSERT(!mRemoved);
		return DotProduct(mNormal, (inPosition - mCentroid)) > 0.0f;
	}

	//==========================================================================
	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::ConvexHullBuilder(Positions inPositions)
		: mPositions(inPositions)
	{
#ifdef JPL_CONVEX_BUILDER_DEBUG
		mIteration = 0;

		// Center the drawing of the first hull around the origin and calculate the delta offset between states
		mOffset = RVec3::sZero();
		if (mPositions.empty())
		{
			// No hull will be generated
			mDelta = Vec3::sZero();
		}
		else
		{
			Vec3 maxv = Vec3::sReplicate(-FLT_MAX), minv = Vec3::sReplicate(FLT_MAX);
			for (Vec3 v : mPositions)
			{
				minv = Vec3::sMin(minv, v);
				maxv = Vec3::sMax(maxv, v);
				mOffset -= v;
			}
			mOffset /= Real(mPositions.size());
			mDelta = Vec3((maxv - minv).GetX() + 0.5f, 0, 0);
			mOffset += mDelta; // Don't start at origin, we're already drawing the final hull there
		}
#endif
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::~ConvexHullBuilder()
	{
		FreeFaces();
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::EResult ConvexHullBuilder<Vec3Type, VectorAllocator>::Initialize(int inMaxVertices, float inTolerance, const char*& outError)
	{
		// Free the faces possibly left over from an earlier hull
		FreeFaces();

		// Test that we have at least 3 points
		if (mPositions.size() < 3)
		{
			outError = "Need at least 3 points to make a hull";
			return EResult::TooFewPoints;
		}

		// Determine a suitable tolerance for detecting that points are coplanar
		float coplanar_tolerance_sq = DetermineCoplanarDistance();
		coplanar_tolerance_sq *= coplanar_tolerance_sq;

		// Increase desired tolerance if accuracy doesn't allow it
		const auto sqrTolerance = inTolerance * inTolerance;
		float tolerance_sq = std::max(coplanar_tolerance_sq, sqrTolerance);

		// Find point furthest from the origin
		int idx1 = -1;
		float max_dist_sq = -1.0f;
		for (int i = 0; i < (int)mPositions.size(); ++i)
		{
			float dist_sq = LengthSquared(mPositions[i]);
			if (dist_sq > max_dist_sq)
			{
				max_dist_sq = dist_sq;
				idx1 = i;
			}
		}
		JPL_ASSERT(idx1 >= 0);

		// Find point that is furthest away from this point
		int idx2 = -1;
		max_dist_sq = -1.0f;
		for (int i = 0; i < (int)mPositions.size(); ++i)
		{
			if (i != idx1)
			{
				float dist_sq = LengthSquared(mPositions[i] - mPositions[idx1]);
				if (dist_sq > max_dist_sq)
				{
					max_dist_sq = dist_sq;
					idx2 = i;
				}
			}
		}
		JPL_ASSERT(idx2 >= 0);

		// Find point that forms the biggest triangle
		// TODO: we may not need this, since we actually want small triangles, just not narrow
		int idx3 = -1;
		float best_triangle_area_sq = -1.0f;
		for (int i = 0; i < (int)mPositions.size(); ++i)
		{
			if (i != idx1 && i != idx2)
			{
				float triangle_area_sq = LengthSquared(CrossProduct(mPositions[idx1] - mPositions[i], mPositions[idx2] - mPositions[i]));
				if (triangle_area_sq > best_triangle_area_sq)
				{
					best_triangle_area_sq = triangle_area_sq;
					idx3 = i;
				}
			}
		}

		JPL_ASSERT(idx3 >= 0);
		if (best_triangle_area_sq < cMinTriangleAreaSq)
		{
			outError = "Could not find a suitable initial triangle because its area was too small";
			return EResult::Degenerate;
		}

		// Check if we have only 3 vertices
		if (mPositions.size() == 3)
		{
			// Create two triangles (back to back)
			// TODO: we may only want a single triangle, instead of two faces of same triangle
			Face* t1 = CreateTriangle(idx1, idx2, idx3);
			Face* t2 = CreateTriangle(idx1, idx3, idx2);

			// Link faces edges
			// TODO: we may not need to link faces
			sLinkFace(t1->mFirstEdge, t2->mFirstEdge->mNextEdge->mNextEdge);
			sLinkFace(t1->mFirstEdge->mNextEdge, t2->mFirstEdge->mNextEdge);
			sLinkFace(t1->mFirstEdge->mNextEdge->mNextEdge, t2->mFirstEdge);

#ifdef JPL_CONVEX_BUILDER_DEBUG
			// Draw current state
			DrawState();
#endif

			return EResult::Success;
		}

		// Find point that forms the biggest tetrahedron
		Vec3 initial_plane_normal = Normalized(CrossProduct(mPositions[idx2] - mPositions[idx1], mPositions[idx3] - mPositions[idx1]));
		Vec3 initial_plane_centroid = (mPositions[idx1] + mPositions[idx2] + mPositions[idx3]) / 3.0f;
		int idx4 = -1;
		float max_dist = 0.0f;
		for (int i = 0; i < (int)mPositions.size(); ++i)
		{
			if (i != idx1 && i != idx2 && i != idx3)
			{
				float dist = DotProduct(mPositions[i] - initial_plane_centroid, initial_plane_normal);
				if (std::abs(dist) > std::abs(max_dist))
				{
					max_dist = dist;
					idx4 = i;
				}
			}
		}

		// Check if the hull is coplanar
		if ((max_dist * max_dist) <= 25.0f * coplanar_tolerance_sq)
		{
			//! Coplanar hull is invalid for 3D VBAP
			// TODO: (may be still useful if we reuse this hull builder for other stuff
#if 1
			JPL_ENSURE(false);
			return EResult::Degenerate;
#else
				// First project all points in 2D space
			Vec3 base1 = GetNormalizedPerpendicular(initial_plane_normal);
			Vec3 base2 = CrossProduct(initial_plane_normal, base1);
			Array<Vec3> positions_2d;
			positions_2d.reserve(mPositions.size());
			for (Vec3 v : mPositions)
				positions_2d.emplace_back(DotProduct(base1, v), DotProduct(base2, v), 0.0f);

			// Build hull
			Array<int> edges_2d;
			ConvexHullBuilder2D builder_2d(positions_2d);
			ConvexHullBuilder2D::EResult result = builder_2d.Initialize(idx1, idx2, idx3, inMaxVertices, inTolerance, edges_2d);

			// Create faces (back to back)
			Face* f1 = CreateFace();
			Face* f2 = CreateFace();

			// Create edges for face 1
			Array<Edge*> edges_f1;
			edges_f1.reserve(edges_2d.size());
			for (int start_idx : edges_2d)
			{
				Edge* edge = new Edge(f1, start_idx);
				if (edges_f1.empty())
					f1->mFirstEdge = edge;
				else
					edges_f1.back()->mNextEdge = edge;
				edges_f1.push_back(edge);
			}
			edges_f1.back()->mNextEdge = f1->mFirstEdge;

			// Create edges for face 2
			Array<Edge*> edges_f2;
			edges_f2.reserve(edges_2d.size());
			for (int i = (int)edges_2d.size() - 1; i >= 0; --i)
			{
				Edge* edge = new Edge(f2, edges_2d[i]);
				if (edges_f2.empty())
					f2->mFirstEdge = edge;
				else
					edges_f2.back()->mNextEdge = edge;
				edges_f2.push_back(edge);
			}
			edges_f2.back()->mNextEdge = f2->mFirstEdge;

			// Link edges
			for (size_t i = 0; i < edges_2d.size(); ++i)
				sLinkFace(edges_f1[i], edges_f2[(2 * edges_2d.size() - 2 - i) % edges_2d.size()]);

			// Calculate the plane for both faces
			f1->CalculateNormalAndCentroid(mPositions.data());
			f2->mNormal = -f1->mNormal;
			f2->mCentroid = f1->mCentroid;

#ifdef JPL_CONVEX_BUILDER_DEBUG
			// Draw current state
			DrawState();
#endif

			return result == ConvexHullBuilder2D::EResult::MaxVerticesReached ? EResult::MaxVerticesReached : EResult::Success;
#endif
		}

		// Ensure the planes are facing outwards
		if (max_dist < 0.0f)
			std::swap(idx2, idx3);

		// Create tetrahedron
		Face* t1 = CreateTriangle(idx1, idx2, idx4);
		Face* t2 = CreateTriangle(idx2, idx3, idx4);
		Face* t3 = CreateTriangle(idx3, idx1, idx4);
		Face* t4 = CreateTriangle(idx1, idx3, idx2);

		// Link face edges
		sLinkFace(t1->mFirstEdge, t4->mFirstEdge->mNextEdge->mNextEdge);
		sLinkFace(t1->mFirstEdge->mNextEdge, t2->mFirstEdge->mNextEdge->mNextEdge);
		sLinkFace(t1->mFirstEdge->mNextEdge->mNextEdge, t3->mFirstEdge->mNextEdge);
		sLinkFace(t2->mFirstEdge, t4->mFirstEdge->mNextEdge);
		sLinkFace(t2->mFirstEdge->mNextEdge, t3->mFirstEdge->mNextEdge->mNextEdge);
		sLinkFace(t3->mFirstEdge, t4->mFirstEdge);

		// Build the initial conflict lists
		Faces faces{ t1, t2, t3, t4 };
		for (int idx = 0; idx < (int)mPositions.size(); ++idx)
			if (idx != idx1 && idx != idx2 && idx != idx3 && idx != idx4)
				AssignPointToFace(idx, faces, tolerance_sq);

#ifdef JPL_CONVEX_BUILDER_DEBUG
		// Draw current state including conflict list
		DrawState(true);

		// Increment iteration counter
		++mIteration;
#endif

		// Overestimate of the actual amount of vertices we use, for limiting the amount of vertices in the hull
		int num_vertices_used = 4;

		// Loop through the remainder of the points and add them
		for (;;)
		{
			// Find the face with the furthest point on it
			Face* face_with_furthest_point = nullptr;
			float furthest_dist_sq = 0.0f;
			for (Face* f : mFaces)
			{
				if (f->mFurthestPointDistanceSq > furthest_dist_sq)
				{
					furthest_dist_sq = f->mFurthestPointDistanceSq;
					face_with_furthest_point = f;
				}
			}

			int furthest_point_idx;
			if (face_with_furthest_point != nullptr)
			{
				// Take the furthest point
				furthest_point_idx = face_with_furthest_point->mConflictList.back();
				face_with_furthest_point->mConflictList.pop_back();
			}
			else if (!mCoplanarList.empty())
			{
				// Try to assign points to faces (this also recalculates the distance to the hull for the coplanar vertices)
				CoplanarList coplanar;
				mCoplanarList.swap(coplanar);
				bool added = false;
				for (const Coplanar& c : coplanar)
					added |= AssignPointToFace(c.mPositionIdx, mFaces, tolerance_sq);

				// If we were able to assign a point, loop again to pick it up
				if (added)
					continue;

				// If the coplanar list is empty, there are no points left and we're done
				if (mCoplanarList.empty())
					break;

				do
				{
					// Find the vertex that is furthest from the hull
					typename CoplanarList::size_type best_idx = 0;
					float best_dist_sq = mCoplanarList.front().mDistanceSq;
					for (typename CoplanarList::size_type idx = 1; idx < mCoplanarList.size(); ++idx)
					{
						const Coplanar& c = mCoplanarList[idx];
						if (c.mDistanceSq > best_dist_sq)
						{
							best_idx = idx;
							best_dist_sq = c.mDistanceSq;
						}
					}

					// Swap it to the end
					std::swap(mCoplanarList[best_idx], mCoplanarList.back());

					// Remove it
					furthest_point_idx = mCoplanarList.back().mPositionIdx;
					mCoplanarList.pop_back();

					// Find the face for which the point is furthest away
					GetFaceForPoint(mPositions[furthest_point_idx], mFaces, face_with_furthest_point, best_dist_sq);
				} while (!mCoplanarList.empty() && face_with_furthest_point == nullptr);

				if (face_with_furthest_point == nullptr)
					break;
			}
			else
			{
				// If there are no more vertices, we're done
				break;
			}

			// Check if we have a limit on the max vertices that we should produce
			if (num_vertices_used >= inMaxVertices)
			{
				// Count the actual amount of used vertices (we did not take the removal of any vertices into account)
				num_vertices_used = GetNumVerticesUsed();

				// Check if there are too many
				if (num_vertices_used >= inMaxVertices)
					return EResult::MaxVerticesReached;
			}

			// We're about to add another vertex
			++num_vertices_used;

			// Add the point to the hull
			Faces new_faces;
			AddPoint(face_with_furthest_point, furthest_point_idx, coplanar_tolerance_sq, new_faces);

			// Redistribute points on conflict lists belonging to removed faces
			for (const Face* face : mFaces)
				if (face->mRemoved)
					for (int idx : face->mConflictList)
						AssignPointToFace(idx, new_faces, tolerance_sq);

			// Permanently delete faces that we removed in AddPoint()
			GarbageCollectFaces();

#ifdef JPL_CONVEX_BUILDER_DEBUG
			// Draw state at the end of this step including conflict list
			DrawState(true);

			// Increment iteration counter
			++mIteration;
#endif
		}

		// Check if we are left with a hull. It is possible that hull building fails if the points are nearly coplanar.
		if (mFaces.size() < 2)
		{
			outError = "Too few faces in hull";
			return EResult::TooFewFaces;
		}

		return EResult::Success;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline int ConvexHullBuilder<Vec3Type, VectorAllocator>::GetNumVerticesUsed() const
	{
		std::unordered_set<int> used_verts;
		for (const Face* f : mFaces)
		{
			const Edge* e = f->mFirstEdge;
			do
			{
				used_verts.insert(e->mStartIdx);
				e = e->mNextEdge;
			} while (e != f->mFirstEdge);
		}
		return (int)used_verts.size();
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline bool ConvexHullBuilder<Vec3Type, VectorAllocator>::ContainsFace(const Array<int>& inIndices) const
	{
		for (Face* f : mFaces)
		{
			Edge* e = f->mFirstEdge;
			Array<int>::const_iterator index = std::find(inIndices.begin(), inIndices.end(), e->mStartIdx);
			if (index != inIndices.end())
			{
				size_t matches = 0;

				do
				{
					// Check if index matches
					if (*index != e->mStartIdx)
						break;

					// Increment number of matches
					matches++;

					// Next index in list of inIndices
					index++;
					if (index == inIndices.end())
						index = inIndices.begin();

					// Next edge
					e = e->mNextEdge;
				} while (e != f->mFirstEdge);

				if (matches == inIndices.size())
					return true;
			}
		}

		return false;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	template<class Vec3i>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::GetTriangles(Array<Vec3i>& outTris)
	{
		outTris.clear();
		outTris.reserve(mFaces.size());

		for (const Face* f : mFaces)
			f->Triangulate(mPositions, outTris);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline float ConvexHullBuilder<Vec3Type, VectorAllocator>::DetermineCoplanarDistance() const
	{
		auto vec3max = [](const Vec3& a, const Vec3& b)
		{
			return Vec3{ std::max(GetX(a), GetX(b)), std::max(GetY(a), GetY(b)), std::max(GetZ(a), GetZ(b)) };
		};
		auto vec3abs = [](const Vec3& v)
		{
			return Vec3{ std::abs(GetX(v)), std::abs(GetY(v)), std::abs(GetZ(v)) };
		};

		// Formula as per: Implementing Quickhull - Dirk Gregorius.
		Vec3 vmax{ 0.0f, 0.0f, 0.0f };
		for (Vec3 v : mPositions)
			vmax = vec3max(vmax, vec3abs(v));
		return 3.0f * FLT_EPSILON * (GetX(vmax) + GetY(vmax) + GetZ(vmax));
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::GetFaceForPoint(const Vec3& inPoint, const Faces& inFaces, Face*& outFace, float& outDistSq) const
	{
		outFace = nullptr;
		outDistSq = 0.0f;

		for (Face* f : inFaces)
		{
			if (!f->mRemoved)
			{
				// Determine distance to face
				const float dot = DotProduct(f->mNormal, inPoint - f->mCentroid);
				if (dot > 0.0f)
				{
					const float dist_sq = dot * dot / LengthSquared(f->mNormal);
					if (dist_sq > outDistSq)
					{
						outFace = f;
						outDistSq = dist_sq;
					}
				}
			}
		}
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline float ConvexHullBuilder<Vec3Type, VectorAllocator>::GetDistanceToEdgeSq(const Vec3& inPoint, const Face* inFace) const
	{
		bool all_inside = true;
		float edge_dist_sq = FLT_MAX;

		// Test if it is inside the edges of the polygon
		Edge* edge = inFace->mFirstEdge;
		Vec3 p1 = mPositions[edge->GetPreviousEdge()->mStartIdx];
		do
		{
			Vec3 p2 = mPositions[edge->mStartIdx];
			if (DotProduct(CrossProduct(p2 - p1, inPoint - p1), inFace->mNormal) < 0.0f)
			{
				// It is outside
				all_inside = false;

				// Measure distance to this edge
				uint32 s;
				edge_dist_sq = std::min(edge_dist_sq, LengthSquared(GetClosestPointOnLine(p1 - inPoint, p2 - inPoint, s)));
			}
			p1 = p2;
			edge = edge->mNextEdge;
		} while (edge != inFace->mFirstEdge);

		return all_inside ? 0.0f : edge_dist_sq;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline bool ConvexHullBuilder<Vec3Type, VectorAllocator>::AssignPointToFace(int inPositionIdx, const Faces& inFaces, float inToleranceSq)
	{
		Vec3 point = mPositions[inPositionIdx];

		// Find the face for which the point is furthest away
		Face* best_face;
		float best_dist_sq;
		GetFaceForPoint(point, inFaces, best_face, best_dist_sq);

		if (best_face != nullptr)
		{
			// Check if this point is within the tolerance margin to the plane
			if (best_dist_sq <= inToleranceSq)
			{
				// Check distance to edges
				float dist_to_edge_sq = GetDistanceToEdgeSq(point, best_face);
				if (dist_to_edge_sq > inToleranceSq)
				{
					// Point is outside of the face and too far away to discard
					mCoplanarList.push_back({ inPositionIdx, dist_to_edge_sq });
				}
			}
			else
			{
				// This point is in front of the face, add it to the conflict list
				if (best_dist_sq > best_face->mFurthestPointDistanceSq)
				{
					// This point is further away than any others, update the distance and add point as last point
					best_face->mFurthestPointDistanceSq = best_dist_sq;
					best_face->mConflictList.push_back(inPositionIdx);
				}
				else
				{
					// Not the furthest point, add it as the before last point
					best_face->mConflictList.insert(best_face->mConflictList.begin() + best_face->mConflictList.size() - 1, inPositionIdx);
				}

				return true;
			}
		}

		return false;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::AddPoint(Face* inFacingFace, int inIdx, float inCoplanarToleranceSq, Faces& outNewFaces)
	{
		// Get position
		Vec3 pos = mPositions[inIdx];

#ifdef JPL_CONVEX_BUILDER_DEBUG
		// Draw point to be added
		DebugRenderer::sInstance->DrawMarker(cDrawScale * (mOffset + pos), Color::sYellow, 0.1f);
		DebugRenderer::sInstance->DrawText3D(cDrawScale * (mOffset + pos), ConvertToString(inIdx), Color::sWhite);
#endif

#ifdef JPL_ENABLE_VALIDATION
		// Check if structure is intact
		ValidateFaces();
#endif

		// Find edge of convex hull of faces that are not facing the new vertex
		FullEdges edges;
		FindEdge(inFacingFace, pos, edges);
		JPL_ASSERT(edges.size() >= 3);

		// Create new faces
		outNewFaces.reserve(edges.size());
		for (const FullEdge& e : edges)
		{
			JPL_ASSERT(e.mStartIdx != e.mEndIdx);
			Face* f = CreateTriangle(e.mStartIdx, e.mEndIdx, inIdx);
			outNewFaces.push_back(f);
		}

		// Link edges
		for (typename Faces::size_type i = 0; i < outNewFaces.size(); ++i)
		{
			sLinkFace(outNewFaces[i]->mFirstEdge, edges[i].mNeighbourEdge);
			sLinkFace(outNewFaces[i]->mFirstEdge->mNextEdge, outNewFaces[(i + 1) % outNewFaces.size()]->mFirstEdge->mNextEdge->mNextEdge);
		}

		// Loop on faces that were modified until nothing needs to be checked anymore
		Faces affected_faces = outNewFaces;
		while (!affected_faces.empty())
		{
			// Take the next face
			Face* face = affected_faces.back();
			affected_faces.pop_back();

			if (!face->mRemoved)
			{
				// Merge with neighbour if this is a degenerate face
				MergeDegenerateFace(face, affected_faces);

				// Merge with coplanar neighbours (or when the neighbour forms a concave edge)
				//! We don't really need to merge coplanar faces, since we do need to retain triangulation.
				//! If we do the merge, we'll have to retriangulate later, which may or may not create beter topology.
#if 0
				if (!face->mRemoved)
					MergeCoplanarOrConcaveFaces(face, inCoplanarToleranceSq, affected_faces);
#endif
			}
		}

#ifdef JPL_ENABLE_VALIDATION
		// Check if structure is intact
		ValidateFaces();
#endif
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::GarbageCollectFaces()
	{
		for (int i = (int)mFaces.size() - 1; i >= 0; --i)
		{
			Face* f = mFaces[i];
			if (f->mRemoved)
			{
				FreeFace(f);
				mFaces.erase(mFaces.begin() + i);
			}
		}
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::Face* ConvexHullBuilder<Vec3Type, VectorAllocator>::CreateFace()
	{
		// Call provider to create face
		Face* f = new Face();

#ifdef JPL_CONVEX_BUILDER_DEBUG
		// Remember iteration counter
		f->mIteration = mIteration;
#endif

		// Add to list
		mFaces.push_back(f);
		return f;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline ConvexHullBuilder<Vec3Type, VectorAllocator>::Face* ConvexHullBuilder<Vec3Type, VectorAllocator>::CreateTriangle(int inIdx1, int inIdx2, int inIdx3)
	{
		Face* f = CreateFace();
		f->Initialize(inIdx1, inIdx2, inIdx3, mPositions);
		return f;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::FreeFace(Face* inFace)
	{
		JPL_ASSERT(inFace->mRemoved);

#ifdef JPL_ENABLE_VALIDATION
		// Make sure that this face is not connected
		if (Edge* e = inFace->mFirstEdge)
		{
			do
			{
				JPL_ASSERT(e->mNeighbourEdge == nullptr);
				e = e->mNextEdge;
			} while (e != inFace->mFirstEdge);
		}
#endif

		// Free the face
		delete inFace;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::FreeFaces()
	{
		for (Face* f : mFaces)
			delete f;
		mFaces.clear();
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::sLinkFace(Edge* inEdge1, Edge* inEdge2)
	{
		// Check not connected yet
		JPL_ASSERT(inEdge1->mNeighbourEdge == nullptr);
		JPL_ASSERT(inEdge2->mNeighbourEdge == nullptr);
		JPL_ASSERT(inEdge1->mFace != inEdge2->mFace);

		// Check vertices match
		JPL_ASSERT(inEdge1->mStartIdx == inEdge2->mNextEdge->mStartIdx);
		JPL_ASSERT(inEdge2->mStartIdx == inEdge1->mNextEdge->mStartIdx);

		// Link up
		inEdge1->mNeighbourEdge = inEdge2;
		inEdge2->mNeighbourEdge = inEdge1;
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::sUnlinkFace(Face* inFace)
	{
		// Unlink from neighbours
		Edge* e = inFace->mFirstEdge;
		do
		{
			if (e->mNeighbourEdge != nullptr)
			{
				// Validate that neighbour points to us
				JPL_ASSERT(e->mNeighbourEdge->mNeighbourEdge == e);

				// Unlink
				e->mNeighbourEdge->mNeighbourEdge = nullptr;
				e->mNeighbourEdge = nullptr;
			}
			e = e->mNextEdge;
		} while (e != inFace->mFirstEdge);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::FindEdge(Face* inFacingFace, const Vec3& inVertex, FullEdges& outEdges) const
	{
		// Assert that we were given an empty array
		JPL_ASSERT(outEdges.empty());

		// Should start with a facing face
		JPL_ASSERT(inFacingFace->IsFacing(inVertex));

		// Flag as removed
		inFacingFace->mRemoved = true;

		// Instead of recursing, we build our own stack with the information we need
		struct StackEntry
		{
			Edge* mFirstEdge;
			Edge* mCurrentEdge;
		};
		constexpr int cMaxEdgeLength = 128;
		StackEntry stack[cMaxEdgeLength];
		int cur_stack_pos = 0;

		static_assert(alignof(Edge) >= 2, "Need lowest bit to indicate to tell if we completed the loop");

		// Start with the face / edge provided
		stack[0].mFirstEdge = inFacingFace->mFirstEdge;
		stack[0].mCurrentEdge = reinterpret_cast<Edge*>(reinterpret_cast<uintptr_t>(inFacingFace->mFirstEdge) | 1); // Set lowest bit of pointer to make it different from the first edge

		for (;;)
		{
			StackEntry& cur_entry = stack[cur_stack_pos];

			// Next edge
			Edge* raw_e = cur_entry.mCurrentEdge;
			Edge* e = reinterpret_cast<Edge*>(reinterpret_cast<uintptr_t>(raw_e) & ~uintptr_t(1)); // Remove the lowest bit which was used to indicate that this is the first edge we're testing
			cur_entry.mCurrentEdge = e->mNextEdge;

			// If we're back at the first edge we've completed the face and we're done
			if (raw_e == cur_entry.mFirstEdge)
			{
				// This face needs to be removed, unlink it now, caller will free
				sUnlinkFace(e->mFace);

				// Pop from stack
				if (--cur_stack_pos < 0)
					break;
			}
			else
			{
				// Visit neighbour face
				Edge* ne = e->mNeighbourEdge;
				if (ne != nullptr)
				{
					Face* n = ne->mFace;
					if (!n->mRemoved)
					{
						// Check if vertex is on the front side of this face
						if (n->IsFacing(inVertex))
						{
							// Vertex on front, this face needs to be removed
							n->mRemoved = true;

							// Add element to the stack of elements to visit
							cur_stack_pos++;
							JPL_ASSERT(cur_stack_pos < cMaxEdgeLength);
							StackEntry& new_entry = stack[cur_stack_pos];
							new_entry.mFirstEdge = ne;
							new_entry.mCurrentEdge = ne->mNextEdge; // We don't need to test this edge again since we came from it
						}
						else
						{
							// Vertex behind, keep edge
							FullEdge full;
							full.mNeighbourEdge = ne;
							full.mStartIdx = e->mStartIdx;
							full.mEndIdx = ne->mStartIdx;
							outEdges.push_back(full);
						}
					}
				}
			}
		}

		// Assert that we have a fully connected loop
#ifdef JPL_ENABLE_VALIDATION
		for (int i = 0; i < (int)outEdges.size(); ++i)
			JPL_ASSERT(outEdges[i].mEndIdx == outEdges[(i + 1) % outEdges.size()].mStartIdx);
#endif

#ifdef JPL_CONVEX_BUILDER_DEBUG
		// Draw edge of facing faces
		for (int i = 0; i < (int)outEdges.size(); ++i)
			DebugRenderer::sInstance->DrawArrow(cDrawScale * (mOffset + mPositions[outEdges[i].mStartIdx]), cDrawScale * (mOffset + mPositions[outEdges[i].mEndIdx]), Color::sWhite, 0.01f);
		DrawState();
#endif
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::MergeFaces(Edge* inEdge)
	{
		// Get the face
		Face* face = inEdge->mFace;

		// Find the previous and next edge
		Edge* next_edge = inEdge->mNextEdge;
		Edge* prev_edge = inEdge->GetPreviousEdge();

		// Get the other face
		Edge* other_edge = inEdge->mNeighbourEdge;
		Face* other_face = other_edge->mFace;

		// Check if attempting to merge with self
		JPL_ASSERT(face != other_face);

#ifdef JPL_CONVEX_BUILDER_DEBUG
		DrawWireFace(face, Color::sGreen);
		DrawWireFace(other_face, Color::sRed);
		DrawState();
#endif

		// Loop over the edges of the other face and make them belong to inFace
		Edge* edge = other_edge->mNextEdge;
		prev_edge->mNextEdge = edge;
		for (;;)
		{
			edge->mFace = face;
			if (edge->mNextEdge == other_edge)
			{
				// Terminate when we are back at other_edge
				edge->mNextEdge = next_edge;
				break;
			}
			edge = edge->mNextEdge;
		}

		// If the first edge happens to be inEdge we need to fix it because this edge is no longer part of the face.
		// Note that we replace it with the first edge of the merged face so that if the MergeFace function is called
		// from a loop that loops around the face that it will still terminate after visiting all edges once.
		if (face->mFirstEdge == inEdge)
			face->mFirstEdge = prev_edge->mNextEdge;

		// Free the edges
		delete inEdge;
		delete other_edge;

		// Mark the other face as removed
		other_face->mFirstEdge = nullptr;
		other_face->mRemoved = true;

		// Recalculate plane
		face->CalculateNormalAndCentroid(mPositions);

		// Merge conflict lists
		if (face->mFurthestPointDistanceSq > other_face->mFurthestPointDistanceSq)
		{
			// This face has a point that's further away, make sure it remains the last one as we add the other points to this faces list
			face->mConflictList.insert(face->mConflictList.end() - 1, other_face->mConflictList.begin(), other_face->mConflictList.end());
		}
		else
		{
			// The other face has a point that's furthest away, add that list at the end.
			face->mConflictList.insert(face->mConflictList.end(), other_face->mConflictList.begin(), other_face->mConflictList.end());
			face->mFurthestPointDistanceSq = other_face->mFurthestPointDistanceSq;
		}
		other_face->mConflictList.clear();

#ifdef JPL_CONVEX_BUILDER_DEBUG
		DrawWireFace(face, Color::sWhite);
		DrawState();
#endif
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::MergeDegenerateFace(Face* inFace, Faces& ioAffectedFaces)
	{
		// Check area of face
		if (LengthSquared(inFace->mNormal) < cMinTriangleAreaSq)
		{
			// Find longest edge, since this face is a sliver this should keep the face convex
			float max_length_sq = 0.0f;
			Edge* longest_edge = nullptr;
			Edge* e = inFace->mFirstEdge;
			Vec3 p1 = mPositions[e->mStartIdx];
			do
			{
				Edge* next = e->mNextEdge;
				Vec3 p2 = mPositions[next->mStartIdx];
				float length_sq = LengthSquared(p2 - p1);
				if (length_sq >= max_length_sq)
				{
					max_length_sq = length_sq;
					longest_edge = e;
				}
				p1 = p2;
				e = next;
			} while (e != inFace->mFirstEdge);

			// Merge with face on longest edge
			MergeFaces(longest_edge);

			// Remove any invalid edges
			RemoveInvalidEdges(inFace, ioAffectedFaces);
		}
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::MergeCoplanarOrConcaveFaces(Face* inFace, float inCoplanarToleranceSq, Faces& ioAffectedFaces)
	{
		bool merged = false;

		Edge* edge = inFace->mFirstEdge;
		do
		{
			// Store next edge since this edge can be removed
			Edge* next_edge = edge->mNextEdge;

			// Test if centroid of one face is above plane of the other face by inCoplanarToleranceSq.
			// If so we need to merge other face into inFace.
			const Face* other_face = edge->mNeighbourEdge->mFace;
			const Vec3 delta_centroid = other_face->mCentroid - inFace->mCentroid;
			const float dist_other_face_centroid = DotProduct(inFace->mNormal, delta_centroid);
			const float signed_dist_other_face_centroid_sq = abs(dist_other_face_centroid) * dist_other_face_centroid;
			const float dist_face_centroid = -DotProduct(other_face->mNormal, delta_centroid);
			const float signed_dist_face_centroid_sq = abs(dist_face_centroid) * dist_face_centroid;
			const float face_normal_len_sq = LengthSquared(inFace->mNormal);
			const float other_face_normal_len_sq = LengthSquared(other_face->mNormal);
			if ((signed_dist_other_face_centroid_sq > -inCoplanarToleranceSq * face_normal_len_sq
				 || signed_dist_face_centroid_sq > -inCoplanarToleranceSq * other_face_normal_len_sq)
				&& DotProduct(inFace->mNormal, other_face->mNormal) > 0.0f) // Never merge faces that are back to back
			{
				MergeFaces(edge);
				merged = true;
			}

			edge = next_edge;
		} while (edge != inFace->mFirstEdge);

		if (merged)
			RemoveInvalidEdges(inFace, ioAffectedFaces);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::sMarkAffected(Face* inFace, Faces& ioAffectedFaces)
	{
		if (std::find(ioAffectedFaces.begin(), ioAffectedFaces.end(), inFace) == ioAffectedFaces.end())
			ioAffectedFaces.push_back(inFace);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::RemoveInvalidEdges(Face* inFace, Faces& ioAffectedFaces)
	{
		// This marks that the plane needs to be recalculated (we delay this until the end of the
		// function since we don't use the plane and we want to avoid calculating it multiple times)
		bool recalculate_plane = false;

		// We keep going through this loop until no more edges were removed
		bool removed;
		do
		{
			removed = false;

			// Loop over all edges in this face
			Edge* edge = inFace->mFirstEdge;
			Face* neighbour_face = edge->mNeighbourEdge->mFace;
			do
			{
				Edge* next_edge = edge->mNextEdge;
				Face* next_neighbour_face = next_edge->mNeighbourEdge->mFace;

				if (neighbour_face == inFace)
				{
					// We only remove 1 edge at a time, check if this edge's next edge is our neighbour.
					// If this check fails, we will continue to scan along the edge until we find an edge where this is the case.
					if (edge->mNeighbourEdge == next_edge)
					{
						// This edge leads back to the starting point, this means the edge is interior and needs to be removed
#ifdef JPL_CONVEX_BUILDER_DEBUG
						DrawWireFace(inFace, Color::sBlue);
						DrawState();
#endif

						// Remove edge
						Edge* prev_edge = edge->GetPreviousEdge();
						prev_edge->mNextEdge = next_edge->mNextEdge;
						if (inFace->mFirstEdge == edge || inFace->mFirstEdge == next_edge)
							inFace->mFirstEdge = prev_edge;
						delete edge;
						delete next_edge;

#ifdef JPL_CONVEX_BUILDER_DEBUG
						DrawWireFace(inFace, Color::sGreen);
						DrawState();
#endif

						// Check if inFace now has only 2 edges left
						if (RemoveTwoEdgeFace(inFace, ioAffectedFaces))
							return; // Bail if face no longer exists

						// Restart the loop
						recalculate_plane = true;
						removed = true;
						break;
					}
				}
				else if (neighbour_face == next_neighbour_face)
				{
					// There are two edges that connect to the same face, we will remove the second one
#ifdef JPL_CONVEX_BUILDER_DEBUG
					DrawWireFace(inFace, Color::sYellow);
					DrawWireFace(neighbour_face, Color::sRed);
					DrawState();
#endif

					// First merge the neighbours edges
					Edge* neighbour_edge = next_edge->mNeighbourEdge;
					Edge* next_neighbour_edge = neighbour_edge->mNextEdge;
					if (neighbour_face->mFirstEdge == next_neighbour_edge)
						neighbour_face->mFirstEdge = neighbour_edge;
					neighbour_edge->mNextEdge = next_neighbour_edge->mNextEdge;
					neighbour_edge->mNeighbourEdge = edge;
					delete next_neighbour_edge;

					// Then merge my own edges
					if (inFace->mFirstEdge == next_edge)
						inFace->mFirstEdge = edge;
					edge->mNextEdge = next_edge->mNextEdge;
					edge->mNeighbourEdge = neighbour_edge;
					delete next_edge;

#ifdef JPL_CONVEX_BUILDER_DEBUG
					DrawWireFace(inFace, Color::sYellow);
					DrawWireFace(neighbour_face, Color::sGreen);
					DrawState();
#endif

					// Check if neighbour has only 2 edges left
					if (!RemoveTwoEdgeFace(neighbour_face, ioAffectedFaces))
					{
						// No, we need to recalculate its plane
						neighbour_face->CalculateNormalAndCentroid(mPositions);

						// Mark neighbour face as affected
						sMarkAffected(neighbour_face, ioAffectedFaces);
					}

					// Check if inFace now has only 2 edges left
					if (RemoveTwoEdgeFace(inFace, ioAffectedFaces))
						return; // Bail if face no longer exists

					// Restart loop
					recalculate_plane = true;
					removed = true;
					break;
				}

				// This edge is ok, go to the next edge
				edge = next_edge;
				neighbour_face = next_neighbour_face;

			} while (edge != inFace->mFirstEdge);
		} while (removed);

		// Recalculate plane?
		if (recalculate_plane)
			inFace->CalculateNormalAndCentroid(mPositions);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline bool ConvexHullBuilder<Vec3Type, VectorAllocator>::RemoveTwoEdgeFace(Face* inFace, Faces& ioAffectedFaces) const
	{
		// Check if this face contains only 2 edges
		Edge* edge = inFace->mFirstEdge;
		Edge* next_edge = edge->mNextEdge;
		JPL_ASSERT(edge != next_edge); // 1 edge faces should not exist
		if (next_edge->mNextEdge == edge)
		{
#ifdef JPL_CONVEX_BUILDER_DEBUG
			DrawWireFace(inFace, Color::sRed);
			DrawState();
#endif

			// Schedule both neighbours for re-checking
			Edge* neighbour_edge = edge->mNeighbourEdge;
			Face* neighbour_face = neighbour_edge->mFace;
			Edge* next_neighbour_edge = next_edge->mNeighbourEdge;
			Face* next_neighbour_face = next_neighbour_edge->mFace;
			sMarkAffected(neighbour_face, ioAffectedFaces);
			sMarkAffected(next_neighbour_face, ioAffectedFaces);

			// Link my neighbours to each other
			neighbour_edge->mNeighbourEdge = next_neighbour_edge;
			next_neighbour_edge->mNeighbourEdge = neighbour_edge;

			// Unlink my edges
			edge->mNeighbourEdge = nullptr;
			next_edge->mNeighbourEdge = nullptr;

			// Mark this face as removed
			inFace->mRemoved = true;

			return true;
		}

		return false;
	}

#ifdef JPL_ENABLE_VALIDATION
	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::DumpFace(const Face* inFace) const
	{
		std::cout << std::format("f:{}", std::uintptr_t(inFace)) << '\n';

		const Edge* e = inFace->mFirstEdge;
		do
		{
			std::cout << std::format("e:{} || i:{} e:{} f:{} ",
									 std::uintptr_t(e),
									 e->mStartIdx,
									 std::uintptr_t(e->mNeighbourEdge),
									 std::uintptr_t(e->mNeighbourEdge->mFace)) << '\n';
			e = e->mNextEdge;
		} while (e != inFace->mFirstEdge);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::DumpFaces() const
	{
		std::cout << std::format("Dump Faces:") << '\n';

		for (const Face* f : mFaces)
			if (!f->mRemoved)
				DumpFace(f);
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::ValidateFace(const Face* inFace) const
	{
		if (inFace->mRemoved)
		{
			const Edge* e = inFace->mFirstEdge;
			if (e != nullptr)
				do
				{
					JPL_ASSERT(e->mNeighbourEdge == nullptr);
					e = e->mNextEdge;
				} while (e != inFace->mFirstEdge);
		}
		else
		{
			int edge_count = 0;

			const Edge* e = inFace->mFirstEdge;
			do
			{
				// Count edge
				++edge_count;

				// Validate that adjacent faces are all different
				if (mFaces.size() > 2)
					for (const Edge* other_edge = e->mNextEdge; other_edge != inFace->mFirstEdge; other_edge = other_edge->mNextEdge)
						JPL_ASSERT(e->mNeighbourEdge->mFace != other_edge->mNeighbourEdge->mFace);

				// Assert that the face is correct
				JPL_ASSERT(e->mFace == inFace);

				// Assert that we have a neighbour
				const Edge* nb_edge = e->mNeighbourEdge;
				JPL_ASSERT(nb_edge != nullptr);
				if (nb_edge != nullptr)
				{
					// Assert that our neighbours edge points to us
					JPL_ASSERT(nb_edge->mNeighbourEdge == e);

					// Assert that it belongs to a different face
					JPL_ASSERT(nb_edge->mFace != inFace);

					// Assert that the next edge of the neighbour points to the same vertex as this edge's vertex
					JPL_ASSERT(nb_edge->mNextEdge->mStartIdx == e->mStartIdx);

					// Assert that my next edge points to the same vertex as my neighbours vertex
					JPL_ASSERT(e->mNextEdge->mStartIdx == nb_edge->mStartIdx);
				}
				e = e->mNextEdge;
			} while (e != inFace->mFirstEdge);

			// Assert that we have 3 or more edges
			JPL_ASSERT(edge_count >= 3);
		}
	}

	template<CVec3 Vec3Type, template<class> class VectorAllocator>
	inline void ConvexHullBuilder<Vec3Type, VectorAllocator>::ValidateFaces() const
	{
		for (const Face* f : mFaces)
			ValidateFace(f);
	}
#endif

} // namespace JPL