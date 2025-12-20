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
#include "JPLSpatial/Math/Vec3Traits.h"

#if defined(JPL_TEST_WITH_JOLT)
#include <Jolt/Jolt.h>
#endif
#include <glm/glm.hpp>

namespace JPL
{
	// TODO: consider removing this struct and moving everything under corresponding namespaces
	//		- or the other way around
#if defined(JPL_TEST_WITH_JOLT)
	template<>
	struct Vec3Access<JPH::Vec3>
	{
		[[nodiscard]] JPL_INLINE static float GetX(const JPH::Vec3& v) noexcept { return v.GetX(); }
		[[nodiscard]] JPL_INLINE static float GetY(const JPH::Vec3& v) noexcept { return v.GetY(); }
		[[nodiscard]] JPL_INLINE static float GetZ(const JPH::Vec3& v) noexcept { return v.GetZ(); }

		JPL_INLINE static void SetX(JPH::Vec3& v, float value) noexcept { v.SetX(value); }
		JPL_INLINE static void SetY(JPH::Vec3& v, float value) noexcept { v.SetY(value); }
		JPL_INLINE static void SetZ(JPH::Vec3& v, float value) noexcept { v.SetZ(value); }
	};
#endif

	template<>
	struct Vec3Access<glm::vec3>
	{
		[[nodiscard]] static JPL_INLINE float GetX(const glm::vec3& v) noexcept { return v.x; }
		[[nodiscard]] static JPL_INLINE float GetY(const glm::vec3& v) noexcept { return v.y; }
		[[nodiscard]] static JPL_INLINE float GetZ(const glm::vec3& v) noexcept { return v.z; }

		JPL_INLINE static void SetX(glm::vec3& v, float value) noexcept { v.x = value; }
		JPL_INLINE static void SetY(glm::vec3& v, float value) noexcept { v.y = value; }
		JPL_INLINE static void SetZ(glm::vec3& v, float value) noexcept { v.z = value; }
	};

} // namespace JPL

#if defined(JPL_TEST_WITH_JOLT)
namespace JPH
{
	[[nodiscard]] JPL_INLINE JPH::Vec3 Abs(const JPH::Vec3& v) { return v.Abs(); }

	JPL_INLINE JPH::Vec3& Normalize(JPH::Vec3& v) { v = v.Normalized(); return v; }
	[[nodiscard]] JPL_INLINE JPH::Vec3 Normalized(const JPH::Vec3& v) { return v.Normalized(); }
	[[nodiscard]] JPL_INLINE float DotProduct(const JPH::Vec3& a, const JPH::Vec3& b) { return a.Dot(b); }
	[[nodiscard]] JPL_INLINE JPH::Vec3 CrossProduct(const JPH::Vec3& a, const JPH::Vec3& b) { return a.Cross(b); }
	[[nodiscard]] JPL_INLINE float LengthSquared(const JPH::Vec3& v) { return v.LengthSq(); }
	[[nodiscard]] JPL_INLINE float Length(const JPH::Vec3& V) { return V.Length(); }
}

static_assert(JPL::CVec3<JPH::Vec3>);
#endif

namespace glm
{
	[[nodiscard]] JPL_INLINE glm::vec3 Abs(const glm::vec3& v) { return glm::abs(v); }

	JPL_INLINE glm::vec3& Normalize(glm::vec3& v) { v = glm::normalize(v); return v; }
	[[nodiscard]] JPL_INLINE glm::vec3 Normalized(const glm::vec3& v) { return glm::normalize(v); }
	[[nodiscard]] JPL_INLINE float DotProduct(const glm::vec3& a, const glm::vec3& b) { return glm::dot(a, b); }
	[[nodiscard]] JPL_INLINE glm::vec3 CrossProduct(const glm::vec3& a, const glm::vec3& b) { return glm::cross(a, b); }
	[[nodiscard]] JPL_INLINE float LengthSquared(const glm::vec3& v) { return glm::dot(v, v); }
	[[nodiscard]] JPL_INLINE float Length(const glm::vec3& v) { return glm::length(v); }
}
