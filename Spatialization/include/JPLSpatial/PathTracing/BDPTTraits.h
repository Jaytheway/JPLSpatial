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
#if JPL_HAS_PATH_TRACING
#include "JPLSpatial/Core.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/Math/SIMD.h"

#include <concepts>
#include <type_traits>


namespace JPL
{
    //======================================================================
    /// Default traits for BDPT
    struct BDPTDefaultTraits
    {
        using Vec3 = MinimalVec3;
    };

    //======================================================================
    /// User-provided ray interface concept
    template<class T, class Vec3Type>
	concept CRay = requires(const T t)
	{
        { t.Origin } -> std::same_as<const Vec3Type&>;
        { t.Direction } -> std::same_as<const Vec3Type&>;
	};

    /// User-provided scene intersection interface concept
	template<class T, class Vec3Type>
    concept CIntersection = requires(const T t)
    {
        { t.Normal } -> std::same_as<const Vec3Type&>;
        { t.Position } -> std::same_as<const Vec3Type&>;
        { t.Material } -> std::same_as<const int32&>; // TODO: maybe let user specify the type for the material id
        { t.SurfaceID } -> std::same_as<const int32&>;
    };

    /// User-provided scene interface concept
    template<class T>
    concept CScene = requires(T t)
    {
        typename T::Vec3;
        requires CRay<typename T::Ray, typename T::Vec3>;
        requires CIntersection<typename T::Intersection, typename T::Vec3>;
        { t.Intersect(std::declval<const typename T::Ray&>(), std::declval<typename T::Intersection&>()) } -> std::same_as<bool>;
        { t.IsOccluded(std::declval<const typename T::Vec3&>(), std::declval<const typename T::Vec3&>()) } -> std::same_as<bool>;

		// GetMaterialFactor function must have a single parameter of type that matches the Material type in Intersection
        { t.GetMaterialFactor(std::declval<decltype(std::declval<typename T::Intersection>().Material)>()) } -> std::same_as<std::pair<float, simd>>;
        // Sample direction direction for the next ray from the hit event (second param) and incoming ray direction (first param)
        //{ t.SampleTraceDirection(std::declval<const typename T::Vec3&>(), std::declval<const typename T::Intersection&>()) } -> std::same_as<typename T::Vec3>;
    };

    /// User-provided source interface concept
    template<class T, class RayType>
    concept CSource = requires(T t)
    {
        { t.SampleRay() } -> std::same_as<RayType>;
        { t.GetIntencity() } -> std::same_as<float>;
    };

    /// User-provided listener interface concept
    template<class T, class RayType>
    concept CListener = requires(T t)
    {
        { t.SampleRay() } -> std::same_as<RayType>;

        // Transform world-space direction vector to listener space, return direction relative to listener
        //{ t.TransformDirection(
        //    std::declval<decltype(std::declval<RayType>().Origin)>()) } -> std::same_as<std::remove_cvref_t<decltype(std::declval<RayType>().Origin)>>;
    };
} // namespace JPL
#endif