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

#include <concepts>

namespace JPL
{
	// Specialization of this has to be provided for custom vec3 type
	template<class Vec3Type>
	struct Vec3Access{};

	template<class Vec3Type>
	using Vec3FloatType = std::remove_cvref_t<decltype(Vec3Access<Vec3Type>::GetX(std::declval<Vec3Type>()))>;

	template<class Vec3Type> [[nodiscard]] JPL_INLINE auto GetX(const Vec3Type& v) noexcept { return Vec3Access<Vec3Type>::GetX(v); }
	template<class Vec3Type> [[nodiscard]] JPL_INLINE auto GetY(const Vec3Type& v) noexcept { return Vec3Access<Vec3Type>::GetY(v); }
	template<class Vec3Type> [[nodiscard]] JPL_INLINE auto GetZ(const Vec3Type& v) noexcept { return Vec3Access<Vec3Type>::GetZ(v); }

	template<class Vec3Type> JPL_INLINE void SetX(Vec3Type& v, Vec3FloatType<Vec3Type> value) noexcept { Vec3Access<Vec3Type>::SetX(v, value); }
	template<class Vec3Type> JPL_INLINE void SetY(Vec3Type& v, Vec3FloatType<Vec3Type> value) noexcept { Vec3Access<Vec3Type>::SetY(v, value); }
	template<class Vec3Type> JPL_INLINE void SetZ(Vec3Type& v, Vec3FloatType<Vec3Type> value) noexcept { Vec3Access<Vec3Type>::SetZ(v, value); }

	template<class T>
	concept CVec3 = requires (T& v)
	{
		// Note: Getters and Setters should be noexcept

		{ Vec3Access<T>::GetX(v) } noexcept;
		{ Vec3Access<T>::GetY(v) } noexcept;
		{ Vec3Access<T>::GetZ(v) } noexcept;

		{ Vec3Access<T>::SetX(v, std::remove_cvref_t<decltype(Vec3Access<T>::GetX(v))>(0.0)) } noexcept;
		{ Vec3Access<T>::SetY(v, std::remove_cvref_t<decltype(Vec3Access<T>::GetX(v))>(0.0)) } noexcept;
		{ Vec3Access<T>::SetZ(v, std::remove_cvref_t<decltype(Vec3Access<T>::GetX(v))>(0.0)) } noexcept;
	};
} // namespace JPL