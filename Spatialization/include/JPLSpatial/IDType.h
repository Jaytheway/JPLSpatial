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

#include <concepts>
#include <atomic>

namespace JPL
{
	//==========================================================================
	/// Simple utility, wrapping incrementing counter to generate strongly typed
	/// runtime identifiers.
	/// 
	/// It is not serializable and is meant to be use for runtime tracking only.
	/// 
	/// If template parameter 'Atomic' is set to 'true', makes the internal
	/// static counter type atomic to make sure IDs can be safely generated
	/// from multiple threads.
	template<typename Tag, std::integral InternalType = uint32_t, bool Atomic = false>
	struct [[nodiscard]] IDType
	{
		using TagType = Tag;
		using CounterType = std::conditional_t<Atomic, std::atomic<InternalType>, InternalType>;
		
		constexpr IDType() = default;

		[[nodiscard]] static constexpr IDType New() noexcept { return IDType(++sLastValue); }

		[[nodiscard]] constexpr bool IsValid() const noexcept { return mValue != std::numeric_limits<InternalType>::max(); }
		[[nodiscard]] constexpr operator bool() const noexcept { return IsValid(); }

		[[nodiscard]] constexpr bool operator==(const IDType other) const noexcept { return mValue == other.mValue; }
		[[nodiscard]] constexpr bool operator!=(const IDType other) const noexcept { return mValue != other.mValue; }

	private:
		constexpr explicit IDType(InternalType value) : mValue(value) {}
		
		friend struct std::hash<JPL::IDType<Tag, InternalType>>;
		
		InternalType mValue = std::numeric_limits<InternalType>::max();
		inline static CounterType sLastValue{ InternalType(0) };
	};
} // namespace JPL


//==============================================================================
namespace std
{
	template<typename Tag, std::integral InternalType>
	struct [[nodiscard]] hash<JPL::IDType<Tag, InternalType>>
	{
		constexpr std::size_t operator()(const JPL::IDType<Tag, InternalType>& id) const
		{
			return id.mValue;
		}
	};
} // namespace std
