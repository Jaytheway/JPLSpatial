//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** MiniaudioCpp **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/MiniaudioCpp
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2024 Jaroslav Pevno, MiniaudioCpp is offered under the terms of the ISC license:
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

// Thanks Jorrit Rouwe and his JoltPhysics library for the boilerplate below
// https://github.com/jrouwe/JoltPhysics

// Determine platform
#if defined(JPL_PLATFORM_BLUE)
	// Correct define already defined, this overrides everything else
#elif defined(_WIN32) || defined(_WIN64)
#include <winapifamily.h>
#if WINAPI_FAMILY == WINAPI_FAMILY_APP
#define JPL_PLATFORM_WINDOWS_UWP // Building for Universal Windows Platform
#endif
#define JPL_PLATFORM_WINDOWS
#elif defined(__ANDROID__) // Android is linux too, so that's why we check it first
#define JPL_PLATFORM_ANDROID
#elif defined(__linux__)
#define JPL_PLATFORM_LINUX
#elif defined(__FreeBSD__)
#define JPL_PLATFORM_FREEBSD
#elif defined(__APPLE__)
#include <TargetConditionals.h>
#if defined(TARGET_OS_IPHONE) && !TARGET_OS_IPHONE
#define JPL_PLATFORM_MACOS
#else
#define JPL_PLATFORM_IOS
#endif
#elif defined(__EMSCRIPTEN__)
#define JPL_PLATFORM_WASM
#endif

// Determine compiler
#if defined(__clang__)
#define JPL_COMPILER_CLANG
#elif defined(__GNUC__)
#define JPL_COMPILER_GCC
#elif defined(_MSC_VER)
#define JPL_COMPILER_MSVC
#endif

// OS-specific includes
#if defined(JPL_PLATFORM_WINDOWS)
#define JPL_BREAKPOINT		__debugbreak()
#elif defined(JPL_PLATFORM_LINUX) || defined(JPL_PLATFORM_ANDROID) || defined(JPL_PLATFORM_MACOS) || defined(JPL_PLATFORM_IOS) || defined(JPL_PLATFORM_FREEBSD)
#if defined(JPL_CPU_X86)
#define JPL_BREAKPOINT	__asm volatile ("int $0x3")
#elif defined(JPL_CPU_ARM)
#define JPL_BREAKPOINT	__builtin_trap()
#elif defined(JPL_CPU_E2K)
#define JPL_BREAKPOINT	__builtin_trap()
#endif
#elif defined(JPL_PLATFORM_WASM)
#define JPL_BREAKPOINT		do { } while (false) // Not supported
#else
#error Unknown platform
#endif

// Detect CPU architecture
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
// X86 CPU architecture
#define JPL_CPU_X86
#if defined(__x86_64__) || defined(_M_X64)
#define JPL_CPU_ADDRESS_BITS 64
#else
#define JPL_CPU_ADDRESS_BITS 32
#endif
#define JPL_USE_SSE
#define JPL_VECTOR_ALIGNMENT 16
#define JPL_DVECTOR_ALIGNMENT 32

// Detect enabled instruction sets
#if defined(__AVX512F__) && defined(__AVX512VL__) && defined(__AVX512DQ__) && !defined(JPL_USE_AVX512)
#define JPL_USE_AVX512
#endif
#if (defined(__AVX2__) || defined(JPL_USE_AVX512)) && !defined(JPL_USE_AVX2)
#define JPL_USE_AVX2
#endif
#if (defined(__AVX__) || defined(JPL_USE_AVX2)) && !defined(JPL_USE_AVX)
#define JPL_USE_AVX
#endif
#if (defined(__SSE4_2__) || defined(JPL_USE_AVX)) && !defined(JPL_USE_SSE4_2)
#define JPL_USE_SSE4_2
#endif
#if (defined(__SSE4_1__) || defined(JPL_USE_SSE4_2)) && !defined(JPL_USE_SSE4_1)
#define JPL_USE_SSE4_1
#endif
#if (defined(__F16C__) || defined(JPL_USE_AVX2)) && !defined(JPL_USE_F16C)
#define JPL_USE_F16C
#endif
#if (defined(__LZCNT__) || defined(JPL_USE_AVX2)) && !defined(JPL_USE_LZCNT)
#define JPL_USE_LZCNT
#endif
#if (defined(__BMI__) || defined(JPL_USE_AVX2)) && !defined(JPL_USE_TZCNT)
#define JPL_USE_TZCNT
#endif
#ifndef JPL_CROSS_PLATFORM_DETERMINISTIC // FMA is not compatible with cross platform determinism
#if defined(JPL_COMPILER_CLANG) || defined(JPL_COMPILER_GCC)
#if defined(__FMA__) && !defined(JPL_USE_FMADD)
#define JPL_USE_FMADD
#endif
#elif defined(JPL_COMPILER_MSVC)
#if defined(__AVX2__) && !defined(JPL_USE_FMADD) // AVX2 also enables fused multiply add
#define JPL_USE_FMADD
#endif
#else
#error Undefined compiler
#endif
#endif
#elif defined(__aarch64__) || defined(_M_ARM64) || defined(__arm__) || defined(_M_ARM)
// ARM CPU architecture
#define JPL_CPU_ARM
#if defined(__aarch64__) || defined(_M_ARM64)
#define JPL_CPU_ADDRESS_BITS 64
#define JPL_USE_NEON
#define JPL_VECTOR_ALIGNMENT 16
#define JPL_DVECTOR_ALIGNMENT 32
#else
#define JPL_CPU_ADDRESS_BITS 32
#define JPL_VECTOR_ALIGNMENT 8 // 32-bit ARM does not support aligning on the stack on 16 byte boundaries
#define JPL_DVECTOR_ALIGNMENT 8
#endif
#elif defined(JPL_PLATFORM_WASM)
// WebAssembly CPU architecture
#define JPL_CPU_WASM
#define JPL_CPU_ADDRESS_BITS 32
#define JPL_VECTOR_ALIGNMENT 16
#define JPL_DVECTOR_ALIGNMENT 32
#ifdef __wasm_simd128__
#define JPL_USE_SSE
#define JPL_USE_SSE4_1
#define JPL_USE_SSE4_2
#endif
#elif defined(__e2k__)
// Elbrus e2k architecture
#define JPL_CPU_E2K
#define JPL_CPU_ADDRESS_BITS 64
#define JPL_USE_SSE
#define JPL_VECTOR_ALIGNMENT 16
#define JPL_DVECTOR_ALIGNMENT 32
#else
#error Unsupported CPU architecture
#endif

// If this define is set, JPL is compiled as a shared library
#ifdef JPL_SHARED_LIBRARY
#ifdef JPL_BUILD_SHARED_LIBRARY
	// While building the shared library, we must export these symbols
#ifdef JPL_PLATFORM_WINDOWS
#define JPL_EXPORT __declspec(dllexport)
#else
#define JPL_EXPORT __attribute__ ((visibility ("default")))
#if defined(JPL_COMPILER_GCC)
	// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
#define JPL_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
#endif
#endif
#else
	// When linking against JPL, we must import these symbols
#ifdef JPL_PLATFORM_WINDOWS
#define JPL_EXPORT __declspec(dllimport)
#else
#define JPL_EXPORT __attribute__ ((visibility ("default")))
#if defined(JPL_COMPILER_GCC)
	// Prevents an issue with GCC attribute parsing (see https://gcc.gnu.org/bugzilla/show_bug.cgi?id=69585)
#define JPL_EXPORT_GCC_BUG_WORKAROUND [[gnu::visibility("default")]]
#endif
#endif
#endif
#else
	// If the define is not set, we use static linking and symbols don't need to be imported or exported
#define JPL_EXPORT
#endif

#ifndef JPL_EXPORT_GCC_BUG_WORKAROUND
#define JPL_EXPORT_GCC_BUG_WORKAROUND JPL_EXPORT
#endif

// Macro used by the RTTI macros to not export a function
#define JPL_NO_EXPORT

#include <cstdint>

namespace JPL
{

	// Standard types
	using uint = unsigned int;
	using uint8 = std::uint8_t;
	using uint16 = std::uint16_t;
	using uint32 = std::uint32_t;
	using uint64 = std::uint64_t;

	// Assert sizes of types
	static_assert(sizeof(uint) >= 4, "Invalid size of uint");
	static_assert(sizeof(uint8) == 1, "Invalid size of uint8");
	static_assert(sizeof(uint16) == 2, "Invalid size of uint16");
	static_assert(sizeof(uint32) == 4, "Invalid size of uint32");
	static_assert(sizeof(uint64) == 8, "Invalid size of uint64");
	static_assert(sizeof(void*) == (JPL_CPU_ADDRESS_BITS == 64 ? 8 : 4), "Invalid size of pointer");

	// Define inline macro
#if defined(JPL_NO_FORCE_INLINE)
#define JPL_INLINE inline
#elif defined(JPL_COMPILER_CLANG) || defined(JPL_COMPILER_GCC)
#define JPL_INLINE __inline__ __attribute__((always_inline))
#elif defined(JPL_COMPILER_MSVC)
#define JPL_INLINE __forceinline
#else
#error Undefined
#endif

// Cache line size (used for aligning to cache line)
#ifndef JPL_CACHE_LINE_SIZE
#define JPL_CACHE_LINE_SIZE 64
#endif

} // namespace JPL
