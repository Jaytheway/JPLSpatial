//
//      ██╗██████╗     ██╗     ██╗██████╗ ███████╗
//      ██║██╔══██╗    ██║     ██║██╔══██╗██╔════╝		** JPLSpatial **
//      ██║██████╔╝    ██║     ██║██████╔╝███████╗
// ██   ██║██╔═══╝     ██║     ██║██╔══██╗╚════██║		https://github.com/Jaytheway/JPLSpatial
// ╚█████╔╝██║         ███████╗██║██████╔╝███████║
//  ╚════╝ ╚═╝         ╚══════╝╚═╝╚═════╝ ╚══════╝
//
//   Copyright 2025 Jaroslav Pevno, JPLSpatial is offered under the terms of the ISC license:
//
//   Permission to use, copy, modify, and/or distribute this software for any purpose with or
//   without fee is hereby granted, provided that the above copyright notice and this permission
//   notice appear in all copies. THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL
//   WARRANTIES WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF MERCHANTABILITY
//   AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY SPECIAL, DIRECT, INDIRECT, OR
//   CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS,
//   WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN
//   CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

#include "TestMemoryLeakDetector.h"

namespace JPL
{
	MallocatorCounting& GetGlobalNewDeleteCounter()
	{
		static MallocatorCounting gGlobalNewDeleteCounter;
		return gGlobalNewDeleteCounter;
	}
} // namepsace JPL

void* operator new (std::size_t inCount) { return JPL::GetGlobalNewDeleteCounter().Allocate(inCount); }
void operator delete (void* inPointer) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }
void* operator new[](std::size_t inCount) { return JPL::GetGlobalNewDeleteCounter().Allocate(inCount); }
void operator delete[](void* inPointer) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }
void* operator new (std::size_t inCount, std::align_val_t inAlignment) { return JPL::GetGlobalNewDeleteCounter().AlignedAllocate(inCount, static_cast<std::size_t>(inAlignment)); }
void operator delete (void* inPointer, std::align_val_t inAlignment) noexcept { JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<std::size_t>(inAlignment)); }
void* operator new[](std::size_t inCount, std::align_val_t inAlignment) { return JPL::GetGlobalNewDeleteCounter().AlignedAllocate(inCount, static_cast<std::size_t>(inAlignment)); }
void operator delete[](void* inPointer, std::align_val_t inAlignment) noexcept { JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<std::size_t>(inAlignment)); }

void* operator new(std::size_t n, const std::nothrow_t&) noexcept
{
	try { return JPL::GetGlobalNewDeleteCounter().Allocate(n); } catch (...) { return nullptr; }
}

void* operator new[](std::size_t n, const std::nothrow_t&) noexcept
{
	try { return JPL::GetGlobalNewDeleteCounter().Allocate(n); } catch (...) { return nullptr; }
}

void operator delete(void* inPointer, std::size_t) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }
void operator delete[](void* inPointer, std::size_t) noexcept { JPL::GetGlobalNewDeleteCounter().Free(inPointer); }

void operator delete(void* inPointer, std::size_t, std::align_val_t inAlignment) noexcept
{
	JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<std::size_t>(inAlignment));
}

void operator delete[](void* inPointer, std::size_t, std::align_val_t inAlignment) noexcept
{
	JPL::GetGlobalNewDeleteCounter().AlignedFree(inPointer, static_cast<std::size_t>(inAlignment));
}
