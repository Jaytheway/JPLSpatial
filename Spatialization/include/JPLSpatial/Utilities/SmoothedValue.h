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

#include <cstdint>
#include <cmath>

namespace JPL
{
    template<class T>
    struct SmoothedValue
    {
    public:
        T Current = T(0.0);
        T Target = T(0.0);
        T Alpha = T(1.0);

    public:
        [[nodiscard]] inline static T GetExpAlpha(uint32_t numOfTicks)
        {
            return numOfTicks == 0
                ? T(1.0)
                : T(1.0 - std::exp(-1.0 / static_cast<double>(numOfTicks)));
        }

        [[nodiscard]] inline static T GetInvAlpha(uint32_t numOfTicks)
        {
            return numOfTicks == 0
                ? T(1.0)
                : T(1.0 / static_cast<double>(numOfTicks));
        }

        [[nodiscard]] inline static SmoothedValue CreateExpSmoothing(uint32_t numOfTicks)
        {
            return SmoothedValue{ .Alpha = GetExpAlpha(numOfTicks) };
        }

        [[nodiscard]] inline static SmoothedValue CreateSimpleSmoothing(uint32_t numOfTicks)
        {
            return SmoothedValue{ .Alpha = GetInvAlpha(numOfTicks) };
        }

        [[nodiscard]] inline static SmoothedValue CreateExpSmoothing(T current, T target, uint32_t numOfTicks)
        {
            return SmoothedValue{ .Current = current, .Target = target, .Alpha = GetExpAlpha(numOfTicks) };
        }

        [[nodiscard]] inline static SmoothedValue CreateSimpleSmoothing(T current, T target, uint32_t numOfTicks)
        {
            return SmoothedValue{ .Current = current, .Target = target, .Alpha = GetInvAlpha(numOfTicks) };
        }

        inline void SetDurationExp(uint32_t numTicks) { Alpha = GetExpAlpha(numTicks); }

        // Advance smoothing to the next value
        inline void Tick()
        {
            // c = a * c + (1.0f - a) * t : or a sing SUB + FMA
            Current = Alpha * (Target - Current) + Current;
        }

        // Advance smoothing to the next value and return it
        [[nodiscard]] inline T GetNext() { Tick(); return Current; }
    };
}
