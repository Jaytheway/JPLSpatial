# JPLSpatial

Simple sound spatialziation utilities library.

## Status
Functional WIP

## Where it's used
JPLSpatial library is used as a sound spatialization solution in [Hazel Engine](https://hazelengine.com/).
If you have access to Hazel Engine code, you can check how JPLSpatial is integrated on the `audio` branch.
## Examples
The simplest use-case is channel panning using `VBAPPanner`.
`VBAPPanner` or Vector Based Amplitude Panner handles panning from any source channel map to any target/output channel map.

The simplest, least efficient, way to just use `VBAPanner<>::ProcessAngle` static function:
```cpp
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/VBAP.h"

#include <span>
...

void CalculateChannelGains(float sourceToListenerAngleRadians,
 std::span<const float> speakerAnglesRadians,
 std::span<float> outGains)
{
	// Process gains based on simple cos/sin rule	
	JPL::VBAPanner<>::ProcessAngle(sourceToListenerAngleRadians, speakerAnglesRadians, outGains);
}
```

The most efficient way to use `VBAPPanner` is with LUT:
```cpp
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/VBAP.h"

#include <span>
...

JPL::VBAPanner panner;

const auto targetChannelMap = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Stereo)

panner.InitializeLUT(targetChannelMap);

...

// outGains.size() must be <= number of channels of the `targetChannelMap`,
// in this case of Stereo channel map, number of channels is 2
void GetChannelGains(float sourceToListenerAngleRadians,
 std::span<float> outGains)
{
	if (panner.IsInitialized())
	{
		panner.GetSpeakerGains(sourceToListenerAngelInRadians, outGains);
	}
}

```
---
A more advanced use cases using `VirtualSource`s, or sometimes they're called Virtual Positions, can be handled manually using low level API of `VBAPPaner`, or higher level API of `PanningService`.

> [!WARNING]
> **Dependency**
> 
> Currently Services depend on utilities pulled from [JoltPhysics](https://github.com/jrouwe/JoltPhysics). `PanningService` requires a couple of includes:
> - "Jolt/Core/Core.h"
> - "Jolt/Core/HashCombine.h"

---
Simple example of `VirtualSource`s API with single-channel sound source and quad panner:
```cpp

#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/VBAP.h"

#include <array>
#include <vector>
#include <span>
...

static constexpr uint32_t NUM_QUAD_CHANNELS = 4;

// Initialize panner for a quad channel map
JPL::VBAPanner panner;

std::array<float, NUM_QUAD_CHANNELS> quadGains;

panner.InitializeLUT(JPL::ChannelMap::FromNumChannels(static_cast<uint32_t>(quadGains.size()))));

...

std::vector<JPL::VirtualSource<>> virtualSources;

// ..assign angles to virtual sources relative to the listener, e.g. based on sound source extent
for (JPL::VirtualSource<>& vs : virtualSources)
	vs.Angle = ... // agnle relative to listener in radians

...

panner.ProcessVirtualSources(virtualSources, quadGains);

```
