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
