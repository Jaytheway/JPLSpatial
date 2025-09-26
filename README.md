# JPLSpatial

Simple sound spatialziation utilities library.

## Status
Functional WIP

## Where it's used
JPLSpatial library is used as a sound spatialization solution in [Hazel Engine](https://hazelengine.com/).
If you have access to Hazel Engine code, you can check how JPLSpatial is integrated on the `audio` branch.
## Features
- Vector-Base Amplitude Panning (VBAP) / Multi-Direction Amplitude Panning (MDAP)
	- Single-channel sources
	- Multi-channel sources
	- Virtual Sources per source channel
	- Most of the common target/output channel layouts
	- Elevation / Height channels
- Distance Attenuation
	- Custom function
	- Curves
	- Predefined models:
		- Inverse
		- Linear
		- Exponential
- Angle Attenuation
- **High level sound source API**:
	- Spatial Manager (top level interface managing Sources and Services)
		- Panning Service
		- Direct Path Service (for now handles just distance and angle attenuation)
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

panner.InitializeLUT(JPL::ChannelMap::FromNumChannels(static_cast<uint32_t>(quadGains.size())));

...

std::vector<JPL::VirtualSource<>> virtualSources;

// ..assign angles to virtual sources relative to the listener, e.g. based on sound source extent
for (JPL::VirtualSource<>& vs : virtualSources)
	vs.Angle = ... // agnle relative to listener in radians

...

panner.ProcessVirtualSources(virtualSources, quadGains);

```
## Folder structure
- **Spatialization** - source code for the library
- **SpatializationTests** - a set of tests to validate the behavior of the features and interfaces
- **docs** - so far non-functioning auto-generated documentation
- **build** - build scripts; running `cmake_vs2022_cl_x64.bat` will create VS 2022 solution
- **cmake** - cmake utilities
## Library structure
As much of the library as possible is header-only.

**JPLSpatial library is structured in a few hierarchical layers**:
- SpatialManager
	- Services
		- Low level features

..any layer can be used on its own for a more manual control.

---
- **SpatialManager.h** - top level interface that manages all the library services on the sound source level.
	- Services - each service handles a specific feature set, relevant data and updates, and serve as a higher level interfaces for low level features.
		- **PanningService.h** - VBAP/MDAP Panning, Virtual Sources
		- **DirectPathService.h** - Distance and Angle based Attenuation
		- ..others.
- *"Low level" features and utilities that can be used on their own:*
	- **ChannelMap.h**
	- **VBAP.h**
	- **DistanceAttenuation.h**
## Documentation
- Most of the things annotated in code.
- For more examples check out tests.
- For an example of integrating **Services** take a look at [SpatialManager.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/SpatialManager.h)
## Compiling
- Some includes can be used as is as a single header include in your project.
- Depends only on the standard template library.
- Tests optionally can be ran with Jolt passing `-DTEST_WITH_JOLT=ON` flag.
- Compiles with Visual Studio 2022, other compiles haven't been tested.
- Uses C++20
## Updates
JPLSpatial library is going to be updated as the need for more features matches my time availability to work on them.

> [!WARNING]
> - API may change
> - Things may get added, removed, and restructured
## License
The project is distributed under the [ISC license](https://github.com/Jaytheway/JPLSpatial?tab=License-1-ov-file).
