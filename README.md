# JPLSpatial

Simple sound spatialziation utilities library.

## Status
Functional WIP

## Where it's used
JPLSpatial library is used as a sound spatialization solution in [Hazel Engine](https://hazelengine.com/).
If you have access to Hazel Engine code, you can check how JPLSpatial is integrated on `dev` or `audio` branch.
## Features
- Vector-Base Amplitude Panning (VBAP) / Multi-Direction Amplitude Panning (MDAP)
	- Source elevation / Height channels

	- <details>
		<summary>Supported/tested source channel layouts</summary>
		<ul>
		<li>Mono</li>
		<li>Stereo</li>
		<li>LCR</li>
		<li>Quad</li>
		<li>Surround 4.1</li>
		<li>Surround 5.0</li>
		<li>Surround 5.1</li>
		<li>Surround 6.0</li>
		<li>Surround 6.1</li>
		<li>Surround 7.0</li>
		<li>Surround 7.1</li>
		<li>Octagonal</li>
		</ul>
	</details>

	- <details>
		<summary>Supported/tested target/output channel layouts</summary>
		<ul>
		<br>
		<b>VBAPanner2D</b>

		---
		
		<li>Stereo</li>
		<li>LCR</li>
		<li>Quad</li>
		<li>Surround 4.1</li>
		<li>Surround 5.0</li>
		<li>Surround 6.0</li>
		<li>Surround 5.1</li>
		<li>Surround 6.1</li>
		<li>Surround 7.0</li>
		<li>Surround 7.1</li>
		<li>Octagonal</li>

		<br>
		<b>VBAPanner3D</b> <i>(layouts with top channels)</i>

		---
		
		<li>Surround 5.0.2</li>
		<li>Surround 5.1.2</li>
		<li>Surround 5.0.4</li>
		<li>Surround 5.1.4</li>
		<br>		
		<i>(as per Dolby Atmos surround, but LFE is always 4th channel)</i>

		<li>Surround 7.0.2</li>
		<li>Surround 7.1.2</li>
		<li>Surround 7.0.4</li>
		<li>Surround 7.1.4</li>
		<li>Surround 7.0.6</li>
		<li>Surround 7.1.6</li>
		<li>Surround 9.0.4</li>
		<li>Surround 9.1.4</li>
		<li>Surround 9.0.6</li>
		<li>Surround 9.1.6</li>
		</ul>
	</details>

- Distance Attenuation
	- Custom function
	- Curves
	- Predefined models:
		- Inverse
		- Linear
		- Exponential
- Angle Attenuation / Cone-based attenuation
- **High level sound source API**:
	- Spatial Manager (top level interface managing Sources and Services)
		- Panning Service
		- Direct Path Service (for now handles just distance and angle attenuation)
## Examples
Initializing `VBAPPanner`, `SourceLayout` and querying target channel gains for a source direction:
```cpp
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Panning/VBAPanning2D.h"

#include <span>
...

using PannerType = typename JPL::VBAPanner2D<>;
using SourceLayout = typename PannerType::SourceLayoutType;
using ChannelGains = typename JPL::VBAPStandartTraits::ChannelGains;

const auto targetChannelMap = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Stereo)
const auto sourceChannelMap = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Mono)

PannerType panner;
panner.InitializeLUT(targetChannelMap);

SourceLayout sourceLayout;
panner.InitializeSourceLayout(sourceChannelMap, sourceLayout)

...

// `outGains` is going to be filled with the computed panning gains
// based on input parameters
void GetChannelGains(
	const SourceLayout& sourceLayout,
 	Vec3 sourceDirection,
  	float focus, float spread
 	ChannelGains& outGains)
{
	if (panner.IsInitialized())
	{
		typename PannerType::PanUpdateData positionData
		{
			.SourceDirection = sourceDirection,
			.Focus = focus,
			.Spread = spread
		};

		panner.ProcessVBAPData(
			sourceLayout,
			positionData,
			[&outGains](uint32 /*channel*/) -> auto& { return outGains; });
	}
}

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
	- **Panning/VBAPEx.h**
## Documentation
- Most of the things annotated in code.
- For more examples check out tests.
- For an example of integrating **Services** take a look at [SpatialManager.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/SpatialManager.h)
## Compiling
- Some includes can be used as is as a single header include in your project.
- Depends only on the standard template library.
- Tests fetch `glm` to validate `glm::vec3` type working with library's interfaces, and optionally can be ran with `JoltPhysics`'s `JPH::Vec3` by passing `-DTEST_WITH_JOLT=ON` flag to test project generator.
- Compiles with Visual Studio 2022, other compiles haven't been tested.
- Uses C++20
## Updates
JPLSpatial library is going to be updated as the need for more features matches my time availability to work on them.

> [!WARNING]
> - API may change
> - Things may get added, removed, and restructured
## License
The project is distributed under the [ISC license](https://github.com/Jaytheway/JPLSpatial?tab=License-1-ov-file).
