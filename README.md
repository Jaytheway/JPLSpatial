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
## Examples & Usage
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

### SpatialManager quickstart
A typical `SpatialManager` workflow wires together high-level constructs such as `SourceInitParameters` for allocating the source,
`Position` for spatial placement, and an `AttenuationCurve` for distance rolloff.

```cpp
#include "JPLSpatial/ChannelMap.h"
#include "JPLSpatial/Math/MinimalVec3.h"
#include "JPLSpatial/SpatialManager.h"

using Vec3 = JPL::MinimalVec3;
using Spatializer = JPL::Spatial::SpatialManager<Vec3>;

Spatializer spatializer;
const auto targetChannels = JPL::ChannelMap::FromChannelMask(JPL::ChannelMask::Quad);

SourceInitParameters initParams{
        .NumChannels = 1,
        .NumTargetChannels = targetChannels.GetNumChannels()
};
const SourceId source = spatializer.CreateSource(initParams);

Position<Vec3> sourcePosition{
        .Location = Vec3(0.0f, 0.0f, -5.0f),
        .Orientation = Orientation<Vec3>::Identity()
};
spatializer.SetSourcePosition(source, sourcePosition);

auto* curve = new AttenuationCurve();
curve->Points = {
        {.Distance = 0.0f, .Value = 1.0f, .FunctionType = Curve::EType::Linear},
        {.Distance = 10.0f, .Value = 0.5f, .FunctionType = Curve::EType::Linear}
};
curve->SortPoints();
const auto curveHandle = spatializer.GetDirectPathService().AssignAttenuationCurve(
        spatializer.GetDirectEffectHandle(source), curve);

spatializer.AdvanceSimulation();

const float distanceAttenuation = spatializer.GetDistanceAttenuation(source, curveHandle);
const auto channelGains = spatializer.GetChannelGains(source, targetChannels);
```

Once `AdvanceSimulation` has processed the scene, `GetLastUpdatedSource` exposes which sources were touched, and the cached
results retrieved through `GetDistanceAttenuation` and `GetChannelGains` can be fed directly into the audio mix for the
current frame.

### Manual <code>PanningService</code> usage

- <details>
	<p>
	When working directly with the panning layer you start by creating the source and target <code>ChannelMap</code> objects that describe each layout you want to support. Those maps are passed to <code>InitializePanningEffect</code>, which returns a <code>PanEffectHandle</code> representing the source's cached panning state. Hold on to that handle for subsequent updates, and query the cached gains after evaluation through <code>GetChannelGainsFor</code>. See <a href="Spatialization/include/JPLSpatial/Services/PanningService.h">PanningService.h</a> and the sequence in <a href="SpatializationTests/src/Tests/PanningServiceTest.h">PanningServiceTest</a> for a full example.
	</p>

	<p>
	A typical update loop mirrors the sequence covered in <code>PanningServiceTest</code>: set the focus/spread shaping via <code>SetPanningEffectParameters</code> (or adjust spread alone with <code>SetPanningEffectSpread</code>) and then call <code>EvaluateDirection</code> with the latest <code>Position</code> to refresh the cached gain buffers. This flow keeps directional data and spread control in sync before the gains are read back for mixing.
	</p>

	<p>
	Remember to release handles that are no longer needed by calling <code>ReleasePanningEffect</code>; consult <a href="Spatialization/include/JPLSpatial/Services/PanningService.h">Services/PanningService.h</a> for the full API surface, including helpers for advanced caching scenarios.
	</p>
</details>

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
