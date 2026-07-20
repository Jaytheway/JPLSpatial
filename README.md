<div align="center">

[![Build](https://github.com/Jaytheway/JPLSpatial/actions/workflows/build.yml/badge.svg)](https://github.com/Jaytheway/JPLSpatial/actions/workflows/build.yml) ![GitHub License](https://img.shields.io/github/license/jaytheway/jplspatial)

</div>

# JPL Spatial

**JPL Spatial** is an engine-independent C++ library for audio spatialization,
sound propagation, and related DSP. It does not own the audio device or mixing
backend. Applications can use their own vector and scene-query types.

The core library **has no external dependencies**.

> [!WARNING]
> **JPL Spatial** is under active development. Unless explicitly documented
> otherwise, public APIs may change without deprecation or migration support.
> Deprecation and migration support may be introduced as the API matures.

<div align="center">

[Hello JPL Spatial](Examples/HelloJPLSpatial) ·
[Build and integrate](build/README.md) ·
[Documentation and API reference](https://jaytheway.github.io/JPLSpatial/) ·
[JPL Spatial Application](https://github.com/Jaytheway/JPLSpatialApplication/)

</div>

## What it provides

JPL Spatial contains independent building blocks as well as service and manager
layers that compose them. A host can adopt only the layer it needs.

### Spatialization

- 2D and 3D Vector-Base Amplitude Panning (VBAP).
- Multi-Direction Amplitude Panning (MDAP) for source spread and focus.
- Mono and multichannel sources targeting conventional speaker layouts,
  including layouts with height channels.
- Distance and source/listener cone attenuation.

<img src="./assets/VBAP2D.gif" width="500" alt="VBAP 2D: quad source panned to a 5.1 speaker layout" />

<img src="./assets/VBAP3D.gif" width="500" alt="VBAP 3D: stereo source panned to a 9.1.4 speaker layout" />

#### Currently represented channel layouts

- **Ground-plane layouts**: mono, stereo, LCR, quad, 4.1, 5.x, 6.x, 7.x, octagonal.
- **Height-channel layouts**: 5.x.2, 5.x.4, 7.x.2, 7.x.4,
7.x.6, 9.x.4, 9.x.6.

See [`ChannelMap.h`](Spatialization/include/JPLSpatial/ChannelMap.h) for the
current definitions and channel ordering. LFE is represented in a channel map
but is not spatialized by VBAP.

### Propagation and acoustics

- Specular reflection tracing using the image-source method.
- Acoustic material definitions and common material presets.
- Air-absorption estimation.
- Sabine and Eyring-Norris reverberation-time estimation.
- Four-band propagation filtering shared by the direct-sound, reflection, and
  late-reverb processing paths.

### Audio processing

- Propagation delay and Doppler processing using interpolating fractional delay
  lines.
- Fourth-order Linkwitz-Riley crossover filters.
- Early-reflection rendering using interpolating fractional delay lines, with
  tap count and parameters that can change at runtime.
- 16th-order Feedback Delay Network (FDN) late reverberation with four-band
  decay control.
- Channel mixing and conversion utilities.

### Integration interface

- Host-provided vector types, with `MinimalVec3` supplied as a fallback.
- Host-provided scene and ray-query implementations.
- Allocator-aware containers and APIs using `std::pmr` where appropriate.
- SIMD implementations for the x86-64 SSE2 baseline and ARM64 NEON, with
  optional AVX2/FMA builds.

## Scene-query ownership

JPL Spatial currently consumes spatial queries through a host-implemented scene
interface. This lets an engine keep ownership of its physics scene, acceleration
structures, and synchronization policy.

## Choose an API layer

| Layer | Use it when |
|---|---|
| Low-level panners, propagation algorithms, and DSP | You want explicit control or need one feature in an existing engine/audio graph. This is the recommended starting point today. |
| Services | You want JPL Spatial to manage reusable layouts, handles, cached parameters, and repeated evaluation. |
| `SpatialManager` | You want one object to coordinate sources, listeners, and services. This is the oldest high-level API and remains under active design review. |

The layers are optional rather than progressive requirements. An integration can
use a low-level panner directly while using a service for another feature.

## Hello JPL Spatial

[`Examples/HelloJPLSpatial`](Examples/HelloJPLSpatial) is a small console
consumer that pans a point source around a quad speaker layout and prints the
resulting gains. It has no dependencies beyond JPL Spatial and the C++ standard
library.

```sh
cmake -S Examples/HelloJPLSpatial -B build/hello
cmake --build build/hello --config Release
```

Example result:

```text
front  FL=0.7071  FR=0.7071  BL=0.0000  BR=0.0000
right  FL=0.0000  FR=0.7071  BL=0.0000  BR=0.7071
back   FL=0.0000  FR=0.0000  BL=0.7071  BR=0.7071
left   FL=0.7071  FR=0.0000  BL=0.7071  BR=0.0000
```

## Building for Visual Studio

- Install [Visual Studio 2026](https://visualstudio.microsoft.com/).
- Run `build\cmake_vs2026_cl_x64.bat`.
- Open and build the generated solution in `Build\VS2026_CL_x64`.

## Integrating with CMake

Vendor the repository and add it directly:

```cmake
add_subdirectory(path/to/JPLSpatial)
target_link_libraries(MyTarget PRIVATE JPL::Spatial)
```

`FetchContent` is supported as well. The complete build guide documents both
paths, build options, CPU requirements, tests, and subproject behavior:
[Building and integrating](build/README.md).

## Coordinate system

JPL Spatial uses a right-handed coordinate system with Y up and -Z forward:

- +X: right;
- +Y: up;
- -Z: forward.

Hosts with a different convention should convert positions, directions, and
orientations at the integration boundary.

## Platform support

The following configurations are built and tested by
[GitHub Actions](https://github.com/Jaytheway/JPLSpatial/actions/workflows/build.yml):

| Platform | Architecture | Compiler | Configuration |
|:---|:---|:---|:---|
| Windows | x64 | MSVC | Debug, Release |
| Linux | x64 | GCC | Debug, Release |
| Linux | x64 | Clang | Debug, Release |
| macOS | x64 | AppleClang | Debug, Release |
| Windows | ARM64* | MSVC | Debug |
| Linux | ARM64* | GCC | Debug |
| macOS | ARM64* | AppleClang | Debug |

\* ARM64 configurations are tested through the extended CI workflow.

Linux/Clang Debug builds are additionally tested with AddressSanitizer.

## Reference integrations

- JPL Spatial is used for sound spatialization in
  [Hazel Engine](https://hazelengine.com/). Its integration is available to
  people with access to Hazel's `dev` or `audio` branch.
- [JPL Spatial Application](https://github.com/Jaytheway/JPLSpatialApplication/)
  is the public reference application and visualization laboratory. It combines
  JPL Spatial with an audio backend, scene queries, playback, and interactive
  inspection tools.

## Documentation

- [Building and integrating](build/README.md)
- [Hello JPL Spatial](Examples/HelloJPLSpatial)
- [Generated documentation and API reference](https://jaytheway.github.io/JPLSpatial/)
- [Tests](SpatializationTests/src/Tests) for detailed behavioral examples

## License

JPL Spatial is distributed under the [ISC license](LICENSE).

## LLM Usage

This disclosure was inspired by Box3D's [LLM usage statement](https://github.com/erincatto/box3d#llm-usage).

I use LLMs as supporting tools for:
- tests
- examples
- build configuration
- code review
- documentation
- benchmarking
- rubber ducking

All other code is developed and written by me. I review and validate all LLM-assisted changes and take responsibility for every line of code in JPL Spatial.
