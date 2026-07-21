# Integrating JPL Spatial {#integrating}

## Choose an API Layer

| Layer | Use it when |
|---|---|
| [Components](#components-api-layer) (panners, propagation algorithms, and DSP) | You want explicit control or need one feature in an existing engine/audio graph. This is the recommended starting point today. |
| [Services](#services-api-layer) | You want JPL Spatial to manage reusable layouts, handles, cached parameters, and repeated evaluation. |
| [Source Manager](#source-manager-api-layer) ([SpatialManager](@ref JPL::Spatial::SpatialManager)) | You want one object to coordinate sources, listeners, and services. |

*Note: the Services and Source Manager layers are older high-level APIs and remain under active design review.*

The layers are optional rather than progressive requirements. An integration can
use a low-level panner directly while using a service for another feature.

## Components API Layer {#components-api-layer}

Components API Lyaer is essentially every feature and utility in JPL Spatial. Many of them can be used individually, or together to build higher level engine and gameplay systems.

For example:

1. VBAP utilities compute panning gains for an incident direction.
2. MDAP utilizes VBAP to simulate effects like [Spread](TODO) and [Focus](TODO).
3. [Volumetric sound sources](TODO) can be spatialized using MDAP.
4. [VBAPanner2D](@ref JPL::VBAPanner2D)/[VBAPanner3D](@ref JPL::VBAPanner3D) provides interface for 2 and 3, while using both, VBAP and MDAP, internally.

### Path Tracing

Includes [Ray Traced Specular Reflections](TODO) and is not part of any service at this moment.

All path tracing utilities located in [PathTracing](https://github.com/Jaytheway/JPLSpatial/tree/main/Spatialization/include/JPLSpatial/PathTracing).

### Audio Rendering

Can be used within your mixing graph to render various acoustic effects based on parameters from JPL Spatial or any other provider.

All the audio processing classes located in [Auralization](https://github.com/Jaytheway/JPLSpatial/tree/main/Spatialization/include/JPLSpatial/Auralization).

## Services API Layer {#services-api-layer}

Services API Layer consists of feature-level managers, responsible for computation- and semantically- similar independent groups of acoustic effects.  
In other words, effects that makes sense to bundle and process together.

In general, services work as follows:

1. Initialize Effect - allocate and setup cached data for the specified parameters and return the handles to cached data.
2. Evaluate/process - process input scene parameters for a specific effect handle and cache the results.
3. Retrieve spatial/audio parameters - retrieve the cached data from the last parameter update.
4. Release Effect - erase the cached data entry for the specified effect handle.

For code example integrating services, see [SpatialManager.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/SpatialManager.h), which makes up [Source Manager API Layer](#source-manager-api-layer).

## Source Manager API Layer {#source-manager-api-layer}

Typical for audio engines and middlware level of abstraction, where the user calls API to initialize, update, and release sources, while the the library handles memory, parameter update, caching, etc.

The output of such model is either audio rendered directly to an endpoint device, or, as in case with JPL Spatial, parameters to drive audio rendering.

### Use-case: Sound Spatializer {#use-case-sound-spatializer}

Out of the box, [SpatialManager](@ref JPL::Spatial::SpatialManager) can be used as a sound spatializer. It provides all of the expected API:

- [SpatialManager::CreateSource](@ref JPL::Spatial::SpatialManager::CreateSource)
- [SpatialManager::DeleteSource](@ref JPL::Spatial::SpatialManager::DeleteSource)
- [SpatialManager::SetSourcePosition](@ref JPL::Spatial::SpatialManager::SetSourcePosition)
- [SpatialManager::SetListenerPosition](@ref JPL::Spatial::SpatialManager::SetListenerPosition)
- [SpatialManager::GetChannelGains](@ref JPL::Spatial::SpatialManager::GetChannelGains)
- ...

For an exhausive API documentation see [SpatialManager class API reference](@ref JPL::Spatial::SpatialManager).

For code example of setting up JPL Spatial Spatializer, see [SpatializerExample](https://github.com/Jaytheway/JPLSpatial/tree/main/Examples/SpatializerExample).

For more details about how Spatial Manager works, see [Spatial Manager Lifecycle](@ref spatial-manager-lifecycle) section.
