# Integrating JPL Spatial {#integrating}

## Coordinate System {#coordinate-system}

JPL Spatial uses a right-handed coordinate system with Y up and -Z forward:

- +X: right
- +Y: up
- -Z: forward

Hosts with a different convention should convert positions, directions, and orientations at the integration boundary.

@note Unless documented otherwise, a direction parameter must be a normalized, non-zero vector.

## Channel Ordering

Channel order determines how channels are laid out in processed audio buffers and in mixing matrices created by JPL Spatial.

JPL Spatial mostly follows WAVE channel ordering. The main difference is that side channels come before back channels. For example, a 7.0 channel map uses:

1. FrontLeft
2. FrontRight
3. FrontCenter
4. SideLeft
5. SideRight
6. BackLeft
7. BackRight

For the exact definitions and ordering, see [EChannel](@ref JPL::EChannel) and [ChannelMask](@ref JPL::ChannelMask).

@note LFE is not spatialized. Channel maps containing LFE are valid, but the corresponding values in panning matrices are zero.

## Memory

The audio-processing entry points documented as real-time safe do not allocate or deallocate dynamic memory. Construction, preparation, and parameter updates may allocate and must be called from the thread stated in their API documentation.

Allocator-aware JPL Spatial storage uses [gDefaultMemoryResource](@ref JPL::gDefaultMemoryResource) unless an API accepts a different [`std::pmr::memory_resource`](https://en.cppreference.com/w/cpp/memory/memory_resource).

If an integration overrides the global resource, it should be treated as a library-wide setting: set it before any JPL Spatial work begins and do not change it until that work has stopped and all objects using it have been destroyed.

- [ScopedGlobalMemoryResource](@ref JPL::ScopedGlobalMemoryResource) is avaialble for isolated scopes where no object or background thread can outlive the override.
- [CountingResource](@ref JPL::CountingResource) can be used to track allocations made through a memory resource.

See [Memory.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/Memory/Memory.h) for the available memory utilities.

## Error Reporting

For the default untagged logging configuration, an integration can replace the global trace and assertion callbacks:

```cpp
JPL::SpatialTrace = MyTraceCallback;

#if defined(JPL_ENABLE_ASSERTS) || defined(JPL_ENABLE_ENSURE)
JPL::AssertFailed = MyAssertFailedCallback;
#endif
```

[ErrorReporting.cpp](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/src/Spatialization/ErrorReporting.cpp) provides fallback callback definitions, so it should remain in a normal CMake build.

When `JPL_TAGGED_LOGGING` is enabled, [ErrorReporting.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/ErrorReporting.h) replaces `SpatialTrace` with separate trace, info, warning, and error callbacks. The integration must define those variables instead of compiling the default `ErrorReporting.cpp`.

## Audio Thread

Thread suitability is part of each function's API documentation.

The `Process*` entry points under [Auralization](https://github.com/Jaytheway/JPLSpatial/tree/main/Spatialization/include/JPLSpatial/Auralization) that are explicitly documented as real-time safe may be called from one audio thread.

Construction, destruction, `Prepare*`, and parameter-update functions belong on a single non-audio thread unless their annotations say otherwise.

[DirectSoundEffect](@ref JPL::DirectSoundEffect), [ERBus](@ref JPL::ERBus), and [ReverbBus](@ref JPL::ReverbBus) synchronize their parameter updates with audio processing internally. The caller must still observe the documented single-audio-thread and single-non-audio-thread model.

For general background, see [Wikipedia: Real-time computing](https://en.wikipedia.org/wiki/Real-time_computing).

## User Vector Types

APIs that operate on three-component vectors are templates and can use an integration's own vector type. Specialize [Vec3Access](@ref JPL::Vec3Access) to provide access to its components and implement required free functions, similar to this `glm::vec3` example:

```cpp
namespace JPL
{
	template<>
	struct Vec3Access<glm::vec3>
	{
		[[nodiscard]] static JPL_INLINE float GetX(const glm::vec3& v) noexcept { return v.x; }
		[[nodiscard]] static JPL_INLINE float GetY(const glm::vec3& v) noexcept { return v.y; }
		[[nodiscard]] static JPL_INLINE float GetZ(const glm::vec3& v) noexcept { return v.z; }

		JPL_INLINE static void SetX(glm::vec3& v, float value) noexcept { v.x = value; }
		JPL_INLINE static void SetY(glm::vec3& v, float value) noexcept { v.y = value; }
		JPL_INLINE static void SetZ(glm::vec3& v, float value) noexcept { v.z = value; }
	};
}

namespace glm
{
	[[nodiscard]] JPL_INLINE glm::vec3 Abs(const glm::vec3& v) { return glm::abs(v); }

	JPL_INLINE glm::vec3& Normalize(glm::vec3& v) { v = glm::normalize(v); return v; }
	[[nodiscard]] JPL_INLINE glm::vec3 Normalized(const glm::vec3& v) { return glm::normalize(v); }
	[[nodiscard]] JPL_INLINE float DotProduct(const glm::vec3& a, const glm::vec3& b) { return glm::dot(a, b); }
	[[nodiscard]] JPL_INLINE glm::vec3 CrossProduct(const glm::vec3& a, const glm::vec3& b) { return glm::cross(a, b); }
	[[nodiscard]] JPL_INLINE float LengthSquared(const glm::vec3& v) { return glm::dot(v, v); }
	[[nodiscard]] JPL_INLINE float Length(const glm::vec3& v) { return glm::length(v); }
}
```

The `Vec3Access` specialization satisfies the [CVec3](@ref JPL::CVec3) component-access contract. The free functions are placed in the vector type's namespace so they can be found through argument-dependent lookup. Some APIs also require three-component construction and arithmetic operators; `glm::vec3` already provides those operations.

[MinimalVec3](@ref JPL::MinimalVec3) is available when an integration does not need its own vector type.

The user vector type may use double-precision components even though much of the internal SIMD processing uses 4-lane 32-bit float [SIMD](@ref JPL::simd) type.

## Choose an API Layer

| Layer | Use it when |
|---|---|
| [Components](#components-api-layer) (panners, propagation algorithms, and DSP) | You want explicit control or need one feature in an existing engine/audio graph. This is the recommended starting point today. |
| [Services](#services-api-layer) | You want JPL Spatial to manage reusable layouts, handles, cached parameters, and repeated evaluation. |
| [Source Manager](#source-manager-api-layer) ([SpatialManager](@ref JPL::Spatial::SpatialManager)) | You want one object to coordinate sources, listeners, and services. |

*Note: the Services and Source Manager layers are older high-level APIs and remain under active design review.*

The layers are optional rather than progressive requirements. An integration can use a component directly while using a service for another feature.

## Components API Layer {#components-api-layer}

The Components layer includes the individual features and utilities in JPL Spatial. Many can be used independently or combined into higher-level engine and gameplay systems.

For example:

1. VBAP utilities compute panning gains for an incident direction.
2. MDAP uses VBAP to model source spread and focus.
3. Volumetric sound sources can be spatialized using MDAP.
4. [VBAPanner2D](@ref JPL::VBAPanner2D) and [VBAPanner3D](@ref JPL::VBAPanner3D) provide a common interface for VBAP and MDAP.

### Path Tracing

Path tracing includes ray-traced specular reflections and is not currently part of a service.

The path-tracing APIs are in [PathTracing](https://github.com/Jaytheway/JPLSpatial/tree/main/Spatialization/include/JPLSpatial/PathTracing).

### Audio Rendering

The audio-processing components can be integrated in a mixing graph to render acoustic effects from parameters supplied by JPL Spatial or another system.

They are in [Auralization](https://github.com/Jaytheway/JPLSpatial/tree/main/Spatialization/include/JPLSpatial/Auralization).

## Services API Layer {#services-api-layer}

Services are feature-level managers for groups of related acoustic effects. They manage reusable state and cache the results of repeated evaluations.

A service generally follows this lifecycle:

1. Initialize an effect, allocate its cached data, and receive a handle.
2. Evaluate or process scene parameters for that handle.
3. Retrieve the cached spatial or audio parameters.
4. Release the effect handle when it is no longer needed.

For an example that combines services, see [SpatialManager.h](https://github.com/Jaytheway/JPLSpatial/blob/main/Spatialization/include/JPLSpatial/SpatialManager.h), which implements the [Source Manager API Layer](#source-manager-api-layer).

## Source Manager API Layer {#source-manager-api-layer}

This is a common audio-engine and middleware abstraction: the integration creates, updates, and releases sources while the manager handles storage, parameter updates, and caching.

Such an API may render directly to an output device. JPL Spatial instead returns parameters that the host uses to drive its own audio rendering.

### Use-case: Sound Spatializer {#use-case-sound-spatializer}

[SpatialManager](@ref JPL::Spatial::SpatialManager) can be used as a sound spatializer. Its source-level API includes:

- [SpatialManager::CreateSource](@ref JPL::Spatial::SpatialManager::CreateSource)
- [SpatialManager::DeleteSource](@ref JPL::Spatial::SpatialManager::DeleteSource)
- [SpatialManager::SetSourcePosition](@ref JPL::Spatial::SpatialManager::SetSourcePosition)
- [SpatialManager::SetListenerPosition](@ref JPL::Spatial::SpatialManager::SetListenerPosition)
- [SpatialManager::GetChannelGains](@ref JPL::Spatial::SpatialManager::GetChannelGains)
- ...

See the [SpatialManager class API reference](@ref JPL::Spatial::SpatialManager) for the complete API.

For setup code, see [SpatializerExample](https://github.com/Jaytheway/JPLSpatial/tree/main/Examples/SpatializerExample). For more detail, see [Spatial Manager Lifecycle](@ref spatial-manager-lifecycle).
