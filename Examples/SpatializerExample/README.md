# Spatializer Example {#spatializer-example}

This example uses the high-level `SpatialManager` API in the role of a game
engine sound spatializer. It moves a mono source around a listener, updates the
spatial simulation, and retrieves quad speaker gains and distance attenuation.
A host audio engine would use these values when mixing the source audio.

`SpatialManager` is an older API that remains under active design review. This
example documents its current workflow; it does not render or play audio.

## Build from the repository

From the JPL Spatial repository root:

```sh
cmake -S Examples/SpatializerExample -B build/spatializer
cmake --build build/spatializer --config Release
```

Run the resulting executable from an open terminal. A single-config build runs
with:

```sh
./build/spatializer/SpatializerExample
```

The Visual Studio Release build runs with:

```sh
./build/spatializer/Release/SpatializerExample.exe
```

Expected result:

```text
front  attenuation=0.2000  FL=0.7071  FR=0.7071  BL=0.0000  BR=0.0000
right  attenuation=0.2000  FL=0.0000  FR=0.7071  BL=0.0000  BR=0.7071
back   attenuation=0.2000  FL=0.0000  FR=0.0000  BL=0.7071  BR=0.7071
left   attenuation=0.2000  FL=0.7071  FR=0.0000  BL=0.7071  BR=0.0000
```

The positions use JPL Spatial's right-handed, Y-up, -Z-forward coordinate
convention. The example prints panning gains and distance attenuation
separately; combining them is the responsibility of the host mixer.

## Use a different JPL Spatial checkout

`JPLSPATIAL_SOURCE_DIR` is a cache variable, so the example can also be pointed
at another checkout:

```sh
cmake -S Examples/SpatializerExample -B build/spatializer \
  -DJPLSPATIAL_SOURCE_DIR=/path/to/JPLSpatial
```

## Spatial Manager Lifecycle {#spatial-manager-lifecycle}

#### 1. Initialize [SpatialManager](@ref JPL::Spatial::SpatialManager)

- JPL::PanningService::CreatePannerFor - preinitialize panners for expected output channel maps
- JPL::PanningService::CreatePanningDataFor - preinitialize panning data for expected JPL::SourceLayoutKey{ source, target } channel map pairs

The expected channel maps can come from user or platform config.  
Or you can initialize all supported source and target channel maps.


Note: each panner has a LUT, which can take up to 1 MB of space for channel maps with height channels; therefore, it's best to initialize only what you know will be needed.

This is an optional step, JPL::Spatial::SpatialManager::CreateSource (in step 2) can initialize the required panning objects itself; explicitly creating panners/layouts beforehand moves LUT construction and allocations out of later execution.

#### 2. Initialize JPL Spatial source(s)

- fill in JPL::Spatial::SourceInitParameters struct
- JPL::Spatial::SpatialManager::CreateSource - let spatial manager create all the internal source bookkeeping data
- store for later all the relevant source data associated to JPL::Spatial::SourceId

#### 3. Update Source & Listener positions in your update loop

- JPL::Spatial::SpatialManager::SetSourcePosition - takes source handle and JPL::Position
- JPL::Spatial::SpatialManager::SetListenerPosition - takes listener handle and JPL::Position

#### 4. Trigger update of spatial parameters

After all the updated positions have been sent to Spatial Manager:
- JPL::Spatial::SpatialManager::AdvanceSimulation - this will trigger the update of all the spatial parameters based on the updated positions.

#### 5. Retrieve the updated parameters

- JPL::Spatial::SpatialManager::GetLastUpdatedSources - get the list of sources that were updated the last simulation frame
- get your previously stored source data, if relevant
- JPL::Spatial::SpatialManager::GetChannelGains - retrieve the last updated channel mixing gains for a source; this is panning gains, to be used for mixing source sample data to the target output
- JPL::Spatial::SpatialManager::GetDistanceAttenuation - get the last updated distance attenuation factor; this can be applied to the mixing gains or any other audio or gameplay parameters
- JPL::Spatial::SpatialManager::GetConeAttenuation - get the last updated angle attenuation factor; this can also be applied to the mixing gains or any other audio or gameplay parameters

#### 6. Mix Source audio according to updated gains

This is engine-specific, but the gist of it is to mix-add each source channel to each output channel with corresponding gain from spatializer.

#### 7. Release no longer needed sources

When sound object is destroyed, or no longer relevant, it's generally a good idea to release no longer needed cached data.

- JPL::Spatial::SpatialManager::DeleteSource - erases all the cached data and releases all the [service effects](@ref services-api-layer) for the specified source handle.
