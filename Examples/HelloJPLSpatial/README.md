# Hello JPL Spatial {#hello-example}

This dependency-free console example uses JPL Spatial's low-level 2D VBAP
panner. It places a point source in front of, to the right of, behind, and to the
left of a listener and prints the gain sent to each speaker in a quad output
layout.

The example deliberately starts with the low-level panner because it is a small,
focused integration surface. It does not require an audio device or audio-file
dependency: the computed gains are the values a host would apply in its own
mixer.

## Build from the repository

From the JPL Spatial repository root:

```sh
cmake -S Examples/HelloJPLSpatial -B build/hello
cmake --build build/hello --config Release
```

The executable location follows the build tree supplied with `-B`. Run it from
an open terminal. For the commands above, a single-config build runs with:

```sh
./build/hello/HelloJPLSpatial
```

The Visual Studio Release build runs with:

```sh
./build/hello/Release/HelloJPLSpatial.exe
```

Expected result:

```text
front  FL=0.7071  FR=0.7071  BL=0.0000  BR=0.0000
right  FL=0.0000  FR=0.7071  BL=0.0000  BR=0.7071
back   FL=0.0000  FR=0.0000  BL=0.7071  BR=0.7071
left   FL=0.7071  FR=0.0000  BL=0.7071  BR=0.0000
```

The source directions are already normalized and use JPL Spatial's right-handed,
Y-up, -Z-forward coordinate convention.

## Use a different JPL Spatial checkout

`JPLSPATIAL_SOURCE_DIR` is a cache variable, so the example can also be pointed
at another checkout:

```sh
cmake -S Examples/HelloJPLSpatial -B build/hello \
  -DJPLSPATIAL_SOURCE_DIR=/path/to/JPLSpatial
```
