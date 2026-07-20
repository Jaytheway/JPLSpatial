## Building and integrating

**JPL Spatial** requires CMake 3.28 or newer and a C++20 compiler. The main `JPLSpatial` static library has no external dependencies.

### Standalone build

```sh
cmake -S . -B build -DBUILD_TESTING=OFF
cmake --build build --config Release
```

Standalone multi-config builds additionally provide:

- `Distribution`: equivalent to `Release`;
- `Profile`: optimized, with debug information.

User-provided configurations are preserved.

### Integration

Add the repository directly:

```cmake
add_subdirectory(path/to/JPLSpatial)
target_link_libraries(MyTarget PRIVATE JPL::Spatial)
```

Alternatively, use `FetchContent`:

```cmake
include(FetchContent)

FetchContent_Declare(
    JPLSpatial
    GIT_REPOSITORY https://github.com/Jaytheway/JPLSpatial.git
    GIT_TAG        <tag-or-commit>
)

FetchContent_MakeAvailable(JPLSpatial)

target_link_libraries(MyTarget PRIVATE JPL::Spatial)
```

When used as a subproject, **JPL Spatial** creates only its library target. It does not:

- enable or add its tests;
- download test dependencies;
- modify the parent’s configuration list;
- override parent output directories;
- change the parent’s compiler or linker flags.

### Build options

| Option | Default | Description |
|---|---:|---|
| `JPL_ENABLE_AVX2_FMA` | `OFF` | Requires AVX2 and FMA on x86-64. |
| `JPL_ENABLE_IPO` | `OFF` | Enables supported interprocedural optimization for Release, Distribution, and Profile. |
| `JPL_MSVC_RUNTIME_LIBRARY` | Context-dependent | Selects `Default`, `Dynamic`, or `Static` MSVC runtime behavior. |

On x86-64, the default build uses the SSE2 baseline. Enabling `JPL_ENABLE_AVX2_FMA` raises the minimum CPU requirement; there is no runtime ISA dispatch. The selected ISA requirements propagate to targets linking `JPL::Spatial`.

ARM64 uses its baseline NEON support without additional compiler flags.

For the MSVC runtime:

- standalone desktop builds default to `Static`;
- subproject builds default to `Default` and inherit the parent/toolchain selection;
- `Dynamic` explicitly selects `/MD` or `/MDd`;
- `Static` explicitly selects `/MT` or `/MTd`;
- Windows Store builds always use the dynamic runtime.

### Tests

Tests are available only when **JPL Spatial** is configured as the top-level project:

```sh
cmake -S . -B build -DBUILD_TESTING=ON
cmake --build build --config Release
ctest --test-dir build -C Release --output-on-failure
```

Test-specific options:

| Option | Default | Description |
|---|---:|---|
| `JPL_FETCH_DEPS` | `ON` | Fetches missing test dependencies through CPM. |
| `JPL_TEST_WITH_JOLT` | `OFF` | Enables Jolt Physics interoperability tests. |

With `JPL_FETCH_DEPS=OFF`, CMake searches for installed GoogleTest, glm, and optional Jolt packages and reports all missing dependencies during configuration. A local Jolt checkout can be supplied through `JOLT_PATH`.

You should link `JPL::Spatial` and should not override exported `JPL_*` feature definitions manually.

> [!NOTE]
> The repository currently supports build-tree integration through `add_subdirectory`/`FetchContent`. 
> A complete installed CMake package workflow is not provided yet.
