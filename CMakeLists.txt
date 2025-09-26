cmake_minimum_required(VERSION 3.28)

# When turning this option on, the library will be compiled with interprocedural optimizations enabled, also known as link-time optimizations or link-time code generation.
# Note that if you turn this on you need to use SET_INTERPROCEDURAL_OPTIMIZATION() or set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON) to enable LTO specifically for your own project as well.
# If you don't do this you may get an error: error adding symbols: file format not recognized
option(INTERPROCEDURAL_OPTIMIZATION "Enable interprocedural optimizations" OFF)

# Option to enable testing Jolt's types with some of the librarie's interfaces
option(TEST_WITH_JOLT "Enable tests that require JoltPhysics" OFF)

option(JPL_FETCH_DEPS "Allow fetching dependencies with FetchContent" ON)

project(JPLSpatial LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_SOURCE_DIR}/cmake")

# Enable Interprocedural Optimization if requested and supported
include(cmake/InterProcessOptimization.cmake)

# Include CPM.cmake for dependency management
include(cmake/CPM.cmake)

# Use folders in IDEs
set_property(GLOBAL PROPERTY USE_FOLDERS ON)

# Configuration types
if(CMAKE_GENERATOR MATCHES "Visual Studio|Xcode")
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release;Distribution;Profile" CACHE STRING "" FORCE)
endif()

# Some projects may not have a Distribution or Profile config, so we need to map them to Release
set(CMAKE_MAP_IMPORTED_CONFIG_PROFILE       Release)
set(CMAKE_MAP_IMPORTED_CONFIG_DISTRIBUTION  Release)

include(CMakeDependentOption)
# Ability to toggle between the static and DLL versions of the MSVC runtime library
# Windows Store only supports the DLL version
cmake_dependent_option(USE_STATIC_MSVC_RUNTIME_LIBRARY "Use the static MSVC runtime library" ON "MSVC;NOT WINDOWS_STORE" OFF)

# Set runtime library
if (MSVC)
  if (USE_STATIC_MSVC_RUNTIME_LIBRARY)
    set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
  endif()
endif()

# Compiler and linker options.
if(MSVC)
  # MSVC flags
  set(CMAKE_CXX_FLAGS_DEBUG "/Od /Zi")
  set(CMAKE_CXX_FLAGS_RELEASE "/O2 /DNDEBUG")
  set(CMAKE_CXX_FLAGS_DISTRIBUTION "/O2 /DNDEBUG")
  set(CMAKE_CXX_FLAGS_PROFILE "/O2 /Zi") # Use /O2 for optimization, /Zi for debug info

  # MSVC linker flags for debug information
  set(CMAKE_EXE_LINKER_FLAGS_DEBUG "/DEBUG")
  set(CMAKE_EXE_LINKER_FLAGS_PROFILE "/DEBUG")
else() # Assumed to be GCC/Clang
  # GCC/Clang flags
  set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DISTRIBUTION "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_PROFILE "-O3 -g")
endif()
  
# Set output directories
set(OUTPUT_BASE "${CMAKE_SOURCE_DIR}/bin")
set(OBJ_BASE    "${CMAKE_SOURCE_DIR}/bin-int")

string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYS)
set(ARCH "${CMAKE_GENERATOR_PLATFORM}") # assuming Visual Studio generator

function(jpl_set_output_dirs tgt)
  set(cfg_dir "${OUTPUT_BASE}/$<CONFIG>-${SYS}-${ARCH}/${tgt}")
  set(obj_dir "${OBJ_BASE}/$<CONFIG>-${SYS}-${ARCH}/${tgt}")
  set_target_properties(${tgt} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${cfg_dir}"
    LIBRARY_OUTPUT_DIRECTORY "${cfg_dir}"
    ARCHIVE_OUTPUT_DIRECTORY "${cfg_dir}"
    PDB_OUTPUT_DIRECTORY     "${obj_dir}"
    COMPILE_PDB_OUTPUT_DIRECTORY "${obj_dir}"
  )
endfunction()

# Add main library project
add_subdirectory(Spatialization)

# Check if we're the root CMakeLists.txt, if not we are included by another CMake file and we should disable everything except for the main library
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
  message(STATUS "JPLSpatial: Building as root project")
  include(CTest) # defines BUILD_TESTING and calls enable_testing()
  
  if(BUILD_TESTING) # Add test project if enabled
    add_subdirectory(SpatializationTests)
    # Set JPLSpatialTest exe as a startup project in Visual Studio
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "JPLSpatialTests")
  endif()
endif()