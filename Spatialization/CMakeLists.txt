add_library(JPLSpatial STATIC)

jpl_enable_ipo()

# Sources
file(GLOB_RECURSE JPLSPATIAL_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
)
target_sources(JPLSpatial PRIVATE ${JPLSPATIAL_SRC})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${JPLSPATIAL_SRC})

# C++ version
target_compile_features(JPLSpatial PUBLIC cxx_std_20)

# Includes: public headers from include/, private headers from src/
target_include_directories(JPLSpatial
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src/Spatialization
)

# Namespaced alias
add_library(JPL::Spatial ALIAS JPLSpatial)

# Place the project in a folder in IDEs
set_target_properties(JPLSpatial PROPERTIES FOLDER "Core")

# Global defines
target_compile_definitions(JPLSpatial
  PRIVATE
    JPL_HAS_ENV_PROPAGATION=0
    JPL_HAS_PATH_TRACING=0
    JPL_HAS_BEAM_TRACING=0
)

# Per-config defines (DEBUG/NDEBUG + JPL_TEST if tests are enabled)
target_compile_definitions(JPLSpatial
  PRIVATE
    $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
    $<$<BOOL:${BUILD_TESTING}>:JPL_TESTS>
)

# Linux PIC
if(UNIX AND NOT APPLE)
  set_target_properties(JPLSpatial PROPERTIES POSITION_INDEPENDENT_CODE ON)
endif()

# x86/AVX2 vs x86/SSE2, skip on non-x86
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
  if(MSVC)
    target_compile_options(JPLSpatial PRIVATE /arch:AVX2)
  else()
    target_compile_options(JPLSpatial PRIVATE -mavx2 -mfma)
  endif()
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86|i[3-6]86)$")
  if(MSVC)
    target_compile_options(JPLSpatial PRIVATE /arch:SSE2)
  else()
    target_compile_options(JPLSpatial PRIVATE -msse2)
  endif()
endif()

# Set output directories
jpl_set_output_dirs(JPLSpatial)