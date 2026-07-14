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

# Setup ISAs
if(JPL_ENABLE_AVX2_FMA)
	if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|AMD64)$")
		target_compile_definitions(JPLSpatial PUBLIC
			JPL_SIMD_CONFIGURED=1
			JPL_USE_AVX2=1
			JPL_USE_FMADD=1
		)
		if(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "MSVC")
			target_compile_options(JPLSpatial PUBLIC /arch:AVX2)
		elseif(CMAKE_CXX_COMPILER_FRONTEND_VARIANT STREQUAL "GNU")
			target_compile_options(JPLSpatial PUBLIC -mavx2 -mfma)
		else()
			message(FATAL_ERROR "Unsupported compiler frontend for AVX2_FMA")
		endif()
	else()
		message(FATAL_ERROR "JPL_ENABLE_AVX2_FMA is only supported on x86-64 architectures")
	endif()
else()
	target_compile_definitions(JPLSpatial PUBLIC JPL_SIMD_CONFIGURED=1)
endif()

# Set output directories
jpl_set_output_dirs(JPLSpatial)