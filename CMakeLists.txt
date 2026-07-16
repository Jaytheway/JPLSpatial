cmake_minimum_required(VERSION 3.28)

project(JPLSpatial LANGUAGES CXX)

# When turning this option on, the library will be compiled with interprocedural optimizations enabled, also known as link-time optimizations or link-time code generation.
# Note that if you turn this on you need to use SET_INTERPROCEDURAL_OPTIMIZATION() or set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON) to enable LTO specifically for your own project as well.
# If you don't do this you may get an error: error adding symbols: file format not recognized
option(JPL_ENABLE_IPO "Enable interprocedural optimizations" OFF)

option(JPL_ENABLE_AVX2_FMA "Require AVX2 and FMA on x86-64" OFF)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# Enable Interprocedural Optimization if requested and supported
include(cmake/InterProcessOptimization.cmake)

if (PROJECT_IS_TOP_LEVEL)
	# Use folders in IDEs
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)
endif()

if(PROJECT_IS_TOP_LEVEL)
    get_property(
        _jpl_is_multi_config
        GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG
    )

    if(_jpl_is_multi_config)
        list(APPEND
            CMAKE_CONFIGURATION_TYPES
            Distribution
            Profile
        )
        list(REMOVE_DUPLICATES CMAKE_CONFIGURATION_TYPES)

        # Update the existing cache entry after preserving configurations
        # supplied by the generator or user.
        set(
            CMAKE_CONFIGURATION_TYPES
            "${CMAKE_CONFIGURATION_TYPES}"
            CACHE STRING
            "Available build configurations"
            FORCE
        )
    endif()

    unset(_jpl_is_multi_config)

    # Imported dependencies generally provide Release but not our standalone custom configurations.
    set(CMAKE_MAP_IMPORTED_CONFIG_PROFILE       Release)
    set(CMAKE_MAP_IMPORTED_CONFIG_DISTRIBUTION  Release)
endif()

# Select the default without overriding an embedding project.
if(PROJECT_IS_TOP_LEVEL)
	set(_jpl_default_msvc_runtime "Static")
else()
	set(_jpl_default_msvc_runtime "Default")
endif()

if(MSVC AND NOT WINDOWS_STORE)
	set(
		JPL_MSVC_RUNTIME_LIBRARY
		"${_jpl_default_msvc_runtime}"
		CACHE STRING
		"MSVC runtime: Default, Dynamic, or Static"
	)
	set_property(
		CACHE JPL_MSVC_RUNTIME_LIBRARY
		PROPERTY STRINGS Default Dynamic Static
	)
elseif(MSVC)
	# Windows Store supports only the DLL runtime.
	set(JPL_MSVC_RUNTIME_LIBRARY "Dynamic")
else()
	# The setting is irrelevant for non-MSVC toolchains.
	set(JPL_MSVC_RUNTIME_LIBRARY "Default")
endif()

unset(_jpl_default_msvc_runtime)

if(MSVC)
	if(JPL_MSVC_RUNTIME_LIBRARY STREQUAL "Static")
		set(
			CMAKE_MSVC_RUNTIME_LIBRARY
			"MultiThreaded$<$<CONFIG:Debug>:Debug>"
		)
	elseif(JPL_MSVC_RUNTIME_LIBRARY STREQUAL "Dynamic")
		set(
			CMAKE_MSVC_RUNTIME_LIBRARY
			"MultiThreaded$<$<CONFIG:Debug>:Debug>DLL"
		)
	elseif(NOT JPL_MSVC_RUNTIME_LIBRARY STREQUAL "Default")
		message(FATAL_ERROR
			"JPL_MSVC_RUNTIME_LIBRARY must be Default, Dynamic, or Static"
		)
	endif()
endif()


# Map our custom compiler and linker options.
if(PROJECT_IS_TOP_LEVEL)
	set(CMAKE_CXX_FLAGS_DISTRIBUTION "${CMAKE_CXX_FLAGS_RELEASE}")
	set(CMAKE_EXE_LINKER_FLAGS_DISTRIBUTION "${CMAKE_EXE_LINKER_FLAGS_RELEASE}")
	set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
	set(CMAKE_EXE_LINKER_FLAGS_PROFILE "${CMAKE_EXE_LINKER_FLAGS_RELWITHDEBINFO}")

	if(MSVC)
		set(
			CMAKE_MSVC_DEBUG_INFORMATION_FORMAT
			"$<$<CONFIG:Debug,RelWithDebInfo,Profile>:ProgramDatabase>"
		)
	endif()
endif()

# Set output directories
if(PROJECT_IS_TOP_LEVEL)
	set(OUTPUT_BASE "${PROJECT_BINARY_DIR}/bin")
	set(OBJ_BASE    "${PROJECT_BINARY_DIR}/bin-int")
	string(TOLOWER "${CMAKE_SYSTEM_NAME}" SYS)
	set(ARCH "${CMAKE_GENERATOR_PLATFORM}") # assuming Visual Studio generator
	if(NOT ARCH)
		set(ARCH "${CMAKE_SYSTEM_PROCESSOR}")
	endif()

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
endif()

# Add main library project
add_subdirectory(Spatialization)

# Check if we're the root CMakeLists.txt, if not we are included by another CMake file and we should disable everything except for the main library
if (PROJECT_IS_TOP_LEVEL)
  message(STATUS "JPLSpatial: Building as root project")
  include(CTest) # defines BUILD_TESTING and calls enable_testing()
  
  if(BUILD_TESTING) # Add test project if enabled
    # Options used only by JPL Spatial's own tests.
    option(JPL_TEST_WITH_JOLT "Enable tests that require JoltPhysics" OFF)
    option(JPL_FETCH_DEPS "Allow fetching missing test dependencies" ON)

    add_subdirectory(SpatializationTests)
    # Set JPLSpatialTest exe as a startup project in Visual Studio
    set_property(DIRECTORY PROPERTY VS_STARTUP_PROJECT "JPLSpatialTests")
  endif()
endif()
