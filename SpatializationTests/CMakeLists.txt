add_executable(JPLSpatialTests)

# Sources
file(GLOB_RECURSE TEST_SRC
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/vendor/**.h"
)
target_sources(JPLSpatialTests PRIVATE ${TEST_SRC})
source_group(TREE "${CMAKE_CURRENT_SOURCE_DIR}" FILES ${TEST_SRC})

# C++ version
target_compile_features(JPLSpatialTests PRIVATE cxx_std_20)

# Includes
target_include_directories(JPLSpatialTests PRIVATE
  "${CMAKE_SOURCE_DIR}/Spatialization/include"
  "${CMAKE_SOURCE_DIR}/Spatialization/src/Spatialization"
)

# === 3rd-party dependencies ===
include(Dependencies.cmake)
jpl_setup_dependencie()

#jpl_enable_ipo()

# Link main library
target_link_libraries(JPLSpatialTests PRIVATE JPLSpatial)

# Place the project in a folder in IDEs
set_target_properties(JPLSpatialTests PROPERTIES FOLDER "Tests")

set_target_properties(JPLSpatialTests PROPERTIES
  VS_DEBUGGER_WORKING_DIRECTORY "$<TARGET_FILE_DIR:JPLSpatialTests>"
)

# Auto-discover gtest cases in the built binary
include(GoogleTest)
gtest_discover_tests(JPLSpatialTests
  WORKING_DIRECTORY $<TARGET_FILE_DIR:JPLSpatialTests>
  DISCOVERY_TIMEOUT 60
  DISCOVERY_MODE PRE_TEST
)

# Per-config defines
target_compile_definitions(JPLSpatialTests PRIVATE
  JPL_TEST
  $<$<BOOL:${TEST_WITH_JOLT}>:JPL_TEST_WITH_JOLT>
  $<$<NOT:$<CONFIG:Debug>>:NDEBUG>
)

# Ensure we build a console app
set_target_properties(JPLSpatialTests PROPERTIES WIN32_EXECUTABLE OFF)

# Set output directories
jpl_set_output_dirs(JPLSpatialTests)
