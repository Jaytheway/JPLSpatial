set(CPM_DOWNLOAD_VERSION 0.40.8)
set(CPM_HASH_SUM "78ba32abdf798bc616bab7c73aac32a17bbd7b06ad9e26a6add69de8f3ae4791")

# 1) Where to place the CPM script itself
set(_cpm_dl_dir "${CMAKE_BINARY_DIR}/cpm")
file(MAKE_DIRECTORY "${_cpm_dl_dir}")
set(CPM_DOWNLOAD_LOCATION "${_cpm_dl_dir}/CPM_${CPM_DOWNLOAD_VERSION}.cmake")

# 2) Where to cache package sources (defaults to build-local; override if desired)
if(NOT DEFINED CPM_SOURCE_CACHE AND NOT DEFINED ENV{CPM_SOURCE_CACHE})
  # build-local cache (wiped with build folder)
  set(CPM_SOURCE_CACHE "${CMAKE_BINARY_DIR}/_cpm")
endif()
# If we need project-local persistence across builds:
# set(CPM_SOURCE_CACHE "${CMAKE_SOURCE_DIR}/.cpm")  # then add ".cpm/" to .gitignore

# Expand relative path. This is important if the provided path contains a tilde (~)
get_filename_component(CPM_DOWNLOAD_LOCATION ${CPM_DOWNLOAD_LOCATION} ABSOLUTE)

# Download CPM into the build dir only
file(DOWNLOAD
  "https://github.com/cpm-cmake/CPM.cmake/releases/download/v${CPM_DOWNLOAD_VERSION}/CPM.cmake"
  "${CPM_DOWNLOAD_LOCATION}"
  EXPECTED_HASH "SHA256=${CPM_HASH_SUM}"
  TLS_VERIFY ON
)

include("${CPM_DOWNLOAD_LOCATION}")

# Optional defaults
set(CPM_USE_LOCAL_PACKAGES ON)
# set(CPM_LOCAL_PACKAGES_ONLY ON)  # force offline/local-only if needed
