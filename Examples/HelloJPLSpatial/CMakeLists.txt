cmake_minimum_required(VERSION 3.28)

project(HelloJPLSpatial LANGUAGES CXX)

set(
    JPLSPATIAL_SOURCE_DIR
    "${CMAKE_CURRENT_LIST_DIR}/../.."
    CACHE PATH
    "Path to the JPL Spatial source tree"
)

if(NOT TARGET JPL::Spatial)
    add_subdirectory(
        "${JPLSPATIAL_SOURCE_DIR}"
        "${CMAKE_CURRENT_BINARY_DIR}/JPLSpatial"
    )
endif()

add_executable(HelloJPLSpatial main.cpp)
target_compile_features(HelloJPLSpatial PRIVATE cxx_std_20)
target_link_libraries(HelloJPLSpatial PRIVATE JPL::Spatial)
