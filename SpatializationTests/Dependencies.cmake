# TODO: move this out if we ever need dependencies in the main library

function(jpl_setup_dependencies)
  set(_missing_dependencies)

  # Prefer targets already supplied by a parent or packages installed locally.
  if(NOT TARGET GTest::gtest)
    find_package(GTest QUIET)
  endif()

  if(NOT TARGET glm::glm)
    find_package(glm CONFIG QUIET)
  endif()

  # Load CPM only if at least one required package must be fetched.
  if(JPL_FETCH_DEPS AND (NOT TARGET GTest::gtest OR NOT TARGET glm::glm))
    include("${JPLSpatial_SOURCE_DIR}/cmake/CPM.cmake")
  endif()

  if(NOT TARGET GTest::gtest)
    if(JPL_FETCH_DEPS)
      CPMFindPackage(
        NAME              GTest
        GITHUB_REPOSITORY google/googletest
        GIT_TAG           v1.14.0
        EXCLUDE_FROM_ALL  YES
        OPTIONS           "INSTALL_GTEST OFF"
      )
    endif()

    if(NOT TARGET GTest::gtest)
      list(APPEND _missing_dependencies "GoogleTest (expected target GTest::gtest)")
    endif()
  endif()

  if(NOT TARGET glm::glm)
    if(JPL_FETCH_DEPS)
      CPMFindPackage(
        NAME              glm
        GITHUB_REPOSITORY g-truc/glm
        GIT_TAG           1.0.3
      )
    endif()

    if(NOT TARGET glm::glm)
      list(APPEND _missing_dependencies "glm (expected target glm::glm)")
    endif()
  endif()

  # stb_image_write is vendored with the tests.
  if(NOT TARGET stb_image_write)
    add_library(stb_image_write INTERFACE)
    target_include_directories(stb_image_write INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/vendor")
  endif()

  # Jolt is optional and needed only by the Jolt interoperability tests.
  set(_jolt_target)
  if(JPL_TEST_WITH_JOLT)
    if(TARGET Jolt)
      set(_jolt_target Jolt)
    elseif(TARGET Jolt::Jolt)
      set(_jolt_target Jolt::Jolt)
    else()
      if(JOLT_PATH AND EXISTS "${JOLT_PATH}/Build/CMakeLists.txt")
        message(STATUS "Using local Jolt from: ${JOLT_PATH}")
        add_subdirectory("${JOLT_PATH}/Build" _build_jolt)
      else()
        find_package(Jolt CONFIG QUIET)
      endif()
    endif()

    if(TARGET Jolt)
      set(_jolt_target Jolt)
    elseif(TARGET Jolt::Jolt)
      set(_jolt_target Jolt::Jolt)
    elseif(JPL_FETCH_DEPS)
      if(NOT COMMAND CPMFindPackage)
        include("${JPLSpatial_SOURCE_DIR}/cmake/CPM.cmake")
      endif()

      CPMFindPackage(
        NAME              Jolt
        GITHUB_REPOSITORY jrouwe/JoltPhysics
        GIT_TAG           v5.3.0
        SOURCE_SUBDIR     "Build"
        OPTIONS
          "INTERPROCEDURAL_OPTIMIZATION OFF"
      )

      if(TARGET Jolt)
        set(_jolt_target Jolt)
      elseif(TARGET Jolt::Jolt)
        set(_jolt_target Jolt::Jolt)
      endif()
    endif()

    if(NOT _jolt_target)
      list(APPEND _missing_dependencies
        "JoltPhysics (expected target Jolt or Jolt::Jolt, or set JOLT_PATH)"
      )
    endif()

    # These warning settings apply only when Jolt was built from source here.
    if(MSVC AND TARGET Jolt)
      message(STATUS "Disabling warnings-as-errors for Jolt")
      target_compile_options(Jolt PRIVATE /WX- /wd5045)
    endif()
  endif()

  if(_missing_dependencies)
    list(JOIN _missing_dependencies "\n  - " _missing_dependencies_text)
    message(FATAL_ERROR
      "JPL Spatial tests require dependencies that were not found:\n"
      "  - ${_missing_dependencies_text}\n"
      "Install/provide these packages, or configure with JPL_FETCH_DEPS=ON."
    )
  endif()

  # Put source-built third-party targets under the Dependencies folder in IDEs.
  function(_put_in_folder tgt folder)
    get_target_property(_aliased "${tgt}" ALIASED_TARGET)
    if(_aliased)
      set_target_properties("${_aliased}" PROPERTIES FOLDER "${folder}")
    else()
      set_target_properties("${tgt}" PROPERTIES FOLDER "${folder}")
    endif()
  endfunction()

  set(_dep_targets glm gtest gtest_main gmock gmock_main stb_image_write)
  if(JPL_TEST_WITH_JOLT)
    list(APPEND _dep_targets Jolt JoltPhysics)
  endif()
  foreach(_t IN LISTS _dep_targets)
    if(TARGET "${_t}")
      _put_in_folder("${_t}" "Dependencies")
    endif()
  endforeach()

  target_link_libraries(JPLSpatialTests PRIVATE
    glm::glm
    GTest::gtest
    stb_image_write
  )

  if(JPL_TEST_WITH_JOLT)
    target_link_libraries(JPLSpatialTests PRIVATE ${_jolt_target})
  endif()
endfunction()
