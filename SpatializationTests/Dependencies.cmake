# TODO: move this out if we ever need dependencies in the main library

function(jpl_setup_dependencie)
  # --- GoogleTest (tests only) ---
  if(NOT TARGET GTest::gtest)
    CPMFindPackage(
      NAME              GTest
      GITHUB_REPOSITORY google/googletest
      GIT_TAG           v1.14.0
      EXCLUDE_FROM_ALL  YES
      OPTIONS           "INSTALL_GTEST OFF"
    )
  endif()

  # --- glm (header-only) ---
  if(NOT TARGET glm::glm)
    CPMFindPackage(
      NAME              glm
      GITHUB_REPOSITORY g-truc/glm
      GIT_TAG           1.0.3
    )
  endif()

  # --- stb image (local header) ---
  add_library(stb_image_write INTERFACE)
  target_include_directories(stb_image_write INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/vendor")

  # --- Jolt ---
  if(TEST_WITH_JOLT)
    if(NOT TARGET Jolt)
      # Local checkout path
      if(JOLT_PATH AND EXISTS "${JOLT_PATH}/Build/CMakeLists.txt")
        message(STATUS "Using local Jolt from: ${JOLT_PATH}")
        add_subdirectory("${JOLT_PATH}/Build" _build_jolt)
      else()
        # Fetch from GitHub, but only download; real project lives in Build/
        CPMFindPackage(
          NAME              Jolt
          GITHUB_REPOSITORY jrouwe/JoltPhysics
          GIT_TAG           v5.3.0
          SOURCE_SUBDIR     "Build"
          OPTIONS
            "JPH_BUILD_SAMPLES OFF"
            "JPH_BUILD_TESTS OFF"
            "INTERPROCEDURAL_OPTIMIZATION OFF"
        )
      endif()
    endif()

    # MSVC tweaks (warnings-as-errors off for Jolt only)
    if(MSVC AND TARGET Jolt)
      message(STATUS "Disabling warnings-as-errors for Jolt")
      target_compile_options(Jolt PRIVATE /WX- /wd5045)
    endif()
  endif()

  # Put 3rd-party targets under the "Dependencies" folder in VS
  function(_put_in_folder tgt folder)
    get_target_property(_aliased "${tgt}" ALIASED_TARGET)
    if(_aliased)
      set_target_properties("${_aliased}" PROPERTIES FOLDER "${folder}")
    else()
      set_target_properties("${tgt}" PROPERTIES FOLDER "${folder}")
    endif()
  endfunction()

  set(_dep_targets glm gtest gtest_main gmock gmock_main stb_image_write)
  if(TEST_WITH_JOLT)
    list(APPEND _dep_targets Jolt JoltPhysics)
  endif()
  foreach(_t IN LISTS _dep_targets)
    if(TARGET "${_t}")
      _put_in_folder("${_t}" "Dependencies")
    endif()
  endforeach()

  # Link the libaries
  target_link_libraries(JPLSpatialTests PRIVATE
    glm::glm
    $<$<BOOL:${TEST_WITH_JOLT}>:Jolt>
    GTest::gtest
    stb_image_write
  )
endfunction()

