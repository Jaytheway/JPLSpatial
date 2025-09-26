function(jpl_enable_ipo)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE OFF PARENT_SCOPE)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DIST OFF PARENT_SCOPE)
    set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_PROFILE OFF PARENT_SCOPE)

    if (INTERPROCEDURAL_OPTIMIZATION
        AND NOT ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "ARM64")
		AND NOT ("${CMAKE_VS_PLATFORM_NAME}" STREQUAL "ARM")
		AND (NOT CROSS_COMPILE_ARM OR ("${CROSS_COMPILE_ARM_TARGET}" STREQUAL "aarch64-linux-gnu")))

        include(CheckIPOSupported)
        check_ipo_supported(RESULT _ipo_supported OUTPUT _ipo_msg)

		if (_ipo_supported)
            # MSVC: IPO doesn’t work with /ZI (Edit&Continue). Guard it out.
            if(MSVC AND CMAKE_MSVC_DEBUG_INFORMATION_FORMAT STREQUAL "EditAndContinue")
                message(STATUS "IPO skipped for MSVC: /ZI is enabled")
                return()
            endif()

			message("Interprocedural optimizations are turned on")
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_RELEASE ON PARENT_SCOPE)
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_DIST ON PARENT_SCOPE)
			set(CMAKE_INTERPROCEDURAL_OPTIMIZATION_PROFILE ON PARENT_SCOPE)
		else()
			message("Warning: Interprocedural optimizations are not supported for this target, turn off the option INTERPROCEDURAL_OPTIMIZATION to disable this warning")
		endif()
	endif()
endfunction()