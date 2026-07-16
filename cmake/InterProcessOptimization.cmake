function(jpl_enable_ipo TARGET)
    set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE FALSE)
    set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_DISTRIBUTION FALSE)
    set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_PROFILE FALSE)

    if (JPL_ENABLE_IPO
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
			set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_RELEASE TRUE)
			set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_DISTRIBUTION TRUE)
			set_property(TARGET ${TARGET} PROPERTY INTERPROCEDURAL_OPTIMIZATION_PROFILE TRUE)
		else()
			message("Warning: Interprocedural optimizations are not supported for this target, turn off the option JPL_ENABLE_IPO to disable this warning")
		endif()
	endif()
endfunction()
