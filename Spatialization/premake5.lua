
local joltPath = _OPTIONS["jolt-path"] or "../../vendor/JoltPhysics/JoltPhysics"
	
project "JPLSpatial"
	kind "StaticLib"
	language "C++"
	cppdialect "C++20"
	staticruntime "off"

	targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

	files {
		"src/**.h",
		"src/**.cpp",
		"include/**.h",
		"vendor/choc/**.h",
	}

	includedirs {
		"include",
		"src/Spatialization",
		--"vendor/nsimd/include",
	}
	
	externalincludedirs {
		joltPath,
	}

	libdirs {
		"vendor/nsimd/build/Release"
	}

	links {
		--"nsimd_NEON128.lib",
		--"nsimd_AArch64.lib",
		--"nsimd_SSE2.lib",
	}

	dependson { "JoltPhysics" }

	-- Temp. Disable work in progress stuff
	defines { "JPL_HAS_ENV_PROPAGATION=0" }
	defines { "JPL_HAS_PATH_TRACING=0" }
	defines { "JPL_HAS_BEAM_TRACING=0" }


	filter "system:windows"
		systemversion "latest"
		cppdialect "C++20"

	filter "system:linux"
		pic "On"
		systemversion "latest"
		cppdialect "C++20"

	filter "configurations:Debug"
		runtime "Debug"
		symbols "on"
		defines { "DEBUG" }
		
	filter "configurations:Release"
		runtime "Release"
		optimize "on"
		defines { "NDEBUG" }
		
	filter "configurations:Dist"
		runtime "Release"
		optimize "on"
		symbols "off"
		defines { "NDEBUG" }
		
	filter "configurations:Test"
		runtime "Debug"
		symbols "on"
		optimize "off"
		defines { "JPL_TEST" }

	filter "configurations:Profile"
		runtime "Release"
		symbols "on"
		optimize "speed"
		defines { "NDEBUG" }
		--defines { "JPL_TEST" }

	-- Enable AVX2 vector extensions for x64 builds on all toolchains
    filter { "architecture:x86_64" }
        vectorextensions "AVX2"

    -- Fallback to SSE2 for 32-bit builds
    filter { "architecture:x86" }
        vectorextensions "SSE2"
