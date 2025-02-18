
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
		
	filter "configurations:Release"
		runtime "Release"
		optimize "on"
		
	filter "configurations:Dist"
		runtime "Release"
		optimize "on"
		symbols "off"
		
	filter "configurations:Test"
		runtime "Debug"
		symbols "on"
		optimize "off"
		defines { "JPL_TEST" }

	filter "configurations:Profile"
		runtime "Release"
		symbols "on"
		optimize "speed"
		defines { "JPL_TEST" }

