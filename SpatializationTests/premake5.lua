project "JPLSpatialTests"
	language "C++"
	cppdialect "C++20"
	staticruntime "off"

	filter {"configurations:not Test and not Profile"}
		kind "None"
	filter {"configurations:Test or Profile"}
		kind "ConsoleApp"

		targetdir ("bin/" .. outputdir .. "/%{prj.name}")
		objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

		files {
			"src/**.h",
			"src/**.cpp",
			"include/**.h",
			"vendor/**.h",
			
			"vendor/googletest/googletest/**.h",
        	"vendor/googletest/googletest/**.hpp",
        	"vendor/googletest/googletest/src/gtest-all.cc"
		}
		
		local joltPath = _OPTIONS["jolt-path"] or "../../vendor/JoltPhysics/JoltPhysics"

		includedirs {
			"../Spatialization/include",
			"../Spatialization/src/Spatialization",
			joltPath,
			
			"vendor/googletest/googletest/include",
			"vendor/googletest/googletest/",

			"vendor/glm",

			--"../Spatialization/vendor/nsimd/include",
		}

		libdirs {
			"../Spatialization/vendor/nsimd/build/Release"
		}

		links {
			"JPLSpatial",
			"JoltPhysics",
			--"nsimd_NEON128.lib",
			--"nsimd_AArch64.lib",
			--"nsimd_SSE2.lib",
		}

		group "Core"
		include "Spatialization"
		group ""

		dependson { "JPLSpatial" }

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
