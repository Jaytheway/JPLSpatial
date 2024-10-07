	project "JPLSpatializationTests"
	language "C++"
	cppdialect "C++20"
	staticruntime "off"

	filter {"configurations:not Test"}
		kind "None"
	filter {"configurations:Test"}
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

		includedirs
		{
			"../Spatialization/include",
			"../Spatialization/vendor",

			"vendor/googletest/googletest/include",
			"vendor/googletest/googletest/"
		}

		links
		{
			"JPLSpatialization",
		}

		group "Core"
		include "Spatialization"
		group ""

		dependson { "JPLSpatialization" }

		filter "configurations:Test"
			runtime "Debug"
			symbols "on"
			optimize "off"
			defines { "JPL_TEST" }
