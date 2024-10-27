project "JoltPhysics"
    kind "StaticLib"
    language "C++"
    cppdialect "C++17"
    staticruntime "off"

    targetdir ("bin/" .. outputdir .. "/%{prj.name}")
	objdir ("bin-int/" .. outputdir .. "/%{prj.name}")

    files
    {
        "JoltPhysics/Jolt/**.cpp",
        "JoltPhysics/Jolt/**.h",
        "JoltPhysics/Jolt/**.inl",
        "JoltPhysics/Jolt/**.gliffy"
    }

    includedirs { "JoltPhysics/Jolt", "JoltPhysics/" }

    filter "system:windows"
        systemversion "latest"

        files { "JoltPhysics/Jolt/Jolt.natvis" }

    filter "not system:windows"
        forceincludes { "stdint.h", "stdarg.h" }

    filter "configurations:Debug or configurations:Debug-AS"
        symbols "on"
        optimize "off"

        defines
        {
            "_DEBUG",
            "JPH_DEBUG_RENDERER",
            "JPH_FLOATING_POINT_EXCEPTIONS_ENABLED",
            "JPH_EXTERNAL_PROFILE",
			"JPH_ENABLE_ASSERTS"
        }

    filter "configurations:Test"
        runtime "Debug"
        optimize "speed"
        vectorextensions "AVX2"
        isaextensions { "BMI", "POPCNT", "LZCNT", "F16C" }
        defines
        {
            "JPH_FLOATING_POINT_EXCEPTIONS_ENABLED",
            "JPH_EXTERNAL_PROFILE"
        }

    filter "configurations:Release or Profile"
        runtime "Release"
        optimize "speed"
        vectorextensions "AVX2"
        isaextensions { "BMI", "POPCNT", "LZCNT", "F16C" }

        defines
        {
            "JPH_DEBUG_RENDERER",
            "JPH_FLOATING_POINT_EXCEPTIONS_ENABLED",
            "JPH_EXTERNAL_PROFILE"
        }

    filter "configurations:Dist"
        optimize "speed"
        symbols "off"
        vectorextensions "AVX2"
        isaextensions { "BMI", "POPCNT", "LZCNT", "F16C" }