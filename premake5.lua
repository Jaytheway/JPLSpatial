-- premake5.lua
workspace "JPLSpatial"
	architecture "x64"
	configurations { "Debug", "Release", "Dist", "Test", "Profile" }
	startproject "JPLSpatialTests"
		

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

newoption {
    trigger     = "jolt-path",
    value       = "path",
    description = "Path to the JoltPhysics project directory"
}

include "Spatialization"
include "SpatializationTests"

-- Only needed for test project
group "Dependencies"
include "SpatializationTests/vendor/JoltPhysics/JoltPhysicsPremake.lua"
group ""
