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
_OPTIONS["jolt-path"] = "%{wks.location}/vendor/JoltPhysics/JoltPhysics"

include "Spatialization"
include "SpatializationTests"

group "Dependencies"
include "vendor/JoltPhysics/JoltPhysicsPremake.lua"
group ""
