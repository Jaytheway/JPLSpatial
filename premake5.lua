-- premake5.lua
workspace "JPLSpatialization"
	architecture "x64"
	configurations { "Debug", "Release", "Dist", "Test" }
	startproject "JPLSpatializationTests"
		

outputdir = "%{cfg.buildcfg}-%{cfg.system}-%{cfg.architecture}"

include "Spatialization"
include "SpatializationTests"
