cmake_minimum_required(VERSION 3.7.2)

##################################################
set (VERSION_NUMBER "0.1.0")
project("SolARModulePCL")
set (SOURCES
	src/SolARModulePCL.cpp
	src/SolARICPNormalsPCL.cpp
	src/SolARICPPCL.cpp
	src/SolARPCFilterCentroidPCL.cpp
	src/SolARPCFilterPCL.cpp
	src/SolARPointCloudLoaderPCL.cpp
)

set (HEADERS
	interfaces/SolARPCLAPI.h
    interfaces/SolARModulePCL_traits.h
	interfaces/SolARICPNormalsPCL.h
	interfaces/SolARICPPCL.h
	interfaces/SolARPCFilterCentroidPCL.h
	interfaces/SolARPCFilterPCL.h
	interfaces/SolARPointCloudLoaderPCL.h
)

#find_package(PCL REQUIRED)

include_directories(
	src/
)

install (FILES "${CMAKE_CURRENT_SOURCE_DIR}/xpcf_SolARModulePCL_registry.xml" DESTINATION $ENV{BCOMDEVROOT}/.xpcf/SolAR/	)

##################################################

# various macros
include("$ENV{BCOMDEVROOT}/bcomBuild/SolARFramework/solarmacros.cmake")
# config setup
setup()
# process packagedependencies.txt
processPackagedependencies()
# define targets (type library)
defineTargets("library" "")
