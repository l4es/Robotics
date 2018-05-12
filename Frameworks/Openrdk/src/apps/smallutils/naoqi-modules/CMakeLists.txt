# WELCOME 
# This is the main cmake configuration file 
# associated with your project
# You can define specific compilation options here
# More about cmake : www.cmake.org
# Using cmake command line : 
# 1. Create a build directory : mkdir build
# 2. Configure cmake project : cd build; cmake ..
#   You can also generate project with your favorite IDE : 
#   Example : For Mac osx use "cmake .. -G XCode"
#   see cmake help for more information
# 3. Edit project option : ccmake .
# 4. Build it : make 

cmake_minimum_required(VERSION 2.6.4 FATAL_ERROR)

include("${CMAKE_CURRENT_SOURCE_DIR}/bootstrap.cmake")
use(NAOQI-PLUGINS-TOOLS)

create_module(openrdk)

# Searching a valid setenv.cmake, if any
IF (NOT "$ENV{RDK_BUILD_DIR}" STREQUAL "")
	EXECUTE_PROCESS(COMMAND rdk-config --cmakeinclude OUTPUT_VARIABLE openrdkExternalInclude OUTPUT_STRIP_TRAILING_WHITESPACE)
	MESSAGE(STATUS "Using ${openrdkExternalInclude}")
	INCLUDE("${openrdkExternalInclude}")
ELSE(NOT "$ENV{RDK_BUILD_DIR}" STREQUAL "")
	MESSAGE(FATAL_ERROR "It seems you tried to configure the OpenRDK NaoQi modules, without sourcing a \"setenv\" file. Usually, you can fix it by doing that (remember to compile OpenRDK, too...).")

ENDIF (NOT "$ENV{RDK_BUILD_DIR}" STREQUAL "")

MESSAGE(STATUS "..:: OpenRDK variables ::..")
RDK_PRINT_VARIABLES()

SET ( openrdk_SRCS
	src/openrdkmain.cpp
	src/rsensorsserver.h
	src/rsensorsserver.cpp
	#src/rvisionserver.h
	#src/rvisionserver.cpp
	src/rledsserver.h
	src/rledsserver.cpp
	src/raudioserver.h
	src/raudioserver.cpp
	${OpenRDK_ROOT}/src/libraries/rdkcore/sharedmemory/sharedmemory.cpp
	)


include_directories(${RDKCORE_INCLUDE_DIR};${CMAKE_CURRENT_SOURCE_DIR}/../../../libraries)
link_directories(${RDKCORE_LINK_DIRECTORIES})
add_definitions(${RDKCORE_DEFINITIONS})

configure_src_module( openrdk ${openrdk_SRCS} )

target_link_libraries(openrdk rdknao)

use_lib( openrdk ALCOMMON ALMATH LIBCORE TOOLS ALVALUE LIBVISION)



