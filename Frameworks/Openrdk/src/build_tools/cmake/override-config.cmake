# This file will override any user change or any configuration
# Please be carefull on modifying it!

# If CMAKE_BUILD_TYPE is empty, choose RELEASE
if(CMAKE_BUILD_TYPE STREQUAL "")
	SET(CMAKE_BUILD_TYPE "RELEASE" CACHE STRING "Building configuration" FORCE)
endif(CMAKE_BUILD_TYPE STREQUAL "")

# Specify flags for standard configurations
if (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
	set(COMPILATION_FLAGS "${COMPILATION_FLAGS} -O3" CACHE STRING "Compilation flags" FORCE)
elseif (CMAKE_BUILD_TYPE STREQUAL "DEBUG")
	set(COMPILATION_FLAGS "${COMPILATION_FLAGS} -ggdb" CACHE STRING "Compilation flags" FORCE)
else (CMAKE_BUILD_TYPE STREQUAL "RELEASE")
	# If unsure, just aske the user!
	error("Invalid build type specified, please choose DEBUG or RELEASE in ccmake.")
endif (CMAKE_BUILD_TYPE STREQUAL "RELEASE")

# Set internal compilation flags
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${COMPILATION_FLAGS}")

# Change compilation flags according to platform
IF("${OpenRDK_ARCH}" STREQUAL "geode")
	SET(COMPILE_RCONSOLE 0)
	SET(COMPILE_USARSIM_SECTIONS 0)
	SET(COMPILE_NAO_SECTIONS 1)
ELSEIF ("${OpenRDK_ARCH}" STREQUAL "atom")
	   SET(COMPILE_RCONSOLE 0)
	   SET(COMPILE_USARSIM_SECTIONS 0)
	   SET(COMPILE_NAO_SECTIONS 1)
ELSEIF ("${OpenRDK_ARCH}" STREQUAL "arm9")
		SET(COMPILE_RCONSOLE 0)
		SET(COMPILE_NAO_SECTIONS 0)	
ENDIF("${OpenRDK_ARCH}" STREQUAL "geode")

# Set UNIS-like path for targets
if(NOT "${CMAKE_SYSTEM}" MATCHES "CYGWIN")
	set(CMAKE_INSTALL_PREFIX "${OpenRDK_OUTPUT_TREE}" CACHE PATH "" FORCE)
	get_filename_component(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/lib" ABSOLUTE)
	get_filename_component(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/lib" ABSOLUTE)
	get_filename_component(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_INSTALL_PREFIX}/bin" ABSOLUTE)
	get_filename_component(OpenRDK_RESOURCES "${OpenRDK_ROOT}/docs" ABSOLUTE)
endif(NOT "${CMAKE_SYSTEM}" MATCHES "CYGWIN")
