# This is the main CMake configuration file, you should always do "cmake ." from here and not from subdirs

# Main project declaration
PROJECT(OpenRDK CXX C)

# Useful declaration for avoiding relative path
SET(OpenRDK_CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/build_tools/cmake")
include(${OpenRDK_CMAKE_MODULE_PATH}/prettymessage.cmake)


# Disable in-source builds
IF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")
	MESSAGE(FATAL_ERROR "In-source builds are not allowed.")
ENDIF("${PROJECT_SOURCE_DIR}" STREQUAL "${PROJECT_BINARY_DIR}")

# Verbose or not verbose
IF($ENV{VERBOSE})
	info("Verbose mode on")
ELSE($ENV{VERBOSE})
	info("Verbose mode off (type 'VERBOSE=1 cmake ${CMAKE_CURRENT_SOURCE_DIR}' to enable it)")
ENDIF($ENV{VERBOSE})

# Needed by CMake versione 2.6 or later (Run "cmake --help-policy CMP0003" for more information.)"
# specifically, we are using full-path when possible for linking libraries
# this should be mandatory for packages found using ROOT directive
CMAKE_MINIMUM_REQUIRED(VERSION 2.6 FATAL_ERROR)

# Version
SET(OpenRDK_V_MAJOR 2)
SET(OpenRDK_V_MINOR 1)
SET(OpenRDK_V_PATCH 9)

# Check for SVN and get current revision
INCLUDE(${OpenRDK_CMAKE_MODULE_PATH}/vcs-revision.cmake)
# Do not use revision information if I was unable to retrieve it
IF(NOT ${PROJECT_WC_REVISION} EQUAL 0)
	SET(OpenRDK_VERSION ${OpenRDK_V_MAJOR}.${OpenRDK_V_MINOR}.${OpenRDK_V_PATCH}.${PROJECT_WC_REVISION})
	SET(OpenRDK_V_REVISION ${PROJECT_WC_REVISION})
ELSE(NOT ${PROJECT_WC_REVISION} EQUAL 0)
	SET(OpenRDK_VERSION ${OpenRDK_V_MAJOR}.${OpenRDK_V_MINOR}.${OpenRDK_V_PATCH})
ENDIF(NOT ${PROJECT_WC_REVISION} EQUAL 0)

# Setting-up necessary paths
STRING(REGEX REPLACE "/src$" "" OpenRDK_ROOT ${OpenRDK_SOURCE_DIR})

# Create libext and binext
IF("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
	SET(OpenRDK_OUTPUT_TREE "${CMAKE_CURRENT_SOURCE_DIR}/..")
ELSE("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
	SET(OpenRDK_OUTPUT_TREE "${CMAKE_CURRENT_BINARY_DIR}")
ENDIF("${CMAKE_CURRENT_BINARY_DIR}" STREQUAL "${CMAKE_CURRENT_SOURCE_DIR}")
GET_FILENAME_COMPONENT(OpenRDK_OUTPUT_TREE ${OpenRDK_OUTPUT_TREE} ABSOLUTE)
IF(NOT EXISTS "${OpenRDK_OUTPUT_TREE}/libext")
	info("Creating directory libext in ${OpenRDK_OUTPUT_TREE}")
	FILE(MAKE_DIRECTORY "${OpenRDK_OUTPUT_TREE}/libext")
ENDIF(NOT EXISTS "${OpenRDK_OUTPUT_TREE}/libext")
IF(NOT EXISTS "${OpenRDK_OUTPUT_TREE}/binext")
	info("Creating directory binext in ${OpenRDK_OUTPUT_TREE}")
	FILE(MAKE_DIRECTORY "${OpenRDK_OUTPUT_TREE}/binext")
ENDIF(NOT EXISTS "${OpenRDK_OUTPUT_TREE}/binext")

# Optional flags and configurable stuff in ccmake
set(COMPILATION_FLAGS "-W -Wall" CACHE STRING "Custom (user defined) compilation flags" FORCE)
SET(COMPILE_RCONSOLE TRUE CACHE BOOL "Enable building of RConsole")
SET(COMPILE_USARSIM_SECTIONS TRUE CACHE BOOL "Enable building of interface modules to USARSim")
SET(COMPILE_NAO_SECTIONS FALSE CACHE BOOL "Enable building of modules and libraries for Aldebaran's Nao robot")
SET(ENABLE_OPENRDK_TESTS FALSE CACHE BOOL "Enable some unit tests on the OpenRDK code")
SET(ENABLE_SEARCHALGORITHMS_TESTS FALSE CACHE BOOL "Enable some unit tests on the OpenRDK code")

# Operating System specific behaviors and switches
INCLUDE(${OpenRDK_CMAKE_MODULE_PATH}/os.cmake)

# Setting overriding preferences (before searching for libraries)
include(${OpenRDK_CMAKE_MODULE_PATH}/override-config.cmake)

# Find and configure switches for external libraries and applications
INCLUDE(${OpenRDK_CMAKE_MODULE_PATH}/deps.cmake)

### file2c
RDK_ADD_EXECUTABLE(file2c file2c.cpp)
SET_TARGET_PROPERTIES(file2c PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR})
GET_TARGET_PROPERTY(FILE2C_EXE file2c LOCATION)

MACRO(CONVERT_FILE2C file out varname)
	MESSAGE( "${file} => ${out}")
	ADD_CUSTOM_COMMAND(
		OUTPUT ${out}
		COMMAND ${FILE2C_EXE} 
		ARGS ${file} ${out} ${varname}
		DEPENDS file2c ${file}
		COMMENT "Converting ${file} to C..")
ENDMACRO(CONVERT_FILE2C file out varname)

# This will be used by the modules and by rdk-config
SET(RDKCORE_NAME "rdkcore")
SET(RDKCORE_INCLUDE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/libraries)
SET(RDKCORE_LINK_DIRECTORIES ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
SET(RDKCORE_LIBRARIES ${RDKCORE_NAME})
# This are the same as before, but for makefile-style directive
# linking flags
SET(RDKCORE_LDFLAGS "")
FOREACH(dir ${RDKCORE_LINK_DIRECTORIES})
	SET(RDKCORE_LDFLAGS "${RDKCORE_LDFLAGS} -L${dir}")
ENDFOREACH(dir ${RDKCORE_LINK_DIRECTORIES})
# pre-processor flags (define)
SET(_tmp_RDKCORE_DEFINITIONS ${RDKCORE_DEFINITIONS})
SET(RDKCORE_DEFINITIONS "")
FOREACH(def ${_tmp_RDKCORE_DEFINITIONS})
	SET(RDKCORE_DEFINITIONS "${RDKCORE_DEFINITIONS} ${def}")
ENDFOREACH(def ${_tmp_RDKCORE_DEFINITIONS})
SET(RDKCORE_CPPFLAGS "${RDKCORE_DEFINITIONS}")
# compiling flags (include)
SET(RDKCORE_CXXFLAGS "")
FOREACH(dir ${RDKCORE_INCLUDE_DIR})
	SET(RDKCORE_CXXFLAGS "${RDKCORE_CXXFLAGS} -I${dir}")
ENDFOREACH(dir ${RDKCORE_INCLUDE_DIR})

# Creation of config.h
IF(CMAKE_CROSSCOMPILING)
	CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake" "${CMAKE_CURRENT_SOURCE_DIR}/libraries/rdkcore/config-cross-${CROSS_ARCH}.h")
ELSE(CMAKE_CROSSCOMPILING)
	CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/config.h.cmake" "${CMAKE_CURRENT_SOURCE_DIR}/libraries/rdkcore/config-generic.h")
ENDIF(CMAKE_CROSSCOMPILING)

# Creation of openrdk_manifest.txt
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/openrdk_manifest.txt.in" "${CMAKE_CURRENT_BINARY_DIR}/openrdk_manifest.txt")

# Creation of setenv
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/scripts/setenv.template" "${CMAKE_CURRENT_BINARY_DIR}/setenv" @ONLY)
execute_process(COMMAND "chmod" "+x" "setenv" WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}")
info("")
info("..:: OpenRDK environment ::..")
execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/rdkenv" "-m" "add" "${CMAKE_CURRENT_BINARY_DIR}" OUTPUT_VARIABLE msg OUTPUT_STRIP_TRAILING_WHITESPACE)
info("${msg}")
add_custom_target(install_rdkenv COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/scripts/rdkenv --color install-me COMMENT Installing rdkenv)
execute_process(COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/scripts/rdkenv" "check-version" OUTPUT_VARIABLE msg OUTPUT_STRIP_TRAILING_WHITESPACE)
info("${msg}")

# Creation of openrdk.cmake
SET(OpenRDK_FINDOPENRDK_CMAKE "${CMAKE_CURRENT_BINARY_DIR}/FindOpenRDK.cmake")
CONFIGURE_FILE("${CMAKE_CURRENT_SOURCE_DIR}/FindOpenRDK.cmake.in" "${OpenRDK_FINDOPENRDK_CMAKE}" @ONLY)

info("")
info("..:: Variables ::..")
info("OpenRDK_VERSION = ${OpenRDK_VERSION}")
info("OpenRDK_ARCH = ${OpenRDK_ARCH}")
info("OpenRDK_OUTPUT_TREE = ${OpenRDK_OUTPUT_TREE}")
info("CMAKE_LIBRARY_OUTPUT_DIRECTORY = ${CMAKE_LIBRARY_OUTPUT_DIRECTORY}")
info("CMAKE_ARCHIVE_OUTPUT_DIRECTORY = ${CMAKE_ARCHIVE_OUTPUT_DIRECTORY}")
info("CMAKE_RUNTIME_OUTPUT_DIRECTORY = ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")
verbose("LIBRARY_OUTPUT_DIRECTORY=${LIBRARY_OUTPUT_DIRECTORY}")
verbose("ARCHIVE_OUTPUT_DIRECTORY=${ARCHIVE_OUTPUT_DIRECTORY}")
verbose("RUNTIME_OUTPUT_DIRECTORY=${RUNTIME_OUTPUT_DIRECTORY}")
verbose("OpenRDK_FINDOPENRDK_CMAKE = ${OpenRDK_FINDOPENRDK_CMAKE}")
verbose("OpenRDK_ROOT = ${OpenRDK_ROOT}")
verbose("OpenRDK_SOURCE_DIR = ${OpenRDK_SOURCE_DIR}")
verbose("RDKCORE_LDFLAGS = ${RDKCORE_LDFLAGS}")
verbose("RDKCORE_CPPFLAGS = ${RDKCORE_CPPFLAGS}")
verbose("RDKCORE_CXXFLAGS = ${RDKCORE_CXXFLAGS}")
verbose("RDKCORE_DEFINITIONS = ${RDKCORE_DEFINITIONS}")
verbose("RDKCORE_INCLUDE_DIR = ${RDKCORE_INCLUDE_DIR}")
verbose("RDKCORE_LINK_DIRECTORIES = ${RDKCORE_LINK_DIRECTORIES}")
verbose("RDKCORE_LIBRARIES = ${RDKCORE_LIBRARIES}")

IF("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON")
	info("")
	info("..:: Cross Compiling Detected ::..")
	info("CMAKE_CXX_COMPILER = ${CMAKE_CXX_COMPILER}")
	info("CMAKE_C_COMPILER = ${CMAKE_C_COMPILER}")
	info("CMAKE_AR = ${CMAKE_AR}")
	info("CMAKE_RANLIB = ${CMAKE_RANLIB}")
	info("CMAKE_LINKER = ${CMAKE_LINKER}")
	info("CMAKE_STRIP = ${CMAKE_STRIP}")
	info("CMAKE_OBJCOPY = ${CMAKE_OBJCOPY}")
	info("CMAKE_OBJDUMP = ${CMAKE_OBJDUMP}")
	# use, i.e. don't skip the full RPATH for the build tree
	SET(CMAKE_SKIP_RPATH  true)
ENDIF("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON")

ENABLE_TESTING()

info("")

# Subdirectories
SUBDIRS(scripts)
SUBDIRS(libraries)
SUBDIRS(apps)

# Packages creation
SET(CPACK_PACKAGE_VERSION_MAJOR "${OpenRDK_V_MAJOR}")
SET(CPACK_PACKAGE_VERSION_MINOR "${OpenRDK_V_MINOR}")
SET(CPACK_PACKAGE_VERSION_PATCH "${OpenRDK_V_PATCH}")
SET(CPACK_GENERATOR "TGZ")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${OpenRDK_V_MAJOR}.${OpenRDK_V_MINOR}")
SET(CPACK_PACKAGE_FILE_NAME "openrdk-binary-${OpenRDK_VERSION}-${CMAKE_SYSTEM_NAME}${CROSS_ARCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "openrdk-source-${OpenRDK_VERSION}-${CMAKE_SYSTEM_NAME}${CROSS_ARCH}")
SET(CPACK_SOURCE_GENERATOR "TGZ")
SET(CPACK_INSTALL_DIRECTORIES "${OpenRDK_ROOT}/data;${OpenRDK_ROOT}/docs")
SET(CPACK_SOURCE_IGNORE_FILES
"/libraries/config.h$"
"/_CPack_Packages/"
"/CMakeCache.txt$"
"\\\\.dir/"
"\\\\.tar\\\\.gz$"
"\\\\.tar\\\\.Z$"
"\\\\.svn/"
"\\\\.cvsignore$"
"\\\\.swp$"
"~$"
"\\\\.#"
"/#"
"/CVS/"
"/cmake_install.cmake$"
"/CMakeOutput.log$"
"/CPackConfig.cmake$"
"/CPackSourceConfig.cmake$"
"/CTestTestfile.cmake$"
"/Makefile$"
"/Makefile2$"
"/CMakeFiles/"
"/\\\\.libs/"
"/\\\\.deps/"
"\\\\.o$"
"\\\\.a$"
"\\\\.lo$"
"\\\\.so$"
"\\\\.so\\\\.0$" 
"\\\\.so\\\\.0\\\\.0$" 
"\\\\.la$"
"Makefile\\\\.in$"
 )

INCLUDE(CPack)

SET(RDK_INSIDE_OpenRDK 1)
# needed because of rdk_* macro
INCLUDE(${OpenRDK_CMAKE_MODULE_PATH}/utils.cmake)
