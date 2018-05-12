## 
## Try to find gnu scientific library GSL  
## (see http://www.gnu.org/software/gsl/)
## Once run this will define: 
## 
## GSL_FOUND       = system has GSL lib
##
## GSL_LIBRARIES   = full path to the libraries
##    on Unix/Linux with additional linker flags from "gsl-config --libs"
## 
## CMAKE_GSL_CXX_FLAGS  = Unix compiler flags for GSL, essentially "`gsl-config --cxxflags`"
##
## GSL_INCLUDE_DIR      = where to find headers 
##
## GSL_LINK_DIRECTORIES = link directories, useful for rpath on Unix
## GSL_EXE_LINKER_FLAGS = rpath on Unix
##
## Felix Woelk 07/2004
## www.mip.informatik.uni-kiel.de
## --------------------------------


SET(GSL_CONFIG_PREFER_PATH "$ENV{GSL_HOME}/bin" CACHE STRING "preferred path to OpenSG (osg-config)")

IF ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON")
	IF ("${OpenRDK_ARCH}" STREQUAL "geode" )
		SET(GSL_LIBRARIES m;gsl;gslcblas)
		SET(GSL_INCLUDE_DIR "${OE_CROSS_ADDONS_DIR}/include")
		SET(GSL_LINK_DIRECTORIES "${OE_CROSS_ADDONS_DIR}/lib")
	ELSEIF ("${OpenRDK_ARCH}" STREQUAL "atom" )
		   SET(GSL_LIBRARIES m;gsl;gslcblas)
		   SET(GSL_INCLUDE_DIR "${OE_CROSS_ADDONS_DIR}/include")
		   SET(GSL_LINK_DIRECTORIES "${OE_CROSS_ADDONS_DIR}/lib")
	ELSEIF ("${OpenRDK_ARCH}" STREQUAL "arm9" )
			SET(GSL_LIBRARIES m;gsl;gslcblas)
			SET(GSL_INCLUDE_DIR "${ARM9_CROSS_ADDONS_DIR}/include")
			SET(GSL_LINK_DIRECTORIES "${ARM9_CROSS_ADDONS_DIR}/lib")
	ELSE ("${OpenRDK_ARCH}" STREQUAL "geode" )
		MESSAGE(STATUS "GSL support is not available for unknown architecture [${OpenRDK_ARCH}]")
	ENDIF ("${OpenRDK_ARCH}" STREQUAL "geode" )
ELSE ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON")
	FIND_PROGRAM(GSL_CONFIG gsl-config
		#      NO_DEFAULT_PATH
		${GSL_CONFIG_PREFER_PATH}
		${CMAKE_FIND_ROOT_PATH}/bin
		${CMAKE_FIND_ROOT_PATH}/usr/bin
		/usr/bin/
		)
	#    MESSAGE("DBG GSL_CONFIG ${GSL_CONFIG}")

	IF (GSL_CONFIG) 
		# set CXXFLAGS to be fed into CXX_FLAGS by the user:
		#SET(GSL_CXX_FLAGS "`${GSL_CONFIG} --cflags`")
		EXEC_PROGRAM(${GSL_CONFIG} ARGS --cflags OUTPUT_VARIABLE GSL_CXX_FLAGS)

		# set INCLUDE_DIRS to prefix+include
		EXEC_PROGRAM(${GSL_CONFIG}
			ARGS --prefix
			OUTPUT_VARIABLE GSL_PREFIX)
		SET(GSL_INCLUDE_DIR ${GSL_PREFIX}/include CACHE STRING INTERNAL)

		# set link libraries and link flags
		#SET(GSL_LIBRARIES "`${GSL_CONFIG} --libs`")

		EXEC_PROGRAM(${GSL_CONFIG} ARGS --libs OUTPUT_VARIABLE GSL_LIBRARIES)


		## extract link dirs for rpath  
		EXEC_PROGRAM(${GSL_CONFIG}
			ARGS --libs
			OUTPUT_VARIABLE GSL_CONFIG_LIBS )

		## split off the link dirs (for rpath)
		## use regular expression to match wildcard equivalent "-L*<endchar>"
		## with <endchar> is a space or a semicolon
		STRING(REGEX MATCHALL "[-][L]([^ ;])+" 
			GSL_LINK_DIRECTORIES_WITH_PREFIX 
			"${GSL_CONFIG_LIBS}" )
		#      MESSAGE("DBG  GSL_LINK_DIRECTORIES_WITH_PREFIX=${GSL_LINK_DIRECTORIES_WITH_PREFIX}")

		## remove prefix -L because we need the pure directory for LINK_DIRECTORIES

		IF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
			STRING(REGEX REPLACE "[-][L]" "" GSL_LINK_DIRECTORIES ${GSL_LINK_DIRECTORIES_WITH_PREFIX} )
		ENDIF (GSL_LINK_DIRECTORIES_WITH_PREFIX)
		SET(GSL_EXE_LINKER_FLAGS "-Wl,-rpath,${GSL_LINK_DIRECTORIES}" CACHE STRING INTERNAL)
		#      MESSAGE("DBG  GSL_LINK_DIRECTORIES=${GSL_LINK_DIRECTORIES}")
		#      MESSAGE("DBG  GSL_EXE_LINKER_FLAGS=${GSL_EXE_LINKER_FLAGS}")

		#ADD_DEFINITIONS("-DHAVE_GSL")
		#SET(GSL_DEFINITIONS "-DHAVE_GSL")
		MARK_AS_ADVANCED(
			GSL_CXX_FLAGS
			GSL_INCLUDE_DIR
			GSL_LIBRARIES
			GSL_LINK_DIRECTORIES
			GSL_DEFINITIONS
			)
		#MESSAGE(STATUS "Using GSL from ${GSL_PREFIX}")

		# make the GSL_LIBRARIES cmake compliant
		STRING(REGEX REPLACE " " ";" _GSL_LIBRARIES ${GSL_LIBRARIES})
		SET(GSL_LIBRARIES "")
		FOREACH(item ${_GSL_LIBRARIES})
			IF (NOT "${item}" MATCHES "-L")
				STRING(REPLACE "-l" "" _item ${item})
				LIST(APPEND GSL_LIBRARIES ${_item})
			ENDIF (NOT "${item}" MATCHES "-L")
		ENDFOREACH(item ${_GSL_LIBRARIES})

	ELSE(GSL_CONFIG)
		MESSAGE("FindGSL.cmake: gsl-config not found. Please set it manually. GSL_CONFIG=${GSL_CONFIG}")
	ENDIF(GSL_CONFIG)

	#  ENDIF(UNIX)
	#ENDIF(WIN32)

ENDIF ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON")

IF(GSL_LIBRARIES)
	IF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
		SET(GSL_FOUND 1)
	ENDIF(GSL_INCLUDE_DIR OR GSL_CXX_FLAGS)
ENDIF(GSL_LIBRARIES)

# needed to avoid putting things in ccmake gui
set(GSL_LIBRARIES "${GSL_LIBRARIES}" CACHE INTERNAL "" FORCE)
set(GSL_CONFIG_PREFER_PATH "${GSL_CONFIG_PREFER_PATH}" CACHE INTERNAL "" FORCE)
set(GSL_CONFIG "${GSL_CONFIG}" CACHE INTERNAL "" FORCE)
set(GSL_EXE_LINKER_FLAGS "${GSL_EXE_LINKER_FLAGS}" CACHE INTERNAL "" FORCE)
