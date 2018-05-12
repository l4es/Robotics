# - Find GeoTIFF library
# Find the native GeoTIFF includes and library
# This module defines
#  GeoTIFF_INCLUDE_DIR, where to find tiff.h, etc.
#  GeoTIFF_LIBRARIES, libraries to link against to use GeoTIFF.
#  GeoTIFF_FOUND, If false, do not try to use GeoTIFF.
#  also defined, but not for general use are
#  GeoTIFF_LIBRARY, where to find the GeoTIFF library.

IF (NOT ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON"))
	FIND_PATH(GEOTIFF_INCLUDE_DIR geotiff.h
		/usr/include
		/usr/include/geotiff
		/usr/local/include
		/opt/local/include
		)

	SET(GEOTIFF_NAMES ${GEOTIFF_NAMES} geotiff)
	FIND_LIBRARY(GEOTIFF_LIBRARY
		NAMES ${GEOTIFF_NAMES}
		PATHS /usr/lib /usr/local/lib /opt/local/lib
		)

	IF(GEOTIFF_INCLUDE_DIR)
		IF(GEOTIFF_LIBRARY)
			SET( GEOTIFF_FOUND "YES" )
			SET( GEOTIFF_LIBRARIES ${GEOTIFF_LIBRARY} )
		ENDIF(GEOTIFF_LIBRARY)
	ENDIF(GEOTIFF_INCLUDE_DIR)

ELSE(NOT ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON"))
	MESSAGE(STATUS "GeoTIFF support is not available for cross-compiled OpenRDK")
ENDIF (NOT ("${CMAKE_CROSSCOMPILING}" STREQUAL "TRUE" OR "${CMAKE_CROSSCOMPILING}" STREQUAL "ON"))

# needed to avoid putting things in ccmake gui
set(GEOTIFF_INCLUDE_DIR "${GEOTIFF_INCLUDE_DIR}" CACHE INTERNAL "" FORCE)
set(GEOTIFF_LIBRARY "${GEOTIFF_LIBRARY}" CACHE INTERNAL "" FORCE)
