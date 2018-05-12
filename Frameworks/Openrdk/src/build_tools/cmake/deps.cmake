INCLUDE(${OpenRDK_CMAKE_MODULE_PATH}/utils.cmake)

# This should be used instead manual adding.
# It ensures main OpenRDK target (aka rdkcore) will be compiled with found libraries
LIST(APPEND _rdkcore_include_directories "")
LIST(APPEND _rdkcore_link_directories "")
LIST(APPEND _rdkcore_libraries "")
LIST(APPEND _rdkcore_definitions "")

info("")
info("..:: Required external libraries ::..")
### Finding NaoQiVersion
#RDK_FIND_LIB(NaoQiVersion optional TRUE)
### Library: GSL
RDK_FIND_LIB(GSL required)
### Library: libxml2
RDK_FIND_LIB(LIBXML2 required)
### Threads
RDK_FIND_LIB(Threads required)

info("")
info("..:: Optional external libraries ::..")

### PkgConfig
INCLUDE(FindPkgConfig)
IF(PKG_CONFIG_FOUND)
	info("pkg-config found")
	verbose("   PKG_CONFIG_EXECUTABLE = ${PKG_CONFIG_EXECUTABLE}")
ELSE(PKG_CONFIG_FOUND)
	info("pkg-config not found, but it is not required")
ENDIF(PKG_CONFIG_FOUND)

### Library: CPPunit
RDK_FIND_LIB(CPPUNIT optional)
### Library: GLUT
RDK_FIND_LIB(GLUT optional)
### Library: Aldebaran
RDK_FIND_LIB(ALDEBARAN optional TRUE)
### Library: ImageMagick's Magick++
RDK_FIND_LIB(MagickPP optional)
### Library: OpenCV
RDK_FIND_LIB(OpenCV optional TRUE)
### Library: libv4l2
RDK_FIND_LIB(libv4l2 optional)
### Library: GeoTIFF library
RDK_FIND_LIB(GEOTIFF optional)
### Library: video4linux 2
RDK_FIND_LIB(V4L2 optional)
### Library: JPEG library
RDK_FIND_LIB(JPEG optional)
### Library: OpenGL
RDK_FIND_LIB(OPENGL optional)

# Player/Stage
RDK_FIND_LIB(PLAYER optional TRUE)

### Library: QT
RDK_FIND_LIB(QT optional)
IF(QT_FOUND)
	verbose("   QT_QT_LIBRARY = ${QT_QT_LIBRARY}")
	verbose("   QT_MOC_EXECUTABLE = ${QT_MOC_EXECUTABLE}")
	verbose("   QT_UIC_EXECUTABLE = ${QT_UIC_EXECUTABLE}")
	SET(FINDOPENRDK_LIBRARIES_SECTION "${FINDOPENRDK_LIBRARIES_SECTION}
SET(QT_QT_LIBRARY "${QT_QT_LIBRARY}")
SET(QT_MOC_EXECUTABLE "${QT_MOC_EXECUTABLE}")
SET(QT_UIC_EXECUTABLE "${QT_UIC_EXECUTABLE}")
")
ELSE(QT_FOUND)
	IF(COMPILE_RCONSOLE OR COMPILE_CONFIGBUILDER)
		error("QT3 not found, it is required to build RConsole and ConfigBuilder:\n"
			"either set COMPILE_RCONSOLE and COMPILE_CONFIGBUILDER to 0 or install qt3-dev package")
	ELSE(COMPILE_RCONSOLE OR COMPILE_CONFIGBUILDER)
		info("QT3 not found, but it is not required (but you will not compile RConsoleQt and configbuilder)")
	ENDIF(COMPILE_RCONSOLE OR COMPILE_CONFIGBUILDER)
ENDIF(QT_FOUND)

# we are now ready to set the global variables
LIST(REMOVE_DUPLICATES _rdkcore_include_directories)
LIST(REMOVE_DUPLICATES _rdkcore_link_directories)
LIST(REMOVE_DUPLICATES _rdkcore_libraries)
LIST(REMOVE_DUPLICATES _rdkcore_definitions)

### Ruby: old stuff (not used anymore
#IF(COMPILE_RUBY)
#  IF(NOT RUBY_FOUND)
#    FIND_PACKAGE(Ruby REQUIRED)

#    # cmake non mette RUBY_FOUND a 1 esplicitamente: lo mettiamo noi
#    IF(EXISTS ${RUBY_INCLUDE_PATH})
#      SET(RUBY_FOUND 1)
#    ENDIF(EXISTS ${RUBY_INCLUDE_PATH})

#    IF(NOT RUBY_FOUND)
#      MESSAGE(STATUS "Ruby not found, but it is not required")
#    ENDIF(NOT RUBY_FOUND)
			
#    SET(RUBY_LIBRARY ruby)
#  ENDIF(NOT RUBY_FOUND)
		 
#  IF(RUBY_FOUND)
#    MESSAGE(STATUS "About Ruby:")
#    MESSAGE(STATUS "   RUBY_INCLUDE_PATH: ${RUBY_INCLUDE_PATH}")
#    MESSAGE(STATUS "   RUBY_EXECUTABLE: ${RUBY_EXECUTABLE}")
#    MESSAGE(STATUS "   RUBY_LIBRARY: ${RUBY_LIBRARY}")
			
#    INCLUDE_DIRECTORIES(${RUBY_INCLUDE_PATH})
#  ENDIF(RUBY_FOUND)
#ENDIF(COMPILE_RUBY)
