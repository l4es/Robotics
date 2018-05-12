
IF (NOT Simox_CONFIGURED)

  # defines Simox_CONFIGURED variable which indicates that this config file has already been included
  SET(Simox_CONFIGURED TRUE)

  # Set up build type
  IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Debug CACHE STRING
        "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
        FORCE)
  ENDIF(NOT CMAKE_BUILD_TYPE)

  GET_FILENAME_COMPONENT (CurrentSimoxPath ${CMAKE_CURRENT_LIST_FILE} PATH)
  MESSAGE (STATUS "** Simox_DIR: ${CurrentSimoxPath}")
  SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CurrentSimoxPath}/CMakeModules)
  SET(Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
  SET(Simox_DIR ${CurrentSimoxPath})
	
	# Offer the user the choice of overriding the installation directories
  set(INSTALL_LIB_DIR lib CACHE PATH "Installation directory for libraries")
  set(INSTALL_BIN_DIR bin CACHE PATH "Installation directory for executables")
  set(INSTALL_INCLUDE_DIR include CACHE PATH
    "Installation directory for header files")
  if(WIN32 AND NOT CYGWIN)
    set(DEF_INSTALL_CMAKE_DIR CMake)
  else()
    set(DEF_INSTALL_CMAKE_DIR share/Simox/cmake)
  endif()
  set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
    "Installation directory for CMake files")
 
  # Make relative paths absolute (needed later on) 
  # -> disabled this since it produced lots of problems with generation of SimoxCOnfig.cmake
  #foreach(p LIB BIN INCLUDE CMAKE)
  #  set(var INSTALL_${p}_DIR)
  #  if(NOT IS_ABSOLUTE "${${var}}")
  #    set(${var} "${CMAKE_INSTALL_PREFIX}/${${var}}")
  #  endif()
  #endforeach()
 
  # set up include-directories
  include_directories(
    "${PROJECT_SOURCE_DIR}"   # to find headers
    "${PROJECT_BINARY_DIR}")  # to find config headers
	
	############################# VERSION #################################
	set(Simox_MAJOR_VERSION 2)
	set(Simox_MINOR_VERSION 3)
	set(Simox_PATCH_VERSION 9)
	set(Simox_VERSION
    ${Simox_MAJOR_VERSION}.${Simox_MINOR_VERSION}.${Simox_PATCH_VERSION})

	MESSAGE (STATUS "** Simox version: ${Simox_VERSION}")

	############################# SETUP PATHS #############################
	SET(Simox_BUILD_DIRECTORY ${CMAKE_BINARY_DIR})
	SET(BIN_DIR bin)
	SET(LIB_DIR lib)
	SET(DATA_DIR data)

	SET(Simox_LIB_DIR ${Simox_BUILD_DIRECTORY}/${LIB_DIR})
	SET(Simox_BIN_DIR ${Simox_BUILD_DIRECTORY}/${BIN_DIR})
	SET(Simox_DATA_DIR ${CurrentSimoxPath}/VirtualRobot/data)

	MESSAGE (STATUS "** SIMOX LIB DIR: ${Simox_LIB_DIR}")
	MESSAGE (STATUS "** SIMOX BIN DIR: ${Simox_BIN_DIR}")
	MESSAGE (STATUS "** SIMOX DATA DIR: ${Simox_DATA_DIR}")

	SET(Simox_INSTALL_LIB_DIR ${LIB_DIR})
	SET(Simox_INSTALL_BIN_DIR ${BIN_DIR})
	SET(Simox_INSTALL_DATA_DIR ${DATA_DIR})
	SET(Simox_INSTALL_HEADER_DIR include)
	MESSAGE (STATUS "** SIMOX INSTALL LIB DIR: ${Simox_INSTALL_LIB_DIR}")
	MESSAGE (STATUS "** SIMOX INSTALL BIN DIR: ${Simox_INSTALL_BIN_DIR}")
	MESSAGE (STATUS "** SIMOX INSTALL DATA DIR: ${Simox_INSTALL_DATA_DIR}")
	MESSAGE (STATUS "** SIMOX INSTALL HEADER DIR: ${Simox_INSTALL_HEADER_DIR}")

  ADD_DEFINITIONS("-DVirtualRobot_SRC_DATA_PATH=\"${Simox_DATA_DIR}\"")
  ADD_DEFINITIONS("-DSimox_DATA_PATH=\"${Simox_INSTALL_DATA_DIR}\"")
	
	########################### IDE settings ################################
	
	# use virtual folders for grouping projects in IDEs 
	set_property(GLOBAL PROPERTY USE_FOLDERS ON)


  ############################# Set OS specific options #############################
  IF(UNIX)
  	# We are on Linux
  	SET(Simox_TEST_DIR ${Simox_BIN_DIR}/tests)
  	IF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  	ADD_DEFINITIONS(-fPIC)
  ENDIF(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  
  IF(${CMAKE_BUILD_TYPE} STREQUAL "Debug")
  	MESSAGE(STATUS "Configuring Debug build")
  	ADD_DEFINITIONS(-D_DEBUG) # -Wall -W -Werror -pedantic)
  ELSE()
  	MESSAGE(STATUS "Configuring Release build")
  ENDIF()
  
  # use, i.e. don't skip the full RPATH for the build tree
  SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
  # when building, don't use the install RPATH already
  # (but later on when installing)
  SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
  SET(CMAKE_INSTALL_RPATH "${Simox_INSTALL_LIB_DIR}")
  
  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  
  # the RPATH to be used when installing, but only if it's not a system directory
  LIST(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${Simox_INSTALL_LIB_DIR}" isSystemDir)
  IF("${isSystemDir}" STREQUAL "-1")
     SET(CMAKE_INSTALL_RPATH "${Simox_INSTALL_LIB_DIR}")
  ENDIF("${isSystemDir}" STREQUAL "-1")
  
  
  ELSE(UNIX)
  	# We are on Windows
  	SET(Simox_TEST_DIR ${Simox_BIN_DIR})
  	ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
  
    # On MSVC we compile with /MP flag (use multiple threads)
    IF(MSVC)
    	ADD_DEFINITIONS(/MP)
    ENDIF(MSVC)
  ENDIF(UNIX)

    
  #######################################################################
  # Setup for testing
  #######################################################################
  ENABLE_TESTING()
  INCLUDE(CTest)
  
  MESSAGE(STATUS "** Test output directory: ${Simox_TEST_DIR}")
  
  ADD_DEFINITIONS(-DSimox_TEST_DIR=\"${Simox_TEST_DIR}/\")
  
  #######################################################################
  # Setup dependencies
  #######################################################################
 
  
  SET (Simox_EXTERNAL_INCLUDE_DIRS "")
  SET (Simox_EXTERNAL_LIBRARIES "")
  SET (Simox_EXTERNAL_LIBRARY_DIRS "")
  SET (Simox_EXTERNAL_LIBRARY_FLAGS "")
  SET (Simox_EXTERNAL_LIBRARY_CMAKE_INCLUDE "")
	
	############################# SETUP MODULES #############################
	MESSAGE (STATUS "** Module path: "  ${CMAKE_MODULE_PATH})
	
	### RBDL
	OPTION (Simox_USE_RBDL "Use RBDL" OFF)
	if (Simox_USE_RBDL)
		FIND_PACKAGE (RBDL)
		if (RBDL_FOUND)
			MESSAGE(STATUS "RBDL found at: ${RBDL_INCLUDE_DIR}")
			SET (Simox_EXTERNAL_INCLUDE_DIRS ${Simox_EXTERNAL_INCLUDE_DIRS} ${RBDL_INCLUDE_DIR})
			SET (Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${RBDL_LIBRARIES})
		else ()
			MESSAGE(STATUS "RBDL not found!")
		endif ()
	endif()
	
	#urdf
	OPTION (Simox_USE_URDF "Use URDF" OFF)

	#### Eigen
	FIND_PACKAGE (Eigen3 REQUIRED)
	if (Eigen3_FOUND)
	    SET (Simox_EXTERNAL_INCLUDE_DIRS ${Simox_EXTERNAL_INCLUDE_DIRS} ${Eigen3_INCLUDE_DIR})
	endif (Eigen3_FOUND)
	
	#### BOOST
        FIND_PACKAGE(Boost 1.46.0 COMPONENTS filesystem system program_options thread REQUIRED)
	if (Boost_FOUND)
	    MESSAGE (STATUS "Boost found at: ${Boost_INCLUDE_DIR}")
	    SET (Simox_EXTERNAL_INCLUDE_DIRS ${Simox_EXTERNAL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
        SET (Simox_EXTERNAL_LIBRARY_DIRS ${Simox_EXTERNAL_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
        SET (Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${Boost_LIBRARIES})
        FIND_PACKAGE(Boost 1.46.0 COMPONENTS unit_test_framework REQUIRED)
        SET (Boost_TEST_LIB "${Boost_LIBRARIES}")
	    # disable boost auto linking
        SET (Simox_EXTERNAL_LIBRARY_FLAGS "${Simox_EXTERNAL_LIBRARY_FLAGS} -DBOOST_ALL_NO_LIB -DBOOST_PROGRAM_OPTIONS_DYN_LINK -DBOOST_FILESYSTEM_DYN_LINK -DBOOST_SYSTEM_DYN_LINK -DBOOST_THREAD_DYN_LINK")
    else (Boost_FOUND)
	    MESSAGE ("!! Could not find Boost !!")
	endif (Boost_FOUND)
	
	#### QT 
	# QT_QMAKE_EXECUTABLE is the only relieable way of setting the qt4 path!
	# convert env var to cmake define	
	IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
	    MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
	    file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
	ENDIF()
	FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)

	#### VISUALIZATION Coin3D+Qt+SoQt / OSG+Qt
	##########################################
	SET (Simox_VISUALIZATION FALSE)
	SET (Simox_VISUALIZATION_LIBS "")
	SET (Simox_VISUALIZATION_INCLUDE_PATHS "")
	SET (Simox_VISUALIZATION_COMPILE_FLAGS "")
	
	OPTION(Simox_USE_COIN_VISUALIZATION "Use Coin3D for visualization" ON)
	OPTION(Simox_USE_OPENSCENEGRAPH_VISUALIZATION "Use OpenSceneGraph for visualization" OFF)
    #	OPTION(Simox_USE_COLLADA "Enable the loading of robots from collada files" OFF)
	
	if (Simox_USE_COIN_VISUALIZATION)
	  MESSAGE(STATUS "Searching Coin3D, Qt and SoQt...")
	    
	  ##### Coin3D
		FIND_PACKAGE(Coin3D REQUIRED)
		if (COIN3D_FOUND)
		    MESSAGE (STATUS "Found Coin3D: " ${COIN3D_INCLUDE_DIRS})
			##INCLUDE_DIRECTORIES(${COIN3D_INCLUDE_DIRS})
			##ADD_DEFINITIONS(-DCOIN_DLL)
		endif (COIN3D_FOUND)
		

		if ( QT_FOUND )
			MESSAGE (STATUS "Found Qt4: " ${QT_INCLUDE_DIR})
			MESSAGE (STATUS "QT_USE_FILE: " ${QT_USE_FILE})
			list(APPEND Simox_EXTERNAL_CMAKE_INCLUDE ${QT_USE_FILE})
			include(${QT_USE_FILE})
			SET (Simox_EXTERNAL_LIBRARY_CMAKE_INCLUDE ${Simox_EXTERNAL_LIBRARY_CMAKE_INCLUDE} ${QT_USE_FILE})
			SET (Simox_EXTERNAL_LIBRARY_DIRS ${Simox_EXTERNAL_LIBRARY_DIRS} ${QT_LIBRARY_DIR})
			SET (Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${QT_LIBRARIES})

			#MESSAGE(STATUS "QT_LIBRARIES: " ${QT_LIBRARIES})
	
			#### SoQt
			# This will set SoQt_INCLUDE_DIRS and SoQt_LIBRARIES
			FIND_PACKAGE(SoQt)
			if (SOQT_FOUND)
				MESSAGE (STATUS "Found SoQt:" ${SoQt_INCLUDE_DIRS})
				##ADD_DEFINITIONS(-DSOQT_DLL)
			else (SOQT_FOUND)
				MESSAGE (STATUS "Did not found SoQt. Disabling SoQt support.")
			endif (SOQT_FOUND)
		else ( QT_FOUND )
			MESSAGE (STATUS "Did not found Qt. Disabling Qt/SoQt support.")
		endif ( QT_FOUND )
		
		if (QT_FOUND AND SOQT_FOUND AND COIN3D_FOUND)
		  MESSAGE (STATUS "Enabling Coin3D/Qt/SoQt support")
			SET (Simox_VISUALIZATION TRUE)
      SET (Simox_VISUALIZATION_LIBS ${QT_LIBRARIES} ${COIN3D_LIBRARIES} ${SoQt_LIBRARIES} )
      SET (Simox_VISUALIZATION_INCLUDE_PATHS ${QT_INCLUDE_DIR} ${SoQt_INCLUDE_DIRS} ${COIN3D_INCLUDE_DIRS} )
      SET (Simox_VISUALIZATION_COMPILE_FLAGS " -DCOIN_DLL -DSOQT_DLL ")
		endif()
		
	elseif (Simox_USE_OPENSCENEGRAPH_VISUALIZATION)
	
	  MESSAGE(STATUS "Searching OSG and Qt...")
	    
	  FIND_PACKAGE(OpenSceneGraph REQUIRED osgViewer osgUtil osgDB osgGA)
		
		if (OPENSCENEGRAPH_FOUND)
		  MESSAGE (STATUS "Found OpenSceneGraph:" ${OPENSCENEGRAPH_INCLUDE_DIRS})
			##INCLUDE_DIRECTORIES(${OPENSCENEGRAPH_INCLUDE_DIRS})
		endif (OPENSCENEGRAPH_FOUND)
		
		if ( QT_FOUND )
			MESSAGE (STATUS "Found Qt4: " ${QT_INCLUDE_DIR})
			list(APPEND Simox_EXTERNAL_CMAKE_INCLUDE ${QT_USE_FILE})
			include(${QT_USE_FILE})
			#MESSAGE(STATUS "QT_LIBRARIES: " ${QT_LIBRARIES})
		else ( QT_FOUND )
			MESSAGE (STATUS "Did not found Qt. Disabling Qt/OSG support.")
		endif ( QT_FOUND )
		
		if (QT_FOUND AND OPENSCENEGRAPH_FOUND)
	    MESSAGE (STATUS "Enabling OSG/Qt support")
	    ### a little hack is needed here since osgQt is not supported in the FindOSG script
	    MESSAGE(STATUS "OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
	    LIST(GET OPENSCENEGRAPH_LIBRARIES 1 firstOsgLib)
	    MESSAGE(STATUS "firstOsgLib: ${firstOsgLib}")
	    GET_FILENAME_COMPONENT(osgLibPath ${firstOsgLib} PATH)
	    MESSAGE(STATUS "osgLibPath: ${osgLibPath}")
	    if (UNIX)
		    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/libosgQt.so)
	    else()
		    list(APPEND OPENSCENEGRAPH_LIBRARIES optimized)
		    list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQt.lib)
	    	list(APPEND OPENSCENEGRAPH_LIBRARIES debug)
	    	list(APPEND OPENSCENEGRAPH_LIBRARIES ${osgLibPath}/osgQtd.lib)
	    endif()
	    MESSAGE(STATUS "OPENSCENEGRAPH_LIBRARIES: ${OPENSCENEGRAPH_LIBRARIES}")
		  SET (Simox_VISUALIZATION TRUE)
    	SET (Simox_VISUALIZATION_LIBS ${QT_LIBRARIES} ${OPENSCENEGRAPH_LIBRARIES} )
    	SET (Simox_VISUALIZATION_INCLUDE_PATHS ${OPENSCENEGRAPH_INCLUDE_DIRS} )
    	SET (Simox_VISUALIZATION_COMPILE_FLAGS "")
		endif()
		
	else()
	    MESSAGE(STATUS "Visualization disabled")
	endif()
	
	if (Simox_USE_COLLADA)
	    MESSAGE(STATUS "Searching for Collada...")

    	FIND_PACKAGE(COLLADA_DOM REQUIRED 2.4)

    	IF(COLLADA_DOM_FOUND)
        	MESSAGE (STATUS "Found Collada")
        	MESSAGE (STATUS "* Collada COLLADA_DOM_ROOT_DIR : ${COLLADA_DOM_ROOT_DIR}")
	        MESSAGE (STATUS "* Collada Include DIRS: ${COLLADA_DOM_INCLUDE_DIRS}")
	        MESSAGE (STATUS "* Collada Libs: ${COLLADA_DOM_LIBRARIES}")
	        MESSAGE (STATUS "* Collada COLLADA_DOM_LIBRARY_DIRS: ${COLLADA_DOM_LIBRARY_DIRS}")

	        FIND_LIBRARY(COLLADA_LIBRARY ${COLLADA_DOM_LIBRARIES} ${COLLADA_DOM_LIBRARY_DIRS})
	        MESSAGE (STATUS "Collada Full Collada lib: ${COLLADA_LIBRARY}")

	        #include_directories(${COLLADA_DOM_INCLUDE_DIRS})

	        SET (Simox_EXTERNAL_INCLUDE_DIRS ${Simox_EXTERNAL_INCLUDE_DIRS} ${COLLADA_DOM_INCLUDE_DIRS})
			    SET (Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${COLLADA_LIBRARY})

			SET (Simox_EXTERNAL_LIBRARY_FLAGS "${Simox_EXTERNAL_LIBRARY_FLAGS} -DDOM_DYNAMIC ")
	    ENDIF()
	endif()
	
  SET (Simox_EXTERNAL_INCLUDE_DIRS ${Simox_EXTERNAL_INCLUDE_DIRS} ${Simox_VISUALIZATION_INCLUDE_PATHS})
  SET (Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${Simox_VISUALIZATION_LIBS})
  SET (Simox_EXTERNAL_LIBRARY_FLAGS "${Simox_EXTERNAL_LIBRARY_FLAGS} ${Simox_VISUALIZATION_COMPILE_FLAGS}")


  ## for historical reasons: set VirtualRobot flags to Simox flags
  SET (VirtualRobot_VISUALIZATION ${Simox_VISUALIZATION})
	SET (VirtualRobot_VISUALIZATION_LIBS ${Simox_VISUALIZATION_LIBS})
	SET (VirtualRobot_VISUALIZATION_INCLUDE_PATHS ${Simox_VISUALIZATION_INCLUDE_PATHS})
	SET (VirtualRobot_VISUALIZATION_COMPILE_FLAGS ${Simox_VISUALIZATION_COMPILE_FLAGS})

  
  INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
  ADD_DEFINITIONS( ${Simox_EXTERNAL_LIBRARY_FLAGS} )

ENDIF(NOT Simox_CONFIGURED)
