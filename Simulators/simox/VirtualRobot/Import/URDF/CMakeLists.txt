PROJECT (SIMOX_URDFREADER)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/CMake )

include(SearchForTinyXML)
FIND_PACKAGE(urdfdom)
#FIND_PACKAGE(PkgConfig)
#PKG_CHECK_MODULES(URDFDOM REQUIRED urdfdom>=0.2.9)
#PKG_CHECK_MODULES(URDFDOM_HEADERS REQUIRED urdfdom_headers>=0.2.3)

if (tinyxml_library AND URDF_FOUND)

    include_directories(${tinyxml_include_dirs})
    link_directories(${tinyxml_library_dirs})
    add_definitions(${tinyxml_cflags})
    add_definitions("-DTIXML_USE_STL")

    FIND_PACKAGE(simox REQUIRED)
    INCLUDE_DIRECTORIES (
	    ${URDFDOM_INCLUDE_DIRS}
	    ${URDFDOM_PARSER_INCLUDE_DIRS}
	    ${URDFDOM_HEADERS_INCLUDE_DIRS}
    )
    setupSimoxExternalLibraries()


    SET (CMAKE_SKIP_BUILD_RPATH FALSE)
    SET (CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
    SET (CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
    SET (CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

    SET ( URDFIMPORT_SOURCES
	    SimoxURDFFactory.cpp
	    )
    SET ( URDFIMPORT_HEADERS
	    SimoxURDFFactory.h
	    )

    ADD_LIBRARY ( simox_urdfimport SHARED ${URDFIMPORT_SOURCES}  ${URDFIMPORT_HEADERS}  )

    add_definitions("-DDEMO_BASE_PATH=\"${PROJECT_SOURCE_DIR}\"")
    ADD_EXECUTABLE (loadURDFobot loadURDFobot.cpp)
    
    TARGET_LINK_LIBRARIES (simox_urdfimport VirtualRobot ${URDFDOM_LIBRARIES})
    TARGET_LINK_LIBRARIES(loadURDFobot simox_urdfimport VirtualRobot ${URDFDOM_LIBRARIES})

endif()
