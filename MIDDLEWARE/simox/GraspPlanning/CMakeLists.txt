PROJECT ( GraspStudio )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "***** CONFIGURING Simox project GraspStudio *****")

MACRO(ADD_GRASPSTUDIO_TEST TEST_NAME)
    	ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    	TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba GraspStudio ${Simox_EXTERNAL_LIBRARIES})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "GraspStudio Tests")
    ADD_TEST(NAME Saba_${TEST_NAME}
    	        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_GRASPSTUDIO_TEST)


########### QHULL #################
OPTION(BUILD_qhull "if on builds qhull external library" OFF)
IF(BUILD_qhull)
	ADD_SUBDIRECTORY(ExternalDependencies/qhull-2003.1)
    set(QHULL_LIBRARIES simox-qhull)
else()
    find_package(QHULL REQUIRED)
    include_directories(${QHULL_INCLUDE_DIRS})
endif()
########### END QHULL #################





########### POWERCRUST #################
OPTION(BUILD_powercrust "if on builds powercrust external library" ON)
IF(BUILD_powercrust)
	ADD_SUBDIRECTORY(ExternalDependencies/powercrust)
ENDIF(BUILD_powercrust)
########### END POWERCRUST #################


SET(SOURCES
ConvexHullGenerator.cpp
ContactConeGenerator.cpp
ApproachMovementGenerator.cpp
ApproachMovementSurfaceNormal.cpp
GraspPlanner/GraspPlanner.cpp
GraspPlanner/GenericGraspPlanner.cpp
GraspQuality/GraspQualityMeasure.cpp
GraspQuality/GraspQualityMeasureWrenchSpace.cpp
Visualization/ConvexHullVisualization.cpp
)

SET(INCLUDES
GraspStudio.h
ConvexHullGenerator.h
ContactConeGenerator.h
ApproachMovementGenerator.h
ApproachMovementSurfaceNormal.h
GraspPlanner/GraspPlanner.h
GraspPlanner/GenericGraspPlanner.h
GraspQuality/GraspQualityMeasure.h
GraspQuality/GraspQualityMeasureWrenchSpace.h
Visualization/ConvexHullVisualization.h
)
#${GRASPSTUDIO_SimoxDir}/VirtualRobot/definesVR.h


IF(BUILD_powercrust)
	# Adding MATGraspPlanner
	SET(SOURCES
		${SOURCES}
		GraspPlanner/MATPlanner/CandidateGrasp.cpp
		GraspPlanner/MATPlanner/CandidateGraspGenerator.cpp
		GraspPlanner/MATPlanner/Converter.cpp
		GraspPlanner/MATPlanner/GraspPlannerConfiguration.cpp
		GraspPlanner/MATPlanner/GridOfMedialSpheres.cpp
		GraspPlanner/MATPlanner/GridParameters.cpp
		GraspPlanner/MATPlanner/LocalNeighborhood.cpp
		GraspPlanner/MATPlanner/MatGraspPlanner.cpp
		GraspPlanner/MATPlanner/MedialSphere.cpp
		GraspPlanner/MATPlanner/MeshConverter.cpp
		GraspPlanner/MATPlanner/SphereHelpers.cpp
		GraspPlanner/MATPlanner/StrOutHelpers.cpp
		GraspPlanner/MATPlanner/TestCases.cpp
		GraspPlanner/MATPlanner/CandidateGraspTester.cpp
	)

	SET(INCLUDES
		${INCLUDES}
		GraspPlanner/MATPlanner/CandidateGrasp.h
		GraspPlanner/MATPlanner/CandidateGraspGenerator.h
		GraspPlanner/MATPlanner/Converter.h
		GraspPlanner/MATPlanner/GraspPlannerConfiguration.h
		GraspPlanner/MATPlanner/GridOfMedialSpheres.h
		GraspPlanner/MATPlanner/GridParameters.h
		GraspPlanner/MATPlanner/LocalNeighborhood.h
		GraspPlanner/MATPlanner/MatGraspPlanner.h
		GraspPlanner/MATPlanner/MedialSphere.h
		GraspPlanner/MATPlanner/MeshConverter.h
		GraspPlanner/MATPlanner/SphereHelpers.h
		GraspPlanner/MATPlanner/StrOutHelpers.h
		GraspPlanner/MATPlanner/TestCases.h
		GraspPlanner/MATPlanner/CandidateGraspTester.h
	)

	if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
		SET(SOURCES
			${SOURCES}
			GraspPlanner/MATPlanner/CoinVisualization/DrawHelpers.cpp
		)
		SET(INCLUDES
			${INCLUDES}
			GraspPlanner/MATPlanner/CoinVisualization/DrawHelpers.h
		)
	endif()
ENDIF()

if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinConvexHullVisualization.h
	)
endif ()


INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})


ADD_LIBRARY (GraspStudio SHARED ${SOURCES} ${INCLUDES})

TARGET_LINK_LIBRARIES (GraspStudio Saba VirtualRobot ${Simox_EXTERNAL_LIBRARIES} ${QHULL_LIBRARIES} powercrust)

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


# include examples
ADD_SUBDIRECTORY(examples/)

# include unit tests
#ADD_SUBDIRECTORY(tests/)


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################


install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
  COMPONENT dev)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
	COMPONENT Headers
        FILES_MATCHING PATTERN "*.h"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)
	

MESSAGE (STATUS "***** Finished CONFIGURING Simox project GraspStudio *****\n")

