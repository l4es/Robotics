PROJECT ( Saba )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "***** CONFIGURING Simox project Saba *****")

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

########################### TESTING #####################################
MACRO(ADD_SABA_TEST TEST_NAME)
    INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
    INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
    INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/..")
    ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})
    ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
    TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba ${Simox_EXTERNAL_LIBRARIES} ${Boost_TEST_LIB})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
    SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "Saba Tests")
    ADD_TEST(NAME Saba_${TEST_NAME}
    	        COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_SABA_TEST)

#################################### FILES ##################################
SET(SOURCES
CSpace/CSpace.cpp
CSpace/CSpaceSampled.cpp
CSpace/CSpaceNode.cpp
CSpace/CSpacePath.cpp
CSpace/CSpaceTree.cpp
CSpace/Sampler.cpp
CSpace/ConfigurationConstraint.cpp
Planner/MotionPlanner.cpp
Planner/Rrt.cpp
Planner/BiRrt.cpp
Planner/GraspIkRrt.cpp
Planner/GraspRrt.cpp
Planner/PlanningThread.cpp
Visualization/RrtWorkspaceVisualization.cpp
PostProcessing/PathProcessor.cpp
PostProcessing/ShortcutProcessor.cpp
PostProcessing/PathProcessingThread.cpp
ApproachDiscretization.cpp
)

SET(INCLUDES
Saba.h
CSpace/CSpace.h
CSpace/CSpaceSampled.h
CSpace/CSpaceNode.h
CSpace/CSpacePath.h
CSpace/CSpaceTree.h
CSpace/Sampler.h
CSpace/ConfigurationConstraint.h
Planner/MotionPlanner.h
Planner/Rrt.h
Planner/BiRrt.h
Planner/GraspIkRrt.h
Planner/GraspRrt.h
Planner/PlanningThread.h
Visualization/RrtWorkspaceVisualization.h
PostProcessing/PathProcessor.h
PostProcessing/ShortcutProcessor.h
PostProcessing/PathProcessingThread.h
ApproachDiscretization.h
)
#../VirtualRobot/definesVR.h


if (Simox_USE_COIN_VISUALIZATION)
	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinRrtWorkspaceVisualization.h
	)
endif (Simox_USE_COIN_VISUALIZATION)

INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})

ADD_LIBRARY (Saba SHARED ${SOURCES} ${INCLUDES})
#MESSAGE(STATUS "VirtualRobot_ROBOT_LINK_LIBRARIES:" ${VirtualRobot_EXTERNAL_LIBRARIES})
TARGET_LINK_LIBRARIES (Saba VirtualRobot ${Simox_EXTERNAL_LIBRARIES})

# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})

# include examples
ADD_SUBDIRECTORY(examples/)

# include unit tests
ADD_SUBDIRECTORY(tests/)

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
MESSAGE (STATUS "***** Finished CONFIGURING Simox project Saba *****\n")
