# Setup doxygen configuration
#
#

find_package(Doxygen)

IF(DOXYGEN_FOUND)
    MESSAGE (STATUS "** Setting up doxygen documetation")
    # set all paths
    SET (CMAKE_DOXYGEN_INPUT_LIST "${CMAKE_CURRENT_SOURCE_DIR}/.."
                         "${CMAKE_CURRENT_SOURCE_DIR}/../GraspPlanning"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../GraspPlanning/GraspPlanner"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../GraspPlanning/GraspQuality"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../GraspPlanning/Visualization"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning/CSpace"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning/Planner"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning/PostProcessing"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning/Visualization"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/CollisionDetection"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/CollisionDetection/PQP"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/CollisionDetection/Dummy"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/EndEffector"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Grasping"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/IK"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Nodes"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Transformation"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Visualization"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Workspace"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/XML"
                         "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot")

    # consider visualization setup
    IF (Simox_USE_COIN_VISUALIZATION)
        SET (CMAKE_DOXYGEN_INPUT_LIST  ${CMAKE_DOXYGEN_INPUT_LIST}
                                        "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Visualization/CoinVisualization"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/../GraspPlanning/Visualization/CoinVisualization"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/../MotionPlanning/Visualization/CoinVisualization")
    ENDIF()

    IF (Simox_USE_OPENSCENEGRAPH_VISUALIZATION)
        SET (CMAKE_DOXYGEN_INPUT_LIST  ${CMAKE_DOXYGEN_INPUT_LIST}
                                        "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/Visualization/OSGVisualization")
    ENDIF()

    IF (Simox_USE_COLLADA)
        SET (CMAKE_DOXYGEN_INPUT_LIST  ${CMAKE_DOXYGEN_INPUT_LIST}
                                        "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/COLLADA")
    ENDIF()

    # set image path
    SET (CMAKE_DOXYGEN_IMAGE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/images")

    # set exclude files
    SET (CMAKE_EXCLUDE_LIST "${CMAKE_CURRENT_SOURCE_DIR}/../VirtualRobot/XML/rapidxml.hpp")

    # replace ; with space
    STRING(REGEX REPLACE ";" " " DOXYGEN_INPUT_DIRECTORIES "${CMAKE_DOXYGEN_INPUT_LIST}")

    CONFIGURE_FILE(Doxyfile.in ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile @ONLY)
    ADD_CUSTOM_TARGET(doc
        ${DOXYGEN_EXECUTABLE} ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Generating API documentation with Doxygen" VERBATIM)
    #DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/Doxyfile)
    #INSTALL( DIRECTORY “${CMAKE_CURRENT_BINARY_DIR}/html/” DESTINATION “/usr/share/doc/libnoise-${LIBNOISE_VERSION}” )
ENDIF()
