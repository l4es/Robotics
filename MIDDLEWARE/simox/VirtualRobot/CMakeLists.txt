PROJECT ( VirtualRobot )

MESSAGE (STATUS "\n***** CONFIGURING Simox project VirtualRobot *****")

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

if(NOT ("${CMAKE_VERSION}" VERSION_LESS 2.8.12))
	cmake_policy(SET CMP0022 OLD) # avoid INTERFACE_LINK_LIBRARIES warnings
ENDIF()

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

if (Simox_USE_RBDL)
	FIND_PACKAGE (RBDL)
endif()

########################### TESTING #####################################
MACRO(ADD_VR_TEST TEST_NAME)  
  INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR})
  INCLUDE_DIRECTORIES("${PROJECT_SOURCE_DIR}/..")
  ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})
  ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK) 
  ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
        TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot ${Simox_EXTERNAL_LIBRARIES} ${Boost_TEST_LIB})
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
	SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "VirtualRobot Tests")
	ADD_TEST(NAME VirtualRobot_${TEST_NAME}
	         COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO()

#################################### FILES ##################################
SET(SOURCES
CollisionDetection/CollisionChecker.cpp
CollisionDetection/CollisionModel.cpp
CollisionDetection/CDManager.cpp
EndEffector/EndEffector.cpp
EndEffector/EndEffectorActor.cpp
Nodes/RobotNode.cpp
Nodes/RobotNodePrismatic.cpp
Nodes/RobotNodePrismaticFactory.cpp
Nodes/RobotNodeRevolute.cpp
Nodes/RobotNodeRevoluteFactory.cpp
Nodes/RobotNodeFixed.cpp
Nodes/RobotNodeFixedFactory.cpp
Nodes/RobotNodeActuator.cpp
Nodes/Sensor.cpp
Nodes/PositionSensor.cpp
Nodes/ForceTorqueSensor.cpp
Nodes/ContactSensor.cpp
Nodes/CameraSensor.cpp
Nodes/PositionSensorFactory.cpp
Nodes/ForceTorqueSensorFactory.cpp
Nodes/ContactSensorFactory.cpp
Nodes/CameraSensorFactory.cpp
Visualization/Visualization.cpp
Visualization/VisualizationNode.cpp
Visualization/ColorMap.cpp
Visualization/TriMeshModel.cpp
XML/BaseIO.cpp
XML/RobotIO.cpp
XML/SceneIO.cpp
XML/ObjectIO.cpp
XML/FileIO.cpp
IK/IKSolver.cpp
IK/DifferentialIK.cpp
IK/GenericIKSolver.cpp
IK/CoMIK.cpp
IK/JacobiProvider.cpp
IK/PoseQualityMeasurement.cpp
IK/PoseQualityManipulability.cpp
IK/PoseQualityExtendedManipulability.cpp
IK/HierarchicalIK.cpp
IK/HierarchicalIKSolver.cpp
IK/JointLimitAvoidanceJacobi.cpp
IK/GazeIK.cpp
IK/StackedIK.cpp
IK/SupportPolygon.cpp
IK/FeetPosture.cpp
IK/ConstrainedIK.cpp
IK/ConstrainedHierarchicalIK.cpp
IK/ConstrainedStackedIK.cpp
IK/Constraint.cpp
IK/constraints/BalanceConstraint.cpp
IK/constraints/PoseConstraint.cpp
IK/constraints/TSRConstraint.cpp
Workspace/WorkspaceDataArray.cpp
Workspace/WorkspaceRepresentation.cpp
Workspace/Reachability.cpp
Workspace/Manipulability.cpp
Workspace/WorkspaceGrid.cpp
Grasping/Grasp.cpp
Grasping/GraspSet.cpp
Grasping/BasicGraspQualityMeasure.cpp
Grasping/GraspEditor/GraspEditorWindow.cpp
MathTools.cpp
Robot.cpp
RobotConfig.cpp
RobotNodeSet.cpp
Trajectory.cpp
KinematicChain.cpp
RobotFactory.cpp
SceneObject.cpp
SceneObjectSet.cpp
Scene.cpp
Obstacle.cpp
Primitive.cpp
VirtualRobotException.cpp
ManipulationObject.cpp
BoundingBox.cpp
RuntimeEnvironment.cpp
Compression/CompressionRLE.cpp
Compression/CompressionBZip2.cpp
LinkedCoordinate.cpp
SphereApproximator.cpp
Import/SimoxXMLFactory.cpp
Import/RobotImporterFactory.cpp
Import/MeshImport/STLReader.cpp

)

if (Simox_USE_RBDL AND RBDL_FOUND)
    SET(SOURCES
		${SOURCES}
		Dynamics/dynamics.cpp
    )

	SET(INCLUDES
		${INCLUDES}
		Dynamics/dynamics.h
	)

	ADD_SUBDIRECTORY(Dynamics/tests)

endif ()


SET(INCLUDES
CollisionDetection/CollisionChecker.h
CollisionDetection/CollisionModel.h
CollisionDetection/CDManager.h
CollisionDetection/CollisionModelImplementation.h
CollisionDetection/CollisionCheckerImplementation.h
EndEffector/EndEffector.h
EndEffector/EndEffectorActor.h
Nodes/RobotNode.h
Nodes/RobotNodeFactory.h
Nodes/RobotNodePrismatic.h
Nodes/RobotNodePrismaticFactory.h
Nodes/RobotNodeRevolute.h
Nodes/RobotNodeRevoluteFactory.h
Nodes/RobotNodeFixed.h
Nodes/RobotNodeFixedFactory.h
Nodes/RobotNodeActuator.h
Nodes/ConditionedLock.h
Nodes/Sensor.h
Nodes/PositionSensor.h
Nodes/ForceTorqueSensor.h
Nodes/CameraSensor.h
Nodes/SensorFactory.h
Nodes/PositionSensorFactory.h
Nodes/ForceTorqueSensorFactory.h
Nodes/CameraSensorFactory.h
Transformation/DHParameter.h
Visualization/VisualizationFactory.h
Visualization/Visualization.h
Visualization/VisualizationNode.h
Visualization/ColorMap.h
Visualization/TriMeshModel.h
XML/BaseIO.h
XML/RobotIO.h
XML/SceneIO.h
XML/ObjectIO.h
XML/FileIO.h
IK/IKSolver.h
IK/DifferentialIK.h
IK/GenericIKSolver.h
IK/CoMIK.h
IK/JacobiProvider.h
IK/PoseQualityMeasurement.h
IK/PoseQualityManipulability.h
IK/PoseQualityExtendedManipulability.h
IK/HierarchicalIK.h
IK/HierarchicalIKSolver.h
IK/JointLimitAvoidanceJacobi.h
IK/GazeIK.h
IK/StackedIK.h
IK/SupportPolygon.h
IK/FeetPosture.h
IK/ConstrainedIK.h
IK/ConstrainedHierarchicalIK.h
IK/ConstrainedStackedIK.h
IK/Constraint.h
IK/constraints/BalanceConstraint.h
IK/constraints/PoseConstraint.h
IK/constraints/TSRConstraint.h
Workspace/WorkspaceData.h
Workspace/WorkspaceDataArray.h
Workspace/WorkspaceRepresentation.h
Workspace/Reachability.h
Workspace/Manipulability.h
Workspace/VoxelTree6D.hpp
Workspace/VoxelTree6DElement.hpp
Workspace/VoxelTreeND.hpp
Workspace/VoxelTreeNDElement.hpp
Workspace/WorkspaceGrid.h
Grasping/Grasp.h
Grasping/GraspSet.h
Grasping/BasicGraspQualityMeasure.h
Grasping/GraspEditor/GraspEditorWindow.h
AbstractFactoryMethod.h
VirtualRobot.h
MathTools.h
Robot.h
RobotConfig.h
RobotNodeSet.h
Trajectory.h
KinematicChain.h
RobotFactory.h
SceneObject.h
SceneObjectSet.h
Scene.h
Obstacle.h
Primitive.h
VirtualRobotException.h
VirtualRobotImportExport.h
VirtualRobotTest.h
ManipulationObject.h
BoundingBox.h
RuntimeEnvironment.h
Compression/CompressionRLE.h
Compression/CompressionBZip2.h
SphereApproximator.h
Import/SimoxXMLFactory.h
Import/RobotImporterFactory.h
Import/MeshImport/STLReader.h
)









ADD_SUBDIRECTORY(Visualization/tests)
ADD_SUBDIRECTORY(Workspace/tests)
ADD_SUBDIRECTORY(Compression/tests)

if (Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)

	SET(SOURCES
	${SOURCES}
	Visualization/CoinVisualization/CoinVisualizationFactory.cpp
	Visualization/CoinVisualization/CoinVisualizationNode.cpp
    Visualization/CoinVisualization/CoinVisualization.cpp
    Import/COLLADA-light/ColladaSimox.cpp
    Import/COLLADA-light/collada.cpp
    Import/COLLADA-light/inventor.cpp
    Import/COLLADA-light/pugixml/pugixml.cpp
    Import/SimoxCOLLADAFactory.cpp
    )

	SET(INCLUDES
	${INCLUDES}
	Visualization/CoinVisualization/CoinVisualizationFactory.h
	Visualization/CoinVisualization/CoinVisualizationNode.h
    Visualization/CoinVisualization/CoinVisualization.h
    Import/COLLADA-light/ColladaSimox.h
    Import/COLLADA-light/collada.h
    Import/COLLADA-light/pugixml/pugixml.hpp
    Import/SimoxCOLLADAFactory.cpp
    Import/COLLADA-light/inventor.h
    Import/SimoxCOLLADAFactory.h
	)
	ADD_SUBDIRECTORY(Visualization/CoinVisualization/tests)

elseif (Simox_VISUALIZATION AND Simox_USE_OPENSCENEGRAPH_VISUALIZATION)

	SET(SOURCES
	${SOURCES}
	Visualization/OSGVisualization/OSGVisualizationFactory.cpp
	Visualization/OSGVisualization/OSGVisualizationNode.cpp
    Visualization/OSGVisualization/OSGVisualization.cpp
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.cpp
	)

	SET(INCLUDES
	${INCLUDES}
	Visualization/OSGVisualization/OSGVisualizationFactory.h
	Visualization/OSGVisualization/OSGVisualizationNode.h
    Visualization/OSGVisualization/OSGVisualization.h
    Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
	)

	set(GUI_MOC_HDRS
        Visualization/OSGVisualization/osgQtWidget/osgViewerWidget.h
    )
    qt4_wrap_cpp(SOURCES ${GUI_MOC_HDRS})

	ADD_SUBDIRECTORY(Visualization/OSGVisualization/tests)
	#ADD_SUBDIRECTORY(Visualization/OSGVisualization/osgQtViewer)

endif ()

#if (Simox_USE_COLLADA)
#
#    SET(SOURCES
#    ${SOURCES}
#    Import/SimoxCOLLADAFactory.cpp
#    COLLADA/ColladaIO.cpp
#    COLLADA/ColladaParser.cpp
#    )
#
#    SET(INCLUDES
#    ${INCLUDES}
#    Import/SimoxCOLLADAFactory.h
#    COLLADA/ColladaIO.h
#    COLLADA/ColladaIterators.h
#    COLLADA/ColladaParser.h
#    )
#endif ()


#check for URDF
if (Simox_USE_URDF)
	include(SearchForTinyXML)
	FIND_PACKAGE(urdfdom)
	#FIND_PACKAGE(PkgConfig)
	#PKG_CHECK_MODULES(URDFDOM REQUIRED urdfdom>=0.2.9)
	#PKG_CHECK_MODULES(URDFDOM_HEADERS REQUIRED urdfdom_headers>=0.2.3)

	if (tinyxml_library AND URDF_FOUND)
	   MESSAGE(STATUS " ** Including URDF importer")

		include_directories(${tinyxml_include_dirs})
		link_directories(${tinyxml_library_dirs})
		add_definitions(${tinyxml_cflags})
		add_definitions(" -DTIXML_USE_STL ")

		INCLUDE_DIRECTORIES (
			${URDFDOM_INCLUDE_DIRS}
			${URDFDOM_PARSER_INCLUDE_DIRS}
			${URDFDOM_HEADERS_INCLUDE_DIRS}
		)

		SET(SOURCES
		  ${SOURCES}
			Import/URDF/SimoxURDFFactory.cpp
			)
		SET(INCLUDES
		  ${INCLUDES}
			Import/URDF/SimoxURDFFactory.h
			)

		SET(Simox_EXTERNAL_LIBRARIES ${Simox_EXTERNAL_LIBRARIES} ${URDFDOM_LIBRARIES})
	else()
		MESSAGE(STATUS " *** urdfDom or TinyXML not found: skipping URDF importer...")
	endif()
endif()

# include collision detection
INCLUDE (CollisionDetection/CMakeLists.txt)

# include unit tests
ADD_SUBDIRECTORY(tests/)
ADD_SUBDIRECTORY(Transformation/tests/)

# include examples
ADD_SUBDIRECTORY(examples/)


# -----------------------------------------------------------------------------------
# Adding the UI files to the library

set(UI
    Grasping/GraspEditor/GraspEditor.ui
)
QT4_WRAP_UI(UI_HEADERS ${UI})
set(INCLUDES ${INCLUDES} ${UI_HEADERS})
QT4_WRAP_CPP(MOC_SRC
    Grasping/GraspEditor/GraspEditorWindow.h
)

SET(SOURCES ${SOURCES} ${MOC_SRC})

# Finished adding UI files
#-----------------------------------------------------------------------------------


ADD_LIBRARY (VirtualRobot SHARED ${SOURCES} ${INCLUDES})
TARGET_LINK_LIBRARIES(VirtualRobot ColCheckerPQP ${Simox_EXTERNAL_LIBRARIES})


# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


#######################################################################################
############################ Setup for installation ###################################
#######################################################################################


#set_target_properties(${PROJECT_NAME} PROPERTIES
#  PUBLIC_HEADER "${INCLUDES}")

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
  ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
  COMPONENT dev)
#  PUBLIC_HEADER DESTINATION "${INSTALL_INCLUDE_DIR}/VirtualRobot"
 
INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} ${CMAKE_CURRENT_BINARY_DIR} DESTINATION "${Simox_INSTALL_HEADER_DIR}"
	COMPONENT Headers
        FILES_MATCHING
        PATTERN "*.h"
        PATTERN "*.hpp"
        PATTERN ".svn" EXCLUDE
        PATTERN "CMakeModules" EXCLUDE
        PATTERN "tests" EXCLUDE
        PATTERN "build*" EXCLUDE
        PATTERN "data" EXCLUDE
        PATTERN "examples" EXCLUDE
	)

INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR}/data DESTINATION ${CMAKE_INSTALL_PREFIX}
	COMPONENT ApplicationData
        FILES_MATCHING 
        PATTERN "*.xml"
        PATTERN "*.XML"
        PATTERN "*.iv"
        PATTERN "*.vrml"
        PATTERN "*.wrl"
        PATTERN "*.stl"
        PATTERN "*.urdf"
        PATTERN "*.dae"
        PATTERN "*.png"
        PATTERN "*.jpg"
        PATTERN "*.bin"
        PATTERN ".svn" EXCLUDE
	)
   
MESSAGE (STATUS "***** Finished CONFIGURING Simox project VirtualRobot *****\n")
