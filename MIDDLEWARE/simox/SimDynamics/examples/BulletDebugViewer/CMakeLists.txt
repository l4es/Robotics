PROJECT ( BulletDebugViewer )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

INCLUDE(${Simox_DIR}/CMakeModules/SimoxMacros.cmake)

IF (SimDynamics_USE_BULLET_USE_GLUT)
	include_directories(${GLUT_INCLUDE_DIR})
	message("GLUT_INCLUDE_DIR:${GLUT_INCLUDE_DIR}")
	# create the executable
	SimoxApplication(${PROJECT_NAME} "BulletDebugViewerGlut.cpp" "")
	TARGET_LINK_LIBRARIES(${PROJECT_NAME} SimDynamics BulletOpenGLSupport)
ELSE(SimDynamics_USE_BULLET_USE_GLUT)

	IF(WIN32)
		SET (SRC
			"BulletDebugViewerWin32.cpp"
			"${Simox_DIR}/SimDynamics/DynamicsEngine/BulletEngine/BulletOpenGL/Win32AppMain.cpp"
		)
		#"Win32BasicDemo.cpp"
		SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} /subsystem:windows")
		SimoxApplication(${PROJECT_NAME} ${SRC} "")
		TARGET_LINK_LIBRARIES(${PROJECT_NAME} SimDynamics BulletOpenGLSupport)
	ELSE(WIN32)
		MESSAGE (STATUS " * !GLUT only supported on windows so far... Skipping BulletDebugViewer")
	ENDIF(WIN32)
ENDIF(SimDynamics_USE_BULLET_USE_GLUT)

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  
#######################################################################################
############################ Setup for installation ###################################
#######################################################################################

install(TARGETS ${PROJECT_NAME}
  # IMPORTANT: Add the library to the "export-set"
  EXPORT SimoxTargets
  RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
  COMPONENT dev)

MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be placed into " ${Simox_BIN_DIR})
MESSAGE( STATUS " ** Simox application ${PROJECT_NAME} will be installed into " ${INSTALL_BIN_DIR})
