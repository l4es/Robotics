PROJECT ( SimDynamics )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.2)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS "***** CONFIGURING Simox project SimDynamics *****")


GET_FILENAME_COMPONENT (CurrentSimDynamicsPath ${CMAKE_CURRENT_LIST_FILE} PATH)
SET(SimDynamics_DIR ${CurrentSimDynamicsPath})
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${SimDynamics_DIR}/CMakeModules)

INCLUDE (Cxx11Test)
SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX11_FLAG}")

############################# SETUP PHYSICS ENGINE #############################
OPTION(SimDynamics_USE_BULLET "Use Bullet Physics Engine" ON)
SET (SimDynamics_DYNAMICSENGINE FALSE)
SET (SimDynamics_PHYSICS_LIBRARIES "")

if (SimDynamics_USE_BULLET)

  IF(NOT "$ENV{BULLET_ROOT}" STREQUAL "")
    MESSAGE (STATUS " ** USING BULLET_ROOT-PATH from environment variable BULLET_ROOT: $ENV{BULLET_ROOT}")
    file(TO_CMAKE_PATH "$ENV{BULLET_ROOT}" BULLET_ROOT)
  ENDIF()
	#SET (SimDynamics_BULLET_ROOT ${Simox_BULLET_ROOT} CACHE PATH "Path to Bullet used by SimDynamics")
  SET (BULLET_ROOT ${BULLET_ROOT} CACHE PATH "Bullet Path")
  Find_Package(Bullet)
  if (BULLET_FOUND)
      MESSAGE (STATUS " ** Found Bullet at ${BULLET_INCLUDE_DIR}")
      #MESSAGE (STATUS "BULLET_LIBRARIES: ${BULLET_LIBRARIES}")
      #MESSAGE (STATUS "BULLET_OPENGL_INCLUDE_DIR: ${BULLET_OPENGL_INCLUDE_DIR}")
      #MESSAGE (STATUS "BULLET_OpenGLSupport_LIBRARY_debug: ${BULLET_OpenGLSupport_LIBRARY_debug}")
      SET(SIMDYNAMICS_INCLUDE_DIRS "${BULLET_INCLUDE_DIR}" "${BULLET_INCLUDE_DIR}/bullet") # "${BULLET_DEMOS_INCLUDE_DIR}" "${BULLET_OPENGL_INCLUDE_DIR}")# PARENT_SCOPE)
     	
     	INCLUDE_DIRECTORIES( 
          ${BULLET_INCLUDE_DIR}
          ${BULLET_INCLUDE_DIR}/bullet
		  )

      OPTION( SimDynamics_USE_BULLET_DOUBLE_PRECISION "Use Bullet Engine built with double precision" OFF )
	  IF (WIN32)
		OPTION(SimDynamics_USE_BULLET_USE_GLUT "Use Glut"	OFF)
	  else()
		OPTION(SimDynamics_USE_BULLET_USE_GLUT "Use Glut"	ON)
	  endif()
      IF( SimDynamics_USE_BULLET_DOUBLE_PRECISION )
          ADD_DEFINITIONS( -DBT_USE_DOUBLE_PRECISION)
          LIST(APPEND SIMDYNAMICS_COMPILE_FLAGS "-DBT_USE_DOUBLE_PRECISION")
      ENDIF( SimDynamics_USE_BULLET_DOUBLE_PRECISION )
      #MESSAGE(STATUS "BULLET_LIBRARIES:${BULLET_LIBRARIES}")
      SET (SimDynamics_PHYSICS_LIBRARIES "${BULLET_LIBRARIES}")
      SET (SimDynamics_DYNAMICSENGINE TRUE)
      
	  FIND_PACKAGE(OpenGL)
	  IF (OPENGL_FOUND)
	  	MESSAGE (STATUS " ** OPENGL FOUND lib:${OPENGL_gl_LIBRARY}")
		#check if there was a distuingish between optimized and debug libs:
      	IF (SimDynamics_PHYSICS_LIBRARIES MATCHES "optimized;")
			LIST(APPEND SimDynamics_PHYSICS_LIBRARIES optimized ${OPENGL_gl_LIBRARY} debug ${OPENGL_gl_LIBRARY})
		ELSE()
			LIST(APPEND SimDynamics_PHYSICS_LIBRARIES ${OPENGL_gl_LIBRARY})
		ENDIF()
	  ENDIF()
	  IF (OPENGL_GLU_FOUND)
		  MESSAGE (STATUS " ** OPENGL_GLU FOUND lib:${OPENGL_glu_LIBRARY}")
		  #check if there was a distuingish between optimized and debug libs:
      	  IF (SimDynamics_PHYSICS_LIBRARIES MATCHES "optimized;")
		    LIST(APPEND SimDynamics_PHYSICS_LIBRARIES optimized ${OPENGL_glu_LIBRARY} debug ${OPENGL_glu_LIBRARY})
		  ELSE()
		    LIST(APPEND SimDynamics_PHYSICS_LIBRARIES ${OPENGL_glu_LIBRARY})
		  ENDIF()
	  ENDIF()

      IF (SimDynamics_USE_BULLET_USE_GLUT)
        FIND_PACKAGE(GLUT)
        IF (GLUT_FOUND)
            MESSAGE(STATUS " ** GLUT FOUND: ${GLUT_glut_LIBRARY}")
		    #check if there was a distuingish between optimized and debug libs:
      	    IF (SimDynamics_PHYSICS_LIBRARIES MATCHES "optimized;")
				LIST(APPEND SimDynamics_PHYSICS_LIBRARIES optimized ${GLUT_glut_LIBRARY} debug ${GLUT_glut_LIBRARY})
			ELSE()
				LIST(APPEND SimDynamics_PHYSICS_LIBRARIES ${GLUT_glut_LIBRARY} )
			ENDIF()

      		IF (WIN32)
      			INCLUDE_DIRECTORIES(${GLUT_INCLUDE_DIR})
      			SET(SIMDYNAMICS_INCLUDE_DIRS "${SIMDYNAMICS_INCLUDE_DIRS}" "${GLUT_INCLUDE_DIR}")# PARENT_SCOPE)
      		ELSE (WIN32)
      			# This is the lines for linux.  This should always work if everything is installed and working fine.
      			INCLUDE_DIRECTORIES(/usr/include /usr/local/include ${GLUT_INCLUDE_DIR}) 
      			SET(SIMDYNAMICS_INCLUDE_DIRS "${SIMDYNAMICS_INCLUDE_DIRS}" "/usr/include" "/usr/local/include" "${GLUT_INCLUDE_DIR}")
      		ENDIF (WIN32)
		ELSE (GLUT_FOUND)
          SET( GLUT_glut_LIBRARY "" CACHE PATH "Glut library." )
        ENDIF (GLUT_FOUND)
        
		#SET (SimDynamics_BULLET_OpenGL TRUE)
      ENDIF(SimDynamics_USE_BULLET_USE_GLUT)

      ADD_SUBDIRECTORY(DynamicsEngine/BulletEngine/BulletOpenGL)

    else()
        MESSAGE (STATUS " ** Could not find Bullet")
    endif()
    
else()
        MESSAGE ("No Physics engine selected...")   	    
endif()


MESSAGE (STATUS " ** SIMDYNAMICS_INCLUDE_DIRS: ${SIMDYNAMICS_INCLUDE_DIRS}")

MESSAGE (STATUS " ** SimDynamics_PHYSICS_LIBRARIES: ${SimDynamics_PHYSICS_LIBRARIES}")


#######################################################################
# Setup for testing
#######################################################################

MACRO(ADD_SIMDYNAMICS_TEST TEST_NAME)
  ADD_EXECUTABLE(${TEST_NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${TEST_NAME}.cpp)
  TARGET_LINK_LIBRARIES(${TEST_NAME} VirtualRobot Saba GraspStudio SimDynamics ${Simox_EXTERNAL_LIBRARIES} ${SimDynamics_PHYSICS_LIBRARIES} ${Boost_TEST_LIB})
  SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_TEST_DIR})
  SET_TARGET_PROPERTIES(${TEST_NAME} PROPERTIES FOLDER "SimDynamics Tests")
  ADD_TEST(NAME SimDynamics_${TEST_NAME}
          COMMAND ${Simox_TEST_DIR}/${TEST_NAME} --output_format=XML --log_level=all --report_level=no)
ENDMACRO(ADD_SIMDYNAMICS_TEST)


if (SimDynamics_DYNAMICSENGINE)

  SET(SOURCES
    DynamicsWorld.cpp
    DynamicsEngine/DynamicsObject.cpp
    DynamicsEngine/DynamicsEngine.cpp
    DynamicsEngine/DynamicsRobot.cpp
    DynamicsEngine/DynamicsUtils.cpp
  )
    
  SET(INCLUDES
    DynamicsWorld.h
    SimDynamics.h
    DynamicsEngine/DynamicsEngineFactory.h
    DynamicsEngine/DynamicsObject.h
    DynamicsEngine/DynamicsEngine.h
    DynamicsEngine/DynamicsRobot.h
    DynamicsEngine/DynamicsUtils.h
  )
 #    ${SimDynamics_SimoxDir}/VirtualRobot/definesVR.h
   
  if (SimDynamics_USE_BULLET)
  	SET(SOURCES
    	${SOURCES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.cpp
    	DynamicsEngine/BulletEngine/BulletEngine.cpp
    	DynamicsEngine/BulletEngine/BulletObject.cpp
    	DynamicsEngine/BulletEngine/BulletRobot.cpp
    	DynamicsEngine/BulletEngine/BulletRobotLogger.cpp
    	DynamicsEngine/BulletEngine/SimoxMotionState.cpp
     	DynamicsEngine/BulletEngine/SimoxCollisionDispatcher.cpp
  	)
  
  	SET(INCLUDES
    	${INCLUDES}
    	DynamicsEngine/BulletEngine/BulletEngineFactory.h
    	DynamicsEngine/BulletEngine/BulletEngine.h
    	DynamicsEngine/BulletEngine/BulletObject.h
    	DynamicsEngine/BulletEngine/BulletRobot.h
    	DynamicsEngine/BulletEngine/BulletRobotLogger.h
    	DynamicsEngine/BulletEngine/SimoxMotionState.h
     	DynamicsEngine/BulletEngine/SimoxCollisionDispatcher.h
  	)
  	
  	if (SimDynamics_BULLET_OpenGL)
      	SET(SOURCES
        	${SOURCES}
			DynamicsEngine/BulletEngine/BulletOpenGLViewer.cpp
			)

      	SET(INCLUDES
        	${INCLUDES}
			DynamicsEngine/BulletEngine/BulletOpenGLViewer.h
			)

      else()
          MESSAGE (STATUS " ** Bullet's OpenGLSupport library not found: Skipping BulletOpenGLViewer")
      endif()
  
  if (Simox_USE_COIN_VISUALIZATION)
      	SET(SOURCES
        	${SOURCES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.cpp
      	)
      
      	SET(INCLUDES
        	${INCLUDES}
        	DynamicsEngine/BulletEngine/BulletCoinQtViewer.h
      	)
      else()
          MESSAGE (STATUS " ** Qt/Coin3D disabled: Skipping BulletCoinQtViewer")
      endif()
  endif ()
    
  # this var is considered for generating Simox_INCLUDE_DIRS_
  SET(Simox_SimDynamics_INCLUDE_DIRS "${SIMDYNAMICS_INCLUDE_DIRS}" PARENT_SCOPE)
  SET(Simox_SimDynamics_EXTERNAL_LIBRARIES "${SimDynamics_PHYSICS_LIBRARIES}" PARENT_SCOPE)
  SET(Simox_SimDynamics_COMPILE_FLAGS "${SIMDYNAMICS_COMPILE_FLAGS}" PARENT_SCOPE)
  #INCLUDE_DIRECTORIES(${SimDynamics_SimoxDir})
  #INCLUDE_DIRECTORIES(${SimDynamics_DIR})
   

  INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
  ADD_DEFINITIONS(${Simox_EXTERNAL_LIBRARY_FLAGS})

  ADD_LIBRARY (SimDynamics SHARED ${SOURCES} ${INCLUDES})
  if (SimDynamics_BULLET_OpenGL)
	TARGET_LINK_LIBRARIES (SimDynamics BulletOpenGLSupport)
	SET (SimDynamics_BULLET_OpenGL TRUE PARENT_SCOPE) # inform parent scope about BulletOpenGLSupport
  ENDIF(SimDynamics_BULLET_OpenGL)
  TARGET_LINK_LIBRARIES (SimDynamics ${Simox_EXTERNAL_LIBRARIES} VirtualRobot ${SimDynamics_PHYSICS_LIBRARIES})

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
  if (SimDynamics_USE_BULLET)
      ADD_SUBDIRECTORY(DynamicsEngine/BulletEngine/tests/)
  endif()
  
    
    
  #######################################################################################
  ############################ Setup for installation ###################################
  #######################################################################################
  
  
  INSTALL(TARGETS ${PROJECT_NAME}
    # IMPORTANT: Add the library to the "export-set"
    EXPORT SimoxTargets
    RUNTIME DESTINATION "${INSTALL_BIN_DIR}" COMPONENT bin
    LIBRARY DESTINATION "${INSTALL_LIB_DIR}" COMPONENT shlib
    ARCHIVE DESTINATION "${INSTALL_LIB_DIR}" COMPONENT library
    COMPONENT dev)
  
  INSTALL(DIRECTORY ${PROJECT_SOURCE_DIR} DESTINATION ${Simox_INSTALL_HEADER_DIR}
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
  
else()
    MESSAGE ("Not building SimDynamics, physics engine not found...")
endif()

MESSAGE (STATUS "***** Finished CONFIGURING Simox project SimDynamics *****")
