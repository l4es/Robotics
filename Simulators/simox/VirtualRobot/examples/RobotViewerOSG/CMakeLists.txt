PROJECT ( RobotViewerOSG )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

INCLUDE(${Simox_DIR}/CMakeModules/SimoxMacros.cmake)

IF(Simox_VISUALIZATION AND Simox_USE_OPENSCENEGRAPH_VISUALIZATION)


	# the variable "demo_SRCS" contains all .cpp files of this project
	FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RobotViewer.cpp ${PROJECT_SOURCE_DIR}/showRobotWindow.cpp)# osgViewerWidget.cpp)
	FILE(GLOB demo_INCS ${PROJECT_SOURCE_DIR}/showRobotWindow.h)# osgViewerWidget.h)


  ################################## moc'ing ##############################
  set(GUI_MOC_HDRS
      ${PROJECT_SOURCE_DIR}/showRobotWindow.h

  )
  #${PROJECT_SOURCE_DIR}/osgViewerWidget.h

  set(GUI_UIS
      ${PROJECT_SOURCE_DIR}/RobotViewer.ui
  )
  
  VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
   
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

  #qt4_wrap_cpp(demo_SRCS ${GUI_MOC_HDRS})
  #qt4_wrap_ui(UI_HEADER ${GUI_UIS})
  #get_filename_component(UI_HEADER_DIR ${UI_HEADER} PATH)
  #list(APPEND demo_INCS ${UI_HEADER})

  #include_directories(${UI_HEADER_DIR})

 
  #ADD_EXECUTABLE(${PROJECT_NAME} ${demo_SRCS} ${demo_INCS})
  #SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")
  #SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})

  #TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${VirtualRobot_VISUALIZATION_LIBS})
  #INCLUDE_DIRECTORIES(${VirtualRobot_VISUALIZATION_INCLUDE_PATHS})
  #INCLUDE_DIRECTORIES(${COLLADA_DOM_INCLUDE_DIRS})
  #INCLUDE(${QT_USE_FILE})
  #ADD_DEFINITIONS(${VirtualRobot_VISUALIZATION_COMPILE_FLAGS})

  #INSTALL(TARGETS ${PROJECT_NAME}
  #  DESTINATION ${VirtualRobot_INSTALL_BIN_DIR}
  #  COMPONENT Applications
  #)

  #  MESSAGE( STATUS ${PROJECT_NAME} " will be placed into " ${VirtualRobot_BIN_DIR})
  #  MESSAGE( STATUS ${PROJECT_NAME} " will be installed into " ${VirtualRobot_INSTALL_BIN_DIR})
ENDIF()
