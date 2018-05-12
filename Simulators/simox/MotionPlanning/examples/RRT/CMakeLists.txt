PROJECT ( RRTdemo )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

INCLUDE(${Simox_DIR}/CMakeModules/SimoxMacros.cmake)

IF(Simox_VISUALIZATION AND Simox_USE_COIN_VISUALIZATION)
    
  # the variable "demo_SRCS" contains all .cpp files of this project
  FILE(GLOB demo_SRCS ${PROJECT_SOURCE_DIR}/RRTdemo.cpp)
    
  VirtualRobotQtApplication(${PROJECT_NAME} "${demo_SRCS}" "${demo_INCS}" "${GUI_MOC_HDRS}" "${GUI_UIS}")
  TARGET_LINK_LIBRARIES(${PROJECT_NAME} Saba)
  
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

ELSE()
	MESSAGE (STATUS "Simox_USE_COIN_VISUALIZATION not defined: Skipping ${PROJECT_NAME}")
ENDIF()
