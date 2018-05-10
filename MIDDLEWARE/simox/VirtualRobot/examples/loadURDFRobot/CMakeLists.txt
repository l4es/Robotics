PROJECT ( loadURDFRobot )

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.4)

CMAKE_POLICY(VERSION 2.6)

if (tinyxml_library AND URDF_FOUND)

  include_directories(${tinyxml_include_dirs})
  link_directories(${tinyxml_library_dirs})
  add_definitions(${tinyxml_cflags})
  add_definitions(" -DTIXML_USE_STL ")
  
  INCLUDE_DIRECTORIES (
    ${URDFDOM_INCLUDE_DIRS}
    ${URDFDOM_PARSER_INCLUDE_DIRS}
    ${URDFDOM_HEADERS_INCLUDE_DIRS}
  )
  add_definitions("-DDEMO_BASE_PATH=\"${PROJECT_SOURCE_DIR}\"")
  ADD_EXECUTABLE(${PROJECT_NAME} loadURDFobot.cpp)
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
  SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "Examples")

  TARGET_LINK_LIBRARIES(${PROJECT_NAME} VirtualRobot ${URDFDOM_LIBRARIES})
  
  
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

endif()


