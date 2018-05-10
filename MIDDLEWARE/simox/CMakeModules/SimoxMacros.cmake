
# Build and helper macros

function(setupSimoxExternalLibraries)
  IF (Simox_VISUALIZATION)
    # we need to check for Qt4
    IF(NOT "$ENV{QT_QMAKE_EXECUTABLE}" STREQUAL "")
      #  if (NOT (Simox_FIND_QUIETLY OR VirtualRobot_FIND_QUIETLY)) 
      #    MESSAGE (STATUS "USING QT-PATH from environment variable QT_QMAKE_EXECUTABLE: $ENV{QT_QMAKE_EXECUTABLE}")
      #endif()
      file(TO_CMAKE_PATH "$ENV{QT_QMAKE_EXECUTABLE}" QT_QMAKE_EXECUTABLE)
    ENDIF()
    FIND_PACKAGE(Qt4 4.6.0 COMPONENTS QtOpenGL QtCore QtGui)
  ENDIF()
  INCLUDE_DIRECTORIES(${Simox_INCLUDE_DIRS})
  INCLUDE_DIRECTORIES(${Simox_EXTERNAL_INCLUDE_DIRS})
  ADD_DEFINITIONS( ${Simox_EXTERNAL_LIBRARY_FLAGS} )
  LINK_DIRECTORIES( ${Simox_LIBRARY_DIRS} )
  
  FOREACH(f ${Simox_EXTERNAL_CMAKE_INCLUDE})
      MESSAGE(STATUS " * Simox_EXTERNAL_CMAKE_INCLUDE: ${f}")
      INCLUDE(${f})
  ENDFOREACH(f) 
endfunction()


function(VirtualRobotApplication name srcs incs)
    setupSimoxExternalLibraries()
    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs})
    TARGET_LINK_LIBRARIES(${name} VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
endfunction()


function(VirtualRobotQtApplication name srcs incs mocFiles uiFiles)
    setupSimoxExternalLibraries() 
    MESSAGE (STATUS "Qt Moc'ing: ${mocFiles}")
    # need this option to work around a qt/boost bug
    qt4_wrap_cpp(generatedMocFiles ${mocFiles} OPTIONS -DBOOST_TT_HAS_OPERATOR_HPP_INCLUDED -DBOOST_NO_TEMPLATE_PARTIAL_SPECIALIZATION)
    MESSAGE (STATUS "Qt ui files: ${uiFiles}")
    qt4_wrap_ui(generatedUiFiles ${uiFiles})
    INCLUDE_DIRECTORIES( ${CMAKE_CURRENT_BINARY_DIR} )
    ################################## EXECUTABLE ##############################
    ADD_EXECUTABLE(${name} ${srcs} ${incs} ${generatedUiFiles} ${generatedMocFiles})
    TARGET_LINK_LIBRARIES(${name} VirtualRobot ${Simox_EXTERNAL_LIBRARIES})
endfunction()


function(SimoxApplication name srcs incs)
    VirtualRobotApplication("${name}" "${srcs}" "${incs}")
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} GraspStudio Saba)
endfunction()


function(SimoxQtApplication name srcs incs mocFiles uiFiles)
    VirtualRobotQtApplication("${name}" "${srcs}" "${incs}" "${mocFiles}" "${uiFiles}")  
    # add Saba and GraspStudio
    TARGET_LINK_LIBRARIES(${name} GraspStudio Saba)
endfunction()
