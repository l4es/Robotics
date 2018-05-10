
PROJECT(powercrust)

CMAKE_MINIMUM_REQUIRED(VERSION 2.6.0)
CMAKE_POLICY(VERSION 2.6)

MESSAGE (STATUS " **** Configuring powercrust")

# Specify sources and headers
FILE(GLOB SRCS ${PROJECT_SOURCE_DIR}/*.cpp)
FILE(GLOB INCS ${PROJECT_SOURCE_DIR}/*.h)

INCLUDE_DIRECTORIES(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include ${PROJECT_SOURCE_DIR}/src)
ADD_LIBRARY(${PROJECT_NAME} STATIC ${SRCS} ${INCS})

SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES FOLDER "3rd_Party")
# .DLL path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${Simox_BIN_DIR})
# .so path
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${Simox_LIB_DIR})
# .lib path (this is needed for setting the DLL-import library path on windows)
SET_TARGET_PROPERTIES(${PROJECT_NAME} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${Simox_LIB_DIR})


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

