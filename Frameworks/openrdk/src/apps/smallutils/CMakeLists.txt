IF (NOT "${CMAKE_CROSS_COMPILING}" STREQUAL TRUE)
	EXECUTE_PROCESS(COMMAND find . -maxdepth 1 -type d -not -iname .svn -not -iname . -not -iname naoqi-modules
									COMMAND tr "\n" ";"
									COMMAND sed -e "s@./@@g"
									WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
									OUTPUT_VARIABLE _toCompile)
	RDK_ADD_ALL_SUBDIRECTORIES(${_toCompile})
ENDIF (NOT "${CMAKE_CROSS_COMPILING}" STREQUAL TRUE)
