# Creation of rdk-config
CONFIGURE_FILE("rdk-config.template" "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/rdk-config" @ONLY)
EXECUTE_PROCESS(COMMAND "chmod" "+x" "rdk-config" WORKING_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})

#EXEC_PROGRAM("find . -maxdepth 1 -type f -not -iname '*.txt' -not -iname '*.cmake' -not -name '*.old' -not -name 'clrenv' -not -iname 'Makefile' -not -iname '*.swp' -not -iname '*.template'" ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE scripts1)
EXEC_PROGRAM("find . -maxdepth 1 -type f -name 'rdk-*' -not -name '*.template'" ${CMAKE_CURRENT_SOURCE_DIR} OUTPUT_VARIABLE scripts1)

IF(NOT ${scripts1} STREQUAL "")
	STRING(REPLACE "\n" ";" scripts ${scripts1})
	FOREACH(s ${scripts})
		GET_FILENAME_COMPONENT(scriptname ${s} NAME)
		ADD_CUSTOM_TARGET(copy_${scriptname} ALL cp ${CMAKE_CURRENT_SOURCE_DIR}/${scriptname} ${CMAKE_RUNTIME_OUTPUT_DIRECTORY} COMMENT "Copying ${scriptname} script in bin directory")
		#INSTALL(PROGRAMS "${s}" DESTINATION ${RUNTIME_INSTALL_PATH})
	ENDFOREACH(s ${scripts})
ENDIF(NOT ${scripts1} STREQUAL "")

