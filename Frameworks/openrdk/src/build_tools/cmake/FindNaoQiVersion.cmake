# - Find NaoQi Version Number		

set(NaoQiVersion_FOUND FALSE)

IF(${OpenRDK_ARCH} STREQUAL "geode" OR ${OpenRDK_ARCH} STREQUAL "atom" OR ${OpenRDK_ARCH} STREQUAL "generic")
	set(AL_DIR /usr/local/opt/nao-sdk)
	
	if(EXISTS "${AL_DIR}")

		info("@@@@@@@@@@@ AL_DIR - $$ENV{AL_DIR}")
		set(AL_VERSION_FILE AL_VERSION_FILE-NOTFOUND CACHE INTERNAL "" FORCE)
		set(__al_config_file_list configcore.h;alcommon/alcommon_revision.h;version;index.html)
		
		foreach(file ${__al_config_file_list})
		   	find_file(AL_VERSION_FILE
				NAMES ${file}
				PATHS $$ENV{AL_DIR}/include;$$ENV{AL_DIR}/doc;$$ENV{AL_DIR};$ENV{OE_CROSS_DIR})
		  	 MESSAGE(STATUS "FILE: ${file}; RT: ${AL_VERSION_FILE}")
		endforeach(file)

		if("${AL_VERSION_FILE}" STREQUAL "AL_VERSION_FILE-NOTFOUND")
			warn("Aldebaran directories found, but \"${AL_VERSION_FILE}\" not found.")
			warn("Probably Aldebaran changes something (again)...")
			warn("Please check the FindALDEBARAN.cmake to fix the problem")
		else("${AL_INC_DIR}" STREQUAL "AL_VERSION_FILE-NOTFOUND")
		info("Found version file: ${AL_VERSION_FILE}")
		# trying to retrieve version number
			EXECUTE_PROCESS(
			COMMAND grep "VERSION:" ${AL_VERSION_FILE}
				OUTPUT_VARIABLE __NaoQi_VERSION
					)
			info("@@@@ __NaoQi_VERSION - ${__NaoQi_VERSION}")
					
	STRING(REGEX REPLACE ".*\\'([0-9]+\).[0-9]+.[0-9]+\\'.*" "\\1" NAOQI_VERSION_MAJOR ${__NaoQi_VERSION})
	STRING(REGEX REPLACE ".*\\'[0-9]+.\([0-9]+\).[0-9]+\\'.*" "\\1" NAOQI_VERSION_MINOR ${__NaoQi_VERSION})
	STRING(REGEX REPLACE ".*\\'[0-9]+.[0-9]+.\([0-9]+\)\\'.*" "\\1" NAOQI_VERSION_PATCH ${__NaoQi_VERSION})
	set(NaoQi_VERSION "${NAOQI_VERSION_MAJOR}.${NAOQI_VERSION_MINOR}.${NAOQI_VERSION_PATCH}" CACHE STRING "NaoQi Version" FORCE)

	info("@@@ NaoQi_VERSION FNQ- ${NaoQi_VERSION}")
	  
	  set(NaoQiVersion_FOUND TRUE)

		endif("${AL_VERSION_FILE}" STREQUAL "AL_VERSION_FILE-NOTFOUND")

	endif(EXISTS "${AL_DIR}")

ENDIF(${OpenRDK_ARCH} STREQUAL "geode" OR ${OpenRDK_ARCH} STREQUAL "atom" OR ${OpenRDK_ARCH} STREQUAL "generic")
