# Macro to simplify output info
FUNCTION(info msg)
	MESSAGE(STATUS "${msg}")
ENDFUNCTION(info msg)

FUNCTION(verbose msg)
	IF($ENV{VERBOSE})
		info("${msg}")
	ENDIF($ENV{VERBOSE})
ENDFUNCTION(verbose msg)

FUNCTION(warn msg)
	MESSAGE(WARNING " ${msg}")
ENDFUNCTION(warn msg)

FUNCTION(error msg)
	MESSAGE(FATAL_ERROR "${msg}")
ENDFUNCTION(error msg)
