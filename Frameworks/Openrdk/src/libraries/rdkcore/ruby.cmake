# This stuff are not used anymore, maybe they should be removed
SET(COMPILE_RUBY 0) # to be sure they are not used

# Aggiungiamo il codice per embedding ruby
IF(COMPILE_RUBY)
	IF(NOT RUBY_FOUND)
		MESSAGE(FATAL_ERROR "COMPILE_RUBY=1 but RUBY_FOUND=0")
	ENDIF(NOT RUBY_FOUND)

	SET(RDK2_FILES ${RDK2_FILES} 
		embedded_ruby2/exposed_to_ruby.cpp 
		embedded_ruby2/embedded_ruby.cpp)

	FILE(GLOB RUBY_TO_INCLUDE "embedded_ruby2/ruby/*.rb")
	MESSAGE("Ruby to include: ${RUBY_TO_INCLUDE}")
	FOREACH(r ${RUBY_TO_INCLUDE})

		get_filename_component(base ${r} NAME_WE)
		set(varname "file_${base}")
		set(cppfile ${r}.cpp)

		MESSAGE("Adding ${cppfile}, varname = ${varname}")

		CONVERT_FILE2C(${r} ${cppfile} ${varname})

		SET(RDK2_FILES ${RDK2_FILES} ${cppfile})

	ENDFOREACH(r ${RUBY_TO_INCLUDE})
	
	# libreria esterna di esempio
	SUBDIRS(embedded_ruby2/sample_external)
ENDIF(COMPILE_RUBY)

# Convertiamo in C i file dentro embedded_ruby2/ruby/
IF(COMPILE_RUBY)
	TARGET_LINK_LIBRARIES(rdkcore ${RUBY_LIBRARY})

	# tests
	SUBDIRS(embedded_ruby2)
	
	MACRO(RUBY_TEST testname)
		SET(bin ${testname})
		ADD_EXECUTABLE(${bin} embedded_ruby2/${testname}.cpp)
		ADD_DEPENDENCIES(${bin} ${r}.c)
		TARGET_LINK_LIBRARIES(${bin} rdkcore)
		TARGET_LINK_LIBRARIES(${bin} cppunit)	
		ADD_TEST(${bin} ${bin})
	ENDMACRO(RUBY_TEST)

	RUBY_TEST(test_rdk)
	RUBY_TEST(test_all)

ENDIF(COMPILE_RUBY)

