# not available yet, thus skipping this for now
#IF(NOT ${CMAKE_SYSTEM} MATCHES "CYGWIN")
#  INSTALL(TARGETS rdkcore LIBRARY DESTINATION ${LIBRARY_INSTALL_PATH})
#ELSE(NOT ${CMAKE_SYSTEM} MATCHES "CYGWIN")
#  INSTALL(TARGETS rdkcore RUNTIME DESTINATION ${LIBRARY_INSTALL_PATH})
#ENDIF(NOT ${CMAKE_SYSTEM} MATCHES "CYGWIN")

#ADD_EXECUTABLE(test_pipequeue posixqueues/unittest_pipequeue.cpp)
#TARGET_LINK_LIBRARIES(test_pipequeue rdkcore)
#INSTALL(TARGETS test_pipequeue RUNTIME DESTINATION ${RUNTIME_INSTALL_PATH})

#ADD_EXECUTABLE(relay network/relay.cpp)
#TARGET_LINK_LIBRARIES(relay rdkcore)
#SET_TARGET_PROPERTIES(relay PROPERTIES LINK_FLAGS "${RDK2_LDFLAGS}")

#IF(COMPILE_TESTS)
#	IF(CPPUNIT_FOUND)
#		ADD_SUBDIRECTORY(test_serialization) 
#	ELSE(CPPUNIT_FOUND)
#		MESSAGE(FATAL_ERROR "You requested to build unittests (in manual.cmake), but you do not have the CppUnit library installed.")
#	ENDIF(CPPUNIT_FOUND)
#ENDIF(COMPILE_TESTS)

