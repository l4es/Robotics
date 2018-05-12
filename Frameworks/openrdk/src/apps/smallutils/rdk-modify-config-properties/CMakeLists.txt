ADD_EXECUTABLE(rdk-modify-config-properties modify-config-properties.cpp)
	TARGET_LINK_LIBRARIES(rdk-modify-config-properties ${RDKCORE_LIBRARIES} dl)
	SET_TARGET_PROPERTIES(rdk-modify-config-properties PROPERTIES COMPILE_FLAGS "${RDKCORE_CPPFLAGS} ${RDKCORE_CXXFLAGS}")
	SET_TARGET_PROPERTIES(rdk-modify-config-properties PROPERTIES LINK_FLAGS "${RDKCORE_LDFLAGS}")

