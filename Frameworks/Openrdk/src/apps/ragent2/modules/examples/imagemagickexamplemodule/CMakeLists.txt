# This file has been generated by rdk-cmake-ragent-module.sh
IF(MagickPP_FOUND)
	RDK_ADD_RAGENT_MODULE(ALL_FILES)
	INCLUDE_DIRECTORIES(${MagickPP_INCLUDE_DIR})
	LINK_DIRECTORIES(${MagickPP_LINK_DIRECTORIES})
	TARGET_LINK_LIBRARIES(${RDK_THIS_MODULE_NAME} ${MagickPP_LIBRARIES})
ENDIF(MagickPP_FOUND)
