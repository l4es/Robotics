# Try to find shared object of the urdf library

SET (URDF_FOUND FALSE)


FIND_PATH(URDFDOM_INCLUDE_DIRS urdfdom_model/model.h "$ENV{URDFDIR}/include" "$ENV{URDF_DIR}/include" "/usr/include")
IF (NOT URDFDOM_INCLUDE_DIRS)
	FIND_PATH(URDFDOM_INCLUDE_DIRS urdf_model/model.h "$ENV{URDFDIR}/include" "$ENV{URDF_DIR}/include" "/usr/include")
endif()
FIND_PATH(URDFDOM_PARSER_INCLUDE_DIRS urdf_parser/urdf_parser.h "$ENV{URDFDIR}/include" "$ENV{URDF_DIR}/include")
#Running the following (commented) command will result in loading liburdf.so, when the ros-indigo is installed (with the respective paths loaded). However this is not the right library to load. See https://github.com/ros/rosdistro/issues/4633
#FIND_LIBRARY(URDFDOM_LIBRARIES urdf "$ENV{URDFDIR}/lib" "$ENV{URDF_DIR}/lib" "/usr/lib" "/usr/lib/x86_64-linux-gnu")
#IF (NOT URDFDOM_LIBRARIES)
	FIND_LIBRARY(URDFDOM_LIBRARIES urdfdom_model "$ENV{URDFDIR}/lib" "$ENV{URDF_DIR}/lib" "/usr/lib" "/usr/lib/x86_64-linux-gnu")
#endif()


IF (URDFDOM_INCLUDE_DIRS AND URDFDOM_PARSER_INCLUDE_DIRS AND URDFDOM_LIBRARIES)
	SET (URDF_FOUND TRUE)
ENDIF ()

IF (URDF_FOUND)
   IF (NOT URDF_FIND_QUIETLY)
      MESSAGE(STATUS " ** Found URDF: ${URDFDOM_LIBRARIES}")
   ENDIF (NOT URDF_FIND_QUIETLY)
ELSE (URDF_FOUND)
   IF (URDF_FIND_REQUIRED)
      MESSAGE(FATAL_ERROR "Could not find URDF")
   ENDIF (URDF_FIND_REQUIRED)
ENDIF (URDF_FOUND)

MARK_AS_ADVANCED (
	URDFDOM_INCLUDE_DIRS
  URDFDOM_PARSER_INCLUDE_DIRS
	URDFDOM_LIBRARIES
	)
	
