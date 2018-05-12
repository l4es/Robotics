
SET(RDKCORE_FILES	 
demangle/demangle.cpp
logging/levelfilter.cpp
logfile/logfile.cpp
logging/logging.cpp
logging/streamsink.cpp
object/objectmanager.cpp

simplethreads/simplethreads.cpp

filesystem/filesystem.cpp

common/threads.cpp

rdkmath/rdkmath.cpp

geometry/cameramatrix.cpp
geometry/dmatrix_algebra.cpp
geometry/geometryfunctions.cpp
geometry/otherpoints.cpp
geometry/output.cpp
geometry/pgmfile.cpp
geometry/posenormaldist.cpp
geometry/reference.cpp
geometry/robotmodels.cpp
geometry/utils.cpp
geometry/viewport.cpp
geometry/viewport_cross.cpp
geometry/segment.cpp
geometry/walk_circle.cpp
geometry/walk_line.cpp
geometry/walk_quad.cpp
geometry/pointstat.cpp

modules/module.cpp
modules/modulemanager.cpp
modules_config/moduleconfig.cpp
modules_config/pair.cpp
modules_config/ragentconfig.cpp

network/nettypes.cpp
network/inetaddress.cpp
network/netexception.cpp
network/socketaddress.cpp
network/tcpserversocket.cpp
network/tcpsocket.cpp
network/udpsocket.cpp

netobjs/socketaddress.cpp
netobjs/inetaddress.cpp
netobjs/tcpserversocket.cpp
netobjs/tcpsocket.cpp
netobjs/udpsocket.cpp

object/object.cpp
object/objectdiff.cpp

posixconstructs/posixmutex.cpp
posixconstructs/posixsem.cpp
posixconstructs/stack.cpp

posixqueues/debug.cpp
posixqueues/pipequeue.cpp

profiling/cpu_time.cpp
profiling/profiling.cpp

repository/remotepropertymanager.cpp
repository/tcpmanager.cpp
repository/tcpmanager_peermanager.cpp
repository/udpmanager.cpp

repository/remotesubscriptionregister.cpp
repository/repository.cpp
repository/repository_queue.cpp
repository/repository_remote.cpp
repository/repository_storage.cpp
repository/repository_utils.cpp
repository/session.cpp
repository/session_diffs.cpp
repository/session_events.cpp
repository/session_get_set_create.cpp
repository/session_queue.cpp
repository/session_utils.cpp
repository/session_network.cpp

repository_struct/property.cpp
repository_struct/robjectqueue.cpp
repository_struct/rpropertydef.cpp
repository_struct/rpropertydefvector.cpp
repository_struct/rpropertyupdate.cpp
repository_struct/rremotesubscription.cpp
repository_struct/url.cpp

rgeometry/rcameramatrix.cpp
rgeometry/rpoint2d.cpp
rgeometry/rpoint2dvector.cpp
rgeometry/rpoint2i.cpp
rgeometry/rpoint2od.cpp
rgeometry/rpoint2odvector.cpp
rgeometry/rpoint3d.cpp
rgeometry/rpoint3od.cpp
rgeometry/rpolarpoint.cpp
rgeometry/rpointstat.cpp

rgraphics/imgdiffs.cpp
rgraphics/rc8set.cpp
rgraphics/rimage.cpp
rgraphics/rimage_conversions.cpp
rgraphics/rimage_magick.cpp
rgraphics/rimage_diffs.cpp
rgraphics/rimage_serialization.cpp
rgraphics/rimagediffpoints.cpp
rgraphics/rimagediffrect.cpp
rgraphics/rimagediffjpegpacket.cpp
rgraphics/rimagepointc8.cpp
rgraphics/rposevector.cpp
rgraphics/color.cpp
rgraphics/colorobjecttable.cpp

rmaps/rfeatureonmapvector.cpp
rmaps/rfidtagonmap.cpp
rmaps/rfrontier.cpp
rmaps/rtopologicalfrontier.cpp
rmaps/rhbdscanresult.cpp
rmaps/ritemonmapvector.cpp
rmaps/rlandmark.cpp
rmaps/rline2donmap.cpp
rmaps/rmapimage.cpp
rmaps/rmapimagediffpoints.cpp
rmaps/rmapimagediffrect.cpp
rmaps/rpathanswer.cpp
rmaps/rpathrequest.cpp
rmaps/rpoint2donmap.cpp
rmaps/rpotentialvictimonmap.cpp
rmaps/rvictimonmap.cpp
rmaps/rellipseonmap.cpp
rmaps/rarrowonmap.cpp
rmaps/rlabelonmap.cpp

rnetobjects/rnetmessage.cpp
rnetobjects/ryellowpages.cpp

robot/robotmodule.cpp

rprimitive/rbool.cpp
rprimitive/rdouble.cpp
rprimitive/rint.cpp
rprimitive/rstring.cpp
rprimitive/rstringvector.cpp

rsensordata/ractuadata.cpp
rsensordata/rblobdata.cpp
rsensordata/rfiducialdata.cpp
rsensordata/rjoystickstatus.cpp
rsensordata/rircdata.cpp
rsensordata/rxsensdata.cpp
rsensordata/rpantiltdata.cpp
rsensordata/rlaserdata.cpp
rsensordata/rodometrydata.cpp
rsensordata/rodometrydatawithspeed.cpp
rsensordata/rsonardata.cpp
rsensordata/rusarinudata.cpp
rsensordata/rusarrfiddata.cpp
rsensordata/rusarvictimrfiddata.cpp
rsensordata/rusarvictimdata.cpp
rsensordata/rtemperaturedata.cpp
rsensordata/rstereoimageheightsdata.cpp
rsensordata/rtouchsensordata.cpp
rsensordata/logtagtoobject.cpp
rsensordata/rpercept.cpp

sensordata/laserdata.cpp
sensordata/laserdata_fisher.cpp
sensordata/laserparser.cpp
sensordata/logreader.cpp
sensordata/laserdatatracker.cpp
sensordata/percept.cpp

serialization_binary/binaryreader_bytes.cpp
serialization_binary/binaryreader_packets.cpp
serialization_binary/binaryreader_recursion.cpp
serialization_binary/binaryreader_stacktrace.cpp
serialization_binary/binarywriter.cpp
serialization_binary/binarywriter_recursion.cpp
serialization_binary/packets.cpp

serialization_xml/xmlreader.cpp
serialization_xml/xmlutils.cpp
serialization_xml/xmlwriter.cpp

stat/stat.cpp

textutils/linestream.cpp
textutils/textutils.cpp
textutils/base64.cpp
textutils/kagetopt.cpp

sharedmemory/sharedmemory.cpp

)

# Adding optional libraries support
IF(OpenCV_FOUND)
	SET(RDKCORE_FILES ${RDKCORE_FILES};interop/opencv.cpp)
ENDIF(OpenCV_FOUND)

IF(MagickPP_FOUND)
	SET(RDKCORE_FILES ${RDKCORE_FILES};interop/magick++.cpp)
ENDIF(MagickPP_FOUND)

# Adding optional libraries support
# include_directories and link_directories apply to target
# created AFTER their call
IF(MagickPP_FOUND)
	INCLUDE_DIRECTORIES(${MagickPP_INCLUDE_DIR})
	LINK_DIRECTORIES(${MagickPP_LINK_DIRECTORIES})
ENDIF(MagickPP_FOUND)

IF(OpenCV_FOUND)
	INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIR})
	LINK_DIRECTORIES(${OpenCV_LINK_DIRECTORIES})
ENDIF(OpenCV_FOUND)

IF(CPPUNIT_FOUND)
	INCLUDE_DIRECTORIES(${CPPUNIT_INCLUDE_DIR})
	LINK_DIRECTORIES(${CPPUNIT_LINK_DIRECTORIES})
ENDIF(CPPUNIT_FOUND)

# Main target of OpenRDK
# every module will use it
set(LIBRARY_OUTPUT_PATH ${CMAKE_LIBRARY_OUTPUT_DIRECTORY})
INCLUDE_DIRECTORIES(${_rdkcore_include_directories};${RDKCORE_INCLUDE_DIR})
LINK_DIRECTORIES(${_rdkcore_link_directories})
# Global definition that should be added
ADD_DEFINITIONS(${_rdkcore_definitions} ${RDKCORE_DEFINITIONS})
ADD_LIBRARY(${RDKCORE_NAME} SHARED ${RDKCORE_FILES})
TARGET_LINK_LIBRARIES(${RDKCORE_NAME} ${_rdkcore_libraries})

# Adding optional libraries support
IF(MagickPP_FOUND)
	info("${RDKCORE_NAME}: Magick++ support enabled")
	TARGET_LINK_LIBRARIES(${RDKCORE_NAME} ${MagickPP_LIBRARIES})
ENDIF(MagickPP_FOUND)

IF(OpenCV_FOUND)
	info("${RDKCORE_NAME}: OpenCV support enabled")
	TARGET_LINK_LIBRARIES(${RDKCORE_NAME} ${OpenCV_LIBRARIES})
ENDIF(OpenCV_FOUND)

IF(CPPUNIT_FOUND)
	info("${RDKCORE_NAME}: CppUnit support enabled")
	TARGET_LINK_LIBRARIES(${RDKCORE_NAME} ${CPPUNIT_LIBRARIES})
ENDIF(CPPUNIT_FOUND)

IF(ENABLE_RDKCORE_TESTS)
	ADD_SUBDIRECTORY(tests)
	#ADD_TEST(rdkcore_url test_url)
	info("${RDKCORE_NAME}: testing enabled")
ENDIF(ENABLE_RDKCORE_TESTS)

#### Old stuff: to be removed
#INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/ruby.cmake)
#INCLUDE(${CMAKE_CURRENT_SOURCE_DIR}/oldstuff.cmake)

