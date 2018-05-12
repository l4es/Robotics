/*
 *    OpenRDK : OpenSource Robot Development Kit
 *    Copyright (C) 2007, 2008  Daniele Calisi, Andrea Censi (<first_name>.<last_name>@dis.uniroma1.it)
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef H_RDK2_GLOBAL
#define H_RDK2_GLOBAL

#include <rdkcore/common/threads.h>
#include <rdkcore/container/container.h>
#include <rdkcore/demangle/demangle.h>
#include <rdkcore/filesystem/filesystem.h>
#include <rdkcore/geometry/angle.h>
#include <rdkcore/geometry/dmatrix.h>
#include <rdkcore/geometry/dmatrix_algebra.h>
#include <rdkcore/geometry/dmatrix_pgm.h>
#include <rdkcore/geometry/dmatrix_point.h>
#include <rdkcore/geometry/dsp.h>
#include <rdkcore/geometry/geometryfunctions.h>
#include <rdkcore/geometry/otherpoints.h>
#include <rdkcore/geometry/output.h>
#include <rdkcore/geometry/pgmfile.h>
#include <rdkcore/geometry/point.h>
#include <rdkcore/geometry/point2.h>
#include <rdkcore/geometry/point2o.h>
#include <rdkcore/geometry/point3.h>
#include <rdkcore/geometry/posenormaldist.h>
#include <rdkcore/geometry/reference.h>
#include <rdkcore/geometry/robotmodels.h>
#include <rdkcore/geometry/segment.h>
#include <rdkcore/geometry/segment2.h>
#include <rdkcore/geometry/utils.h>
#include <rdkcore/geometry/viewport.h>
#include <rdkcore/geometry/walk_circle.h>
#include <rdkcore/geometry/walk_line.h>
#include <rdkcore/geometry/walk_quad.h>
#include <rdkcore/logging/levelfilter.h>
#include <rdkcore/logging/logfile.h>
#include <rdkcore/logging/logging.h>
#include <rdkcore/logging/logging_imp.h>
#include <rdkcore/logging/streamsink.h>
#include <rdkcore/modules/module.h>
#include <rdkcore/modules/modulemanager.h>
#include <rdkcore/modules_config/moduleconfig.h>
#include <rdkcore/modules_config/pair.h>
#include <rdkcore/modules_config/ragentconfig.h>
#include <rdkcore/network/inetaddress.h>
#include <rdkcore/network/netexception.h>
#include <rdkcore/network/socketaddress.h>
#include <rdkcore/network/tcpserversocket.h>
#include <rdkcore/network/tcpsocket.h>
#include <rdkcore/network/udpsocket.h>
#include <rdkcore/object/object.h>
#include <rdkcore/object/objectdiff.h>
#include <rdkcore/object/objectmanager.h>
#include <rdkcore/posixconstructs/posixmutex.h>
#include <rdkcore/posixconstructs/posixsem.h>
#include <rdkcore/posixconstructs/posixset.h>
#include <rdkcore/posixconstructs/stack.h>
#include <rdkcore/posixqueues/interests.h>
#include <rdkcore/posixqueues/pipequeue.h>
#include <rdkcore/posixqueues/queueinterface.h>
#include <rdkcore/profiling/profiling.h>
#include <rdkcore/rcoordination/rcoordobject.h>
#include <rdkcore/repository/events.h>
#include <rdkcore/repository/exceptions.h>
#include <rdkcore/repository/remotepropertymanager.h>
#include <rdkcore/repository/remotesubscriptionregister.h>
#include <rdkcore/repository/repository.h>
#include <rdkcore/repository/session.h>
#include <rdkcore/repository/tcpmanager.h>
#include <rdkcore/repository/udpmanager.h>
#include <rdkcore/repository_struct/exceptions.h>
#include <rdkcore/repository_struct/property.h>
#include <rdkcore/repository_struct/robjectqueue.h>
#include <rdkcore/repository_struct/rpropertydef.h>
#include <rdkcore/repository_struct/rpropertydefvector.h>
#include <rdkcore/repository_struct/rpropertyupdate.h>
#include <rdkcore/repository_struct/rremotesubscription.h>
#include <rdkcore/repository_struct/url.h>
#include <rdkcore/rgeometry/rpoint2d.h>
#include <rdkcore/rgeometry/rpoint2dvector.h>
#include <rdkcore/rgeometry/rpoint2i.h>
#include <rdkcore/rgeometry/rpoint2od.h>
#include <rdkcore/rgeometry/rpoint2odvector.h>
#include <rdkcore/rgeometry/rpoint3d.h>
#include <rdkcore/rgraphics/color.h>
#include <rdkcore/rgraphics/imgdiffs.h>
#include <rdkcore/rgraphics/rc8set.h>
#include <rdkcore/rgraphics/rcolor.h>
#include <rdkcore/rgraphics/rimage.h>
#include <rdkcore/rgraphics/rimagediffpoints.h>
#include <rdkcore/rgraphics/rimagediffrect.h>
#include <rdkcore/rgraphics/rimagepointc8.h>
#include <rdkcore/rgraphics/rposevector.h>
#include <rdkcore/rmaps/rfeatureonmapvector.h>
#include <rdkcore/rmaps/rfidtagonmap.h>
#include <rdkcore/rmaps/rfrontier.h>
#include <rdkcore/rmaps/rfrontiervector.h>
#include <rdkcore/rmaps/rhbdscanresult.h>
#include <rdkcore/rmaps/ritemonmapvector.h>
#include <rdkcore/rmaps/rline2donmap.h>
#include <rdkcore/rmaps/rmapimage.h>
#include <rdkcore/rmaps/rmapimagediffpoints.h>
#include <rdkcore/rmaps/rmapimagediffrect.h>
#include <rdkcore/rmaps/rpathanswer.h>
#include <rdkcore/rmaps/rpathrequest.h>
#include <rdkcore/rmaps/rpoint2donmap.h>
#include <rdkcore/rmaps/rpotentialvictimonmap.h>
#include <rdkcore/rmaps/rvictimonmap.h>
#include <rdkcore/rnetobjects/rnetmessage.h>
#include <rdkcore/rnetobjects/rnetmessage_interests.h>
#include <rdkcore/rnetobjects/ryellowpages.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rprimitive/rbool.h>
#include <rdkcore/rprimitive/rdouble.h>
#include <rdkcore/rprimitive/rint.h>
#include <rdkcore/rprimitive/rprimitive.h>
#include <rdkcore/rprimitive/rstring.h>
#include <rdkcore/rprimitive/rstringvector.h>
#include <rdkcore/rsensordata/ractuadata.h>
#include <rdkcore/rsensordata/rblobdata.h>
#include <rdkcore/rsensordata/rircdata.h>
#include <rdkcore/rsensordata/rlaserdata.h>
#include <rdkcore/rsensordata/rodometrydata.h>
#include <rdkcore/rsensordata/rsonardata.h>
#include <rdkcore/rsensordata/rusarinudata.h>
#include <rdkcore/rsensordata/rusarrfiddata.h>
#include <rdkcore/rsensordata/rusarvictimrfiddata.h>
#include <rdkcore/sensordata/actuadata.h>
#include <rdkcore/sensordata/ircdata.h>
#include <rdkcore/sensordata/laserdata.h>
#include <rdkcore/sensordata/laserparser.h>
#include <rdkcore/sensordata/logreader.h>
#include <rdkcore/sensordata/odometrydata.h>
#include <rdkcore/sensordata/sensordata.h>
#include <rdkcore/sensordata/sonardata.h>
#include <rdkcore/serialization/exceptions.h>
#include <rdkcore/serialization/read.h>
#include <rdkcore/serialization/utils.h>
#include <rdkcore/serialization/write.h>
#include <rdkcore/serialization_binary/binaryreader.h>
#include <rdkcore/serialization_binary/binarywriter.h>
#include <rdkcore/serialization_binary/constants.h>
#include <rdkcore/serialization_binary/packets.h>
#include <rdkcore/serialization_xml/xmlreader.h>
#include <rdkcore/serialization_xml/xmlutils.h>
#include <rdkcore/serialization_xml/xmlwriter.h>
#include <rdkcore/stat/stat.h>
#include <rdkcore/textutils/linestream.h>
#include <rdkcore/textutils/textutils.h>
#include <rdkcore/time/time.h>

#endif

