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

#ifndef RDK2_MODULE_PLAYER203MODULE
#define RDK2_MODULE_PLAYER203MODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rsensordata/rlaserdata.h>
#include <rdkcore/rsensordata/rsonardata.h>

#include <libplayerc++/playerc++.h>

using namespace PlayerCc;
using namespace RDK2::RSensorData;
using namespace RDK2::Geometry;
using namespace RDK2::SensorData;

namespace RDK2 { namespace RAgent {

/**
 * @brief Client for Player
 *
 * @ingroup RAgentModules
 */
class Player2Module : public Module {
public:
	Player2Module() :
		playerClient(0),
		positionProxy(0),
		sonarProxy(0),
		laserProxy(0),
		ptzProxy(0),
		blobfinderProxy(0),
		fiducialProxy(0),
		simulationProxy(0) { }
	virtual ~Player2Module() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void cleanup();

	void asyncAgentCmd(cstr cmd);
	
private:
	PlayerClient* playerClient;
	PlayerClient* simClient;
	Position2dProxy* positionProxy;
	SonarProxy* sonarProxy;
	LaserProxy* laserProxy;
	PtzProxy* ptzProxy;
	BlobfinderProxy* blobfinderProxy;
	FiducialProxy* fiducialProxy;
	SimulationProxy* simulationProxy;

	void readLaser() throw (SessionException);
	void readSonar() throw (SessionException);
	void readImage() throw (SessionException);
	void readFiducial() throw (SessionException);
	void readOdometry() throw (SessionException);
	void readGroundTruthPose() throw (SessionException);

	void updatePlayerSpeed(Session* session) throw (SessionException);

	bool parseRobotCmd(cstr cmd, Session* session);
};

}} // namespace

#endif
