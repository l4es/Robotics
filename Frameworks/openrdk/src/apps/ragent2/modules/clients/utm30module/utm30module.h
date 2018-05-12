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

#ifndef RDK2_MODULE_UTM30MODULE
#define RDK2_MODULE_UTM30MODULE

#include <rdkcore/modules/module.h>
#include <rdkcore/robot/robotmodule.h>
#include <rdkcore/rsensordata/rlaserdata.h>

using namespace RDK2::RSensorData;
using namespace RDK2::Geometry;
using namespace RDK2::SensorData;

extern "C" {
#include "hokuyo.h"
}


namespace RDK2 { namespace RAgent {

/**
 * @brief Client for Hokuyo UTM 30 laser
 *
 * @ingroup RAgentModules
 */
class UTM30Module : public Module {
public:
	UTM30Module()  { }
	virtual ~UTM30Module() { }

	bool initConfigurationProperties();
	bool init();
	void exec();
	void cleanup();
	
private:
	void readLaser() throw (SessionException);
	HokuyoLaser urg;
	char buf[HOKUYO_BUFSIZE];
};

}} // namespace

#endif
