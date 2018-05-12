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

#include "rblobdata.h"
#include "rircdata.h"
#include "rjoystickstatus.h"
#include "rlaserdata.h"
#include "rodometrydata.h"
#include "rpantiltdata.h"
#include "rsonardata.h"
#include "rstereoimageheightsdata.h"
#include "rtemperaturedata.h"
#include "rtouchsensordata.h"
#include "rusarinudata.h"
#include "rusarrfiddata.h"
#include "rusarvictimdata.h"
#include "rusarvictimrfiddata.h"
#include "rxsensdata.h"

#include "logtagtoobject.h"

namespace RDK2 { namespace RSensorData {

RDK2::Object* logTagToObject(cstr logtag)
{
	if (logtag == "FLASER") return new RLaserData;
	else if (logtag == "ODOM") return new ROdometryData;
	else if (logtag == "PANTILT") return new RPantiltData;
	else if (logtag == "TEMPERATURE") return new RTemperatureData;
	else return 0;
}

}} // ns
