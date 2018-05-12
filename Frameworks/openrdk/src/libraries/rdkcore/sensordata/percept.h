/*
 * =====================================================================================
 *
 *       Filename:  percept.h
 *
 *    Description:  A base representations for percepts
 *
 *        Version:  1.0
 *        Created:  05/21/2009 04:37:24 PM
 *
 *         Author:  Luca Marchetti (lm), ximilian@gmail.com
 *        Company:  OpenRDK Developers Team
 *
 * =====================================================================================
 */


#ifndef  PERCEPT_INC
#define  PERCEPT_INC


#include <rdkcore/geometry/pointstat.h>
#include <rdkcore/sensordata/sensordata.h>

namespace RDK2 { namespace SensorData {


	struct Percept: BaseSensorData, RDK2::Geometry::PolarPointCov
	{
		/** *structors */
		Percept(const std::string& s="PCPTGEN"):
			BaseSensorData(s),
			PolarPointCov()
		{
		}

		virtual ~Percept();

		//Percept(const Percept& p):
		//  BaseSensorData(p),
		//  PolarPoint(p),
		//  confidence(p.confidence)
		//{
		//}
	};				/* ----------  end of struct Percept  ---------- */
}}

#endif   /* ----- #ifndef PERCEPT_INC  ----- */
